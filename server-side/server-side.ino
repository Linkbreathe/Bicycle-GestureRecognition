#include "BluetoothSerial.h"
#include <CapacitiveSensor.h>

// Define preprocessor macros
#define USE_NAME  // Comment to use MAC address instead of a slaveName

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to enable it.
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. Only available for ESP32.
#endif

BluetoothSerial SerialBT;

// Bluetooth configuration
#ifdef USE_NAME
String slaveName = "ESP32-BT-Slave";
#else
String MACadd = "88:BE:D1:22:C9:30";
uint8_t address[6] = { 0x88, 0xBE, 0xD1, 0x22, 0xC9, 0x30 };
#endif

String myName = "ESP32-BT-Master";

// Slider configuration
const int senderPin = 25;
struct SliderConfig {
  CapacitiveSensor sensor;
  int minThreshold;
  int maxValue;
  double weight;
};

SliderConfig leftSliders[] = {
  { CapacitiveSensor(senderPin, 13), 150, 300, 0.0 },
  { CapacitiveSensor(senderPin, 12), 150, 300, 0.5 },
  { CapacitiveSensor(senderPin, 14), 150, 300, 1.0 }
};

SliderConfig rightSliders[] = {
  { CapacitiveSensor(senderPin, 26), 150, 300, 1.0 },
  { CapacitiveSensor(senderPin, 33), 150, 300, 0.5 },
  { CapacitiveSensor(senderPin, 32), 150, 300, 0.0 }
};

// Sliding state variables
const int meanWindowSize = 5;
const double slideThreshold = 0.1;
const double touchSensitivity = 0.3;
const unsigned long slideEndTimeout = 1500;
const double restingDecayFactor = 0.95;
double leftPositionHistory[meanWindowSize] = { 0 };
double rightPositionHistory[meanWindowSize] = { 0 };
int leftHistoryIndex = 0, rightHistoryIndex = 0;
double previousLeftMeanPos = -1.0, previousRightMeanPos = -1.0;
bool isSlidingLeft = false, isSlidingRight = false;
unsigned long lastSlideLeftTime = 0, lastSlideRightTime = 0;

// System states
enum SystemState {
  IDLE,
  WAIT_FOR_BOTH_CONTACT
};

SystemState currentState = IDLE;

// Messaging state
bool stopMessageSent = false;

// Parameter settings for braking
const int fsrPin = 2;
bool isBraking = false;

const unsigned long calibrationTime = 5000;
const int numCalibrationSamples = 100;
float baselineVoltage = 0.0;
float brakingThreshold = 0.0;  // Will be set dynamically after calibration

// Parameter for reconnecting
bool wasConnected = false;
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000;  // Try reconnect every 5s if disconnected

String previousMessage = "";

// Function prototypes
double calculateWeightedPosition(SliderConfig sliders[], double positionHistory[], int &historyIndex, double &previousMeanPos, bool &isSliding, unsigned long &lastSlideTime, String direction);
double calculateRollingMean(double positionHistory[]);
void detectSlide(double meanPos, double &previousMeanPos, bool &isSliding, unsigned long &lastSlideTime, String direction);
void sendMessage(String message);
void calibrateBaseline();
void calibrateBrakeThreshold();
void calibrateSliders(SliderConfig sliders[], int numSliders, String side);
void detectBraking();
void attemptReconnectIfNeeded();
bool bothSidesInContact();

void setup() {
  Serial.begin(9600);

  analogReadResolution(12);
  calibrateBaseline();

  SerialBT.begin(myName, true);
  Serial.printf("The device \"%s\" started in master mode, ensure slave BT device is on!\n", myName.c_str());

  bool connected;
#ifdef USE_NAME
  Serial.printf("Connecting to slave BT device named \"%s\"...\n", slaveName.c_str());
  connected = SerialBT.connect(slaveName);
#else
  Serial.printf("Connecting to slave BT device with MAC %s...\n", MACadd.c_str());
  connected = SerialBT.connect(address);
#endif

  int maxRetries = 5;
  while (!connected && maxRetries--) {
    Serial.println("Retrying Bluetooth connection...");
    delay(1000);
    connected =
#ifdef USE_NAME
      SerialBT.connect(slaveName);
#else
      SerialBT.connect(address);
#endif
  }

  if (connected) {
    Serial.println("Connected Successfully!");
    wasConnected = true;
  } else {
    Serial.println("Failed to connect after multiple attempts.");
  }

  calibrateSliders(leftSliders, 3, "Left");
  calibrateSliders(rightSliders, 3, "Right");
  calibrateBrakeThreshold();

  currentState = IDLE; // Start in IDLE
}

void loop() {
  attemptReconnectIfNeeded();

  if (SerialBT.hasClient()) {
    double leftMeanPos = calculateWeightedPosition(leftSliders, leftPositionHistory, leftHistoryIndex, previousLeftMeanPos, isSlidingLeft, lastSlideLeftTime, "LEFT");
    double rightMeanPos = calculateWeightedPosition(rightSliders, rightPositionHistory, rightHistoryIndex, previousRightMeanPos, isSlidingRight, lastSlideRightTime, "RIGHT");

    detectBraking();
    
    // Handle states
    if (currentState == WAIT_FOR_BOTH_CONTACT) {
      // We have sent a direction message. Now wait until both sides are back in contact to send STOP.
      if (bothSidesInContact()) {
        if (!stopMessageSent) {
          sendMessage("STOP");
          stopMessageSent = true;
        }
        // Transition back to IDLE after STOP
        currentState = IDLE;
      }
    }
  }

  delay(10);
}


void calibrateBaseline() {
  Serial.println("Calibrating baseline (no press on brake)... Please do not press the brake.");
  unsigned long startTime = millis();
  unsigned long endTime = startTime + calibrationTime;
  unsigned long sum = 0;
  int samples = 0;

  while (millis() < endTime && samples < numCalibrationSamples) {
    int raw = analogRead(fsrPin);
    sum += raw;
    samples++;
    delay(10);
  }

  baselineVoltage = (sum / (float)samples) * (3.3 / 4095.0);
  Serial.print("Baseline Voltage: ");
  Serial.println(baselineVoltage, 3);
  Serial.println("Baseline calibration complete.");
}

void calibrateBrakeThreshold() {
  Serial.println("\nCalibrating brake threshold...");
  Serial.println("Please press and hold the brake now. The calibration will start once a press is detected.");

  float pressDetectionThreshold = 0.05;  
  float netVoltage = 0.0;

  while (true) {
    int rawValue = analogRead(fsrPin);
    float voltage = rawValue * (3.3 / 4095.0);
    netVoltage = voltage - baselineVoltage;

    if (netVoltage > pressDetectionThreshold) {
      Serial.println("Brake press detected. Starting calibration measurement...");
      break;
    } else {
      static unsigned long lastPromptTime = 0;
      unsigned long currentTime = millis();
      if (currentTime - lastPromptTime > 3000) {
        Serial.println("Please press and hold the brake firmly...");
        lastPromptTime = currentTime;
      }
      delay(100);
    }
  }

  unsigned long pressCalibrationTime = 3000; 
  unsigned long startTime = millis();
  float maxNetVoltage = 0.0;

  while (millis() - startTime < pressCalibrationTime) {
    int rawValue = analogRead(fsrPin);
    float voltage = rawValue * (3.3 / 4095.0);
    netVoltage = voltage - baselineVoltage;
    if (netVoltage > maxNetVoltage) {
      maxNetVoltage = netVoltage;
    }
    delay(10);
  }

  Serial.println("You can release the brake now.");

  if (maxNetVoltage < pressDetectionThreshold) {
    Serial.println("[WARNING] Detected very low press values. Using a default threshold of 0.3");
    brakingThreshold = 0.3;
  } else {
    brakingThreshold = maxNetVoltage * 0.5;
    Serial.print("Calibrated braking threshold: ");
    Serial.println(brakingThreshold, 3);
  }
  Serial.println("Brake threshold calibration complete.\n");
}

double calculateWeightedPosition(SliderConfig sliders[], double positionHistory[], int &historyIndex, double &previousMeanPos, bool &isSliding, unsigned long &lastSlideTime, String direction) {
  double overallPosition = -1.0;
  int touchCount = 0;
  double totalWeight = 0.0;
  double weightedPosition = 0.0;

  for (int i = 0; i < 3; i++) {
    double sensorValue = sliders[i].sensor.capacitiveSensor(10);
    if (sensorValue > sliders[i].minThreshold) {
      touchCount++;
      double weight = (double)sensorValue / (double)sliders[i].maxValue;
      weightedPosition += sliders[i].weight * weight;
      totalWeight += weight;
    }
  }

  if (touchCount > 0) {
    overallPosition = weightedPosition / totalWeight;
  }

  positionHistory[historyIndex] = overallPosition;
  historyIndex = (historyIndex + 1) % meanWindowSize;

  double meanPos = calculateRollingMean(positionHistory);
  detectSlide(meanPos, previousMeanPos, isSliding, lastSlideTime, direction);

  return meanPos;
}

double calculateRollingMean(double positionHistory[]) {
  double mean = 0.0;
  double totalWeight = 0.0;
  double weight = 1.0;

  for (int i = meanWindowSize - 1; i >= 0; i--) {
    if (positionHistory[i] >= 0) {
      mean += positionHistory[i] * weight;
      totalWeight += weight;
      weight *= 0.8; 
    }
  }

  return (totalWeight > 0) ? mean / totalWeight : -1.0;
}

void detectSlide(double meanPos, double &previousMeanPos, bool &isSliding, unsigned long &lastSlideTime, String direction) {
  if (meanPos >= 0.0) {
    if (previousMeanPos == -1.0) {
      previousMeanPos = meanPos;
    } else if (meanPos < previousMeanPos - slideThreshold) {
      Serial.printf("Slide %s Detected\n", direction.c_str());
      isSliding = true;
      lastSlideTime = millis();
    }
    previousMeanPos = meanPos;
  } else {
    if (isSliding && millis() - lastSlideTime >= slideEndTimeout) {
      Serial.printf("LIGHT %s\n", direction.c_str());
      sendMessage(direction);
      isSliding = false;

      // After sending direction, we will wait until both sides are in contact before sending STOP.
      stopMessageSent = false;
      currentState = WAIT_FOR_BOTH_CONTACT; 
    }
    previousMeanPos *= restingDecayFactor;
  }
}

void detectBraking() {
  int rawValue = analogRead(fsrPin);
  float voltage = rawValue * (3.3 / 4095.0);
  float netVoltage = voltage - baselineVoltage;

  if (netVoltage > brakingThreshold) {
    if (!isBraking) {
      sendMessage("BRAKING");
      isBraking = true;
    }
  } else {
    if (isBraking) {
      sendMessage("STOP BRAKING");
      isBraking = false;
    }
  }
}

void calibrateSliders(SliderConfig sliders[], int numSliders, String side) {
  Serial.printf("\n--- %s Sliders Calibration Start ---\n", side.c_str());

  for (int i = 0; i < numSliders; i++) {
    long baseValue = 0;
    for (int j = 0; j < 1000; j++) {
      baseValue += sliders[i].sensor.capacitiveSensor(10);
      delay(10);
    }
    baseValue /= 1000;
    sliders[i].minThreshold = baseValue;
    Serial.printf("[INFO] Slider %d (%s) baseline: %ld | Min Threshold: %d\n", i + 1, side.c_str(), baseValue, sliders[i].minThreshold);
  }

  Serial.println("[INFO] Waiting for all sliders to be touched...");

  bool allTouched = false;
  while (!allTouched) {
    allTouched = true;
    for (int i = 0; i < numSliders; i++) {
      long sensorValue = sliders[i].sensor.capacitiveSensor(10);
      if (sensorValue <= sliders[i].minThreshold) {
        allTouched = false;
        break;
      }
    }
    delay(100);
  }

  Serial.println("[INFO] All sliders touched! Starting touch calibration...");

  for (int i = 0; i < numSliders; i++) {
    long touchValue = 0;
    for (int j = 0; j < 1000; j++) {
      touchValue += sliders[i].sensor.capacitiveSensor(10);
      delay(10);
    }
    touchValue /= 1000;
    sliders[i].maxValue = touchValue;
    Serial.printf("[INFO] Slider %d (%s) touch value: %ld | Max Value: %d\n", i + 1, side.c_str(), touchValue, sliders[i].maxValue);
  }

  for (int i = 0; i < numSliders; i++) {
    sliders[i].minThreshold = sliders[i].minThreshold + (sliders[i].maxValue - sliders[i].minThreshold) * touchSensitivity;
    Serial.printf("[INFO] Final minThreshold for Slider %d: %d | Max Value: %d | Sensitivity: %f\n",
                  i + 1, sliders[i].minThreshold, sliders[i].maxValue, touchSensitivity);
  }

  Serial.printf("--- %s Sliders Calibration Complete ---\n", side.c_str());
}


bool bothSidesInContact() {
  // Checks if both left and right side have at least one slider touched.
  bool leftContact = false;
  bool rightContact = false;

  for (int i = 0; i < 3; i++) {
    double leftValue = leftSliders[i].sensor.capacitiveSensor(10);
    if (leftValue > leftSliders[i].minThreshold) {
      leftContact = true;
      break;
    }
  }

  for (int i = 0; i < 3; i++) {
    double rightValue = rightSliders[i].sensor.capacitiveSensor(10);
    if (rightValue > rightSliders[i].minThreshold) {
      rightContact = true;
      break;
    }
  }

  return leftContact && rightContact;
}

void sendMessage(String message) {
  if (SerialBT.hasClient()) {
    Serial.println("Sent to Slave: " + message);
    SerialBT.println(message);
  } else {
    Serial.println("[WARNING] No slave connected. Cannot send: " + message);
  }
}

// Attempt reconnection only if needed and at controlled intervals
void attemptReconnectIfNeeded() {
  if (!SerialBT.hasClient()) {
    unsigned long currentTime = millis();
    if (!wasConnected || (currentTime - lastReconnectAttempt >= reconnectInterval)) {
      lastReconnectAttempt = currentTime;
      Serial.println("[INFO] Trying to reconnect...");
#ifdef USE_NAME
      bool isReconnect = SerialBT.connect(slaveName);
#else
      bool isReconnect = SerialBT.connect(address);
#endif
      if (isReconnect) {
        Serial.println("[INFO] RECONNECTED SUCCESSFULLY");
        wasConnected = true;
        sendMessage("RECONNECT");
      } else {
        Serial.println("[INFO] Reconnect attempt failed.");
      }
    }
  }
}
