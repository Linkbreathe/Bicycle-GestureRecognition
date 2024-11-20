#include "BluetoothSerial.h"
#include <CapacitiveSensor.h>

#define USE_NAME  // Comment this to use MAC address instead of a slaveName

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. Only available for ESP32.
#endif

BluetoothSerial SerialBT;

#ifdef USE_NAME
String slaveName = "ESP32-BT-Slave";
#else
String MACadd = "AA:BB:CC:11:22:33";
uint8_t address[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};
#endif

String myName = "ESP32-BT-Master";

const unsigned long sendInterval = 5000;
unsigned long previousSendMillis = 0;
bool stopMessageSent = false;

int senderPin = 25;

// Slider configuration
struct SliderConfig {
  CapacitiveSensor sensor;
  int minThreshold;
  int maxValue;
  double weight;
};

struct BrakeConfig {
  CapacitiveSensor sensor;
  int minThreshold;
  int maxValue;
};

SliderConfig leftSliders[] = {
  {CapacitiveSensor(senderPin, 13), 60, 147, 0.0},
  {CapacitiveSensor(senderPin, 12), 20, 130, 0.5},
  {CapacitiveSensor(senderPin, 14), 30, 154, 1.0}
};

SliderConfig rightSliders[] = {
  {CapacitiveSensor(senderPin, 27), 35, 147, 1.0},
  {CapacitiveSensor(senderPin, 33), 50, 130, 0.5},
  {CapacitiveSensor(senderPin, 32), 21, 154, 0.0}
};

BrakeConfig brake = {CapacitiveSensor(senderPin, 2), 100, 1000};

// Slide detection configuration
const int meanWindowSize = 5;
const double slideThreshold = 0.09;
const unsigned long slideEndTimeout = 500;
double leftPositionHistory[meanWindowSize] = {0};
double rightPositionHistory[meanWindowSize] = {0};
int leftHistoryIndex = 0, rightHistoryIndex = 0;
double previousLeftMeanPos = -1.0, previousRightMeanPos = -1.0;
bool isSlidingLeft = false, isSlidingRight = false;
unsigned long lastSlideLeftTime = 0, lastSlideRightTime = 0;

// Brake detection configuration
bool isBreaking = false;
bool lastBrakeState = false;
int brakeThreshold = 2000;
long smoothedTouchValue = 0;
const float smoothingFactor = 0.2;

void setup() {
  Serial.begin(9600);
  SerialBT.begin(myName, true);

  Serial.printf("The device \"%s\" started in master mode, ensure slave BT device is on!\n", myName.c_str());

  bool connected;
  #ifdef USE_NAME
    Serial.printf("Connecting to slave BT device named \"%s\"\n", slaveName.c_str());
    connected = SerialBT.connect(slaveName);
  #else
    Serial.printf("Connecting to slave BT device with MAC %s\n", MACadd.c_str());
    connected = SerialBT.connect(address);
  #endif

  if (connected) {
    Serial.println("Connected Successfully!");
  } else {
    Serial.println("Failed to connect. Retrying...");
    // Optional: add retry logic
  }
}

void loop() {
  // Calculate positions for sliders
  double leftMeanPos = calculateWeightedPosition(leftSliders, leftPositionHistory, leftHistoryIndex, previousLeftMeanPos, isSlidingLeft, lastSlideLeftTime, "LEFT");
  double rightMeanPos = calculateWeightedPosition(rightSliders, rightPositionHistory, rightHistoryIndex, previousRightMeanPos, isSlidingRight, lastSlideRightTime, "RIGHT");

  // Brake check and message sending
  checkBrakeState();

  // Check if stop message should be sent
  checkAndSendStopMessage();

  delay(100);
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
      double weight = sensorValue / sliders[i].maxValue;
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

  if (meanPos >= 0.0) {
    Serial.printf("%s, %.2f\n", direction.c_str(), meanPos);
  }

  return meanPos;
}

double calculateRollingMean(double positionHistory[]) {
  double mean = 0.0;
  int validReadings = 0;

  for (int i = 0; i < meanWindowSize; i++) {
    if (positionHistory[i] >= 0) {
      mean += positionHistory[i];
      validReadings++;
    }
  }

  return (validReadings > 0) ? mean / validReadings : -1.0;
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
    if (isSliding) {
      if (millis() - lastSlideTime < slideEndTimeout) {
        Serial.printf("LIGHT %s\n", direction.c_str());
        sendMessage(direction);
        delay(5000);
        stopMessageSent = false;
      } else {
        Serial.println("Slide Ended");
      }
      isSliding = false;
    }
    previousMeanPos = -1.0;
  }
}

void checkBrakeState() {
  // Read and smooth the brake sensor value
  long rawBrakeValue = brakeSensor.capacitiveSensor(10);
  smoothedBrakeValue = (smoothingFactor * rawBrakeValue) + ((1 - smoothingFactor) * smoothedBrakeValue);

  // Determine if braking is happening
  isBreaking = smoothedBrakeValue >= brakeThreshold;

  // If the brake state has changed, send the appropriate message
  if (isBreaking != lastBrakeState) {
    if (isBreaking) {
      sendMessage("BRAKE");
    } else {
      sendMessage("STOP BRAKE");
    }
    lastBrakeState = isBreaking; // Update the last known state
  }
}

// Send message over Bluetooth
void sendMessage(String message) {
  Serial.println("Sent to Slave: " + message);
  SerialBT.println(message);
}

void sendMessage(String message) {
  Serial.println("Sent to Slave: " + message);
  SerialBT.println(message);
}

void checkAndSendStopMessage() {
  if (!stopMessageSent) {
    sendMessage("STOP");
    stopMessageSent = true;
  }
}
