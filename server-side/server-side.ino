#include "BluetoothSerial.h"
#include <CapacitiveSensor.h>

// Define preprocessor macros
#define USE_NAME // Comment to use MAC address instead of a slaveName

// Check if Bluetooth is enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it.
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. Only available for ESP32.
#endif

BluetoothSerial SerialBT;

// Bluetooth configuration
#ifdef USE_NAME
String slaveName = "ESP32-BT-Slave";
#else
String MACadd = "AA:BB:CC:11:22:33";
uint8_t address[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};
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

// Initialize left and right sliders with reduced thresholds
SliderConfig leftSliders[] = {
  {CapacitiveSensor(senderPin, 13), 60, 147, 0.0},
  {CapacitiveSensor(senderPin, 12), 30, 130, 0.5},
  {CapacitiveSensor(senderPin, 14), 40, 154, 1.0}
};

SliderConfig rightSliders[] = {
  {CapacitiveSensor(senderPin, 26), 80, 147, 0.0},
  {CapacitiveSensor(senderPin, 33), 65, 130, 0.5},
  {CapacitiveSensor(senderPin, 32), 60, 154, 1.0}
};

// Sliding state variables
const int meanWindowSize = 3; // Reduced for responsiveness
const double slideThreshold = 0.03; // Reduced for greater sensitivity
const unsigned long slideEndTimeout = 500; // Increased for slower movements
const double restingDecayFactor = 0.95; // Decay for resting pressure
double leftPositionHistory[meanWindowSize] = {0};
double rightPositionHistory[meanWindowSize] = {0};
int leftHistoryIndex = 0, rightHistoryIndex = 0;
double previousLeftMeanPos = -1.0, previousRightMeanPos = -1.0;
bool isSlidingLeft = false, isSlidingRight = false;
unsigned long lastSlideLeftTime = 0, lastSlideRightTime = 0;

// Messaging state
bool stopMessageSent = false;

// Function prototypes
double calculateWeightedPosition(SliderConfig sliders[], double positionHistory[], int &historyIndex, double &previousMeanPos, bool &isSliding, unsigned long &lastSlideTime, String direction);
double calculateRollingMean(double positionHistory[]);
void detectSlide(double meanPos, double &previousMeanPos, bool &isSliding, unsigned long &lastSlideTime, String direction);
void sendMessage(String message);
void checkAndSendStopMessage();

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

  int maxRetries = 0;
  while (!connected && maxRetries--) {
    Serial.println("Retrying Bluetooth connection...");
    delay(1000);
    connected = SerialBT.connect(slaveName);
  }

  if (connected) {
    Serial.println("Connected Successfully!");
  } else {
    Serial.println("Failed to connect after multiple attempts.");
  }
}

void loop() {
  // Calculate positions for sliders
  double leftMeanPos = calculateWeightedPosition(leftSliders, leftPositionHistory, leftHistoryIndex, previousLeftMeanPos, isSlidingLeft, lastSlideLeftTime, "LEFT");
  double rightMeanPos = calculateWeightedPosition(rightSliders, rightPositionHistory, rightHistoryIndex, previousRightMeanPos, isSlidingRight, lastSlideRightTime, "RIGHT");

  // Check if stop message should be sent
  checkAndSendStopMessage();

  delay(10);
}

double calculateWeightedPosition(SliderConfig sliders[], double positionHistory[], int &historyIndex, double &previousMeanPos, bool &isSliding, unsigned long &lastSlideTime, String direction) {
  double overallPosition = -1.0;
  int touchCount = 0;
  double totalWeight = 0.0;
  double weightedPosition = 0.0;

  Serial.printf("Raw sensor values for %s sliders:\n", direction.c_str());

  for (int i = 0; i < 3; i++) {
    double sensorValue = sliders[i].sensor.capacitiveSensor(20); // Increased sampling for better sensitivity
    Serial.printf("  Slider %d: %lf (minThreshold: %d)\n", i, sensorValue, sliders[i].minThreshold); // Debug output for raw values
    
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
    Serial.printf("%s, Weighted Mean Position: %.2f\n", direction.c_str(), meanPos);
  }

  return meanPos;
}

double calculateRollingMean(double positionHistory[]) {
  double mean = 0.0;
  double totalWeight = 0.0;
  double weight = 1.0;

  // Apply a decay factor to prioritize recent readings
  for (int i = meanWindowSize - 1; i >= 0; i--) {
    if (positionHistory[i] >= 0) {
      mean += positionHistory[i] * weight;
      totalWeight += weight;
      weight *= 0.8; // Decay factor for older readings
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
      stopMessageSent = false;
      delay(5000);
    }
    // Apply decay to previousMeanPos to simulate resting state
    previousMeanPos *= restingDecayFactor;
  }
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
