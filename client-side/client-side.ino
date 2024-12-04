#include "BluetoothSerial.h"

// Device Bluetooth Name
String device_name = "ESP32-BT-Slave";

BluetoothSerial SerialBT;

// GPIO Pins for EL Displays
const int gpioPin_left = 23;      
const int gpioPin_right = 15;     
const int gpioPin_braking = 27;   

// Blinking states
bool blinkingLeft = false;
bool blinkingRight = false;
bool blinkBrake = false;

// Separate timing and state variables for each pin
unsigned long lastBlinkTimeLeft = 0;
bool pinStateLeft = LOW;

unsigned long lastBlinkTimeRight = 0;
bool pinStateRight = LOW;

unsigned long lastBlinkTimeBrake = 0;
bool pinStateBrake = LOW;

const unsigned long blinkInterval = 500; // Flashing interval (ms)

void setup() {
  Serial.begin(9600);

  // Set GPIO pins as outputs
  pinMode(gpioPin_left, OUTPUT);
  pinMode(gpioPin_right, OUTPUT);
  pinMode(gpioPin_braking, OUTPUT);
  
  digitalWrite(gpioPin_left, HIGH);
  digitalWrite(gpioPin_right, HIGH);
  digitalWrite(gpioPin_braking, HIGH);
  
  // Start Bluetooth
  SerialBT.begin(device_name);
  Serial.printf("Device \"%s\" started. Pair it via Bluetooth!\n", device_name.c_str());
}

void loop() {

  // Handle incoming Bluetooth commands
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    // clear the buffer
    SerialBT.read();
    incoming.trim();
    Serial.println("Received from Master: " + incoming);

    if (incoming == "LEFT") {
      blinkingLeft = true;
      blinkingRight = false;
      // Ensure braking light state is maintained
    } else if (incoming == "RIGHT") {
      blinkingRight = true;
      blinkingLeft = false;
      // Ensure braking light state is maintained
    } else if (incoming == "STOP") {
      blinkingLeft = false;
      blinkingRight = false;
      digitalWrite(gpioPin_left, HIGH);
      digitalWrite(gpioPin_right, HIGH);
    } else if (incoming == "BRAKING") {
      blinkBrake = true;
      // Optionally, set braking light to start blinking immediately
    } else if (incoming == "STOP BRAKING") {
      blinkBrake = false;
      digitalWrite(gpioPin_braking, LOW); // Switch off the brake lights.
    }
  }

  unsigned long currentTime = millis();

  // Handle left blinking
  if (blinkingLeft) {
    if (currentTime - lastBlinkTimeLeft >= blinkInterval) {
      lastBlinkTimeLeft = currentTime;
      pinStateLeft = !pinStateLeft;
      digitalWrite(gpioPin_left, pinStateLeft ? HIGH : LOW);
    }
  }

  // Handle right blinking
  if (blinkingRight) {
    if (currentTime - lastBlinkTimeRight >= blinkInterval) {
      lastBlinkTimeRight = currentTime;
      pinStateRight = !pinStateRight;
      digitalWrite(gpioPin_right, pinStateRight ? HIGH : LOW);
    }
  }

  // Handle braking blinking
  if (blinkBrake) {
    if (currentTime - lastBlinkTimeBrake >= blinkInterval) {
      lastBlinkTimeBrake = currentTime;
      pinStateBrake = !pinStateBrake;
      digitalWrite(gpioPin_braking, pinStateBrake ? HIGH : LOW);
    }
  }

  // Small delay to prevent high CPU usage
  delay(10);
}
