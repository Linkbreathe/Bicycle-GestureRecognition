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

void setup() {
  Serial.begin(9600);

  // Set GPIO pins as outputs
  pinMode(gpioPin_left, OUTPUT);
  pinMode(gpioPin_right, OUTPUT);
  pinMode(gpioPin_braking, OUTPUT);

  // Start Bluetooth
  SerialBT.begin(device_name);
  Serial.printf("Device \"%s\" started. Pair it via Bluetooth!\n", device_name.c_str());
}

void loop() {
  // Handle incoming Bluetooth commands
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    incoming.trim();
    Serial.println("Received from Master: " + incoming);

    if (incoming == "LEFT") {
      blinkingLeft = true;
      blinkingRight = false;
    } else if (incoming == "RIGHT") {
      blinkingRight = true;
      blinkingLeft = false;
    } else if (incoming == "STOP") {
      blinkingLeft = false;
      blinkingRight = false;
      digitalWrite(gpioPin_left, LOW);
      digitalWrite(gpioPin_right, LOW);
    } else if (incoming == "BRAKING") {
      digitalWrite(gpioPin_braking, HIGH);
    } else if (incoming == "STOP BRAKING") {
      Serial.print("Hallo");
      digitalWrite(gpioPin_braking, LOW);
    }
  }

  // Handle continuous blinking
  if (blinkingLeft) {
    blink(gpioPin_left);
  } else if (blinkingRight) {
    blink(gpioPin_right);
  } else {
    delay(20); // Avoid busy-waiting
  }
}

void blink(int pin) {
  static unsigned long lastBlinkTime = 0;
  static bool pinState = LOW;

  unsigned long currentTime = millis();
  if (currentTime - lastBlinkTime >= 1000) {
    lastBlinkTime = currentTime;
    pinState = !pinState;
    digitalWrite(pin, pinState);
  }
}
