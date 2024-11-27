#include "BluetoothSerial.h"
#include <FastLED.h>


String device_name = "ESP32-BT-Slave";

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

unsigned long ledStateStartTime = 0;
const unsigned long ledStateDuration = 1000;  // Duration for each LED state in milliseconds

// Parameters to define to function to control the EL displays
const int frequency = 100;     // Hz
const int dutyResolution = 8;  // 8-bit resolution
const int gpioPin = 26;        // GPIO pin to the PWM signal
const int maxDutyCycle = 255;  // Maximum value for 8-bit resolution
const int delayTime = 8;       // 255 * 8

const int gpioPin_collar = 27;          // gpion Pin for collar
const int gpioPin_right = 15;          // gpion Pin for right
const int gpioPin_left = 23;          // gpion Pin for left

const int channelNumber_collar = 0;     // channel number collar
const int channelNumber_right = 1;     // channel number right
const int channelNumber_left = 2;     // channel number left

const int GlowTiming = 15000; // Continuous glow time (ms) for turning right and left

const int BlinkingTiming = 10000; // Continuous glow time (ms) for collar

const int BlinkingInterval = 3000; // Continuous blinking time(ms) for collar part

void setup() {
  Serial.begin(9600);

  // set up EL Displays
  // EL displays for collar
  ledcSetup(channelNumber_collar, frequency, dutyResolution);
  ledcAttachPin(gpioPin_collar, channelNumber_collar);

  // EL displays for turning right
  ledcSetup(channelNumber_right, frequency, dutyResolution);
  ledcAttachPin(gpioPin_right, channelNumber_right);

  // EL displays for turning left
  ledcSetup(gpioPin_left, frequency, dutyResolution);
  ledcAttachPin(gpioPin_left, gpioPin_left);

  // set up Bluetooth
  SerialBT.begin(device_name);  // Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
}

void loop() {
  
  // Check for messages from the master device
  // if (SerialBT.available()) {
  //   String incoming = SerialBT.readStringUntil('\n');
  //   Serial.println("Received from Master: " + incoming);
  // }
  left_ELDisplays();
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    incoming.trim();
    Serial.println("Received from Master: " + incoming);

    if (incoming == "LEFT") {
      left_ELDisplays();
    } else if(incoming == "RIGHT"){
      right_ElDisplays();
    }else if (incoming == "STOP") {
      turnOffLEDs();
    } 
  }

  delay(20);
}

void left_ELDisplays() {
  unsigned long startTime = millis();
  while (millis() - startTime < GlowTiming) {
    // Turn on the left LED at maximum brightness
    ledcWrite(channelNumber_left, maxDutyCycle);
    delay(500); // Delay for 500 ms

    // Turn off the left LED
    ledcWrite(channelNumber_left, 0);
    delay(500); // Delay for 500 ms
  }
  // Ensure the left LED is turned off after blinking
  
  ledcWrite(channelNumber_left, 0);
}

void right_ElDisplays() {
  unsigned long startTime = millis();
  while (millis() - startTime < GlowTiming) {
    // Turn on the right LED at maximum brightness
    ledcWrite(channelNumber_right, maxDutyCycle);
    delay(500); // Delay for 500 ms

    // Turn off the right LED
    ledcWrite(channelNumber_right, 0);
    delay(500); // Delay for 500 ms
  }
  // Ensure the right LED is turned off after blinking
  ledcWrite(channelNumber_right, 0);
}

 // Blinks once per BlinkingInterval within the specified BlinkingTiming. 
void turnOffLEDs() {
   unsigned long startTime = millis();
  
  // Blink the collar LED to indicate LEDs are turning off
  while (millis() - startTime < BlinkingTiming) {
    // Turn on the collar LED at maximum brightness
    ledcWrite(channelNumber_collar, maxDutyCycle);
    delay(500); // LED on for 500 ms

    // Turn off the collar LED
    ledcWrite(channelNumber_collar, 0);
    delay(500); // LED off for 500 ms

    // Wait for the remaining interval time
    unsigned long elapsed = millis() - startTime;
    if ((elapsed % BlinkingInterval) < (500 + 500)) {
      delay(BlinkingInterval - (elapsed % BlinkingInterval));
    }
  }

  // Ensure all LEDs are turned off after blinking
  ledcWrite(channelNumber_collar, 0);

  // Serial.println("All LEDs have been turned off.");
}


