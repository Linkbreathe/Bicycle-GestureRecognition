#include "BluetoothSerial.h" 
#include <FastLED.h>

#define NUM_LEDS 4
#define LED_PIN 13

CRGB leds[NUM_LEDS];

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

void setup() {
  Serial.begin(9600);
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);

  
  SerialBT.begin(device_name);  // Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
}

void loop() {
   // Check for messages from the master device
  // if (SerialBT.available()) {
  //   String incoming = SerialBT.readStringUntil('\n');
  //   Serial.println("Received from Master: " + incoming);
  // }
   if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    incoming.trim();
    Serial.println("Received from Master: " + incoming);

    if (incoming == "LEFT") {
      lightLEDs(CRGB::Red);
    } else if (incoming == "STOP") {
      turnOffLEDs();
    }
  }

   delay(20);
}

void lightLEDs(CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
  FastLED.show();
}

void turnOffLEDs() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

