#include "BluetoothSerial.h"
#include <CapacitiveSensor.h>

#define USE_NAME  // Comment this to use MAC address instead of a slaveName

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#ifdef USE_NAME
String slaveName = "ESP32-BT-Slave";  // Change this to reflect the real name of your slave BT device
#else
String MACadd = "AA:BB:CC:11:22:33";                        // This only for printing
uint8_t address[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};  // Change this to reflect real MAC address of your slave BT device
#endif

String myName = "ESP32-BT-Master";

// Define the interval (in milliseconds) for timed transmissions
const unsigned long sendInterval = 5000;
unsigned long previousSendMillis = 0;

double analog_slider_right1;
double analog_slider_right2;
double analog_slider_right3;

double analog_slider_left1;
double analog_slider_left2;
double analog_slider_left3;

int sender = 25;

int slider_pin_left1 = 13;
int slider_pin_left2 = 12;
int slider_pin_left3 = 14;

int slider_pin_right1 = 27;
int slider_pin_right2 = 33;
int slider_pin_right3 = 32;

int samples_touch = 10;

// Thresholds and maximum values for each sensor
int sliderLeft1MinThresh = 60;
int sliderLeft2MinThresh = 30;
int sliderLeft3MinThresh = 30;

int sliderRight1MinThresh = 20;
int sliderRight2MinThresh = 25;
int sliderRight3MinThresh = 21;

int sliderLeft1MaxValue = 147;
int sliderLeft2MaxValue = 130;
int sliderLeft3MaxValue = 154;

int sliderRight1MaxValue = 147;
int sliderRight2MaxValue = 130;
int sliderRight3MaxValue = 154;

// Capacitive sensor objects
CapacitiveSensor slideSensorLeft1 = CapacitiveSensor(sender, slider_pin_left1);
CapacitiveSensor slideSensorLeft2 = CapacitiveSensor(sender, slider_pin_left2);
CapacitiveSensor slideSensorLeft3 = CapacitiveSensor(sender, slider_pin_left3);

CapacitiveSensor slideSensorRight1 = CapacitiveSensor(sender, slider_pin_right1);
CapacitiveSensor slideSensorRight2 = CapacitiveSensor(sender, slider_pin_right2);
CapacitiveSensor slideSensorRight3 = CapacitiveSensor(sender, slider_pin_right3);

const int mean_window_size = 5;        // Number of readings in the rolling mean
double position_history[mean_window_size] = {0};
int history_index = 0;
double previous_mean_position = -1.0;
bool is_sliding = false;
bool last_slide_left = false;
unsigned long last_left_slide_time = 0;
const double slide_threshold = 0.09;    // Smaller threshold to detect gradual sliding
const unsigned long slide_end_timeout = 500;  // Timeout for detecting slide end after a left slide (in milliseconds)


void setup() {
  bool connected;
  Serial.begin(9600);

  SerialBT.begin(myName, true);
  //SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
  Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());

#ifndef USE_NAME
  // If connecting with a MAC address, you can set the PIN code here (if required)
  // SerialBT.setPin(pin);
  // Serial.println("Using PIN");
#endif

  // Connecting to a slave device
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
    // Retry logic can be added here
  }
}

void loop() {
  unsigned long currentMillis = millis();

   // Get raw sensor values
  analog_slider_left1 = slideSensorLeft1.capacitiveSensor(samples_touch);
  analog_slider_left2 = slideSensorLeft2.capacitiveSensor(samples_touch);
  analog_slider_left3 = slideSensorLeft3.capacitiveSensor(samples_touch);

  analog_slider_right1 = slideSensorRight1.capacitiveSensor(samples_touch);
  analog_slider_right2 = slideSensorRight2.capacitiveSensor(samples_touch);
  analog_slider_right3 = slideSensorRight3.capacitiveSensor(samples_touch);

  // Check touch thresholds
  int is_touchedLeft1 = (analog_slider_left1 > sliderLeft1MinThresh) ? 1 : 0;
  int is_touchedLeft2 = (analog_slider_left2 > sliderLeft2MinThresh) ? 1 : 0;
  int is_touchedLeft3 = (analog_slider_left3 > sliderLeft3MinThresh) ? 1 : 0;

  int touch_count_left = is_touchedLeft1 + is_touchedLeft2 + is_touchedLeft3;
  double overall_position = -1.0;

  // Calculate position based on touch count
  if (touch_count_left > 0) {
    double total_weight = 0.0;
    double weighted_position = 0.0;

    // Calculate weighted position based on each sensor's contribution
    if (is_touchedLeft1) {
      double weight = analog_slider_left1 / sliderLeft1MaxValue;
      weighted_position += 0.0 * weight;
      total_weight += weight;
    }
    if (is_touchedLeft2) {
      double weight = analog_slider_left2 / sliderLeft2MaxValue;
      weighted_position += 0.5 * weight;
      total_weight += weight;
    }
    if (is_touchedLeft3) {
      double weight = analog_slider_left3 / sliderLeft3MaxValue;
      weighted_position += 1.0 * weight;
      total_weight += weight;
    }

    // Compute final position
    if (total_weight > 0) {
      overall_position = weighted_position / total_weight;
    }
  }

  // Add the current position to the history buffer and update the rolling mean
  position_history[history_index] = overall_position;
  history_index = (history_index + 1) % mean_window_size;

  // Calculate the rolling mean of the last `mean_window_size` positions
  double mean_position = 0.0;
  int valid_readings = 0;
  for (int i = 0; i < mean_window_size; i++) {
    if (position_history[i] >= 0) {  // Ignore uninitialized entries
      mean_position += position_history[i];
      valid_readings++;
    }
  }
  mean_position = (valid_readings > 0) ? mean_position / valid_readings : -1.0;

  // Slide detection logic using rolling mean
  if (mean_position >= 0.0) {
    if (previous_mean_position == -1.0) {
      // First valid mean position, no slide detected yet
      previous_mean_position = mean_position;
    } else {
      // Detect slide based on mean position change
      if (mean_position > previous_mean_position + slide_threshold) {
        Serial.println("Slide Right Detected");
        is_sliding = true;
        last_slide_left = false;
      } else if (mean_position < previous_mean_position - slide_threshold) {
        Serial.println("Slide Left Detected");
        is_sliding = true;
        last_slide_left = true;
        last_left_slide_time = millis();  // Record the time of the left slide
      }
      // Update the previous mean position for the next loop
      previous_mean_position = mean_position;
    }
  } else {
    // No touch detected, reset sliding state
    if (is_sliding) {
      unsigned long current_time = millis();
      
      if (last_slide_left && (current_time - last_left_slide_time < slide_end_timeout)) {
        Serial.println("LIGHT LEFT");  // Special output for left slide followed by end
      } else {
        Serial.println("Slide Ended");
      }
      
      is_sliding = false;
      last_slide_left = false;
    }
    previous_mean_position = -1.0;
  }

 

  if (mean_position >= 0.2 && mean_position <= 1.0) {
    Serial.print("LEFT,");
    Serial.print(mean_position, 2);
    Serial.println();
    String message = "LEFT";
    Serial.println("Sent to Slave: " + message);
    SerialBT.println(message);
  }

  if(mean_position < 0.2){
    Serial.print("STOP");
    Serial.print(mean_position, 2);
    Serial.println();
    String message = "STOP";
    Serial.println("Sent to Slave: " + message);
    SerialBT.println(message);
  }
 

  // Check for messages from the slave device
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    Serial.println("Received from Slave: " + incoming);
  }


  delay(20);
}
