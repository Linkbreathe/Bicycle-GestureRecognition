#include <WiFi.h>
#include <esp_now.h>

// GPIO Pins for EL Displays
const int gpioPin_left = 23;      
const int gpioPin_right = 15;     
const int gpioPin_braking = 27;   

// Blinking states
bool blinkingLeft = false;
bool blinkingRight = false;
bool brakeActive = false;

// Separate timing and state variables for each pin
unsigned long lastBlinkTimeLeft = 0;
bool pinStateLeft = LOW;

unsigned long lastBlinkTimeRight = 0;
bool pinStateRight = LOW;

unsigned long lastBlinkTimeBrake = 0;
bool pinStateBrake = LOW;

const unsigned long blinkInterval = 500; // Flashing interval (ms)


void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // this code was in loop for bluetooth, but moved here for Wifi approach 
  char message[50];
  if (len < 50) {
    memcpy(message, incomingData, len);
    message[len] = '\0'; // Null-terminate the string
    Serial.println("Received from Bike: " + String(message));

    if (String(message) == "LEFT") {
      blinkingLeft = true;
      blinkingRight = false;
    } 
    else if (String(message) == "RIGHT") {
      blinkingRight = true;
      blinkingLeft = false;
    } 
    else if (String(message) == "STOP") {
      blinkingLeft = false;
      blinkingRight = false;
      digitalWrite(gpioPin_left, HIGH);
      digitalWrite(gpioPin_right, HIGH);
    } 
    else if (String(message) == "BRAKING") {
      brakeActive = true;
      digitalWrite(gpioPin_braking, HIGH); // turn on brake light continuously
    } 
    else if (String(message) == "STOP BRAKING") {
      brakeActive = false;
      digitalWrite(gpioPin_braking, LOW);
    }
    else if (String(message) == "RECONNECT") {
      Serial.println("Bike reconnected yay");
    }
  }
}

void setup() {
  Serial.begin(9600);

  // Set GPIO pins as outputs
  pinMode(gpioPin_left, OUTPUT);
  pinMode(gpioPin_right, OUTPUT);
  pinMode(gpioPin_braking, OUTPUT);
  
  digitalWrite(gpioPin_left, HIGH);
  digitalWrite(gpioPin_right, HIGH);
  digitalWrite(gpioPin_braking, HIGH);
  
  
  // init esp now
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {

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

// solid brake no need for blinkBrake

  // Small delay to prevent high CPU usage
  delay(10);
}
