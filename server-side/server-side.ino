#include "BluetoothSerial.h"

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

// 定义定时发送的间隔（毫秒）
const unsigned long sendInterval = 5000;
unsigned long previousSendMillis = 0;

void setup() {
  bool connected;
  Serial.begin(115200);

  SerialBT.begin(myName, true);
  //SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
  Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());

#ifndef USE_NAME
  // 如果使用 MAC 地址连接，您可以在此设置 PIN 码（如果需要）
  // SerialBT.setPin(pin);
  // Serial.println("Using PIN");
#endif

  // 连接到从设备
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
    // 可以在这里添加重试逻辑
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // 每隔 sendInterval 毫秒发送一次消息
  if (currentMillis - previousSendMillis >= sendInterval) {
    previousSendMillis = currentMillis;
    String message = "Hello from Master at " + String(currentMillis / 1000) + "s";
    SerialBT.println(message);
    Serial.println("Sent to Slave: " + message);
  }

  // 检查是否有来自从设备的消息
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    Serial.println("Received from Slave: " + incoming);
  }

  delay(20);
}
