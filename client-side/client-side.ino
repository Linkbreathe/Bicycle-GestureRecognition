#include "BluetoothSerial.h"

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

// 定义定时发送的间隔（毫秒）
const unsigned long sendInterval = 5000;
unsigned long previousSendMillis = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin(device_name);  // Bluetooth device name
  //SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
}

void loop() {
  unsigned long currentMillis = millis();

  // 每隔 sendInterval 毫秒发送一次消息
  if (currentMillis - previousSendMillis >= sendInterval) {
    previousSendMillis = currentMillis;
    String message = "Hello from Slave at " + String(currentMillis / 1000) + "s";
    SerialBT.println(message);
    Serial.println("Sent to Master: " + message);
  }

  // 检查是否有来自主设备的消息
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    Serial.println("Received from Master: " + incoming);

    // 可选：根据收到的消息进行响应
    String response = "Acknowledged: " + incoming;
    SerialBT.println(response);
    Serial.println("Sent to Master: " + response);
  }

  delay(20);
}
