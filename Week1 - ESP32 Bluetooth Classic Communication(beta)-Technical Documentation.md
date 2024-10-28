# ESP32 Bluetooth Classic Communication Technical Documentation

This document aims to guide you on how to use two ESP32 development boards to communicate via Bluetooth Classic. You will learn how to set up a Master device and a Slave device, and how to call the appropriate functions to send and receive data.

## Table of Contents

1. Introduction
2. Hardware Requirements
3. Software Preparation
4. Code Structure and Functionality
   - Master Device Code
   - Slave Device Code
5. Function Calls and Communication Flow
   - Master Device Function Calls
   - Slave Device Function Calls
6. Configuration and Execution Steps
7. Debugging and Troubleshooting
8. Extensions and Optimizations
9. References

## 1. Introduction

The ESP32 is a powerful microcontroller with built-in Wi-Fi and Bluetooth capabilities. Using Bluetooth Classic, two ESP32 development boards can establish a stable Serial Port Profile (SPP) communication channel for bidirectional data transfer. This guide provides detailed instructions on how to configure and write code to enable the Master device to actively connect to the Slave device and exchange data.

## 2. Hardware Requirements

- **Two ESP32 Development Boards**
- **USB Data Cables**: For connecting ESP32 boards to a computer for programming and serial monitoring.
- **Computer**: Installed with Arduino IDE for writing and uploading code.

## 3. Software Preparation

### 3.1 Install Arduino IDE

1. Download and install the [Arduino IDE](https://www.arduino.cc/en/software).

2. Open Arduino IDE, navigate to **File > Preferences**, and add the ESP32 JSON URL to the **Additional Board Manager URLs** field:

   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```

3. Go to **Tools > Board > Boards Manager**, search for `esp32`, and install the **esp32** board support.

### 3.2 Install Necessary Libraries

The ESP32 Arduino core library already includes the `BluetoothSerial` library, so no additional installation is required.

## 4. Code Structure and Functionality

This document provides two main code examples:

1. **Master Device Code**: Actively connects to the Slave device and performs data transmission.
2. **Slave Device Code**: Acts as an SPP server, waiting for the Master device to connect and handle data.

### Master Device Code

Below is the complete code for the Master device:

```ino
// Master Device Code
#include "BluetoothSerial.h"

#define USE_NAME  // Comment this to use MAC address instead

// Check if Bluetooth is enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable Bluetooth.
#endif

// Check Serial Port Profile (SPP)
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not enabled. It is only available for ESP32.
#endif

BluetoothSerial SerialBT;

#ifdef USE_NAME
String slaveName = "ESP32-BT-Slave";  // Bluetooth name of the Slave device
#else
String MACadd = "AA:BB:CC:11:22:33";                        // MAC address of the Slave device (for printing only)
uint8_t address[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};  // Actual MAC address of the Slave device
#endif

String myName = "ESP32-BT-Master";

// Define the interval for sending messages (milliseconds)
const unsigned long sendInterval = 5000;
unsigned long previousSendMillis = 0;

void setup() {
  bool connected;
  Serial.begin(115200);

  SerialBT.begin(myName, true);
  //SerialBT.deleteAllBondedDevices(); // Uncomment to delete paired devices
  Serial.printf("Device \"%s\" started in master mode. Ensure the Slave Bluetooth is on!\n", myName.c_str());

  #ifndef USE_NAME
  // If connecting using MAC address, you can set a PIN here (if required)
  // SerialBT.setPin("1234");
  // Serial.println("Using PIN for connection");
  #endif

  // Connect to the Slave device
  #ifdef USE_NAME
  Serial.printf("Connecting to Slave Bluetooth device named \"%s\"\n", slaveName.c_str());
  connected = SerialBT.connect(slaveName);
  #else
  Serial.printf("Connecting to Slave Bluetooth device with MAC %s\n", MACadd.c_str());
  connected = SerialBT.connect(address);
  #endif

  if (connected) {
    Serial.println("Successfully connected to Slave!");
  } else {
    Serial.println("Connection failed. Retrying...");
    // Retry logic can be added here
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Send a message every sendInterval milliseconds
  if (currentMillis - previousSendMillis >= sendInterval) {
    previousSendMillis = currentMillis;
    String message = "Master Message, Time: " + String(currentMillis / 1000) + "s";
    SerialBT.println(message);
    Serial.println("Sent to Slave: " + message);
  }

  // Check for incoming messages from the Slave
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    Serial.println("Received from Slave: " + incoming);
  }

  delay(20);
}
```

### Slave Device Code

Below is the complete code for the Slave device:

```ino
// Slave Device Code
#include "BluetoothSerial.h"

String device_name = "ESP32-BT-Slave";

// Check if Bluetooth is enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable Bluetooth.
#endif

// Check Serial Port Profile (SPP)
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not enabled. It is only available for ESP32.
#endif

BluetoothSerial SerialBT;

// Define the interval for sending messages (milliseconds)
const unsigned long sendInterval = 5000;
unsigned long previousSendMillis = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin(device_name);  // Set the Bluetooth name of the Slave device
  //SerialBT.deleteAllBondedDevices(); // Uncomment to delete paired devices
  Serial.printf("Device name \"%s\" started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
}

void loop() {
  unsigned long currentMillis = millis();

  // Send a message every sendInterval milliseconds
  if (currentMillis - previousSendMillis >= sendInterval) {
    previousSendMillis = currentMillis;
    String message = "Slave Message, Time: " + String(currentMillis / 1000) + "s";
    SerialBT.println(message);
    Serial.println("Sent to Master: " + message);
  }

  // Check for incoming messages from the Master
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    Serial.println("Received from Master: " + incoming);

    // Optional: Send an acknowledgment back to the Master
    String response = "Acknowledged: " + incoming;
    SerialBT.println(response);
    Serial.println("Sent to Master: " + response);
  }

  delay(20);
}
```

## 5. Function Calls and Communication Flow

This section explains the key functions used in both Master and Slave devices and their roles in the communication process.

### Master Device Function Calls

#### Initialize Bluetooth

```
SerialBT.begin(myName, true);
```

- **Parameters**:
  - `myName`: The Bluetooth name of the Master device (string).
  - `true`: Enables pairing functionality.
- **Purpose**: Initializes Bluetooth and sets the device name to "ESP32-BT-Master", enabling pairing.

#### Connect to Slave Device

```
connected = SerialBT.connect(slaveName);
```

- **Parameters**: `slaveName`: The Bluetooth name of the Slave device (string).
- **Purpose**: Attempts to connect to the Slave device named "ESP32-BT-Slave". Returns `true` if successful, `false` otherwise.

Alternatively, if using MAC address:

```
connected = SerialBT.connect(address);
```

- **Parameters**: `address`: The MAC address of the Slave device (6-byte array).
- **Purpose**: Attempts to connect to the Slave device with the specified MAC address.

#### Send Data

```
SerialBT.println(message);
```

- **Parameters**: `message`: The string message to send.
- **Purpose**: Sends a message to the Slave device via Bluetooth serial, automatically appending a newline character `\n`.

#### Receive Data

```
String incoming = SerialBT.readStringUntil('\n');
```

- **Parameters**: `'\n'`: Specifies to stop reading when a newline character is encountered.
- **Purpose**: Receives a complete message from the Slave device until a newline character is found.

### Slave Device Function Calls

#### Initialize Bluetooth

```
SerialBT.begin(device_name);
```

- **Parameters**: `device_name`: The Bluetooth name of the Slave device (string).
- **Purpose**: Initializes Bluetooth and sets the device name to "ESP32-BT-Slave".

#### Send Data

```
SerialBT.println(message);
```

- **Parameters**: `message`: The string message to send.
- **Purpose**: Sends a message to the Master device via Bluetooth serial, automatically appending a newline character `\n`.

#### Receive Data

```
String incoming = SerialBT.readStringUntil('\n');
```

- **Parameters**: `'\n'`: Specifies to stop reading when a newline character is encountered.
- **Purpose**: Receives a complete message from the Master device until a newline character is found.

#### Send Acknowledgment (Optional)

```
String response = "Acknowledged: " + incoming;
SerialBT.println(response);
```

- **Parameters**: `response`: The acknowledgment message string.
- **Purpose**: Sends an acknowledgment message back to the Master device.

## 6. Configuration and Execution Steps

### 6.1 Configure the Slave Device

1. **Connect the Slave Device to the Computer**: Use a USB data cable to connect the Slave ESP32 to your computer.
2. **Open Arduino IDE**: Select the correct board and port: Tools > Board > ESP32 Dev Module, Tools > Port > Select the appropriate port.
3. **Upload Slave Device Code**: Copy the Slave device code provided above into Arduino IDE. Click the Upload button to flash the code to the ESP32.
4. **Open Serial Monitor**: In Arduino IDE, go to Tools > Serial Monitor and set the baud rate to 115200.

You should see output similar to:

```
Device name "ESP32-BT-Slave" started.
Now you can pair it with Bluetooth!
```

### 6.2 Configure the Master Device

1. **Connect the Master Device to the Computer**: Use a USB data cable to connect the Master ESP32 to your computer.
2. **Open Another Arduino IDE Instance**: Alternatively, open a new window in the same Arduino IDE.
3. **Upload Master Device Code**: Copy the Master device code provided above into Arduino IDE. Ensure that `slaveName` matches exactly with the Slave device's `device_name`. Click the Upload button to flash the code to the ESP32.
4. **Open Serial Monitor**: In Arduino IDE, go to Tools > Serial Monitor and set the baud rate to 115200.

You should see output similar to:

```
Device "ESP32-BT-Master" started in master mode. Ensure the Slave Bluetooth is on!
Connecting to Slave Bluetooth device named "ESP32-BT-Slave"
Successfully connected to Slave!
```

### 6.3 Run and Test

- **Ensure Both Devices are Powered On and Close Proximity**

- **Observe Serial Monitors**: The Master device will send a message to the Slave every 5 seconds. The Slave device will receive the message, print it to its serial monitor, and optionally send an acknowledgment back.

- **Sample Output**:

  - **Master Device Serial Monitor**:

    ```
    Device "ESP32-BT-Master" started in master mode. Ensure the Slave Bluetooth is on!
    Connecting to Slave Bluetooth device named "ESP32-BT-Slave"
    Successfully connected to Slave!
    Sent to Slave: Master Message, Time: 5s
    Received from Slave: Slave Message, Time: 5s
    Sent to Slave: Master Message, Time: 10s
    Received from Slave: Slave Message, Time: 10s
    ...
    ```

  - **Slave Device Serial Monitor**:

    ```
    Device name "ESP32-BT-Slave" started.
    Now you can pair it with Bluetooth!
    Sent to Master: Slave Message, Time: 5s
    Received from Master: Master Message, Time: 5s
    Sent to Master: Acknowledged: Master Message, Time: 5s
    Sent to Master: Slave Message, Time: 10s
    Received from Master: Master Message, Time: 10s
    Sent to Master: Acknowledged: Master Message, Time: 10s
    ...
    ```

## 7. Debugging and Troubleshooting

### 7.1 Unable to Connect to Slave Device

- **Check Device Name or MAC Address**: Ensure that the `slaveName` or `address` in the Master device code matches exactly with the Slave device's Bluetooth name or MAC address.
- **Ensure Slave Device is in Connectable Mode**: The Slave device should be powered on and in Bluetooth pairing mode.
- **Proximity and Interference**: Make sure both ESP32 boards are within a reasonable distance and there are no significant physical obstacles or interference sources.
- **Restart Devices**: Try restarting both Master and Slave devices to re-establish the connection.

### 7.2 Data Sending Fails or Data is Lost

- **Check Serial Monitor**: Ensure the Serial Monitor's baud rate is correctly set to 115200.
- **Increase Delay**: If data is sent too frequently, it may cause buffer overflows. Try increasing the send interval or reducing the send frequency.
- **Check Power Supply**: Ensure the ESP32 boards are receiving adequate power to prevent communication issues due to insufficient power.

### 7.3 Bluetooth Pairing Issues

- **Delete Paired Devices**: Uncomment the `SerialBT.deleteAllBondedDevices();` line in the code, upload the code to clear paired devices, and then re-pair.
- **Manual Pairing**: Manually pair the two ESP32 boards through your computer or phone's Bluetooth settings.

### 7.4 Inaccurate Debug Information

- **Check Serial Connection**: Ensure the Serial Monitor is connected to the correct port and board.
- **Review Debug Prints**: Use `Serial.println()` statements to output debug information and help locate issues.

## 8. Extensions and Optimizations

### 8.1 Add Error Handling and Retry Mechanism

Enhance the Master device code to include automatic retry mechanisms for improved connection stability. For example:

```ino
void loop() {
  unsigned long currentMillis = millis();

  // Send a message every sendInterval milliseconds
  if (currentMillis - previousSendMillis >= sendInterval) {
    previousSendMillis = currentMillis;
    if (SerialBT.connected()) {
      String message = "Master Message, Time: " + String(currentMillis / 1000) + "s";
      SerialBT.println(message);
      Serial.println("Sent to Slave: " + message);
    } else {
      Serial.println("Not connected. Attempting to reconnect...");
      #ifdef USE_NAME
      bool connected = SerialBT.connect(slaveName);
      #else
      bool connected = SerialBT.connect(address);
      #endif
      if (connected) {
        Serial.println("Reconnected successfully!");
      } else {
        Serial.println("Reconnection failed!");
      }
    }
  }

  // Check for incoming messages from the Slave
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    Serial.println("Received from Slave: " + incoming);
  }

  delay(20);
}
```

### 8.2 Optimize Data Transmission

- **Use Fixed-Length Data Packets**: Design fixed-length data packets to ensure data integrity and simplify parsing.
- **Data Protocol Design**: Define a data protocol, such as JSON format, to facilitate parsing and handling complex data.

### 8.3 Enhance Security

- **Encrypt Communication**: Implement encryption algorithms to secure data transmission.
- **Authenticate Devices**: Implement authentication mechanisms to ensure only authorized devices can connect and communicate.

### 8.4 Support Multiple Device Connections

- **Support Multiple Slaves**: Extend the Master device code to support simultaneous connections to multiple Slave devices, managing multiple Bluetooth connections.

## 9. References

[ESP32 BluetoothSerial Library Documentation](https://github.com/espressif/arduino-esp32/tree/master/libraries/BluetoothSerial)