# ESP32-C3 CAN + BLE + GPIO Multiplexer Firmware

This project provides firmware for the **ESP32-C3** that integrates **CAN bus (TWAI)** communication, **Bluetooth Low Energy (BLE)** notifications, and scanning of multiple digital inputs through a **16-channel multiplexer**. It is designed for applications such as battery management systems, automotive projects, or embedded monitoring where CAN data and GPIO status need to be sent wirelessly.

---

## Overview

- **CAN (TWAI)**: Receives frames from a 500 kbit/s CAN bus.  
- **BLE**: Acts as a GATT server and streams both CAN messages and GPIO status updates to connected clients.  
- **GPIO Multiplexer**: Reads up to 16 digital inputs through a 74HC4067 or similar multiplexer and packs their states into a 16-bit word.  
- **Watchdog Support**: Interfaces with a TPL5010 watchdog timer using `WD_DONE` (pulse) and `WD_WAKE` (interrupt).  

The firmware is lightweight, self-contained, and runs on the Arduino framework for ESP32.

---

## Features

- **BLE Notifications**:
  - Service UUID: `7E400001-B5A3-F393-E0A9-E50E24DCCA9E`
  - Characteristic UUID: `7E400002-B5A3-F393-E0A9-E50E24DCCA9E`
  - Two message types:
    - **CAN Frame (`0x01`)** → includes CAN ID and up to 8 data bytes
    - **GPIO Status (`0x02`)** → includes a packed 16-bit bitmap of inputs

- **GPIO Packing**:
  - Bit 0: KL15  
  - Bits 1–12: DI1..DI12  
  - Bit 13: USB OK (1 = OK, 0 = Fault)  
  - Bits 14–15: reserved  

- **Multiplexer Scanning**:
  - Select lines: `MuxS0..S3`  
  - Input read on `MuxIN`  
  - Channels map to DI1–DI12, DI13–DI15, and USB status  

- **CAN Decoding**:
  - Supports raw message forwarding  
  - Prints known IDs with descriptive labels to Serial  

- **Watchdog Integration**:
  - Periodic 2-minute pulse on `WD_DONE`  
  - Immediate pulse on `WD_WAKE` interrupt  

---

## Pin Assignments

| Function         | GPIO |
|------------------|------|
| CAN TX           | 3    |
| CAN RX           | 4    |
| KL15 Input       | 0    |
| Relay Output     | 7    |
| MUX S0..S3       | 8, 9, 5, 10 |
| MUX IN           | 6    |
| USB Power Enable | 19   |
| WD_DONE          | 18   |
| WD_WAKE          | 1    |

⚠️ **Note:** A 3.3V CAN transceiver (e.g., SN65HVD230) is required between ESP32-C3 pins and the CAN bus.

---

## Build Instructions

### Arduino IDE
1. Install **ESP32 board support** via Board Manager.  
2. Select **ESP32-C3 Dev Module**.  
3. Open the `.ino` file and upload to the board.  
4. Monitor Serial at **115200 baud** for logs.  

### PlatformIO
Example `platformio.ini`:

```ini
[env:esp32c3]
platform = espressif32
board = esp32-c3-devkitm-1
framework = arduino
monitor_speed = 115200
