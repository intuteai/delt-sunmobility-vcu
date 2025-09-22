#include <Arduino.h>
#include <cstring>
#include "driver/twai.h"
#include "BLEDevice.h"
#include "esp_err.h"

// ======================= Pin Assignments =======================
// CAN pins
#define CAN_TX 3
#define CAN_RX 4

// BLE UUIDs
#define SERVICE_UUID        "7E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "7E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// Digital IO pins
int KL15    = 0;
int Relay   = 7;
int MuxS0   = 8;
int MuxS1   = 9;
int MuxS2   = 5;
int MuxS3   = 10;
int MuxIN   = 6;
int USBpower = 19;

#define WD_DONE 18
#define WD_WAKE 1

// ======================= Digital IO variables =======================
int DI13, DI14, DI15, DI10, DI11, DI12;
int DI7, DI8, DI9, DI4, DI5, DI6;
int DI1, DI2, DI3;

int    KL15_Status;
String usbFaultStatus;

// ======================= BLE/CAN globals =======================
BLEServer*         bleServer = nullptr;
BLECharacteristic* bleCharacteristic = nullptr;
BLEAdvertising*    advertising = nullptr;

bool          clientConnected = false;
bool          wasConnected    = false;
unsigned long lastMessageTime = 0;
int           totalMessages   = 0;

unsigned long lastDIPrint = 0;
// NOTE: value is 500 ms (0.5 s) even though original comment said 1s
const unsigned long DI_PRINT_PERIOD_MS = 500; // 0.5 second interval

// ======================= BLE callbacks =======================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("✅ BLE Client Connected");
    clientConnected = true;
  }
  void onDisconnect(BLEServer* pServer) override {
    Serial.println("⚠ BLE Client Disconnected");
    clientConnected = false;
    delay(100);
    advertising->start();  // Restart advertising
  }
};

// ======================= Helper: MUX select =======================
void selectMuxChannel(byte channel) {
  digitalWrite(MuxS0, bitRead(channel, 0));
  digitalWrite(MuxS1, bitRead(channel, 1));
  digitalWrite(MuxS2, bitRead(channel, 2));
  digitalWrite(MuxS3, bitRead(channel, 3));
}

// ======================= Pack Digital IO states =======================
uint16_t packDigitalInputs() {
  KL15_Status = digitalRead(KL15);

  for (int ch = 0; ch < 16; ch++) {
    selectMuxChannel(ch);
    delay(5);
    int value = digitalRead(MuxIN);

    switch (ch) {
      case 0:  DI13 = value; break;
      case 1:  DI14 = value; break;
      case 2:  DI15 = value; break;
      case 3:  DI10 = value; break;
      case 4:  DI11 = value; break;
      case 5:  DI12 = value; break;
      case 6:  DI7  = value; break;
      case 7:  DI8  = value; break;
      case 8:  DI9  = value; break;
      case 9:  DI4  = value; break;
      case 10: DI5  = value; break;
      case 11: DI6  = value; break;
      case 12: DI1  = value; break;
      case 13: DI2  = value; break;
      case 14: DI3  = value; break;
      case 15: usbFaultStatus = (value == HIGH) ? "USB OK" : "USB FAULT"; break;
    }
  }

  // Pack all statuses into a single 16-bit value (bit = 1 = HIGH)
  uint16_t packed = 0;
  packed |= (KL15_Status & 1) << 0;
  packed |= (DI1         & 1) << 1;
  packed |= (DI2         & 1) << 2;
  packed |= (DI3         & 1) << 3;
  packed |= (DI4         & 1) << 4;
  packed |= (DI5         & 1) << 5;
  packed |= (DI6         & 1) << 6;
  packed |= (DI7         & 1) << 7;
  packed |= (DI8         & 1) << 8;
  packed |= (DI9         & 1) << 9;
  packed |= (DI10        & 1) << 10;
  packed |= (DI11        & 1) << 11;
  packed |= (DI12        & 1) << 12;

  // bit 13: USB OK (1 = OK)
  packed |= ((usbFaultStatus == "USB OK") ? 1 : 0) << 13;

  return packed;
}

// ======================= Print Digital IO Table =======================
void printDigitalInputs() {
  Serial.println("---- MUX Readings");
  Serial.print("KL15 Input (A8) Status : "); Serial.println(KL15_Status);
  Serial.print("Digital Input 1 (B7)   : "); Serial.println(DI1);
  Serial.print("Digital Input 2 (A7)   : "); Serial.println(DI2);
  Serial.print("Digital Input 3 (B6)   : "); Serial.println(DI3);
  Serial.print("Digital Input 4 (A6)   : "); Serial.println(DI4);
  Serial.print("Digital Input 5 (B5)   : "); Serial.println(DI5);
  Serial.print("Digital Input 6 (A5)   : "); Serial.println(DI6);
  Serial.print("Digital Input 7 (B4)   : "); Serial.println(DI7);
  Serial.print("Digital Input 8 (A4)   : "); Serial.println(DI8);
  Serial.print("Digital Input 9 (B3)   : "); Serial.println(DI9);
  Serial.print("Digital Input 10 (A3)  : "); Serial.println(DI10);
  Serial.print("Digital Input 11 (B2)  : "); Serial.println(DI11);
  Serial.print("Digital Input 12 (A2)  : "); Serial.println(DI12);
  Serial.print("USB Fault Status       : "); Serial.println(usbFaultStatus);
  Serial.println();
}

// ======================= CAN ID decode =======================
void decodeCANID(uint32_t id) {
  if      (id == 0x1038FF50) Serial.println(" -> Msg_DIU1 (Faults)");
  else if (id == 0x18265040) Serial.println(" -> Msg_sigControllerTemperature + 2 more");
  else if (id == 0x18305040) Serial.println(" -> Msg_sigThrottleVoltage");
  else if (id == 0x18275040) Serial.println(" -> Msg_sigTorqueCommand");
  else if (id == 0x104FFF50) Serial.println(" -> Msg_DIU10");
  else if (id == 0x14234050) Serial.println(" -> Msg_DIU2 (Current)");
  else if (id == 0x14244050) Serial.println(" -> Msg_DIU3 (Cell Voltages)");
  else if (id == 0x10281050) Serial.println(" -> Msg_DIU4 (SOC)");
  else if (id == 0x1031FF50) Serial.println(" -> Msg_DIU14 (DIU14 Faults)");
  else if (id == 0x14498250) Serial.println(" -> Msg_DriveParameters");
  else                       Serial.println(" -> Unknown Message ID");
}

// ======================= WAKE ISR =======================
volatile bool wakeTriggered = false;
void IRAM_ATTR onWakeISR() { wakeTriggered = true; }

// ======================= setup =======================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // BLE Setup
  BLEDevice::init("Battery_ESP32_C3");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());

  BLEService* service = bleServer->createService(SERVICE_UUID);
  bleCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  service->start();

  advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();
  Serial.println("✅ BLE GATT Server Started");

  // CAN Setup
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("✅ TWAI (CAN) Driver Installed");
  } else {
    Serial.println("❌ TWAI Driver Installation Failed!");
    while (1) {}
  }

  if (twai_start() == ESP_OK) {
    Serial.println("✅ TWAI (CAN) Driver Started");
  } else {
    Serial.println("❌ Failed to Start TWAI (CAN) Driver");
    while (1) {}
  }

  // Digital IO Setup
  pinMode(KL15,    INPUT);
  pinMode(Relay,   OUTPUT);
  pinMode(MuxS0,   OUTPUT);
  pinMode(MuxS1,   OUTPUT);
  pinMode(MuxS2,   OUTPUT);
  pinMode(MuxS3,   OUTPUT);
  pinMode(MuxIN,   INPUT);
  pinMode(USBpower, OUTPUT);
  digitalWrite(USBpower, HIGH); // keep USB power on

  digitalWrite(Relay, LOW);

  pinMode(WD_DONE, OUTPUT);
  digitalWrite(WD_DONE, LOW);

  pinMode(WD_WAKE, INPUT_PULLUP); // WAKE from TPL5010
  attachInterrupt(digitalPinToInterrupt(WD_WAKE), onWakeISR, FALLING); // Trigger on falling edge (WAKE goes LOW)

  lastMessageTime = millis();
}

// ======================= loop =======================
void loop() {
  // CAN Receive
  twai_message_t     message;
  twai_status_info_t status;
  twai_get_status_info(&status);

  if (wasConnected && !clientConnected) {
    Serial.println("🔄 BLE disconnected, restarting advertisement...");
    delay(100); // short gap
    advertising->start();
    Serial.println("📡 BLE advertising restarted");
    wasConnected = false;
  }
  if (!wasConnected && clientConnected) {
    wasConnected = true;
  }

  esp_err_t res = twai_receive(&message, pdMS_TO_TICKS(100));
  if (res == ESP_OK) {
    lastMessageTime = millis();
    totalMessages++;

    Serial.printf("CAN Msg [%d] ID: 0x%08X, Data:", totalMessages, message.identifier);
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.printf(" 0x%02X", message.data[i]);
    }
    Serial.println();

    decodeCANID(message.identifier);

    // Send CAN over BLE if client connected
    if (clientConnected) {
      uint8_t sendData[12];
      sendData[0] = 0x01; // Type = CAN
      sendData[1] = (message.identifier >> 24) & 0xFF;
      sendData[2] = (message.identifier >> 16) & 0xFF;
      sendData[3] = (message.identifier >> 8)  & 0xFF;
      sendData[4] =  message.identifier        & 0xFF;

      memcpy(&sendData[5], message.data, message.data_length_code);
      bleCharacteristic->setValue(sendData, 5 + message.data_length_code);
      bleCharacteristic->notify();
      Serial.println("📤 CAN Data sent via BLE");
    }

  } else if (millis() - lastMessageTime > 5000) {
    Serial.println("❗ No CAN data for 5 seconds!");
    if (status.state == TWAI_STATE_BUS_OFF) {
      Serial.println("⚠ CAN Bus OFF. Attempting recovery...");
      twai_stop();
      if (twai_start() == ESP_OK) {
        Serial.println("✅ CAN Bus Recovered");
      } else {
        Serial.println("❌ Recovery Failed");
      }
    } else {
      Serial.println("⚠ No RX activity. Check BMS power or wiring.");
    }
    lastMessageTime = millis();
  }

  // Digital IO updates (always Serial print + BLE if connected)
  if (millis() - lastDIPrint >= DI_PRINT_PERIOD_MS) {
    uint16_t packedGPIO = packDigitalInputs();

    // Only print every build with DEBUG
    #ifdef DEBUG
    printDigitalInputs();
    #endif

    lastDIPrint = millis();

    if (clientConnected) {
      uint8_t data[3];
      data[0] = 0x02; // Type = GPIO
      data[1] = (packedGPIO >> 8) & 0xFF;
      data[2] =  packedGPIO       & 0xFF;

      bleCharacteristic->setValue(data, 3);
      bleCharacteristic->notify();

      // Optional: light debug log every 5s
      #ifdef DEBUG
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 5000) {
        Serial.println("📤 GPIO BLE notify");
        lastPrint = millis();
      }
      #endif
    }
  }

  // Watchdog DONE pulse every 2 minutes
  const unsigned long WD_PULSE_INTERVAL_MS = 2 * 60 * 1000UL;
  static unsigned long lastWDPulseTime = 0;
  if (millis() - lastWDPulseTime >= WD_PULSE_INTERVAL_MS) {
    digitalWrite(WD_DONE, HIGH);
    delay(10); // 10ms pulse
    digitalWrite(WD_DONE, LOW);
    Serial.println("✅ Watchdog DONE pulse sent");
    lastWDPulseTime = millis();
  }

  // Handle WAKE interrupt (safe outside ISR)
  if (wakeTriggered) {
    wakeTriggered = false;
    digitalWrite(WD_DONE, HIGH);
    delay(20);
    digitalWrite(WD_DONE, LOW);
    Serial.println("✅ Watchdog DONE pulse sent from WAKE");
  }
}
