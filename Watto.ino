// by: Matt Gaidica

#define VERSION 230410

#include <ArduinoBLE.h>
#include <INA226.h>
#include <Wire.h>

const size_t MAX_BUFFER_SIZE = 200; // 200 * 5ms = 1s

INA226 INA(0x40);

BLEService customService("A000");
BLECharacteristic voltageCharacteristic("A001", BLERead | BLENotify, MAX_BUFFER_SIZE);
BLECharacteristic currentCharacteristic("A002", BLERead | BLENotify, MAX_BUFFER_SIZE);
BLECharacteristic powerCharacteristic("A003", BLERead | BLENotify, MAX_BUFFER_SIZE);

float uCurrent, uVoltage, uPower;
uint8_t voltageData[MAX_BUFFER_SIZE];
uint8_t currentData[MAX_BUFFER_SIZE];
uint8_t powerData[MAX_BUFFER_SIZE];
size_t dataSize = 0;
unsigned long lastSentTime = 0;

void setup() {
  Serial.begin(115200);
  // while (!Serial);

  Wire.begin();
  if (!INA.begin() )
  {
    Serial.println("could not connect. Fix and Reboot");
  }
  // 20mA, 0.1ohm
  INA.setMaxCurrentShunt(0.02, 0.1);
  INA.setAverage(1);
  INA.setShuntVoltageConversionTime(3);

  Serial.println("POWER2 = busVoltage x current\n");
  Serial.println("BUS\tCURRENT\tPOWER\tPOWER2\tDELTA");

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("WATTO");
  BLE.setAdvertisedService(customService);
  customService.addCharacteristic(voltageCharacteristic);
  customService.addCharacteristic(currentCharacteristic);
  customService.addCharacteristic(powerCharacteristic);
  BLE.addService(customService);
  uint8_t emptyData[MAX_BUFFER_SIZE] = {0};
  currentCharacteristic.writeValue(emptyData, MAX_BUFFER_SIZE);
  BLE.advertise();

  Serial.println("BLE Peripheral - Current Sensor");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    lastSentTime = millis();
    dataSize = 0;

    while (central.connected()) {
      if (dataSize == MAX_BUFFER_SIZE) {
        sendData();
        dataSize = 0;
      }

      if (dataSize < MAX_BUFFER_SIZE) {
        uVoltage = INA.getShuntVoltage_uV();
        uCurrent = INA.getCurrent_uA();
        uPower = INA.getPower_uW();
        memcpy(&voltageData[dataSize], &uVoltage, sizeof(float));
        memcpy(&currentData[dataSize], &uCurrent, sizeof(float));
        memcpy(&powerData[dataSize], &uPower, sizeof(float));
        dataSize += sizeof(float);
        delay(5);
      }
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void sendData() {
  if (dataSize == 0) return;

  size_t remainingData = dataSize;
  uint8_t* dataPtr = currentData;

  while (remainingData > 0) {
    size_t notifSize = min((size_t)20, remainingData); // Max 20 bytes per notification
    currentCharacteristic.writeValue(dataPtr, notifSize);
    dataPtr += notifSize;
    remainingData -= notifSize;
  }
}