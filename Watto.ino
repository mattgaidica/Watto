// by: Matt Gaidica

#define VERSION 230410

#include <ArduinoBLE.h>
#include <INA226.h>
#include <Wire.h>
#include <math.h>

// set these up to minimize optimize delay for conversion
const int USE_AVG = 1;
const int USE_SVCT = 4;
const int averageTable[8] = { 1, 4, 16, 64, 128, 256, 512, 1024 };                                      // samples
const int conversionTable[8] = { 140, 204, 332, 588, 1.1 * 1000, 2.1 * 1000, 4.2 * 1000, 8.3 * 1000 };  // us
float intermediateValue = (averageTable[USE_AVG] * conversionTable[USE_SVCT]) / 1000.0;
int loopDelayMs = ceil(intermediateValue);

INA226 INA(0x40);

BLEService customService("A000");
BLECharacteristic voltageCharacteristic("A001", BLERead | BLENotify, sizeof(float));
BLECharacteristic currentCharacteristic("A002", BLERead | BLENotify, sizeof(float));
BLECharacteristic powerCharacteristic("A003", BLERead | BLENotify, sizeof(float));

float uVoltage = 0, uCurrent = 0, uPower = 0;
size_t dataSize = 0;
unsigned long lastSentTime = 0;

void setup() {
  Serial.begin(115200);
  // while (!Serial);

  Wire.begin();
  if (!INA.begin()) {
    Serial.println("could not connect. Fix and Reboot");
  }
 
  INA.setMaxCurrentShunt(0.05, 0.115); // R=0.1ohm but empirically slightly different
  INA.setAverage(USE_AVG);
  INA.setShuntVoltageConversionTime(USE_SVCT);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1)
      ;
  }

  BLE.setLocalName("Watto");
  BLE.setDeviceName("Watto");
  BLE.setAdvertisedService(customService);
  customService.addCharacteristic(voltageCharacteristic);
  customService.addCharacteristic(currentCharacteristic);
  customService.addCharacteristic(powerCharacteristic);
  BLE.addService(customService);
  currentCharacteristic.writeValue(&uVoltage, sizeof(uVoltage));
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
      uVoltage = INA.getShuntVoltage_uV();
      uCurrent = INA.getCurrent_uA();
      // uPower = INA.getPower_uW(); // not working
      uPower = uVoltage * uCurrent;

      // voltageCharacteristic.writeValue(&uVoltage, sizeof(float));
      currentCharacteristic.writeValue(&uCurrent, sizeof(float));
      powerCharacteristic.writeValue(&uPower, sizeof(float));
      delay(loopDelayMs);
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}