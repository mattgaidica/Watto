// by: Matt Gaidica
// ...many thanks Rob Tillaart for the INA226 library

#include <ArduinoBLE.h>
#include <INA226.h>
#include <Wire.h>
#include <math.h>

#define VERSION 230412  // yymmdd
INA226 INA(0x40);

// always send current, others are opt to make loop fast
const bool SEND_VOLTAGE = false;
const bool SEND_POWER = false;

// set these up to minimize optimize delay for conversion, see INA226 data sheet
const int USE_AVG = 1;
const int USE_SVCT = 4;
const int averageTable[8] = { 1, 4, 16, 64, 128, 256, 512, 1024 };                                      // samples
const int conversionTable[8] = { 140, 204, 332, 588, 1.1 * 1000, 2.1 * 1000, 4.2 * 1000, 8.3 * 1000 };  // us
float floatDelay = (averageTable[USE_AVG] * conversionTable[USE_SVCT]) / 1000.0;
int loopDelayMs = ceil(floatDelay);

BLEService customService("A000");
BLECharacteristic voltageCharacteristic("A001", BLERead | BLENotify, sizeof(float));
BLECharacteristic currentCharacteristic("A002", BLERead | BLENotify, sizeof(float));
BLECharacteristic powerCharacteristic("A003", BLERead | BLENotify, sizeof(float));

float uVoltage = 0, uCurrent = 0, uPower = 0;

// Linear model Poly3:
//      f(x) = p1*x^3 + p2*x^2 + p3*x + p4
// Coefficients (with 95% confidence bounds):
//        p1 =  -3.312e-09  (-6.615e-09, -9.924e-12)
//        p2 =   3.426e-05  (3.731e-06, 6.479e-05)
//        p3 =       2.413  (2.341, 2.484)
//        p4 =      -150.4  (-174.1, -126.7)
float correctCurrent(float x) {
  return -3.312e-09 * pow(x, 3) + 3.426e-05 * pow(x, 2) + 2.413 * x + -150.4;
}

void setup() {
  Serial.begin(115200);
  // while (!Serial);

  Wire.begin();
  if (!INA.begin()) {
    Serial.println("could not connect. Fix and Reboot");
  }

  INA.setMaxCurrentShunt(.02, 0.1);  // R=0.1ohm but empirically slightly different
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

  Serial.println("BLE Peripheral - Watto");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      uCurrent = correctCurrent(INA.getCurrent_uA());
      Serial.println(uCurrent);
      currentCharacteristic.writeValue(&uCurrent, sizeof(float));

      if (SEND_VOLTAGE || SEND_POWER) {
        uVoltage = INA.getShuntVoltage_uV();
        uPower = uVoltage * uCurrent;
        // uPower = INA.getPower_uW(); // not working
        if (SEND_VOLTAGE) {
          voltageCharacteristic.writeValue(&uVoltage, sizeof(float));
        }
        if (SEND_POWER) {
          powerCharacteristic.writeValue(&uPower, sizeof(float));
        }
      }
      // delays minimal amount based on averaging/conversion time, but may be yoked by BLE
      delay(loopDelayMs);
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}