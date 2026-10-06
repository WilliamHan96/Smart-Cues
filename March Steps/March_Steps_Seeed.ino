/**
 * @brief    Smart Cues march-step BLE sender for Seeed hardware
 * @author   Smart Cues Group
 * @note     Reads the IMU hardware step counter and sends updated step counts over BLE.
 */

#include "LSM6DS3.h"
#include "Wire.h"
#include <ArduinoBLE.h>

LSM6DS3 pedometer(I2C_MODE, 0x6A);

BLEService stepService("180D");
BLECharacteristic stepChar("2A53",
  BLERead | BLENotify, 16);

void setup() {


  Wire.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (pedometer.begin() != 0) {
    while (1);
  }
  if (config_pedometer(false) != 0) {
    while (1);
  }

  if (!BLE.begin()) {
    while (1);
  }

  BLE.setLocalName("XIAO_Step");
  BLE.setAdvertisedService(stepService);
  stepService.addCharacteristic(stepChar);
  BLE.addService(stepService);
  stepChar.writeValue("Waiting...");
  BLE.advertise();

}
void loop() {
  BLEDevice central = BLE.central();
  static uint16_t lastStep = 0;

  if (central) {

    while (central.connected()) {
      uint16_t stepCount = readStepCount();

      if (stepCount != lastStep) {
        stepChar.writeValue(stepCount);
        lastStep = stepCount;
      }

      delay(200);
    }
  }
}


uint16_t readStepCount() {
  uint8_t dataH, dataL;
  pedometer.readRegister(&dataH, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
  pedometer.readRegister(&dataL, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
  return (dataH << 8) | dataL;
}

int config_pedometer(bool clearStep) {
  uint8_t err = 0;
  uint8_t val = LSM6DS3_ACC_GYRO_FS_XL_2g | LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
  err += pedometer.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, val);

  pedometer.writeRegister(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80);
  pedometer.writeRegister(0x14, 0x6A);
  pedometer.writeRegister(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00);

  err += pedometer.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, clearStep ? 0x3E : 0x3C);
  err += pedometer.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);
  err += pedometer.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10);
  return err;
}
