/**
 * @brief    Smart Cues 6D wireless controller for Feather V2
 * @author   Smart Cues Group
 * @note     Handles 6D motion sensing and wireless communication for Smart Cues.
 */

#include <Wire.h>
#include <ArduinoBLE.h>

#define LSM6DSOX_ADDR 0x6A
#define WHO_AM_I      0x0F
#define CTRL1_XL      0x10
#define CTRL10_C      0x19
#define TAP_CFG2      0x58
#define TAP_THS_6D    0x59
#define MD1_CFG       0x5E
#define D6D_SRC       0x1D

BLEService imuService("A9F1");
BLECharacteristic imuChar("B7D2", BLERead | BLENotify, 8);

uint8_t last = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LSM6DSOX_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(LSM6DSOX_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSOX_ADDR, 1);
  return Wire.read();
}

void setupIMU() {
  writeReg(CTRL1_XL, 0x60);     // Accelerometer: 416Hz ±2g
  writeReg(CTRL10_C, 0x3C);     // Enable embedded functions
  writeReg(TAP_CFG2, 0x80);     // Enable embedded function interface
  writeReg(TAP_THS_6D, 0x60);   // 50° threshold
  writeReg(MD1_CFG, 0x04);      // Enable 6D on INT1
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (readReg(WHO_AM_I) != 0x6C) {
    Serial.println("LSM6DSOX not found");
    while (1);
  }

  setupIMU();
  delay(100);
  BLE.begin();
  BLE.setLocalName("FeatherIMU");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(imuChar);
  BLE.addService(imuService);
  imuChar.writeValue("INIT");  // optional
  BLE.advertise();

  Serial.println("Feather IMU BLE ready.");
}

void loop() {
  BLE.poll();
  uint8_t val = readReg(D6D_SRC);
  uint8_t dir = val & 0x3F;
  if (dir != last && dir != 0) {
    last = dir;
    if (dir & 0x20) imuChar.writeValue("ZH");
    else if (dir & 0x10) imuChar.writeValue("ZL");
    else if (dir & 0x08) imuChar.writeValue("YH");
    else if (dir & 0x04) imuChar.writeValue("YL");
    else if (dir & 0x02) imuChar.writeValue("XH");
    else if (dir & 0x01) imuChar.writeValue("XL");
  }

  delay(100);
}

