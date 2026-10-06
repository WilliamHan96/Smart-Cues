/**
 * @brief    Smart Cues IMU balance BLE sender
 * @author   Smart Cues Group
 * @note     Calculates balance stability from IMU gyroscope data and sends the stable/unstable state over BLE.
 */

#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include <Wire.h>

LSM6DS3 imu(I2C_MODE, 0x6A);

BLEService imuService("180A");
BLECharacteristic imuDataChar("2A57", BLERead | BLENotify, 16);

float gyroBuffer[3][100];
int bufferIndex = 0;
bool bufferFilled = false;
unsigned long lastEvalTime = 0;
const int evalInterval = 1000;

String stabilityStatus = "Unknown";
float stabilityScore = 0.0;

static int lastSentStatus = -1;


void setup() {
  Serial.begin(115200);

  Wire.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  imu.settings.gyroEnabled = 1;
  imu.settings.gyroRange = 245;
  imu.settings.gyroSampleRate = 104;
  imu.settings.accelEnabled = 1;
  imu.settings.accelRange = 2;
  imu.settings.accelSampleRate = 104;

  if (imu.begin() != 0) {
    while (1);
  }

  if (!BLE.begin()) {
    while (1);
  }

  BLE.setLocalName("XIAO_IMU");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(imuDataChar);
  BLE.addService(imuService);
  imuDataChar.writeValue("Waiting...");
  BLE.advertise();

}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {

    while (central.connected()) {
      float gx = imu.readFloatGyroX() * DEG_TO_RAD;
      float gy = imu.readFloatGyroY() * DEG_TO_RAD;
      float gz = imu.readFloatGyroZ() * DEG_TO_RAD;

      gyroBuffer[0][bufferIndex] = gx;
      gyroBuffer[1][bufferIndex] = gy;
      gyroBuffer[2][bufferIndex] = gz;
      bufferIndex++;
      if (bufferIndex >= 100) {
        bufferIndex = 0;
        bufferFilled = true;
      }

      if (bufferFilled && millis() - lastEvalTime > evalInterval) {
        lastEvalTime = millis();
        float varX = computeVariance(gyroBuffer[0], 100);
        float varY = computeVariance(gyroBuffer[1], 100);
        float varZ = computeVariance(gyroBuffer[2], 100);
        float totalVar = varX + varY + varZ;

        stabilityScore = 100.0 - totalVar * 1000.0;
        stabilityScore = constrain(stabilityScore, 0, 100);

        stabilityStatus = (stabilityScore > 80) ? "Stable" : "Unstable";

      }

      char buffer[16] = {0};

      int statusInt = (stabilityStatus == "Stable") ? 1 : 0;
      if (statusInt != lastSentStatus) {
        char buffer[4] = {0};
        snprintf(buffer, sizeof(buffer), "%d", statusInt);
        imuDataChar.writeValue(buffer);
        lastSentStatus = statusInt;
      }

      delay(5);
    }

  }
}

float computeVariance(float* data, int size) {
  float mean = 0;
  for (int i = 0; i < size; i++) {
    mean += data[i];
  }
  mean /= size;

  float variance = 0;
  for (int i = 0; i < size; i++) {
    float diff = data[i] - mean;
    variance += diff * diff;
  }
  return variance / size;
}
