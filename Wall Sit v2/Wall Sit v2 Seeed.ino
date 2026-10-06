/**
 * @brief    Smart Cues wall-sit wireless controller for Seeed hardware
 * @author   Smart Cues Group
 * @note     Handles wall-sit sensing and wireless communication for Smart Cues.
 */

#include <ArduinoBLE.h>

const int fsrPin = A1;

BLEService fsrService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEByteCharacteristic fsrChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

unsigned long lastSendTime = 0;
const unsigned long interval = 100;

void setup() {

  if (!BLE.begin()) {
    while (1);
  }

  BLE.setLocalName("FSR_Sender");
  BLE.setAdvertisedService(fsrService);
  fsrService.addCharacteristic(fsrChar);
  BLE.addService(fsrService);
  BLE.advertise();

}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {

    while (central.connected()) {
      unsigned long currentTime = millis();
      if (currentTime - lastSendTime >= interval) {
        lastSendTime = currentTime;

        int fsrValue = analogRead(fsrPin);                 // 0–1023
        byte mappedValue = map(fsrValue, 0, 1023, 0, 255); // 0–255
        fsrChar.writeValue(mappedValue);

      }
    }

  }
}
