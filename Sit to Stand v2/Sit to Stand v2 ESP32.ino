/**
 * @brief    Smart Cues sit-to-stand wireless controller for ESP32
 * @author   Smart Cues Group
 * @note     Handles sit-to-stand sensing and wireless communication for Smart Cues.
 */

#include <ArduinoBLE.h>

const int fsrPin = 2;
const int threshold = 100;

BLEService fsrService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEByteCharacteristic fsrChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

void setup() {

  pinMode(fsrPin, INPUT);

  if (!BLE.begin()) {
    return;
  }

  BLE.setDeviceName("FSR_ESP32");
  BLE.setLocalName("FSR_ESP32");
  BLE.setAdvertisedService(fsrService);

  fsrService.addCharacteristic(fsrChar);
  BLE.addService(fsrService);
  BLE.advertise();

}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {

    while (central.connected()) {
      int fsrValue = analogRead(fsrPin);
      byte state = fsrValue > threshold ? 1 : 0;
      fsrChar.writeValue(state);
      delay(100);  // update every 100ms
    }

  }
}
