/**
 * @brief    FSR BLE sensor for Smart Cues
 * @author   Smart Cues Group
 * @note     Reads an FSR value and sends a binary state over BLE.
 */

#include <ArduinoBLE.h>

const int fsrPin = 2;          // FSR analog input pin
const int threshold = 100;     // Adjust according to the ESP32 ADC reading

BLEService fsrService("19B10000-E8F2-537E-4F6C-D104768A1214");

BLEByteCharacteristic fsrChar(
    "19B10001-E8F2-537E-4F6C-D104768A1214",
    BLERead | BLENotify
);

void setup() {
    pinMode(fsrPin, INPUT);

    // Initialize BLE
    if (!BLE.begin()) {
        return;
    }

    // Configure BLE device information
    BLE.setDeviceName("FSR_ESP32");
    BLE.setLocalName("FSR_ESP32");
    BLE.setAdvertisedService(fsrService);

    // Add the FSR characteristic to the BLE service
    fsrService.addCharacteristic(fsrChar);
    BLE.addService(fsrService);

    // Start BLE advertising
    BLE.advertise();
}

void loop() {
    // Wait for a BLE central device to connect
    BLEDevice central = BLE.central();

    if (central) {
        while (central.connected()) {
            // Read the FSR value
            int fsrValue = analogRead(fsrPin);

            // Convert the reading to a binary pressed/not-pressed state
            byte state = fsrValue > threshold ? 1 : 0;

            // Send the current state over BLE
            fsrChar.writeValue(state);

            // Update every 100 ms
            delay(100);
        }
    }
}
