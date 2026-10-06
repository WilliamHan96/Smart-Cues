/**
 * @brief    Smart Cues puppy interaction controller
 * @author   Smart Cues Group
 * @note     Controls the Smart Cues puppy interaction and wireless communication.
 */

#include <ArduinoBLE.h>
#include <Adafruit_APDS9960.h>
#include <Wire.h>

Adafruit_APDS9960 apds;
BLEService sensorService("a9f13584-02d4-4e6c-81a7-f03ce6df0023");
BLEByteCharacteristic proximityChar("c1b74b7e-b8da-4d67-bf55-7d84efcab831", BLERead | BLENotify);

bool handWasClose = false;
unsigned long lastTrigger = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin();
  apds.begin();
  apds.enableProximity(true);

  BLE.begin();
  BLE.setLocalName("XIAO_Sensor");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(proximityChar);
  BLE.addService(sensorService);
  BLE.advertise();
}

void loop() {
  BLE.poll();

  uint8_t proximity = apds.readProximity();
  unsigned long now = millis();

  if (proximity > 10 && !handWasClose && (now - lastTrigger > 1000)) {
    handWasClose = true;
    lastTrigger = now;
    uint8_t value = 1;
    proximityChar.writeValue(value);
    blink();
  } else if (proximity <= 5) {
    handWasClose = false;
  }
}

void blink() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
}
