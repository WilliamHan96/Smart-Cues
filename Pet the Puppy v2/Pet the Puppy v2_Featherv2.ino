/**
 * @brief    Smart Cues puppy interaction controller
 * @author   Smart Cues Group
 * @note     Controls the Smart Cues puppy interaction and wireless communication.
 */

#include <ArduinoBLE.h>
#include <Adafruit_APDS9960.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define PIN_NEOPIXEL     0
#define NEOPIXEL_POWER   NEOPIXEL_I2C_POWER

Adafruit_APDS9960 apds;
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// UUID
BLEService sensorService("a9f13584-02d4-4e6c-81a7-f03ce6df0023");
BLEByteCharacteristic proximityChar(
  "c1b74b7e-b8da-4d67-bf55-7d84efcab831",
  BLERead | BLENotify
);

bool handWasClose = false;
unsigned long lastTrigger = 0;
bool bleConnected = false;

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

void setup() {
  Serial.begin(115200);

  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH); // enable power
  pixel.begin();
  pixel.setBrightness(50);
  setColor(50, 0, 80);

  // --- APDS9960 ---
  Wire.begin();
  apds.begin();
  apds.enableProximity(true);

  // --- BLE ---
  BLE.begin();
  BLE.setLocalName("XIAO_Sensor");
  BLE.setAdvertisedService(sensorService);

  sensorService.addCharacteristic(proximityChar);
  BLE.addService(sensorService);
  BLE.advertise();

  Serial.println("ESP32 Feather V2 BLE Peripheral Ready");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central && !bleConnected) {
    bleConnected = true;
    Serial.println("BLE Connected");
    setColor(0, 80, 0);
  }

  if (!central && bleConnected) {
    bleConnected = false;
    Serial.println("BLE Disconnected");
    setColor(50, 0, 80);
  }

  uint8_t proximity = apds.readProximity();
  unsigned long now = millis();

  if (proximity > 10 && !handWasClose && (now - lastTrigger > 1000)) {
    handWasClose = true;
    lastTrigger = now;

    uint8_t val = 1;
    proximityChar.writeValue(val);

    setColor(80, 50, 0);
    delay(120);
    if (bleConnected)
      setColor(0, 80, 0);
    else
      setColor(50, 0, 80);
  }
  else if (proximity <= 5) {
    handWasClose = false;
  }
}
