/**
 * @brief    Smart Cues sit-to-stand wireless controller for Arduino
 * @author   Smart Cues Group
 * @note     Handles sit-to-stand sensing and wireless communication for Smart Cues.
 */

#include <ArduinoBLE.h>

// RGB LED pins
const int redPin = 3;
const int greenPin = 5;
const int bluePin = 6;

// Buzzer
const int buzzerPin = 2;

BLEDevice espPeripheral;
BLECharacteristic fsrChar;

bool isSitting = false;
bool prevSitting = false;
int sitStandCount = 0;

unsigned long sitStartTime = 0;
bool sitTooLongAlerted = false;

#define MAX_EVENTS 20
unsigned long movementTimes[MAX_EVENTS];
int eventCount = 0;
float frequency5s = 0;

bool ledOn = false;
unsigned long lastBlinkTime = 0;

// ===== Manually defined red-to-white gradient =====
struct RGB { byte r, g, b; };

RGB colorCycle[30] = {
  {255,249,249}, {255,240,240}, {255,231,231}, {255,222,222}, {255,213,213},
  {255,204,204}, {255,195,195}, {255,186,186}, {255,177,177}, {255,168,168},
  {255,159,159}, {255,150,150}, {255,141,141}, {255,132,132}, {255,123,123},
  {255,114,114}, {255,105,105}, {255,96,96},   {255,87,87},   {255,78,78},
  {255,69,69},   {255,60,60},   {255,51,51},   {255,42,42},   {255,33,33},
  {255,24,24},   {255,15,15},   {255,6,6},     {255,3,3},     {255,0,0}
};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1);
  }

  Serial.println("Scanning for ESP32_FSR...");
  BLE.scanForName("FSR_ESP32");
}

void loop() {
  if (!espPeripheral || !espPeripheral.connected()) {
    espPeripheral = BLE.available();
    if (espPeripheral && espPeripheral.localName() == "FSR_ESP32") {
      Serial.println("Found ESP32, connecting...");
      if (espPeripheral.connect()) {
        Serial.println("Connected to ESP32");
        if (espPeripheral.discoverAttributes()) {
          fsrChar = espPeripheral.characteristic("19B10001-E8F2-537E-4F6C-D104768A1214");
          if (fsrChar) {
            fsrChar.subscribe();
            Serial.println("Subscribed to FSR characteristic");
          } else {
            Serial.println("FSR characteristic not found!");
          }
        }
      } else {
        Serial.println("Connection failed");
      }
    }
  }

  // Main loop with BLE FSR value
  if (espPeripheral.connected() && fsrChar && fsrChar.valueUpdated()) {
    byte value = 0;
    fsrChar.readValue(value);
    isSitting = (value == 1);
    Serial.print("FSR state received: ");
    Serial.println(isSitting ? "SITTING" : "STANDING");

    unsigned long now = millis();

    if (isSitting != prevSitting) {
      if (prevSitting && !isSitting) {
        sitStandCount++;
        if (sitStandCount % 5 == 0) {
          digitalWrite(buzzerPin, HIGH);
          delay(100);
          digitalWrite(buzzerPin, LOW);
        }
        updateRGBColor(sitStandCount);
        if (sitStandCount >= 30) {
          celebrateRGB();
          sitStandCount = 0;
        }
        if (eventCount < MAX_EVENTS) {
          movementTimes[eventCount++] = now;
        } else {
          for (int i = 1; i < MAX_EVENTS; i++) movementTimes[i - 1] = movementTimes[i];
          movementTimes[MAX_EVENTS - 1] = now;
        }
      }
      if (!isSitting) {
        sitStartTime = 0;
        sitTooLongAlerted = false;
        ledOn = false;
      }
      prevSitting = isSitting;
    }

    // Sitting response
    if (isSitting) {
      if (sitStartTime == 0) sitStartTime = now;
      unsigned long sittingTime = now - sitStartTime;
      Serial.print("Sitting time: ");
      Serial.println(sittingTime);

      if (!sitTooLongAlerted && sittingTime >= 10000) {
        for (int i = 0; i < 3; i++) {
          digitalWrite(buzzerPin, HIGH);
          delay(150);
          digitalWrite(buzzerPin, LOW);
          delay(150);
        }
        sitTooLongAlerted = true;
      }

      if (sittingTime >= 10000 && sittingTime < 20000) {
        if (now - lastBlinkTime >= 2000) {
          ledOn = !ledOn;
          if (ledOn) {
            analogWrite(redPin, 255);
            analogWrite(greenPin, 100);
            analogWrite(bluePin, 100);
          } else {
            analogWrite(redPin, 0);
            analogWrite(greenPin, 0);
            analogWrite(bluePin, 0);
          }
          lastBlinkTime = now;
        }
      } else if (sittingTime >= 20000) {
        if (now - lastBlinkTime >= 3000) {
          ledOn = !ledOn;
          if (ledOn) {
            analogWrite(redPin, 255);
            analogWrite(greenPin, 0);
            analogWrite(bluePin, 0);
          } else {
            analogWrite(redPin, 0);
            analogWrite(greenPin, 0);
            analogWrite(bluePin, 0);
          }
          lastBlinkTime = now;
        }
      }

      if (sittingTime >= 10000) {
        sitStandCount = 0;
      }
    }

    // Frequency update
    int recentCount = 0;
    for (int i = 0; i < eventCount; i++) {
      if (now - movementTimes[i] <= 5000) {
        movementTimes[recentCount++] = movementTimes[i];
      }
    }
    eventCount = recentCount;
    frequency5s = (eventCount / 5.0) * 60.0;
  }

  delay(100);
}

void updateRGBColor(int count) {
  int index = count % 30;
  RGB c = colorCycle[index];
  analogWrite(redPin, c.r);
  analogWrite(greenPin, c.g);
  analogWrite(bluePin, 100);
}

void celebrateRGB() {
  RGB rainbow[] = {
    {255, 0, 0}, {255, 128, 0}, {255, 255, 0}, {0, 255, 0},
    {0, 255, 255}, {0, 0, 255}, {128, 0, 255}, {255, 0, 255}
  };
  int len = sizeof(rainbow) / sizeof(rainbow[0]);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < len; j++) {
      analogWrite(redPin, rainbow[j].r);
      analogWrite(greenPin, rainbow[j].g);
      analogWrite(bluePin, rainbow[j].b);
      delay(100);
    }
  }
}
