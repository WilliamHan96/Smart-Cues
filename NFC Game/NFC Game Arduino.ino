/**
 * @brief    Smart Cues NFC game controller for Arduino
 * @author   Smart Cues Group
 * @note     Handles NFC game interaction and communication for Smart Cues.
 */

#include <Wire.h>
#include <RTClib.h>
#include <U8g2lib.h>
#include "ArduinoBLE.h"

// ---------- OLED ----------
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ---------- RTC ----------
RTC_DS1307 rtc;

// ---------- BLE Configuration ----------
#define SERVICE_UUID        "a9f238b1-7c3e-4f2c-9359-91b47f8e2fcb"
#define CHARACTERISTIC_UUID "d473cf0c-3e09-4811-bd6e-8fc3d42e6b26"

BLEDevice esp32Peripheral;
BLECharacteristic esp32Characteristic;

// ---------- Pins ----------
const int pirPin = 2;
const int buttonPin = 4;
const int buzzerPin = 7;
const int ledSufficient = 8;
const int ledSome = 9;
const int ledNone = 10;

// ---------- Variables ----------
int exerciseCount = 0;
int lastButtonState = HIGH;
int lastPirState = LOW;
unsigned long buttonPressTime = 0;
bool buttonHeld = false;

const unsigned long longPressDuration = 2000;
const unsigned long debounceDelay = 50;

bool statusTriggered = false;
int beepCount = 0;
int beepTotal = 0;
unsigned long lastBeepTime = 0;
bool buzzerOn = false;
int statusLED = -1;
unsigned long ledStartTime = 0;
const unsigned long ledDuration = 1000;
const unsigned long beepInterval = 400;

unsigned long lastClockUpdate = 0;

// ---------- Game State Variables ----------
bool gameInProgress = false;
unsigned long lastMotionTime = 0;
const unsigned long motionCooldown = 5000;
bool deviceConnected = false;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  delay(1000);

  Serial.println("OLED...");
  u8g2.begin();
  delay(200);

  u8g2.clearBuffer();
  u8g2.sendBuffer();
  delay(100);

  u8g2.setFont(u8g2_font_ncenB14_tr);

  u8g2.clearBuffer();
  u8g2.setCursor(15, 35);
  u8g2.print("Starting...");
  u8g2.sendBuffer();
  delay(1500);

  Serial.println("✅ OLED");

  // RTC
  if (!rtc.begin()) {
    Serial.println("❌ Couldn't find RTC");
    u8g2.clearBuffer();
    u8g2.setCursor(10, 35);
    u8g2.print("RTC Error!");
    u8g2.sendBuffer();
    while (1);
  }
  if (!rtc.isrunning()) {
    Serial.println("⚠️ RTC not running, setting to compile time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Pins
  pinMode(pirPin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledSufficient, OUTPUT);
  pinMode(ledSome, OUTPUT);
  pinMode(ledNone, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledSufficient, LOW);
  digitalWrite(ledSome, LOW);
  digitalWrite(ledNone, LOW);

  lastButtonState = digitalRead(buttonPin);

  setupBLE();

  u8g2.clearBuffer();
  u8g2.sendBuffer();
  delay(200);

  showCurrentTime();

  Serial.println("✅ ");
}

void loop() {
  unsigned long currentTime = millis();

  if (!deviceConnected) {
    scanForESP32();
  }

  if (deviceConnected && !esp32Peripheral.connected()) {
    Serial.println("⚠️ Lost connection to ESP32, resetting state");
    deviceConnected = false;
    esp32Peripheral.disconnect();
  }


  // Show time every 30 seconds
  if (currentTime - lastClockUpdate >= 30000) {
    lastClockUpdate = currentTime;
    if (!statusTriggered && !gameInProgress) {
      showCurrentTime();
    }
  }

  handleButton(currentTime);
  handlePIR(currentTime);
  updateBuzzerAndLED();
}

void scanForESP32() {
  static unsigned long lastScan = 0;
  if (millis() - lastScan < 5000) return;

  lastScan = millis();
  Serial.println("🔍 Scanning for ESP32...");

  BLE.scanForName("ESP32_NFC_Game_Controller");
  BLEDevice foundDevice = BLE.available();

  if (foundDevice) {
    Serial.println("🎯 Found ESP32! Connecting...");
    BLE.stopScan();

    if (foundDevice.connect()) {
      Serial.println("✅ Connected to ESP32!");
      deviceConnected = true;
      esp32Peripheral = foundDevice;

      if (esp32Peripheral.discoverAttributes()) {
        esp32Characteristic = esp32Peripheral.characteristic(CHARACTERISTIC_UUID);
        if (esp32Characteristic) {
          Serial.println("✅ Found characteristic");
          showConnectionStatus("Connected!");
        } else {
          Serial.println("❌ Characteristic not found");
          deviceConnected = false;
          esp32Peripheral.disconnect();
        }
      } else {
        Serial.println("❌ Failed to discover attributes");
        deviceConnected = false;
        esp32Peripheral.disconnect();
      }
    } else {
      Serial.println("❌ Connection failed");
      deviceConnected = false;
    }
  }
}

// ---------- BLE Setup ----------
void setupBLE() {
  Serial.println("🔵 Initializing BLE Client...");

  if (!BLE.begin()) {
    Serial.println("❌ Starting BLE failed!");
    return;
  }

  Serial.println("✅ BLE initialized");
}

// ---------- Handle Game Status from ESP32 ----------
void handleGameStatusUpdate() {
}

// ---------- Display Connection Status ----------
void showConnectionStatus(String status) {
  if (statusTriggered || gameInProgress) return;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.setCursor(10, 35);
  u8g2.print(status.c_str());
  u8g2.sendBuffer();
  delay(1500);

  showCurrentTime();
}

// ---------- Display Game Status ----------
void showGameStatus(String status) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.setCursor(10, 35);
  u8g2.print(status.c_str());
  u8g2.sendBuffer();
}

// ---------- Display RTC Time ----------
void showCurrentTime() {
  if (statusTriggered || gameInProgress) {
    return;
  }

  delay(10);

  DateTime now = rtc.now();
  char timeStr[6];
  sprintf(timeStr, "%02d:%02d", now.hour(), now.minute());

  if (now.year() < 2020 || now.year() > 2030) {
    return;
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.setCursor(25, 35);
  u8g2.print(timeStr);
  u8g2.sendBuffer();
  delay(15);
}

// ---------- Handle Button Logic ----------
void handleButton(unsigned long currentTime) {
  int reading = digitalRead(buttonPin);

  if (lastButtonState == LOW && reading == HIGH) {
    buttonPressTime = currentTime;
    buttonHeld = false;
  }

  if (reading == HIGH && !buttonHeld && (currentTime - buttonPressTime >= longPressDuration)) {
    buttonHeld = true;
  }

  if (lastButtonState == HIGH && reading == LOW) {
    if (buttonHeld) {
      exerciseCount = 0;
      Serial.println("🔄 Long press reset: exerciseCount = 0");
      beepTotal = 1;
      statusLED = ledNone;
      ledStartTime = currentTime;
      statusTriggered = true;
      beepCount = 0;
      buzzerOn = false;
      lastBeepTime = currentTime;
      digitalWrite(statusLED, HIGH);
    } else {
      exerciseCount++;
      Serial.print("Exercise count +1: ");
      Serial.println(exerciseCount);
      showLedOnly();
    }
    buttonHeld = false;
  }

  lastButtonState = reading;
}

// ---------- Handle PIR Sensor ----------
void handlePIR(unsigned long currentTime) {
  int pirValue = digitalRead(pirPin);

  if (pirValue == HIGH && lastPirState == LOW) {
    Serial.println("👣 PIR: Detected person");

    if (deviceConnected && !gameInProgress &&
        (currentTime - lastMotionTime > motionCooldown)) {

      lastMotionTime = currentTime;

      if (esp32Characteristic) {
        esp32Characteristic.writeValue("motion_detected");
        Serial.println("📡 Motion signal sent to ESP32");
      } else {
        Serial.println("❌ Cannot send motion signal - not connected");
      }

      showGameStatus("Motion Detected");
      delay(1000);
      showCurrentTime();
    }

    if (!deviceConnected || gameInProgress) {
      delay(20);
      prepareStatusFeedback();
    }
  }
  lastPirState = pirValue;
}

// ---------- Prepare Status Feedback ----------
void prepareStatusFeedback() {
  statusTriggered = true;
  beepCount = 0;
  lastBeepTime = millis();
  buzzerOn = false;

  if (exerciseCount == 0) {
    beepTotal = 3;
    statusLED = ledNone;
  } else if (exerciseCount == 1) {
    beepTotal = 2;
    statusLED = ledSome;
  } else {
    beepTotal = 0;
    statusLED = ledSufficient;
  }

  if (statusLED != -1) {
    delay(10);
    digitalWrite(statusLED, HIGH);
    ledStartTime = millis();
    delay(10);
  }
}

// ---------- Show LED Only ----------
void showLedOnly() {
  if (exerciseCount == 0) {
    statusLED = ledNone;
  } else if (exerciseCount == 1) {
    statusLED = ledSome;
  } else {
    statusLED = ledSufficient;
  }

  if (statusLED != -1) {
    delay(10);
    digitalWrite(statusLED, HIGH);
    ledStartTime = millis();
    statusTriggered = true;
    beepTotal = 0;
    beepCount = 0;
    buzzerOn = false;
    delay(10);
  }
}

// ---------- Non-blocking Feedback Controller ----------
void updateBuzzerAndLED() {
  if (!statusTriggered) return;

  unsigned long currentTime = millis();

  if (statusLED != -1 && currentTime - ledStartTime >= ledDuration) {
    delay(5);
    digitalWrite(statusLED, LOW);
    statusLED = -1;
    delay(5);
  }

  if (beepCount < beepTotal) {
    if (currentTime - lastBeepTime >= beepInterval) {
      lastBeepTime = currentTime;
      if (buzzerOn) {
        delay(5);
        digitalWrite(buzzerPin, LOW);
        buzzerOn = false;
        beepCount++;
        delay(5);
      } else {
        delay(5);
        digitalWrite(buzzerPin, HIGH);
        buzzerOn = true;
        delay(5);
      }
    }
  }

  if (beepCount >= beepTotal && statusLED == -1) {
    statusTriggered = false;
  }
  if (beepTotal == 0 && statusLED == -1) {
    statusTriggered = false;
  }
}
