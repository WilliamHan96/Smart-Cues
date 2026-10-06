/**
 * @brief    Smart Cues wall-sit wireless controller
 * @author   Smart Cues Group
 * @note     Handles wall-sit sensing and wireless communication for Smart Cues.
 */

#include "ArduinoGraphics.h"
#include <Arduino_LED_Matrix.h>
#define TONE_PITCH 440
#include <TonePitch.h>
#include <ArduinoBLE.h>

ArduinoLEDMatrix matrix;

int melody[] = {
  NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5,
  NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6
};
int duration = 100;

const int trigPin = 9;
const int echoPin = 10;
const int buzzerPin = 8;

const float distanceThreshold = 50.0; // cm
const int holdConfirmTime = 500;
const int buzzInterval = 5000;
const unsigned long melodyInterval = 30000;
const int exitConfirmTime = 800;

unsigned long holdStartTime = 0;
unsigned long sessionStartTime = 0;
unsigned long lastBuzzTime = 0;
unsigned long lastMelodyTime = 0;
unsigned long exitStartTime = 0;

bool holding = false;
bool readyToStart = false;

float lastDistance = 0;
bool wallTouched = false;

BLEDevice fsrPeripheral;
BLECharacteristic fsrChar;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  matrix.begin();

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed");
    while (1);
  }

  BLE.scanForName("FSR_Sender");
  Serial.println("BLE Ready, scanning for FSR_Sender...");
}

void loop() {
  if (!fsrPeripheral || !fsrPeripheral.connected()) {
    fsrPeripheral = BLE.available();
    if (fsrPeripheral && fsrPeripheral.localName() == "FSR_Sender") {
      Serial.print("Found FSR_Sender, connecting...");
      if (fsrPeripheral.connect()) {
        Serial.println(" connected!");
        if (fsrPeripheral.discoverAttributes()) {
          fsrChar = fsrPeripheral.characteristic("19B10001-E8F2-537E-4F6C-D104768A1214");
          if (fsrChar) {
            fsrChar.subscribe();
          }
        }
      } else {
        Serial.println(" connection failed.");
      }
    }
    return;
  }

  if (fsrChar && fsrChar.valueUpdated()) {
    int32_t intValue = 0;
    fsrChar.readValue(intValue);
    wallTouched = (intValue > 100);
  }

  float distance = readDistance();
  unsigned long currentTime = millis();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm | WallTouched: ");
  Serial.println(wallTouched);

  if (distance > 0 && distance < distanceThreshold && wallTouched) {
    exitStartTime = 0;

    if (!readyToStart) {
      holdStartTime = currentTime;
      readyToStart = true;
    } else if ((currentTime - holdStartTime > holdConfirmTime) && !holding) {
      holding = true;
      sessionStartTime = currentTime;
      lastBuzzTime = currentTime;
      lastMelodyTime = currentTime;
      Serial.println("Wall sit started.");
    }

  } else {
    readyToStart = false;

    if (holding) {
      if (exitStartTime == 0) {
        exitStartTime = currentTime;
      } else if (currentTime - exitStartTime > exitConfirmTime) {
        Serial.println("Wall sit ended.");
        holding = false;
        lastMelodyTime = 0;
        exitStartTime = 0;
        clearMatrix();
      }
    } else {
      exitStartTime = 0;
    }
  }

  if (holding) {
    int secondsHeld = (currentTime - sessionStartTime) / 1000;
    showCount(secondsHeld);

    if (currentTime - lastBuzzTime >= buzzInterval) {
      beepOnce();
      lastBuzzTime = currentTime;
    }

    if (currentTime - lastMelodyTime >= melodyInterval) {
      playMelody();
      lastMelodyTime = currentTime;
    }
  }

  delay(100);
}

float readDistance() {
  float total = 0;
  int validSamples = 0;
  for (int i = 0; i < 5; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration > 0) {
      total += duration * 0.0343 / 2;
      validSamples++;
    }
    delay(10);
  }
  if (validSamples == 0) return -1;
  return total / validSamples;
}

void showCount(int count) {
  matrix.beginDraw();
  matrix.stroke(255);
  matrix.textFont(Font_5x7);
  matrix.clear();
  matrix.text(String(count), 0, 0);
  matrix.endDraw();
}

void clearMatrix() {
  matrix.beginDraw();
  matrix.clear();
  matrix.endDraw();
}

void beepOnce() {
  tone(buzzerPin, NOTE_G4, duration);
  delay(duration + 20);
  noTone(buzzerPin);
}

void playMelody() {
  Serial.println("🎵 Playing melody!");
  int numNotes = sizeof(melody) / sizeof(melody[0]);
  for (int i = 0; i < numNotes; i++) {
    tone(buzzerPin, melody[i], 50);
    delay(duration + 10);
    noTone(buzzerPin);
  }
}
