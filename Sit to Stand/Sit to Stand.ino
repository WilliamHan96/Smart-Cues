/**
 * @brief    Smart Cues sit-to-stand controller using an FSR sensor
 * @author   Smart Cues Group
 * @note     Detects sit-to-stand transitions from force-sensor readings and provides interaction feedback.
 */

#include <Wire.h>

// FSR
const int fsrPin = A0;
int fsrReading;
int threshold = 100;
bool isSitting = false;
bool prevSitting = false;
int sitStandCount = 0;

// Timestamps for frequency calculation (optional)
#define MAX_EVENTS 20
unsigned long movementTimes[MAX_EVENTS];
int eventCount = 0;
float frequency5s = 0;

// RGB LED pins
const int redPin = 3;
const int greenPin = 5;
const int bluePin = 6;

// Buzzer pin
const int buzzerPin = 2;

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
  Serial.begin(9600);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
}

void loop() {
  // 1. Read FSR value
  fsrReading = analogRead(fsrPin);
  isSitting = (fsrReading > threshold);

  // 2. Detect sit-to-stand transition
  if (isSitting != prevSitting) {
    if (prevSitting && !isSitting) {
      sitStandCount++;

      // Beep every 5 sit-to-stand movements
      if (sitStandCount % 5 == 0) {
        digitalWrite(buzzerPin, HIGH);
        delay(100);
        digitalWrite(buzzerPin, LOW);
      }

      updateRGBColor(sitStandCount);

      // Celebrate and reset after 30 movements
      if (sitStandCount >= 30) {
        celebrateRGB();
        sitStandCount = 0;
      }

      // (Optional) Log timestamp for frequency calculation
      unsigned long now = millis();
      if (eventCount < MAX_EVENTS) {
        movementTimes[eventCount++] = now;
      } else {
        for (int i = 1; i < MAX_EVENTS; i++) movementTimes[i - 1] = movementTimes[i];
        movementTimes[MAX_EVENTS - 1] = now;
      }
    }
    prevSitting = isSitting;
  }

  // (Optional) Calculate frequency over past 5 seconds
  unsigned long now = millis();
  int recentCount = 0;
  for (int i = 0; i < eventCount; i++) {
    if (now - movementTimes[i] <= 5000) {
      movementTimes[recentCount++] = movementTimes[i];
    }
  }
  eventCount = recentCount;
  frequency5s = (eventCount / 5.0) * 60.0;

  delay(100);
}

// ✅ Red-to-white gradient based on count
void updateRGBColor(int count) {
  int index = count % 30;
  RGB c = colorCycle[index];
  analogWrite(redPin, c.r);
  analogWrite(greenPin, c.g);
  analogWrite(bluePin, 100);  // Fixed blue value (optional customization)
}

// ✅ RGB rainbow celebration animation
void celebrateRGB() {
  RGB rainbow[] = {
    {255, 0, 0}, {255, 128, 0}, {255, 255, 0}, {0, 255, 0},
    {0, 255, 255}, {0, 0, 255}, {128, 0, 255}, {255, 0, 255}
  };
  int len = sizeof(rainbow) / sizeof(rainbow[0]);

  for (int i = 0; i < 3; i++) {  // Flash 3 rainbow cycles
    for (int j = 0; j < len; j++) {
      analogWrite(redPin, rainbow[j].r);
      analogWrite(greenPin, rainbow[j].g);
      analogWrite(bluePin, rainbow[j].b);
      delay(100);
    }
  }
}
