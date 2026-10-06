/**
 * @brief    Smart Cues knee-extension exercise controller
 * @author   Smart Cues Group
 * @note     Handles sensing, feedback, and exercise interaction for knee-extension activity.
 */

#include <Adafruit_APDS9960.h>
#include <ArduinoGraphics.h>
#include <Arduino_LED_Matrix.h>

Adafruit_APDS9960 apds;
ArduinoLEDMatrix matrix;

uint16_t count = 0;
bool handWasClose = false;

unsigned long lastTime = 0;
unsigned long interval = 0;
float frequency = 0;
const float maxFrequency = 30.0;

void setup() {
  Serial.begin(115200);

  if (!apds.begin()) {
    Serial.println("Failed to initialize APDS9960 sensor.");
    while (1);
  }

  apds.enableProximity(true);
  matrix.begin();

  showCount();
}

void loop() {
  uint8_t proximity = apds.readProximity();
  Serial.print("Proximity: ");
  Serial.println(proximity);

  if (proximity > 3) {
    if (!handWasClose) {
      unsigned long currentTime = millis();
      if (lastTime > 0) {
        interval = currentTime - lastTime;
        frequency = 60000.0 / interval;
      }
      lastTime = currentTime;

      count++;
      handWasClose = true;

      showSmile();
      delay(1000);
      showCount();
    }
  } else {
    handWasClose = false;
  }

  delay(100);
}

void showCount() {
  matrix.beginDraw();
  matrix.clear();
  matrix.stroke(255);
  matrix.textFont(Font_5x7);

  matrix.text(String(count), 0, 0);

  int barLength = 0;
  if (frequency >= maxFrequency) barLength = 12;
  else if (frequency < 1) barLength = 1;
  else barLength = (int)(frequency / maxFrequency * 12);

  if (barLength > 0) {
    matrix.line(0, 7, barLength - 1, 7);
  }

  matrix.endDraw();
}

void showSmile() {
  uint32_t frame[] = {
    0x19819800,
    0x00001081,
    0xF8000000
  };

  int barLength = 0;
  if (frequency >= maxFrequency) barLength = 12;
  else if (frequency < 1) barLength = 1;
  else barLength = (int)(frequency / maxFrequency * 12);

  for (int i = 0; i < barLength; i++) {
    int bitIndex = 11 - i;
    frame[2] |= (1UL << bitIndex);
  }

  matrix.loadFrame(frame);
}
