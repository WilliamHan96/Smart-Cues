/**
 * @brief    Smart Cues gesture-controlled timer
 * @author   Smart Cues Group
 * @note     Uses an APDS9960 gesture sensor and LED matrix to control and display a stopwatch.
 */

#include <Adafruit_APDS9960.h>
#include <ArduinoGraphics.h>
#include <Arduino_LED_Matrix.h>

Adafruit_APDS9960 apds;
ArduinoLEDMatrix matrix;

enum TimerState { IDLE, RUNNING, PAUSED };
TimerState timerState = IDLE;

unsigned long startTime = 0;
unsigned long pausedAt = 0;
unsigned long totalPausedTime = 0;

void setup() {
  Serial.begin(115200);

  if (!apds.begin()) {
    Serial.println("Failed to initialize APDS9960 sensor.");
    while (1);
  }


  apds.enableProximity(true);
  apds.enableGesture(true);

  matrix.begin();
  showMessage("Ready");
}

void loop() {
  uint8_t gesture = apds.readGesture();
  if (gesture != 0) {
    handleGesture(gesture);
    Serial.print("Gesture: ");
    Serial.println(gesture);
  }

  updateDisplay();
  delay(300);
}

void handleGesture(uint8_t gesture) {
  switch (gesture) {
    case APDS9960_UP:
      if (timerState == IDLE) {
        startTime = millis();
        totalPausedTime = 0;
        timerState = RUNNING;
        showMessage("Start");
      }
      break;

    case APDS9960_LEFT:
      if (timerState == RUNNING) {
        pausedAt = millis();
        timerState = PAUSED;
        showMessage("Pause");
      }
      break;

    case APDS9960_RIGHT:
      if (timerState == PAUSED) {
        totalPausedTime += (millis() - pausedAt);
        timerState = RUNNING;
        showMessage("Resume");
      }
      break;

    case APDS9960_DOWN:
      if (timerState != IDLE) {
        timerState = IDLE;
        showMessage("Stop");
      }
      break;
  }
}

void updateDisplay() {
  matrix.beginDraw();
  matrix.clear();
  matrix.stroke(255);
  matrix.textFont(Font_5x7);

  if (timerState == RUNNING) {
    unsigned long now = millis();
    unsigned long elapsed = (now - startTime - totalPausedTime) / 1000;
    matrix.text(String(elapsed), 0, 0);
  } else if (timerState == PAUSED) {
    unsigned long elapsed = (pausedAt - startTime - totalPausedTime) / 1000;
    matrix.text(String(elapsed), 0, 0);
  } else {
    matrix.text("0", 0, 0);
  }

  matrix.endDraw();
}

void showMessage(const char* msg) {
  matrix.beginDraw();
  matrix.clear();
  matrix.stroke(255);
  matrix.textScrollSpeed(80);
  matrix.textFont(Font_5x7);

  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(msg);
  matrix.endText(SCROLL_LEFT);
  matrix.endDraw();

  delay(1000);
}

