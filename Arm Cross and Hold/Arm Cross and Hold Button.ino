/**
 * @brief    Smart Cues button-controlled exercise timer
 * @author   Smart Cues Group
 * @note     Provides button-based timer control and local visual/audio feedback for exercise sessions.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define BTN_START 2
#define BTN_PAUSE 4
#define BTN_RESUME 5
#define BTN_STOP 3

LiquidCrystal_I2C lcd(0x27, 16, 2);

enum TimerState { IDLE, RUNNING, PAUSED };
TimerState timerState = IDLE;

unsigned long startTime = 0;
unsigned long pausedAt = 0;
unsigned long totalPausedTime = 0;

void setup() {
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_PAUSE, INPUT_PULLUP);
  pinMode(BTN_RESUME, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Timer Ready");
}

void loop() {
  checkButtons();

  if (timerState == RUNNING) {
    unsigned long now = millis();
    unsigned long elapsed = (now - startTime - totalPausedTime) / 1000;
    showTime(elapsed);
  }
}

void checkButtons() {
  static unsigned long lastDebounce = 0;
  const unsigned long debounceDelay = 200;

  if (millis() - lastDebounce < debounceDelay) return;
  lastDebounce = millis();

  if (digitalRead(BTN_START) == LOW && timerState == IDLE) {
    startTime = millis();
    totalPausedTime = 0;
    timerState = RUNNING;
    lcd.setCursor(0, 0);
    lcd.print("Running        ");
  }
  else if (digitalRead(BTN_PAUSE) == LOW && timerState == RUNNING) {
    pausedAt = millis();
    timerState = PAUSED;
    lcd.setCursor(0, 0);
    lcd.print("Paused         ");
  }
  else if (digitalRead(BTN_RESUME) == LOW && timerState == PAUSED) {
    totalPausedTime += (millis() - pausedAt);
    timerState = RUNNING;
    lcd.setCursor(0, 0);
    lcd.print("Running        ");
  }
  else if (digitalRead(BTN_STOP) == LOW) {
    timerState = IDLE;
    lcd.setCursor(0, 0);
    lcd.print("Stopped        ");
    lcd.setCursor(0, 1);
    lcd.print("Time: 00:00    ");
  }
}

void showTime(unsigned long sec) {
  int minutes = sec / 60;
  int seconds = sec % 60;
  lcd.setCursor(0, 1);
  lcd.print("Time: ");
  if (minutes < 10) lcd.print('0');
  lcd.print(minutes);
  lcd.print(':');
  if (seconds < 10) lcd.print('0');
  lcd.print(seconds);
  lcd.print("    ");
}
