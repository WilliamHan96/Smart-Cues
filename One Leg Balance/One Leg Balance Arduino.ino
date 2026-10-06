/**
 * @brief    Smart Cues IMU balance BLE receiver and exercise timer
 * @author   Smart Cues Group
 * @note     Receives balance stability over BLE and provides timer, LCD, keypad, and audio feedback.
 */

#include <ArduinoBLE.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define TONE_PITCH 440
#include <TonePitch.h>
#include <DIYables_Keypad.h>

BLEDevice peripheral;
BLECharacteristic imuDataChar;

int melody[] = {
  NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5,
  NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6
};

const int buzzerPin = 11;
String lastStatus = "";

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int buttonPin = 10;
bool isTiming = false;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
bool lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
unsigned long lastMelodyTime = 0;

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2};
DIYables_Keypad keypad = DIYables_Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

enum TimerMode { NONE, STOPWATCH, COUNTDOWN_INPUT, COUNTDOWN_RUN };
TimerMode mode = NONE;

String inputBuffer = "";
int countdownSeconds = 0;
unsigned long countdownStart = 0;
bool isCountingDown = false;

void playMelody() {
  int numNotes = sizeof(melody) / sizeof(melody[0]);
  for (int i = 0; i < numNotes; i++) {
    tone(buzzerPin, melody[i], 50);
    delay(100 + 10);
    noTone(buzzerPin);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);

  lcd.init();
  lcd.backlight();
  lcd.print("Waiting BLE...");

  bool bleReady = false;
  for (int i = 0; i < 5; i++) {
    if (BLE.begin()) {
      bleReady = true;
      break;
    }
    Serial.println("BLE init failed, retrying...");
    delay(1000);
  }

  if (!bleReady) {
    Serial.println("Failed to initialize BLE after retries!");
    while (1);
  }

  Serial.println("Scanning for XIAO_IMU...");
}

void loop() {
  if (!peripheral || !peripheral.connected()) {
    digitalWrite(12, LOW);
    BLE.scanForName("XIAO_IMU");
    peripheral = BLE.available();

    if (peripheral) {
      Serial.println("Connecting to XIAO_IMU...");
      if (peripheral.connect()) {
        Serial.println("Connected.");
        digitalWrite(12, HIGH);

        if (peripheral.discoverAttributes()) {
          imuDataChar = peripheral.characteristic("2A57");
          if (!imuDataChar) {
            Serial.println("IMU Characteristic not found.");
            peripheral.disconnect();
            return;
          }
          imuDataChar.subscribe();
          Serial.println("Subscribed to IMU data.");
          lcd.clear();
        } else {
          Serial.println("Attribute discovery failed.");
          peripheral.disconnect();
          return;
        }
      } else {
        Serial.println("Connection failed.");
        return;
      }
    }
  }

  String status = "Unknown";
  if (peripheral && peripheral.connected() && imuDataChar.valueUpdated()) {
    String imuData = String((const char*)imuDataChar.value());
    imuData.trim();

    Serial.print("[Raw BLE] ");
    Serial.println(imuData);

    int statusCode = 0;
    if (imuData.length() > 0 && isDigit(imuData.charAt(0))) {
      statusCode = imuData.charAt(0) - '0';
    }

    String status = (statusCode == 1) ? "Stable" : "Unstable";
    Serial.print("Status string: ");
    Serial.println(statusCode);

    if (status != lastStatus) {
      if (status == "Unstable") {
        tone(buzzerPin, NOTE_G4, 100);
        delay(50);
        tone(buzzerPin, NOTE_G4, 100);
      } else if (status == "Stable") {
        tone(buzzerPin, NOTE_G5, 100);
      }
      lastStatus = status;
    }
  }

  char key = keypad.getKey();
  if (key) {
    if (mode == COUNTDOWN_INPUT || mode == COUNTDOWN_RUN) {
      if (isdigit(key)) {
        if (mode == COUNTDOWN_INPUT) {
          inputBuffer += key;
        }
      } else if (key == 'B' && mode == COUNTDOWN_INPUT) {
        countdownSeconds = inputBuffer.toInt();
        inputBuffer = "";
        countdownStart = millis();
        isCountingDown = true;
        mode = COUNTDOWN_RUN;
      } else if (key == 'C') {
        inputBuffer = "";
        isCountingDown = false;
        countdownSeconds = 0;
        mode = COUNTDOWN_INPUT;
        lcd.setCursor(0, 0);
        lcd.print("Countdown reset ");
      } else if (key == 'D') {
        inputBuffer = "";
        isCountingDown = false;
        countdownSeconds = 0;
        mode = NONE;
        lcd.clear();
      } else if (key == '*' && mode == COUNTDOWN_INPUT) {
        inputBuffer = "";
      }
    } else {
      if (key == 'A') {
        mode = COUNTDOWN_INPUT;
        inputBuffer = "";
        lcd.clear();
      }
    }
  }

  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
    if (reading == HIGH && lastButtonState == LOW) {
      isTiming = !isTiming;
      if (isTiming) {
        startTime = millis();
        lastMelodyTime = startTime;
        mode = STOPWATCH;
      } else {
        mode = NONE;
        lcd.setCursor(0, 0);
        lcd.print("Timer stopped     ");
      }
    }
  }
  lastButtonState = reading;

  if (mode == COUNTDOWN_INPUT) {
    lcd.setCursor(0, 0);
    lcd.print("Input: ");
    lcd.print(inputBuffer);
    lcd.print("s       ");
  } else if (mode == COUNTDOWN_RUN && isCountingDown) {
    unsigned long elapsed = (millis() - countdownStart) / 1000;
    int remaining = countdownSeconds - elapsed;
    lcd.setCursor(0, 0);
    lcd.print("Countdown: ");
    lcd.print(remaining);
    lcd.print("s     ");
    if (remaining <= 0) {
      isCountingDown = false;
      playMelody();
      mode = NONE;
      lcd.setCursor(0, 0);
      lcd.print("Countdown done     ");
    }
  } else if (mode == STOPWATCH && isTiming) {
    elapsedTime = millis() - startTime;
    lcd.setCursor(0, 0);
    lcd.print("Time: ");
    lcd.print(elapsedTime / 1000.0, 1);
    lcd.print("s     ");

    if (millis() - lastMelodyTime >= 30000) {
      playMelody();
      lastMelodyTime = millis();
    }
  } else if (mode == NONE) {
    lcd.setCursor(0, 0);
    lcd.print("Timer stopped     ");
  }

  lcd.setCursor(0, 1);
  lcd.print("Status: ");
  lcd.print(lastStatus + "       ");

  delay(100);
}
