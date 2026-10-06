/**
 * @brief    Smart Cues 6D wireless controller for Arduino
 * @author   Smart Cues Group
 * @note     Handles 6D motion sensing and wireless communication for Smart Cues.
 */

#include <ArduinoBLE.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define TONE_PITCH 440
#include <TonePitch.h>

#define NOTE_E5 659
#define NOTE_G5 784

LiquidCrystal_I2C lcd(0x27, 16, 2);

BLEDevice peripheral;
BLECharacteristic imuChar;

String lastValue = "";
String currentTask = "";
String previousTask = "";
String currentIMURaw = "ZH";

unsigned long taskStartTime = 0;
bool waitingForAnswer = false;

const int buzzerPin = 11;

String directions[] = {"Hand Down", "Hand Up", "Side", "Front", "Squat&Flip"};
String shuffledTasks[5];
int taskIndex = 0;

String mapDirection(const String& raw) {
  if (raw == "XH") return "Hand Down";
  if (raw == "XL") return "Hand Up";
  if (raw == "YH" || raw == "YL") return "Side";
  if (raw == "ZH") return "Front";
  if (raw == "ZL") return "Squat&Flip";
  return "Unknown";
}

void shuffleTasks() {
  for (int i = 0; i < 5; i++) {
    shuffledTasks[i] = directions[i];
  }

  for (int i = 4; i > 0; i--) {
    int j = random(0, i + 1);
    String temp = shuffledTasks[i];
    shuffledTasks[i] = shuffledTasks[j];
    shuffledTasks[j] = temp;
  }

  if (shuffledTasks[0] == previousTask) {
    for (int i = 1; i < 5; i++) {
      if (shuffledTasks[i] != previousTask) {
        String temp = shuffledTasks[0];
        shuffledTasks[0] = shuffledTasks[i];
        shuffledTasks[i] = temp;
        break;
      }
    }
  }

  taskIndex = 0;
}


String getNewTask() {
  if (taskIndex >= 5) shuffleTasks();
  currentTask = shuffledTasks[taskIndex++];
  previousTask = currentTask;
  return currentTask;
}
void buzz(bool correct) {
  if (correct) {
    tone(buzzerPin, NOTE_E5, 120);
    delay(140);
    tone(buzzerPin, NOTE_G5, 120);
    delay(140);
    noTone(buzzerPin);
  } else {
    for (int i = 0; i < 3; i++) {
      analogWrite(buzzerPin, 50); delay(100); analogWrite(buzzerPin, 0);
      delay(100);
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  randomSeed(analogRead(A0));

  pinMode(buzzerPin, OUTPUT);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Connecting...");

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1);
  }

  BLE.scanForName("FeatherIMU");
  shuffleTasks();
}

void loop() {
  if (!peripheral || !peripheral.connected()) {
    peripheral = BLE.available();
    if (peripheral && peripheral.localName() == "FeatherIMU") {
      if (peripheral.connect()) {
        Serial.println("Connected!");
        peripheral.discoverAttributes();
        imuChar = peripheral.characteristic("B7D2");
        if (imuChar) imuChar.subscribe();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Connected");
        delay(1000);
        shuffleTasks();
        currentTask = getNewTask();
        lcd.clear();
        lcd.print("Next: ");
        lcd.print(currentTask);
        taskStartTime = millis();
        waitingForAnswer = true;
      } else {
        BLE.scanForName("FeatherIMU");
      }
    }
    return;
  }

  if (imuChar && imuChar.valueUpdated()) {
    String val = String((const char*)imuChar.value());
    Serial.print("IMU Raw Update: ");
    Serial.println(val);
    if (val == "XH" || val == "XL" || val == "YH" || val == "YL" || val == "ZH" || val == "ZL") {
      currentIMURaw = val;
    }
  }

  if (waitingForAnswer) {
    unsigned long elapsed = millis() - taskStartTime;
    int remaining = 5 - elapsed / 1000;

    String friendly = mapDirection(currentIMURaw);

    if (remaining >= 0) {
      lcd.setCursor(0, 1);
      lcd.print("Wait... ");
      lcd.print(remaining);
      lcd.print("s   ");
    }

    if (elapsed >= 5000) {
      lcd.setCursor(0, 1);
      lcd.print("You: ");
      lcd.print(friendly);
      Serial.print("Current: ");
      Serial.println(friendly);
      delay(500);

      if (friendly == currentTask) {
        buzz(true);
        delay(1000);
        lcd.clear();
        currentTask = getNewTask();
        lcd.setCursor(0, 0);
        lcd.print("Do: ");
        lcd.print(currentTask);
        taskStartTime = millis();
      } else {
        buzz(false);
        delay(500);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Retry: ");
        lcd.print(currentTask);
        taskStartTime = millis();
      }
    }
  }
}
