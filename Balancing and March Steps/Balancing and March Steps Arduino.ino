/**
 * @brief    Smart Cues step and balance countdown controller for Arduino
 * @author   Smart Cues Group
 * @note     Handles stepping and balance countdown interactions for Smart Cues.
 */

#define TONE_PITCH 440
/**
 * Arduino R4 WiFi
 * ESP32Seeed
 * 1: Step - LED
 * 2: Balancing - LCD +  +
 */

#include <ArduinoBLE.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TonePitch.h>
#include <DIYables_Keypad.h>

BLEDevice esp32Peripheral;
BLECharacteristic esp32ModeChar;
BLECharacteristic esp32CountdownChar;

BLEDevice seedPeripheral;
BLECharacteristic seedDataChar;

enum SystemMode { BALANCE_MODE = 2, STEP_MODE = 1 };
SystemMode currentMode = BALANCE_MODE;
bool modeChanged = false;

uint16_t stepOffset = 0;
bool needSetOffset = false;

const uint8_t pinSS = 10;
const uint8_t MAX_ROWS = 8;
const uint16_t OP_DIGIT0 = 1;
const uint16_t OP_DECODEMODE = 9;
const uint16_t OP_INTENSITY = 10;
const uint16_t OP_SCANLIMIT = 11;
const uint16_t OP_SHUTDOWN = 12;
const uint16_t OP_DISPLAYTEST = 15;
uint8_t ledMatrix[8] = {0};
int litIndex = 0;
uint16_t lastStepCount = 0;

#define SPI_PARAMETER SPISettings(8000000, MSBFIRST, SPI_MODE0)

LiquidCrystal_I2C lcd(0x27, 16, 2);
const int buzzerPin = A1;
const int buttonPin = A0;

int melody[] = {
  NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5,
  NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6
};

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {9, 7, 6, 5};
byte colPins[COLS] = {4, 3, 2, 1};
DIYables_Keypad keypad = DIYables_Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

enum TimerMode { NONE, STOPWATCH, COUNTDOWN_INPUT, COUNTDOWN_RUN };
TimerMode timerMode = NONE;
bool isTiming = false;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
bool lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
unsigned long lastMelodyTime = 0;

String inputBuffer = "";
int countdownSeconds = 0;
unsigned long countdownStart = 0;
bool isCountingDown = false;

String lastBalanceStatus = "";
unsigned long lastStatusTime = 0;

uint32_t countdownDuration = 0;
uint32_t countdownStartTime = 0;
bool countdownActive = false;

bool esp32Connected = false;
bool seedConnected = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin();
  setupMAX7219();

  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);

  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  lcd.init();
  lcd.backlight();
  lcd.print("System Starting");

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
    Serial.println("Failed to initialize BLE!");
    lcd.setCursor(0, 1);
    lcd.print("BLE Failed!");
    while (1);
  }

  Serial.println("Arduino R4 Unified Receiver Ready");
  lcd.clear();
  lcd.print("Mode: BALANCE");
  lcd.setCursor(0, 1);
  lcd.print("Scanning BLE...");
}

void loop() {
  if (!esp32Connected) {
    scanForESP32();
  } else {
    if (esp32ModeChar.valueUpdated()) {
      Serial.println("ESP32 mode data updated!");
      handleModeChange();
    }

    if (esp32CountdownChar.valueUpdated()) {
      Serial.println("ESP32 countdown data updated!");
      handleCountdownUpdate();
    }
  }

  if (!seedConnected) {
    scanForSeed();
  } else {
    if (seedDataChar.valueUpdated()) {
      Serial.println("Seeed data updated!");
      handleSensorData();
    }
  }

  if (currentMode == BALANCE_MODE) {
    handleBalanceMode();
  }

  checkConnections();

  delay(50);
}

void scanForESP32() {
  static unsigned long lastScan = 0;
  if (millis() - lastScan < 2000) return;

  BLE.scanForName("ESP32_Mode_Controller");
  esp32Peripheral = BLE.available();

  if (esp32Peripheral) {
    Serial.println("Found ESP32 Mode Controller");
    BLE.stopScan();

    if (esp32Peripheral.connect()) {
      Serial.println("Connected to ESP32!");
      esp32Connected = true;

      if (esp32Peripheral.discoverAttributes()) {
        esp32ModeChar = esp32Peripheral.characteristic("87654321-4321-4321-4321-cba987654321");
        esp32CountdownChar = esp32Peripheral.characteristic("87654321-4321-4321-4321-cba987654322");

        if (esp32ModeChar && esp32CountdownChar) {
          esp32ModeChar.subscribe();
          esp32CountdownChar.subscribe();
          Serial.println("Subscribed to ESP32 mode control and countdown");
          digitalWrite(12, HIGH);

          if (esp32ModeChar.canRead()) {
            String initialMode = String((const char*)esp32ModeChar.value());
            Serial.print("Initial mode from ESP32: ");
            Serial.println(initialMode);
            handleModeChange();
          }
        } else {
          Serial.println("Mode or countdown characteristic not found!");
        }
      } else {
        Serial.println("Failed to discover ESP32 attributes");
      }
    } else {
      Serial.println("Failed to connect to ESP32");
    }
  }
  lastScan = millis();
}

void scanForSeed() {
  BLE.scanForName("XIAO_Unified");

  seedPeripheral = BLE.available();

  if (seedPeripheral) {
    Serial.println("Found XIAO_Unified device");
    BLE.stopScan();

    if (seedPeripheral.connect()) {
      Serial.println("Connected to Seeed!");
      seedConnected = true;

      delay(500);

      if (seedPeripheral.discoverAttributes()) {
        seedDataChar = seedPeripheral.characteristic("2A53");

        if (seedDataChar) {
          seedDataChar.subscribe();
          Serial.println("Subscribed to unified Seeed data");

          lcd.setCursor(0, 1);
          lcd.print("Sensor Ready    ");
        } else {
          Serial.println("Failed to find characteristics");
        }
      } else {
        Serial.println("Failed to discover attributes");
        seedPeripheral.disconnect();
        seedConnected = false;
      }
    } else {
      Serial.println("Failed to connect to Seeed");
    }
  }
}

void handleModeChange() {
  String modeData = String((const char*)esp32ModeChar.value());
  modeData.trim();

  int newMode = modeData.toInt();
  if (newMode == 1 || newMode == 2) {
    SystemMode oldMode = currentMode;
    currentMode = (SystemMode)newMode;

    if (oldMode != currentMode) {
      Serial.print("Mode changed to: ");
      Serial.println(currentMode == STEP_MODE ? "STEP" : "BALANCE");

      switchModeDisplay();
      modeChanged = true;
    }
  }
}

void switchModeDisplay() {
  if (currentMode == STEP_MODE) {
    lcd.clear();
    lcd.print("Mode: STEP");
    lcd.setCursor(0, 1);
    lcd.print("Steps: 0        ");

    clearLEDMatrix();
    litIndex = 0;
    lastStepCount = 0;

    needSetOffset = true;
    Serial.println("Step mode activated - will reset on next data");

  } else {
    lcd.clear();
    lcd.print("Mode: BALANCE");
    lcd.setCursor(0, 1);
    lcd.print("Status: Unknown");

    timerMode = NONE;
    isTiming = false;
    isCountingDown = false;
    inputBuffer = "";
    lastBalanceStatus = "";
  }

  Serial.println("Mode switched");
}

void handleSensorData() {
  String unifiedData = String((const char*)seedDataChar.value());
  unifiedData.trim();

  if (unifiedData.length() > 0) {
    Serial.print("Unified data received: ");
    Serial.println(unifiedData);

    int commaIndex = unifiedData.indexOf(',');
    if (commaIndex > 0) {
      uint16_t rawStepCount = unifiedData.substring(0, commaIndex).toInt();
      int balanceStatus = unifiedData.substring(commaIndex + 1).toInt();

      if (currentMode == STEP_MODE) {
        if (needSetOffset) {
          stepOffset = rawStepCount;
          needSetOffset = false;
          modeChanged = false;
          Serial.print("Step offset set to: ");
          Serial.println(stepOffset);
        }
        handleStepData(rawStepCount);
      } else if (currentMode == BALANCE_MODE) {
        handleBalanceData(balanceStatus);
      }
    }
  }
}

void handleStepData(uint16_t rawStepCount) {
  uint16_t adjustedStepCount = (rawStepCount >= stepOffset) ? (rawStepCount - stepOffset) : 0;

  Serial.print("Raw steps: ");
  Serial.print(rawStepCount);
  Serial.print(", Offset: ");
  Serial.print(stepOffset);
  Serial.print(", Adjusted: ");
  Serial.println(adjustedStepCount);

  lcd.setCursor(0, 1);
  lcd.print("Steps: ");
  lcd.print(adjustedStepCount);
  lcd.print("    ");

  if (adjustedStepCount > lastStepCount) {
    int newSteps = adjustedStepCount - lastStepCount;
    for (int i = 0; i < newSteps && i < 5; i++) {
      lightNextLED();
      delay(100);
    }
    lastStepCount = adjustedStepCount;
  }
}

void handleBalanceData(int balanceStatus) {
  String balanceStatusStr = (balanceStatus == 1) ? "Stable" : "Unstable";

  if (balanceStatusStr != lastBalanceStatus) {
    Serial.print("Balance status changed to: ");
    Serial.println(balanceStatusStr);

    if (balanceStatusStr == "Unstable") {
      tone(buzzerPin, NOTE_G4, 100);
      delay(50);
      tone(buzzerPin, NOTE_G4, 100);
    } else if (balanceStatusStr == "Stable") {
      tone(buzzerPin, NOTE_G5, 100);
    }

    lastBalanceStatus = balanceStatusStr;
    lastStatusTime = millis();
  }
}

void handleBalanceMode() {
  char key = keypad.getKey();
  if (key) {
    handleKeypadInput(key);
  }

  handleButtonInput();

  updateBalanceLCD();
}

void handleKeypadInput(char key) {
  if (timerMode == COUNTDOWN_INPUT || timerMode == COUNTDOWN_RUN) {
    if (isdigit(key)) {
      if (timerMode == COUNTDOWN_INPUT) {
        inputBuffer += key;
      }
    } else if (key == 'B' && timerMode == COUNTDOWN_INPUT) {
      countdownSeconds = inputBuffer.toInt();
      inputBuffer = "";
      countdownStart = millis();
      isCountingDown = true;
      timerMode = COUNTDOWN_RUN;
    } else if (key == 'C') {
      inputBuffer = "";
      isCountingDown = false;
      countdownSeconds = 0;
      timerMode = COUNTDOWN_INPUT;
    } else if (key == 'D') {
      inputBuffer = "";
      isCountingDown = false;
      countdownSeconds = 0;
      timerMode = NONE;
    } else if (key == '*' && timerMode == COUNTDOWN_INPUT) {
      inputBuffer = "";
    }
  } else {
    if (key == 'A') {
      timerMode = COUNTDOWN_INPUT;
      inputBuffer = "";
    }
  }
}

void handleButtonInput() {
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
    if (reading == HIGH && lastButtonState == LOW) {
      isTiming = !isTiming;
      if (isTiming) {
        startTime = millis();
        lastMelodyTime = startTime;
        timerMode = STOPWATCH;
      } else {
        timerMode = NONE;
      }
    }
  }
  lastButtonState = reading;
}

void updateBalanceLCD() {
  checkCountdownFinished();

  lcd.setCursor(0, 0);
  if (countdownActive) {
    uint32_t remaining = getRemainingTime();
    lcd.print("Timer: ");
    lcd.print(remaining);
    lcd.print("s       ");

    static uint32_t lastBeep = 0;
    static uint32_t lastRemaining = 0;
    if (remaining != lastRemaining && remaining <= 5 && remaining > 0) {
      if (millis() - lastBeep > 500) {
        if (remaining <= 3) {
          tone(buzzerPin, NOTE_C6, 100);
        } else {
          tone(buzzerPin, NOTE_C5, 100);
        }
        lastBeep = millis();
      }
      lastRemaining = remaining;
    }
  } else if (timerMode == COUNTDOWN_INPUT) {
    lcd.print("Input: ");
    lcd.print(inputBuffer);
    lcd.print("s       ");
  } else if (timerMode == COUNTDOWN_RUN && isCountingDown) {
    unsigned long elapsed = (millis() - countdownStart) / 1000;
    int remaining = countdownSeconds - elapsed;
    lcd.print("Countdown: ");
    lcd.print(remaining);
    lcd.print("s     ");
    if (remaining <= 0) {
      isCountingDown = false;
      playMelody();
      timerMode = NONE;
    }
  } else if (timerMode == STOPWATCH && isTiming) {
    elapsedTime = millis() - startTime;
    lcd.print("Time: ");
    lcd.print(elapsedTime / 1000.0, 1);
    lcd.print("s     ");

    if (millis() - lastMelodyTime >= 30000) {
      playMelody();
      lastMelodyTime = millis();
    }
  } else {
    lcd.print("Mode: BALANCE   ");
  }

  lcd.setCursor(0, 1);
  lcd.print("Status: ");
  lcd.print(lastBalanceStatus);
  lcd.print("       ");
}

void sendCmd(uint16_t cmd, uint8_t data) {
  SPI.beginTransaction(SPI_PARAMETER);
  digitalWrite(pinSS, LOW);
  SPI.transfer(cmd);
  SPI.transfer(data);
  digitalWrite(pinSS, HIGH);
  SPI.endTransaction();
}

void sendData() {
  for (uint8_t i = 0; i < 8; i++) {
    sendCmd(OP_DIGIT0 + i, ledMatrix[i]);
  }
}

void setupMAX7219() {
  pinMode(pinSS, OUTPUT);
  digitalWrite(pinSS, HIGH);

  sendCmd(OP_DISPLAYTEST, 0x00);
  sendCmd(OP_SCANLIMIT, 0x07);
  sendCmd(OP_DECODEMODE, 0x00);
  sendCmd(OP_SHUTDOWN, 0x01);
  sendCmd(OP_INTENSITY, 0x08);
  sendData();
}

void lightNextLED() {
  if (litIndex >= 64) {
    clearLEDMatrix();
    litIndex = 0;
    delay(500);
    return;
  }

  uint8_t row = litIndex / 8;
  uint8_t col = litIndex % 8;
  ledMatrix[row] |= (1 << col);
  sendData();
  litIndex++;
}

void clearLEDMatrix() {
  for (uint8_t i = 0; i < 8; i++) {
    ledMatrix[i] = 0;
  }
  sendData();
}

void handleCountdownUpdate() {
  String countdownData = String((const char*)esp32CountdownChar.value());
  countdownData.trim();

  Serial.print("Countdown data received: ");
  Serial.println(countdownData);

  uint32_t newDuration = countdownData.toInt();

  if (newDuration > 0) {
    countdownDuration = newDuration;
    countdownStartTime = millis();
    countdownActive = true;

    Serial.print("Countdown started: ");
    Serial.print(countdownDuration);
    Serial.println(" seconds");

    tone(buzzerPin, NOTE_C5, 200);
  }
}

uint32_t getRemainingTime() {
  if (!countdownActive) return 0;

  uint32_t elapsed = (millis() - countdownStartTime) / 1000;
  if (elapsed >= countdownDuration) {
    return 0;
  }
  return countdownDuration - elapsed;
}

bool checkCountdownFinished() {
  if (!countdownActive) return false;

  uint32_t remaining = getRemainingTime();
  if (remaining == 0) {
    countdownActive = false;
    Serial.println("Countdown finished!");

    playMelody();
    return true;
  }
  return false;
}
void playMelody() {
  int numNotes = sizeof(melody) / sizeof(melody[0]);
  for (int i = 0; i < numNotes; i++) {
    tone(buzzerPin, melody[i], 50);
    delay(110);
    noTone(buzzerPin);
  }
}

void checkConnections() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) {

    if (esp32Connected && !esp32Peripheral.connected()) {
      esp32Connected = false;
      digitalWrite(12, LOW);
      Serial.println("ESP32 disconnected, reverting to BALANCE mode");
      currentMode = BALANCE_MODE;
      switchModeDisplay();
    }

    if (seedConnected && !seedPeripheral.connected()) {
      seedConnected = false;
      Serial.println("Seeed disconnected");
    }

    lastCheck = millis();
  }
}
