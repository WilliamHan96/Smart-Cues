/**
 * @brief    Smart Cues march-step BLE receiver
 * @author   Smart Cues Group
 * @note     Receives step counts over BLE and provides local feedback for marching activity.
 */

#include <ArduinoBLE.h>
#include <SPI.h>

// ==== MAX7219 ====
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

#define SPI_PARAMETER SPISettings(8000000, MSBFIRST, SPI_MODE0)

// ==== BLE ====
BLEDevice peripheral;
BLECharacteristic stepChar;
uint16_t lastStep = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin();
  setupMAX7219();

  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);

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

  Serial.println("Scanning for XIAO_Step...");
}

void loop() {
  if (!peripheral || !peripheral.connected()) {
    digitalWrite(12, LOW);
    BLE.scanForName("XIAO_Step");
    peripheral = BLE.available();

    if (peripheral) {
      Serial.print("Connecting to ");
      Serial.println(peripheral.address());

      if (peripheral.connect()) {
        Serial.println("Connected!");
        digitalWrite(12, HIGH);

        if (peripheral.discoverAttributes()) {
          stepChar = peripheral.characteristic("2A53");
          if (!stepChar) {
            Serial.println("Step characteristic not found!");
            peripheral.disconnect();
            return;
          }

          stepChar.subscribe();
          lastStep = 0;
        } else {
          Serial.println("Failed to discover attributes");
          peripheral.disconnect();
        }
      } else {
        Serial.println("Connection failed");
      }
    }
  } else {
    if (stepChar.valueUpdated()) {
      uint16_t stepCount;
      stepChar.readValue(stepCount);

      Serial.print("Step Count: ");
      Serial.println(stepCount);

      if (stepCount > lastStep) {
        int steps = stepCount - lastStep;
        for (int i = 0; i < steps; i++) {
          lightNextLED();
          delay(100);
        }
        lastStep = stepCount;
      }
    }

    if (!peripheral.connected()) {
      digitalWrite(12, LOW);
    }
  }
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
    for (uint8_t i = 0; i < 8; i++) {
      ledMatrix[i] = 0;
    }
    sendData();
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
