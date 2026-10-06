/**
 * @brief    Smart Cues puppy interaction controller
 * @author   Smart Cues Group
 * @note     Controls the Smart Cues puppy interaction and wireless communication.
 */


#include <ArduinoBLE.h>
#include <ArduinoGraphics.h>
#include <Arduino_LED_Matrix.h>
#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>

#define Start_Byte     0x7E
#define Version_Byte   0xFF
#define Command_Length 0x06
#define End_Byte       0xEF
#define Acknowledge    0x01

ArduinoLEDMatrix matrix;
SoftwareSerial mySerial(6, 7);  // DFPlayer TX->D6, RX->D7
DFRobotDFPlayerMini myDFPlayer;
bool dfplayer_ready = false;

BLEDevice seeedPeripheral;
BLECharacteristic seeedChar;

const char* peripheralName = "XIAO_Sensor";
const char* serviceUUID = "a9f13584-02d4-4e6c-81a7-f03ce6df0023";
const char* charUUID    = "c1b74b7e-b8da-4d67-bf55-7d84efcab831";

uint16_t count = 0;
unsigned long lastTime = 0;
unsigned long interval = 0;
float frequency = 0;
const float maxFrequency = 30.0;
bool bleConnected = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  matrix.begin();
  mySerial.begin(9600);
  if (myDFPlayer.begin(mySerial)) {
    dfplayer_ready = true;
    myDFPlayer.volume(25);
  }
  myDFPlayer.volume(25);

  BLE.begin();
  BLE.setLocalName("ArduinoBLE_Ctrl");
  BLE.scanForName(peripheralName);
  showCount();
}

void loop() {
  if (!seeedPeripheral || !seeedPeripheral.connected()) {
    BLEDevice discovered = BLE.available();
    if (discovered && discovered.localName() == peripheralName) {
      if (discovered.connect() && discovered.discoverAttributes()) {
        seeedPeripheral = discovered;
        seeedChar = seeedPeripheral.characteristic(charUUID);
        if (seeedChar) seeedChar.subscribe();
        bleConnected = true;
        showBleConnected();
        delay(1000);
        showCount();
      } else {
        BLE.scanForName(peripheralName);
      }
    }
    return;
  }

  BLE.poll();

  if (seeedChar && seeedChar.valueUpdated()) {
    uint8_t val;
    seeedChar.readValue(&val, 1);
    if (val == 1) {
      onProximityDetected();
    }
  }
}

void onProximityDetected() {
  unsigned long currentTime = millis();
  if (lastTime > 0) {
    interval = currentTime - lastTime;
    frequency = 60000.0 / interval;
  }
  lastTime = currentTime;
  count++;
  showSmile();
  delay(1000);
  showCount();
}

void showCount() {
  matrix.beginDraw();
  matrix.clear();
  matrix.stroke(255);
  matrix.textFont(Font_5x7);
  matrix.text(String(count), 0, 0);
  int bar = (frequency >= maxFrequency) ? 12 : (frequency < 1 ? 1 : (int)(frequency / maxFrequency * 12));
  if (bar > 0) matrix.line(0, 7, bar - 1, 7);
  if (bleConnected) matrix.point(11, 0);
  matrix.endDraw();
}

void showSmile() {
  play_sound();
  uint32_t frame[] = {
    0x19819800,
    0x00001081,
    0xF8000000
  };
  int bar = (frequency >= maxFrequency) ? 12 : (frequency < 1 ? 1 : (int)(frequency / maxFrequency * 12));
  for (int i = 0; i < bar; i++) frame[2] |= (1UL << (11 - i));
  if (bleConnected) frame[0] |= (1UL << 31);
  matrix.loadFrame(frame);
}

void showBleConnected() {
  matrix.beginDraw();
  matrix.clear();
  matrix.stroke(255);
  matrix.text("BLE", 0, 0);
  matrix.endDraw();
}

void execute_CMD(byte CMD, byte Par1, byte Par2) {
  word checksum = -(Version_Byte + Command_Length + CMD + Acknowledge + Par1 + Par2);
  byte Command_line[10] = {
    Start_Byte, Version_Byte, Command_Length, CMD, Acknowledge, Par1, Par2,
    highByte(checksum), lowByte(checksum), End_Byte
  };
  for (byte k = 0; k < 10; k++) mySerial.write(Command_line[k]);
}

void play_sound() {
  if (count % 5 == 0) execute_CMD(0x03, 0, 2);
  else execute_CMD(0x03, 0, 1);
}
