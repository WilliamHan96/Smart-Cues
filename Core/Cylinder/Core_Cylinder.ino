/**
 * ESP32 NFC exercise game controller - Waveshare 1.91" AMOLED version
 * Touch: CST816  (shares the same I2C bus with NFC / IMU: SDA=11, SCL=10)
 * LVGL: uses the LVGL_Driver wrapper (Lvgl_Init / Lvgl_Loop / Touch_Loop)
 * Note: retains BLE / WiFi / NFC / IMU / UI / game logic while refactoring display and I2C
 */
/**
 * @brief    Smart Cues multi-mode exercise controller
 * @author   Smart Cues Group
 * @note     Integrates BLE, Wi-Fi, NFC, IMU, ultrasonic sensing, LVGL UI, and game logic.
 */

#include "Display_ST77916.h"
#include "RTC_PCF85063.h"
#include "BAT_Driver.h"

extern "C" {
  #include "esp_coexist.h"
}
#include <esp_heap_caps.h>

#define TONE_PITCH 440
#include <TonePitch.h>
int melody[] = {
  NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5,
  NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6
};
int duration = 100;

#include <stdio.h>

#include "BLE2902.h"
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"

#include "esp_bt.h"
#include <WiFi.h>
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"

// ==== Display / touch / I2C wrappers ====
#include "LVGL_Driver.h"
#include "I2C_Driver.h"

#include <Adafruit_PN532.h>
#include "esp_random.h"

#include <Wire.h>

// ======================================================
// Device config system
// Change only DEVICE_NO for each device
// ======================================================
#define DEVICE_NO 20   // 👈 set this to the device number, e.g. 1 / 2 / 3 ...

// Secondary I2C controller
TwoWire WireExt = TwoWire(1);

// I2C bus used by PN532 / IMU
TwoWire* SensorWire = &Wire;

// Create the PN532 instance after loading the device configuration
Adafruit_PN532* nfc = nullptr;

// Assigned automatically from the device configuration
int BUZZER_PIN = 13;
int US_TRIG = 3;
int US_ECHO = 6;

int I2C2_SDA = 11;
int I2C2_SCL = 10;

bool USE_EXT_I2C_FOR_SENSORS = false;

// ======================================================
// Per-device configuration table
// id, buzzer, ultrasonic trig, ultrasonic echo,
// whether to use secondary I2C, secondary SDA, secondary SCL
// ======================================================
struct DeviceConfig {
  int id;

  int buzzerPin;
  int usTrig;
  int usEcho;

  bool useExtI2CForSensors;
  int i2c2Sda;
  int i2c2Scl;
};

DeviceConfig deviceConfigs[] = {
  // id, buzzer, trig, echo, useExtI2C, SDA2, SCL2

  // Example devices
  {1, 9, 7, 6,  false, -1, -1},
  {2, 13, 3, 6,  false, -1, -1},
  {3, 13, 3, 6,  false, -1, -1},
  {4, 13, 3, 6,  false, -1, -1},
  {5, 13, 7, 6,  true, 12, 0},
  {6, 13, 3, 6,  false, -1, -1},
  {7, 13, 3, 6,  false, -1, -1},
  {8, 13, 3, 6,  false, -1, -1},
  {9, 13, 3, 6,  false, -1, -1},
  {10, 13, 7, 6,  false, -1, -1},
  {11, 13, 7, 6,  false, -1, -1},
  {12, 13, 3, 7,  false, -1, -1},
  {13, 13, 12, 6,  false, -1, -1},
  {14, 13, 3, 7,  false, -1, -1},
  {15, 13, 3, 6,  true,  12,  9},
  {16, 12, 3, 6,  false, -1, -1},
  {17, 13, 3, 6,  true,  12, 9},
  {18, 13, 3, 6,  true, 12, 9},
  {19, 3, 6, 9,  true,  7,  12},
  {20, 13, 3, 6,  false, -1, -1},


  // Add additional device configurations below
  // {6, 13, 3, 6, true, 12, 9},
  // {7, 13, 3, 6, false, -1, -1},
};

const int DEVICE_CONFIG_COUNT =
  sizeof(deviceConfigs) / sizeof(deviceConfigs[0]);

bool applyDeviceConfig(int id) {
  for (int i = 0; i < DEVICE_CONFIG_COUNT; i++) {
    if (deviceConfigs[i].id == id) {
      DeviceConfig cfg = deviceConfigs[i];

      BUZZER_PIN = cfg.buzzerPin;
      US_TRIG = cfg.usTrig;
      US_ECHO = cfg.usEcho;

      USE_EXT_I2C_FOR_SENSORS = cfg.useExtI2CForSensors;
      I2C2_SDA = cfg.i2c2Sda;
      I2C2_SCL = cfg.i2c2Scl;

      Serial.println();
      Serial.println("==============================");
      Serial.printf("This is device no %d\n", id);
      Serial.printf("BUZZER_PIN = GPIO%d\n", BUZZER_PIN);
      Serial.printf("US_TRIG    = GPIO%d\n", US_TRIG);
      Serial.printf("US_ECHO    = GPIO%d\n", US_ECHO);

      if (USE_EXT_I2C_FOR_SENSORS) {
        Serial.printf("Sensor I2C = WireExt, SDA GPIO%d, SCL GPIO%d\n",
                      I2C2_SDA, I2C2_SCL);
      } else {
        Serial.println("Sensor I2C = Wire, onboard SDA GPIO11, SCL GPIO10");
      }

      Serial.println("==============================");
      Serial.println();

      return true;
    }
  }

  Serial.printf("No config found for device no %d\n", id);
  return false;
}


lv_obj_t* startup_overlay = nullptr;
bool systemReady = false;

#define SERVICE_UUID        "a9f238b1-7c3e-4f2c-9359-91b47f8e2fcb"
#define CHARACTERISTIC_UUID "d473cf0c-3e09-4811-bd6e-8fc3d42e6b26"

// === Squat BLE UUIDs ===
#define SQUAT_SERVICE_UUID  "19B10000-E8F2-537E-4F6C-D104768A1215"
#define SQUAT_CHAR_UUID     "19B10001-E8F2-537E-4F6C-D104768A1215"

// ====== Hub WiFi ======
const char* HUB_SSID = "SmartHub_AP";
const char* HUB_PASSWORD = "SmartCues";
const char* HUB_IP = "192.168.4.1";
const uint16_t HUB_PORT = 8888;

bool MODE_MARCH = false;

#include <Preferences.h>
Preferences prefs;
int targetBalance = 0;
int targetMarch   = 0;
int targetPose    = 0;
int targetSit     = 0;
int targetSquat   = 0;
int targetNFC     = 0;
String sitMAC = "";


WiFiServer server(8888);

bool nfcReady = false;

// ========== IMU (LSM6DSOX) ==========
#define LSM6DSOX_ADDR 0x6A
#define WHO_AM_I      0x0F
#define CTRL1_XL      0x10
#define CTRL10_C      0x19
#define TAP_CFG2      0x58
#define TAP_THS_6D    0x59
#define MD1_CFG       0x5E
#define D6D_SRC       0x1D
#define FUNC_CFG_ACCESS 0x01
#define EMB_FUNC_EN_A   0x04
#define STEP_COUNTER_L  0x62
#define STEP_COUNTER_H  0x63

// ========== Global runtime state ==========
static const char *MAIN_TAG = "nfc_exercise_game";
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;
bool motionDetected = false;
// ===== Forward Declarations =====
void show_motion_dialog();
void show_balance_screen();
void show_march_screen();
void show_pose_screen();
void show_sit_screen();
void show_squat_screen();


// ===== Balance detection variables =====
float avgMag = 0;
int stableCount = 0;
float lastMagnitude = 0;


lv_obj_t* btn_yes = nullptr;
lv_obj_t* btn_no = nullptr;
lv_obj_t* label_status = nullptr;
lv_obj_t* main_screen = nullptr;
lv_obj_t* dialog_screen = nullptr;
lv_obj_t* game_screen = nullptr;
lv_obj_t* label_ble = nullptr;
lv_obj_t* squat_screen = nullptr;
lv_obj_t* squat_label = nullptr;
lv_obj_t* sit_label = nullptr;
lv_obj_t* pose_screen = nullptr;
lv_obj_t* pose_label = nullptr;
lv_obj_t* countdown_label = nullptr;
lv_obj_t *label_hub_status = nullptr;

// === Ultrasonic squat (wired) ===
//const int US_TRIG = 3; //7 or 3 37mm 38 better?
//const int US_ECHO = 6;

bool squatMeasuring = false;
unsigned long squatStartTime = 0;
unsigned long squatElapsed = 0;
bool squatActive = false;
lv_timer_t* squatTimer = nullptr;
unsigned long last10sBeep = 0;

BLEClient* squatClient = nullptr;
BLERemoteCharacteristic* squatChar = nullptr;
bool squatConnected = false;
const float squatThresholdCM = 50.0f;

static bool example_lvgl_lock(int /*timeout_ms*/) { return true; }
static void example_lvgl_unlock(void) {}


BLEClient* sitClient = nullptr;
BLERemoteCharacteristic* sitChar = nullptr;
bool sitConnected = false;
int sitCount = 0;
bool lastSitState = 0;
lv_obj_t* sit_hint_label = NULL;

BLEClient* hubClient = nullptr;
BLERemoteCharacteristic* hubChar = nullptr;
bool hubConnected = false;

// === Other state variables ===
uint8_t lastDirection = 0;
uint32_t lastStepTime = 0;
int stepCount = 0;
bool isBalanced = true;
uint16_t stepCountHW = 0;
lv_timer_t* marchTimer = NULL;

int batteryPercentage(float volts) {
    if (volts >= 4.15) return 100;
    if (volts >= 4.10) return 90;
    if (volts >= 4.00) return 80;
    if (volts >= 3.90) return 70;
    if (volts >= 3.80) return 60;
    if (volts >= 3.70) return 50;
    if (volts >= 3.60) return 40;
    if (volts >= 3.50) return 30;
    if (volts >= 3.40) return 20;
    if (volts >= 3.30) return 10;
    return 5;
}

// === Pose: new flow without 6D pose checking ===

enum PosePhase {
  POSE_PREP,
  POSE_WAIT_STABLE,
  POSE_HOLD,
  POSE_REST
};
int restCountdown = 2;

PosePhase posePhase = POSE_PREP;

lv_timer_t* pose_timer = nullptr;

int prepCountdown = 5;
int holdCountdown = 5;


// === Pose Challenge ===
const char* poseList[] = {"Front", "Squat", "Side (left/right)", "Hand Down", "Hand Up"};
int generatedPoses[5];
int currentPoseIndex = 0;
int totalPoses = 5;
int countdownValue = 5;
bool poseActive = false;

// === Game ===
enum GameState { WAITING, DIALOG, PLAYING, COMPLETED };
GameState currentGameState = WAITING;
uint8_t gameSequence[4];
int currentStep = 0;
lv_obj_t* sequence_labels[4];
lv_obj_t* progress_label = nullptr;
lv_obj_t* instruction_label = nullptr;

static uint32_t dialogStartTime = 0;
static bool dialogVisible = false;

struct NFCCard {
  uint8_t uid[4];
  uint8_t number;
};
const NFCCard nfcCards[] = {
  {{0x36, 0xbb, 0xcc, 0x05}, 1},
  {{0x85, 0x72, 0xb0, 0x05}, 2},
  {{0xbd, 0x8b, 0xb3, 0x05}, 3},
  {{0x53, 0xb2, 0xb3, 0x05}, 4}
};

void scanI2CBus(TwoWire &bus, const char* busName) {
    Serial.printf("\n🔎 Scanning %s...\n", busName);

    int found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        bus.beginTransmission(addr);
        uint8_t error = bus.endTransmission();

        if (error == 0) {
            Serial.printf("✅ %s device found at 0x%02X\n", busName, addr);
            found++;
        } else if (error == 4) {
            Serial.printf("⚠️ %s unknown error at 0x%02X\n", busName, addr);
        }

        delay(2);
    }

    if (found == 0) {
        Serial.printf("❌ No I2C devices found on %s\n", busName);
    } else {
        Serial.printf("✅ %s scan done, found %d device(s)\n", busName, found);
    }
}

uint8_t getNFCCardNumber(uint8_t* uid, uint8_t uidLength) {
  if (uidLength < 4) return 0;
  for (int i = 0; i < 4; i++) {
    bool match = true;
    for (int j = 0; j < 4; j++) {
      if (uid[j] != nfcCards[i].uid[j]) { match = false; break; }
    }
    if (match) {
      Serial.printf("🎯 Recognized card %d\n", nfcCards[i].number);
      return nfcCards[i].number;
    }
  }
  Serial.print("❓ Unknown card: ");
  for (int i = 0; i < uidLength; i++) { if (uid[i] < 0x10) Serial.print("0"); Serial.print(uid[i], HEX); Serial.print(" "); }
  Serial.println();
  return 0;
}


void imu_write(uint8_t reg, uint8_t value) {
  SensorWire->beginTransmission(LSM6DSOX_ADDR);
  SensorWire->write(reg);
  SensorWire->write(value);
  SensorWire->endTransmission();
}

uint8_t imu_read(uint8_t reg) {
  SensorWire->beginTransmission(LSM6DSOX_ADDR);
  SensorWire->write(reg);

  if (SensorWire->endTransmission(false) != 0) {
    return 0xFF;
  }

  SensorWire->requestFrom(LSM6DSOX_ADDR, 1);

  if (SensorWire->available()) {
    return SensorWire->read();
  }

  return 0xFF;
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) override {
    deviceConnected = true;
    Serial.println("🔗 Connected to Arduino!");
  }
  void onDisconnect(BLEClient* pclient) override {
    deviceConnected = false;
    Serial.println("❌ Disconnected from Arduino!");
  }
};
static void notifyCallback(BLERemoteCharacteristic*, uint8_t*, size_t, bool) {
  motionDetected = true;
}

void squatNotifyCallback(BLERemoteCharacteristic* pCharacteristic, uint8_t* data, size_t length, bool isNotify) {
  if (length < 1) return;
  int value = data[0];
  Serial.printf("📩 Squat notify: %d\n", value);

  if (value == 1 && !squatActive) {
    squatActive = true;
    squatStartTime = millis();
    squatElapsed = 0;
    lv_async_call([](void*) {
        char buf[32];
        sprintf(buf, "0 / %ds", targetSquat);
        lv_label_set_text(squat_label, buf);
        lv_obj_set_style_text_font(squat_label, &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(squat_label, lv_color_hex(0xFFFFFF), 0);
    }, NULL);


    if (squatTimer) lv_timer_del(squatTimer);
    squatTimer = lv_timer_create([](lv_timer_t *timer) {
      squatElapsed = (millis() - squatStartTime) / 1000;
      char *buf = (char*)malloc(32);
      sprintf(buf, "%lu / %ds", squatElapsed, targetSquat);
      lv_async_call([](void* txt) {
          lv_label_set_text(squat_label, (const char*)txt);
          free(txt);
      }, strdup(buf));


      if (squatElapsed > 0 && squatElapsed % 10 == 0 && squatElapsed % 30 != 0) {
          tone(BUZZER_PIN, 880, 200);
          Serial.println("🔔 Beep (10s milestone)");
      }

      if (squatElapsed > 0 && squatElapsed % 30 == 0) {
          Serial.println("🎵 Playing melody (30s milestone)");
          for (int i = 0; i < 8; i++) {
              tone(BUZZER_PIN, melody[i], duration);
              delay(duration * 1.3);
          }
          noTone(BUZZER_PIN);
      }
    }, 1000, NULL);
  } else if (value == 0 && squatActive) {
    squatActive = false;
    if (squatTimer) { lv_timer_del(squatTimer); squatTimer = nullptr; }
    char *buf = (char*)malloc(64);
    sprintf(buf, "Finished: %lus", squatElapsed);
    lv_async_call([](void* txt) {
      lv_label_set_text(squat_label, (const char*)txt);
      lv_obj_set_style_text_font(squat_label, &lv_font_montserrat_28, 0);
      lv_obj_set_style_text_color(squat_label, lv_color_hex(0x00FFFF), 0);
      free(txt);
    }, buf);
    Serial.printf("Squat finished after %lus\n", squatElapsed);
  }
}

void sitNotifyCallback(BLERemoteCharacteristic*, uint8_t* data, size_t length, bool) {
  if (length < 1) return;
  int value = data[0];  // 1 = sitting, 0 = stand
  Serial.printf("📩 Sit notify: %d\n", value);

    if (lastSitState == 1 && value == 0) {
        sitCount++;

        Serial.printf("Sit-to-Stand Count: %d\n", sitCount);

        if (sitCount % 5 == 0 && sitCount % 15 != 0) {
            tone(BUZZER_PIN, 880, 200);
            Serial.println("🔔 Beep (5x milestone)");
        }

        if (sitCount % 15 == 0) {
            Serial.println("🎵 Playing melody (15x milestone)");
            for (int i = 0; i < 8; i++) {
                tone(BUZZER_PIN, melody[i], duration);
                delay(duration * 1.3);
            }
            noTone(BUZZER_PIN);
        }
    }
    lastSitState = value;

  lv_async_call([](void*) {
    char buf[64];
    sprintf(buf, "Sit-to-Stand: %d / %d", sitCount, targetSit);
    lv_label_set_text(sit_label, buf);
    lv_obj_set_style_text_font(sit_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(sit_label, lv_color_hex(0xFFFFFF), 0);
  }, NULL);
}

// ========== BLE Functions ==========

void update_ble_status_label() {
  if (!label_ble) return;

  if (example_lvgl_lock(50)) {
    const char* status = deviceConnected ? "Status: Connected" : "Status: Waiting...";
    lv_label_set_text(label_ble, status);
    example_lvgl_unlock();
  }
}

void setup_ble_server() {
    Serial.println("🔵 Initializing BLE Server...");

    if (!btStart()) {
        Serial.println("❌ Failed to initialize BT controller");
        return;
    }

    BLEDevice::init("ESP32_NFC_Game_Controller");
    Serial.println("✅ BLE Device initialized");


    delay(100);

    pServer = BLEDevice::createServer();
    if (!pServer) {
        Serial.println("❌ Failed to create BLE server");
        return;
    }

    // Create server callback class
    class ServerCallbacks: public BLEServerCallbacks {
        void onConnect(BLEServer* pServer) {
          deviceConnected = true;
          Serial.println("🔗 Arduino connected!");
          update_ble_status_label();
        };

        void onDisconnect(BLEServer* pServer) {
          deviceConnected = false;
          Serial.println("❌ Arduino disconnected!");
          update_ble_status_label();
          pServer->getAdvertising()->start();
          Serial.println("🔄 Restarted advertising");
        }
    };

    pServer->setCallbacks(new ServerCallbacks());
    Serial.println("✅ BLE Server created");

    BLEService *pService = pServer->createService(SERVICE_UUID);
    if (!pService) {
        Serial.println("❌ Failed to create BLE service");
        return;
    }
    Serial.println("✅ BLE Service created");

    pCharacteristic = pService->createCharacteristic(
                         CHARACTERISTIC_UUID,
                         BLECharacteristic::PROPERTY_READ |
                         BLECharacteristic::PROPERTY_WRITE |
                         BLECharacteristic::PROPERTY_NOTIFY
                       );

    if (!pCharacteristic) {
        Serial.println("❌ Failed to create BLE characteristic");
        return;
    }

    // Create characteristic callback class
    class CharacteristicCallbacks: public BLECharacteristicCallbacks {
        void onWrite(BLECharacteristic* pCharacteristic) {
          String value = pCharacteristic->getValue();
          Serial.print("📡 Received from Arduino: ");
          Serial.println(value);
          Serial.printf("🧪 Before show_motion_dialog: dialogVisible=%d, currentGameState=%d\n",
              dialogVisible, currentGameState);

          if (value == "motion_detected") {
            motionDetected = true;
            Serial.println("💬 Motion detected, will show dialog");
            show_motion_dialog();

          }
        }
    };

    // Set write callback to receive motion signals
    pCharacteristic->setCallbacks(new CharacteristicCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    Serial.println("✅ BLE Characteristic created");

    pCharacteristic->setValue("ready");

    pService->start();
    Serial.println("✅ BLE Service started");

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    if (!pAdvertising) {
        Serial.println("❌ Failed to get advertising");
        return;
    }

    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);

    pAdvertising->start();
    Serial.println("🎯 BLE Server ready!");
}


bool connectSquatPeripheral(const char* targetName = "Squ_Sender") {
    if (squatClient) {
    if (squatClient->isConnected()) squatClient->disconnect();
    delete squatClient;
    squatClient = nullptr;
    delay(50);
    }

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);


    Serial.println("🔍 Scanning for Squ_Sender...");
    BLEScan* scanner = BLEDevice::getScan();
    scanner->setActiveScan(true);
    BLEScanResults results = *scanner->start(3);

    for (int i = 0; i < results.getCount(); i++) {
        BLEAdvertisedDevice device = results.getDevice(i);
        if (device.getName() == targetName) {
            Serial.println("✅ Found Squ_Sender! Connecting...");
            squatClient = BLEDevice::createClient();
            squatClient->connect(&device);
            Serial.println("🔗 Connected to Squ_Sender");

            BLERemoteService* svc = squatClient->getService(SQUAT_SERVICE_UUID);
            if (!svc) {
                Serial.println("❌ Service not found");
                return false;
            }

            squatChar = svc->getCharacteristic(SQUAT_CHAR_UUID);
            if (!squatChar) {
                Serial.println("❌ Characteristic not found");
                return false;
            }

            squatChar->registerForNotify(squatNotifyCallback);
            squatConnected = true;
            Serial.println("✅ Notifications enabled for squat characteristic");
            return true;
        }
    }
    Serial.println("❌ Squ_Sender not found");
    return false;
}

bool connectSquatByMAC(const char* macAddr = "26:a1:9e:e3:6a:e1") {

    if (squatClient) {
        if (squatClient->isConnected()) squatClient->disconnect();
        delete squatClient;
        squatClient = nullptr;
        delay(50);
    }

    Serial.printf("🔗 Connecting to Squat device (MAC: %s)...\n", macAddr);

    BLEAddress address(macAddr);
    squatClient = BLEDevice::createClient();
    squatClient->setClientCallbacks(new MyClientCallback());

    if (!squatClient->connect(address)) {
        Serial.println("❌ Failed to connect to Squat (MAC)");
        return false;
    }

    Serial.println("✅ Connected to Squat Peripheral via MAC!");

    // === Get service ===
    BLERemoteService* svc =
        squatClient->getService(SQUAT_SERVICE_UUID);
    if (!svc) {
        Serial.println("❌ Squat Service not found!");
        return false;
    }

    // === Get characteristic ===
    squatChar = svc->getCharacteristic(SQUAT_CHAR_UUID);
    if (!squatChar) {
        Serial.println("❌ Squat Characteristic not found!");
        return false;
    }

    // === Register notification callback ===
    if (squatChar->canNotify()) {
        squatChar->registerForNotify(squatNotifyCallback);
    }

    squatConnected = true;
    Serial.println("📡 Squat notify enabled!");
    return true;
}

bool connectToSitPeripheral() {

    Serial.println("🔍 Scanning (UUID filter)...");

    BLEScan* scanner = BLEDevice::getScan();
    scanner->setActiveScan(true);
    scanner->setInterval(100);
    scanner->setWindow(99);

    scanner->clearResults();

    BLEScanResults* results = scanner->start(2, false);

    int count = results->getCount();
    Serial.printf("Found %d devices\n", count);

    BLEUUID targetUUID("19B10000-E8F2-537E-4F6C-D104768A1214");

    for (int i = 0; i < count; i++) {

        BLEAdvertisedDevice device = results->getDevice(i);

        String addr = device.getAddress().toString().c_str();
        Serial.printf("Device: %s\n", addr.c_str());

        if (device.haveServiceUUID() &&
            device.isAdvertisingService(targetUUID)) {

            Serial.println("✅ Found device by UUID");

            if (!sitClient) {
                sitClient = BLEDevice::createClient();
                sitClient->setClientCallbacks(new MyClientCallback());
            }

            BLEAddress address = device.getAddress();

            if (!sitClient->connect(address)) {
                Serial.println("❌ Connect failed");
                return false;
            }

            BLERemoteService* service =
                sitClient->getService(targetUUID);

            if (!service) {
                Serial.println("❌ Service not found after connect");
                sitClient->disconnect();
                return false;
            }

            sitChar =
                service->getCharacteristic(
                    "19B10001-E8F2-537E-4F6C-D104768A1214");

            if (!sitChar) {
                Serial.println("❌ Characteristic not found");
                sitClient->disconnect();
                return false;
            }

            if (sitChar->canNotify())
                sitChar->registerForNotify(sitNotifyCallback);

            Serial.println("✅ Sit ready (UUID)");

            // Save MAC address
            prefs.begin("bleStore", false);
            prefs.putString("sit_mac", addr);
            prefs.end();

            sitMAC = addr;

            return true;
        }
    }

    Serial.println("❌ UUID device not found");
    return false;
}

bool connectToSitPeripheral_orf(const char* name = "FSR_ESP32") {

    if (sitClient) {
    if (sitClient->isConnected()) sitClient->disconnect();
    delete sitClient;
    sitClient = nullptr;
    delay(50);
    }

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);


    Serial.printf("🔍 Scanning for Sit Peripheral (%s)...\n", name);
    BLEScan* scanner = BLEDevice::getScan();
    scanner->setActiveScan(true);
    BLEScanResults results = *scanner->start(3);

    for (int i = 0; i < results.getCount(); i++) {
        BLEAdvertisedDevice dev = results.getDevice(i);
        if (dev.haveName() && dev.getName() == name) {
            Serial.printf("✅ Found Sit Peripheral: %s\n", dev.getAddress().toString().c_str());
            sitClient = BLEDevice::createClient();
            sitClient->connect(&dev);

            BLERemoteService* service = sitClient->getService("19B10000-E8F2-537E-4F6C-D104768A1214");
            if (!service) {
                Serial.println("❌ Sit Service not found!");
                return false;
            }

            sitChar = service->getCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214");
            if (!sitChar) {
                Serial.println("❌ Sit Characteristic not found!");
                return false;
            }

            if (sitChar->canNotify()) sitChar->registerForNotify(sitNotifyCallback);
            sitConnected = true;
            Serial.println("✅ Connected to FSR_ESP32 (Sit)");
            return true;
        }
    }

    Serial.println("❌ Sit Peripheral not found");
    return false;
}

bool connectToSitByMAC(const char* macAddr) {

    Serial.println("⚡ Trying MAC connection...");

    if (!sitClient) {
        sitClient = BLEDevice::createClient();
        sitClient->setClientCallbacks(new MyClientCallback());
    }

    BLEAddress address(macAddr);

    if (!sitClient->connect(address)) {
        Serial.println("❌ MAC connect failed");
        return false;
    }

    Serial.println("✅ Connected via MAC");

    BLERemoteService* service =
        sitClient->getService("19B10000-E8F2-537E-4F6C-D104768A1214");

    if (!service) {
        Serial.println("❌ Service not found");
        sitClient->disconnect();
        return false;
    }

    sitChar =
        service->getCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214");

    if (!sitChar) {
        Serial.println("❌ Characteristic not found");
        sitClient->disconnect();
        return false;
    }

    if (sitChar->canNotify())
        sitChar->registerForNotify(sitNotifyCallback);

    Serial.println("✅ Sit ready (MAC)");
    return true;
}

bool connectToSitByMAC_org(const char* macAddr = "6e:13:1d:ef:8a:64") {

    if (sitClient) {
        if (sitClient->isConnected()) sitClient->disconnect();
        delete sitClient;
        sitClient = nullptr;
        delay(50);
    }

    Serial.printf("🔗 Connecting to Sit device (MAC: %s)...\n", macAddr);

    BLEAddress address(macAddr);
    sitClient = BLEDevice::createClient();

    sitClient->setClientCallbacks(new MyClientCallback());

    if (!sitClient->connect(address)) {
        Serial.println("❌ Failed to connect (MAC)");
        return false;
    }

    Serial.println("✅ Connected to Sit Peripheral via MAC!");

    BLERemoteService* service =
        sitClient->getService("19B10000-E8F2-537E-4F6C-D104768A1214");
    if (!service) {
        Serial.println("❌ Sit Service not found!");
        return false;
    }

    sitChar = service->getCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214");
    if (!sitChar) {
        Serial.println("❌ Sit Characteristic not found!");
        return false;
    }

    if (sitChar->canNotify())
        sitChar->registerForNotify(sitNotifyCallback);

    sitConnected = true;
    Serial.println("📡 Sit notify enabled!");
    return true;
}

void sendResultToHub(const char* exercise, int value, const char* unit = "") {
  char msg[64];
  sprintf(msg, "%s:%d%s\n", exercise, value, unit);
  bool sent = false;

  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    if (client.connect(HUB_IP, HUB_PORT)) { client.print(msg); client.stop(); sent = true; Serial.printf("📡 [WiFi] Sent to Hub: %s", msg); }
    else { Serial.println("⚠️ [WiFi] Hub connection failed"); }
  }
  if (!sent) Serial.println("⚠️ No available link to Hub (Wi-Fi failed)");
}


// ===== Global mode index =====
int currentModeIndex = 0;

const char* modeNames[] = {
    "Balance",     // 0
    "March",       // 1
    "Pose",        // 2
    "Sit",         // 3
    "Squat",       // 4
    "NFC"          // 5
};
const int modeCount = 6;


lv_obj_t* mode_label_global = NULL;

// ===== Refresh mode display =====
void refreshModeLabel() {
    if (mode_label_global)
        lv_label_set_text(mode_label_global, modeNames[currentModeIndex]);

    if (sit_hint_label)
        lv_label_set_text(sit_hint_label, "");
}

void runCurrentMode() {
    switch (currentModeIndex) {

        case 0:  // Balance
            show_balance_screen();
            break;

        case 1:  // March
            MODE_MARCH = true;
            show_march_screen();
            break;

        case 2:  // Pose
            show_pose_screen();
            break;

        case 3:  // Sit-to-Stand (FSR)
            if (sit_hint_label)
                lv_label_set_text(sit_hint_label, "Connecting...");

            xTaskCreatePinnedToCore([](void*) {

                bool ok = false;

                if (sitMAC.length() > 0) {
                    Serial.println("⚡ Trying fast MAC connection...");
                    ok = connectToSitByMAC(sitMAC.c_str());
                }

                if (!ok) {
                    Serial.println("🔍 MAC failed, scanning...");
                    ok = connectToSitPeripheral();

                    if (ok && sitClient) {
                        String newMAC = sitClient->getPeerAddress().toString().c_str();
                        Serial.printf("💾 Saving new MAC: %s\n", newMAC.c_str());

                        prefs.begin("bleStore", false);
                        prefs.putString("sit_mac", newMAC);
                        prefs.end();

                        sitMAC = newMAC;
                    }
                }

                lv_async_call([](void* param) {

                    bool success = (bool)param;

                    if (success) {
                        lv_label_set_text(sit_hint_label, "Connected");
                        show_sit_screen();
                    } else {
                        lv_label_set_text(sit_hint_label, "Connection Failed");
                    }

                }, (void*)ok);

                vTaskDelete(NULL);

            }, "sit_connect_task", 4096, NULL, 1, NULL,1);


            break;


            //break;

            //break;

        case 4:  // Long Squat (Ultrasonic)
            //break;

            //break;

            show_squat_screen();
            break;

        case 5:  // NFC Game
            show_motion_dialog();
            break;
    }
}


// ===== Button events =====
void on_center_btn(lv_event_t* e) {
    runCurrentMode();
}

void on_left_btn(lv_event_t* e) {
    currentModeIndex--;
    if (currentModeIndex < 0) currentModeIndex = modeCount - 1;
    refreshModeLabel();
}

void on_right_btn(lv_event_t* e) {
    currentModeIndex++;
    if (currentModeIndex >= modeCount) currentModeIndex = 0;
    refreshModeLabel();
}


void create_main_ui() {
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_PART_MAIN);

    main_screen = lv_obj_create(NULL);
    lv_scr_load(main_screen);
    lv_obj_set_style_bg_color(main_screen, lv_color_hex(0x383030), LV_PART_MAIN);
    lv_obj_set_style_pad_all(main_screen, 0, LV_PART_MAIN);

    // ===== Title =====
    lv_obj_t *title = lv_label_create(main_screen);
    lv_label_set_text(title, "Exercise Menu");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 35);

    // ===== Main centre button =====
    lv_obj_t* mode_btn = lv_btn_create(main_screen);
    lv_obj_set_size(mode_btn, 150, 120);
    lv_obj_align(mode_btn, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_bg_color(mode_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_radius(mode_btn, 14, 0);
    lv_obj_add_event_cb(mode_btn, on_center_btn, LV_EVENT_CLICKED, NULL);

    mode_label_global = lv_label_create(mode_btn);
    lv_label_set_text(mode_label_global, modeNames[currentModeIndex]);
    lv_obj_set_style_text_font(mode_label_global, &lv_font_montserrat_36, 0);
    lv_obj_set_style_text_color(mode_label_global, lv_color_hex(0x000000), 0);
    lv_obj_center(mode_label_global);

    // ===== Left button =====
    lv_obj_t* btn_left = lv_btn_create(main_screen);
    lv_obj_set_size(btn_left, 80, 80);
    lv_obj_set_style_bg_color(btn_left, lv_color_hex(0x777777), LV_PART_MAIN);
    lv_obj_align(btn_left, LV_ALIGN_LEFT_MID, 20, 20);
    lv_obj_add_event_cb(btn_left, on_left_btn, LV_EVENT_CLICKED, NULL);

    lv_obj_t* lbl_left = lv_label_create(btn_left);
    lv_label_set_text(lbl_left, "<");
    lv_obj_set_style_text_font(lbl_left, &lv_font_montserrat_36, 0);
    lv_obj_center(lbl_left);

    // ===== Right button =====
    lv_obj_t* btn_right = lv_btn_create(main_screen);
    lv_obj_set_size(btn_right, 80, 80);
    lv_obj_set_style_bg_color(btn_right, lv_color_hex(0x777777), LV_PART_MAIN);
    lv_obj_align(btn_right, LV_ALIGN_RIGHT_MID, -20, 20);
    lv_obj_add_event_cb(btn_right, on_right_btn, LV_EVENT_CLICKED, NULL);

    lv_obj_t* lbl_right = lv_label_create(btn_right);
    lv_label_set_text(lbl_right, ">");
    lv_obj_set_style_text_font(lbl_right, &lv_font_montserrat_36, 0);
    lv_obj_center(lbl_right);

    // Sit Status hint label (empty by default)
    sit_hint_label = lv_label_create(main_screen);
    lv_label_set_text(sit_hint_label, "");
    lv_obj_set_style_text_font(sit_hint_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(sit_hint_label, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align_to(sit_hint_label, mode_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);


    // ===== Hub Status =====
    label_hub_status = lv_label_create(main_screen);
    lv_label_set_text(label_hub_status, "Hub: Disconnected");
    lv_obj_set_style_text_color(label_hub_status, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(label_hub_status, LV_ALIGN_BOTTOM_MID, 0, -25);

    // ==== Battery Label ====
    static lv_obj_t* battery_label;
    battery_label = lv_label_create(main_screen);
    lv_label_set_text(battery_label, "");
    lv_obj_set_style_text_color(battery_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_30, 0);
    lv_obj_align(battery_label, LV_ALIGN_TOP_MID, 0, 70);

    lv_timer_create([](lv_timer_t* timer){
        lv_obj_t* lbl = (lv_obj_t*)timer->user_data;
        float volts = BAT_analogVolts;

        // ✅ Hide invalid voltage values
        if (volts < 3.0f || volts > 4.35f) {
            lv_label_set_text(lbl, "");
            return;
        }

        int percent = batteryPercentage(volts);  // Convert voltage to percentage

        if (percent <= 15) {
            lv_label_set_text(lbl, "BAT LOW");
            lv_obj_set_style_text_color(lbl, lv_color_hex(0xFF0000), 0);
        } else {
                lv_label_set_text(lbl, "");
                lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
        }

        Serial.printf("UI Voltage Update: %.3f V (%d%%)\n", volts, percent);


    }, 10000, battery_label);


    Serial.println("✅ Single-item UI created successfully");
}


void generate_non_repeating_sequence(uint8_t* seq, size_t len = 4) {
  uint8_t last = 0;
  for (size_t i = 0; i < len; ++i) {
    uint8_t r;
    do {
      r = (esp_random() % 4) + 1;
    } while (i > 0 && r == last);
    seq[i] = r;
    last = r;
  }

  Serial.print("🎲 Generated truly random sequence: ");
  for (size_t i = 0; i < len; i++) {
    Serial.print(seq[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void generate_pose_sequence() {
    uint8_t last = 255;
    for (int i = 0; i < totalPoses; i++) {
        int r;
        do {
            r = esp_random() % 5;  // 0~4
        } while (r == last);
        generatedPoses[i] = r;
        last = r;
    }

    Serial.print("🎲 Pose sequence: ");
    for (int i = 0; i < totalPoses; i++) {
        Serial.printf("%s ", poseList[generatedPoses[i]]);
    }
    Serial.println();
}


void create_dialog_ui() {
    // Create dialog screen
    dialog_screen = lv_obj_create(NULL);

    // Set background color
    lv_obj_set_style_bg_color(dialog_screen, lv_color_hex(0x1F1F1F), LV_PART_MAIN);

    // Dialog box
    lv_obj_t *dialog_box = lv_obj_create(dialog_screen);
    lv_obj_set_size(dialog_box, 400, 180);
    lv_obj_align(dialog_box, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(dialog_box, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_border_color(dialog_box, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_set_style_border_width(dialog_box, 2, LV_PART_MAIN);

    // Motion detected text
    lv_obj_t *motion_text = lv_label_create(dialog_box);
    lv_label_set_text(motion_text, "NFC GAME?");
    lv_obj_set_style_text_color(motion_text, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(motion_text, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(motion_text, LV_ALIGN_TOP_MID, 0, 5);

    // YES button
    btn_yes = lv_btn_create(dialog_box);
    lv_obj_set_size(btn_yes, 150, 80);
    lv_obj_align(btn_yes, LV_ALIGN_BOTTOM_LEFT, 30, -20);
    lv_obj_set_style_bg_color(btn_yes, lv_color_hex(0x00AA00), LV_PART_MAIN);
    lv_obj_t *yes_label = lv_label_create(btn_yes);
    lv_label_set_text(yes_label, "YES");
    lv_obj_set_style_text_font(yes_label, &lv_font_montserrat_28, 0);
    lv_obj_center(yes_label);

    // NO button
    btn_no = lv_btn_create(dialog_box);
    lv_obj_set_size(btn_no, 150, 80);
    lv_obj_align(btn_no, LV_ALIGN_BOTTOM_RIGHT, -30, -20);
    lv_obj_set_style_bg_color(btn_no, lv_color_hex(0xAA0000), LV_PART_MAIN);
    lv_obj_t *no_label = lv_label_create(btn_no);
    lv_label_set_text(no_label, "NO");
    lv_obj_set_style_text_font(no_label, &lv_font_montserrat_28, 0);
    lv_obj_center(no_label);

    // Button events
    lv_obj_add_event_cb(btn_yes, [](lv_event_t *e) {
        Serial.println("✅ User tapped YES - Starting game!");

        generate_non_repeating_sequence(gameSequence);
        currentStep = 0;
        currentGameState = PLAYING;
        dialogVisible = false;

        Serial.print("🎲 Generated sequence: ");
        for (int i = 0; i < 4; i++) {
            Serial.print(gameSequence[i]);
            Serial.print(" ");
        }
        Serial.println();

        Serial.println("🎮 Switching to game screen");
        lv_scr_load(game_screen);
        update_sequence_display_direct();

        if (deviceConnected && pCharacteristic) {
            pCharacteristic->setValue("game_started");
            pCharacteristic->notify();
            Serial.println("📡 Notified Arduino: game_started");
        }

        Serial.println("✅ Game screen loaded");
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(btn_no, [](lv_event_t *e) {
        Serial.println("❌ User clicked NO");
        hide_dialog();
    }, LV_EVENT_CLICKED, NULL);

    Serial.println("✅ Dialog UI created");
}

void create_game_ui() {
    // Create game screen
    game_screen = lv_obj_create(NULL);

    // Set background color
    lv_obj_set_style_bg_color(game_screen, lv_color_hex(0x0F0F0F), LV_PART_MAIN);

    // Title

    // Instruction
    instruction_label = lv_label_create(game_screen);
    lv_label_set_text(instruction_label, "Tag cards in this order:");
    lv_obj_set_style_text_color(instruction_label, lv_color_hex(0xFFFF00), 0);
    lv_obj_set_style_text_font(instruction_label, &lv_font_montserrat_22, LV_PART_MAIN);
    lv_obj_align(instruction_label, LV_ALIGN_TOP_MID, 0, 60);

    // Sequence display (horizontal layout)
    lv_obj_t *sequence_container = lv_obj_create(game_screen);
    lv_obj_set_size(sequence_container, 420, 100);
    lv_obj_align(sequence_container, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_bg_color(sequence_container, lv_color_hex(0x2F2F2F), LV_PART_MAIN);
    lv_obj_set_style_border_width(sequence_container, 0, LV_PART_MAIN);

    // Create sequence labels
    for (int i = 0; i < 4; i++) {
        sequence_labels[i] = lv_label_create(sequence_container);
        lv_obj_set_style_text_font(sequence_labels[i], &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_color(sequence_labels[i], lv_color_hex(0xFFFFFF), 0);
        lv_obj_align(sequence_labels[i], LV_ALIGN_LEFT_MID, 50 + i * 80, 0);
        lv_label_set_text(sequence_labels[i], "?");
    }

    // Progress label
    progress_label = lv_label_create(game_screen);
    lv_label_set_text(progress_label, "Step 1/4 - Tag the first card");
    lv_obj_set_style_text_color(progress_label, lv_color_hex(0x00FFFF), 0);
    lv_obj_set_style_text_font(progress_label, &lv_font_montserrat_28, 0);
    lv_obj_align(progress_label, LV_ALIGN_BOTTOM_MID, 0, -80);

    // Back button
    lv_obj_t *back_btn = lv_btn_create(game_screen);
    lv_obj_set_size(back_btn, 140, 60);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "Back");
    lv_obj_set_style_text_font(back_label, &lv_font_montserrat_28, 0);
    lv_obj_center(back_label);

    lv_obj_add_event_cb(back_btn, [](lv_event_t *e) {
      Serial.println("🔙 Back button pressed - user cancelled game");

      if (deviceConnected && pCharacteristic) {
          pCharacteristic->setValue("game_ended");
          pCharacteristic->notify();
          Serial.println("📡 Notified Arduino: game_ended");
      }

      show_game_cancelled();
    }, LV_EVENT_CLICKED, NULL);

    Serial.println("✅ Game UI created");
}

void show_squat_screen_org() {

    Serial.println("📺 Showing Local Squat Screen");

    // ==== UI ====
    squat_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(squat_screen, lv_color_hex(0x000000), LV_PART_MAIN);

    lv_obj_t *title = lv_label_create(squat_screen);
    lv_label_set_text(title, "Squat Timer");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    squat_label = lv_label_create(squat_screen);
    lv_label_set_text(squat_label, "Squat and hold");
    lv_obj_set_style_text_font(squat_label, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(squat_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(squat_label, LV_ALIGN_CENTER, 0, 0);

    // End button
    lv_obj_t *back_btn = lv_btn_create(squat_screen);
    lv_obj_set_size(back_btn, 150, 60);
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_text_font(back_btn, &lv_font_montserrat_28, 0);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x555555), LV_PART_MAIN);
    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "End");
    lv_obj_center(back_label);

    lv_obj_add_event_cb(back_btn, [](lv_event_t *e) {

        Serial.println("🔙 Exit Squat Screen");

        squatMeasuring = false;
        squatActive    = false;
        noTone(BUZZER_PIN);

        if (squatTimer) {
            lv_timer_del(squatTimer);
            squatTimer = nullptr;
        }

        if (squatElapsed > 0) {
            sendResultToHub("Squat", squatElapsed, "s");
        }

        lv_scr_load(main_screen);

    }, LV_EVENT_CLICKED, NULL);

    // ==== Initialize state ====
    squatMeasuring = true;
    squatActive    = false;
    squatElapsed   = 0;
    squatStartTime = 0;
    last10sBeep    = 0;

    squatTimer = lv_timer_create([](lv_timer_t *t) {

        if (!squatMeasuring) return;

        float d = readUltrasonicCM();
        if (d < 0) return;

        bool nowSquatting;
        nowSquatting = (d < squatThresholdCM);

        static bool wasSquatting = false;
        static unsigned long lastShownSec = 0;
        static unsigned long last10sBeepSec  = 0;
        static unsigned long last30sMelodySec = 0;

        if (nowSquatting && !wasSquatting) {
            squatActive    = true;
            squatStartTime = millis();
            squatElapsed   = 0;
            lastShownSec   = 0;
            last10sBeepSec = 0;
            last30sMelodySec = 0;

            lv_async_call([](void*) {
                char *txt = (char*)malloc(32);
                if (!txt) return;
                sprintf(txt, "0s / %ds", targetSquat);
                lv_label_set_text(squat_label, txt);
                lv_obj_set_style_text_font(squat_label, &lv_font_montserrat_32, 0);
                lv_obj_set_style_text_color(squat_label, lv_color_hex(0xFFFFFF), 0);
                free(txt);
            }, NULL);
        }

        if (nowSquatting && squatActive) {
            unsigned long elapsed = (millis() - squatStartTime) / 1000;
            squatElapsed = elapsed;

            if (elapsed != lastShownSec) {
                lastShownSec = elapsed;

                lv_async_call([](void* param) {
                    char *txt = (char*)param;
                    lv_label_set_text(squat_label, txt);
                    free(txt);
                }, []() -> void* {
                    char *buf = (char*)malloc(32);
                    if (!buf) return nullptr;
                    sprintf(buf, "%lus / %ds", squatElapsed, targetSquat);
                    return buf;
                }());
            }

            if (elapsed > 0 &&
                (elapsed % 10 == 0) &&
                (elapsed % 30 != 0) &&
                (elapsed != last10sBeepSec)) {

                last10sBeepSec = elapsed;
                tone(BUZZER_PIN, 880, 200);
                Serial.println("🔔 Beep (10s milestone)");
            }

            if (elapsed > 0 &&
                (elapsed % 30 == 0) &&
                (elapsed != last30sMelodySec)) {

                last30sMelodySec = elapsed;
                Serial.println("🎵 Playing melody (30s milestone)");
                for (int i = 0; i < 8; i++) {
                    tone(BUZZER_PIN, melody[i], duration);
                    delay(duration * 1.3);
                }
                noTone(BUZZER_PIN);
            }
        }

        if (!nowSquatting && wasSquatting && squatActive) {

            squatActive  = false;
            noTone(BUZZER_PIN);

            squatElapsed = (millis() - squatStartTime) / 1000;

            // UI：Finished Xs / targetSquat
            lv_async_call([](void* param) {
                char *txt = (char*)param;
                lv_label_set_text(squat_label, txt);
                lv_obj_set_style_text_font(squat_label, &lv_font_montserrat_28, 0);
                lv_obj_set_style_text_color(squat_label, lv_color_hex(0x00FFFF), 0);
                free(txt);
            }, []() -> void* {
                char *buf = (char*)malloc(40);
                if (!buf) return nullptr;
                sprintf(buf, "Finished %lus / %ds", squatElapsed, targetSquat);
                return buf;
            }());

            Serial.printf("✅ Local squat finished: %lus / %ds\n", squatElapsed, targetSquat);
        }

        if (!nowSquatting && !wasSquatting && !squatActive) {
        }

        wasSquatting = nowSquatting;

    }, 100, NULL);

    // Show screen
    lv_scr_load(squat_screen);
}

void show_squat_screen() {

    Serial.println("📺 Showing Local Squat Screen");

    // =========================
    // UI
    // =========================
    squat_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(squat_screen, lv_color_hex(0x000000), LV_PART_MAIN);

    lv_obj_t *title = lv_label_create(squat_screen);
    lv_label_set_text(title, "Squat Timer");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    squat_label = lv_label_create(squat_screen);
    lv_label_set_text(squat_label, "Squat and hold");
    lv_obj_set_style_text_font(squat_label, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(squat_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(squat_label, LV_ALIGN_CENTER, 0, 0);

    // End button
    lv_obj_t *back_btn = lv_btn_create(squat_screen);
    lv_obj_set_size(back_btn, 150, 60);
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x555555), LV_PART_MAIN);
    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "End");
    lv_obj_set_style_text_font(back_label, &lv_font_montserrat_28, 0);
    lv_obj_center(back_label);

    // =========================
    // Session state (THIS PAGE ONLY)
    // =========================
    squatMeasuring = true;
    squatActive    = false;
    squatElapsed   = 0;
    squatStartTime = 0;

    static bool     sessionStartedOnce = false;
    static bool     inSquat            = false;
    static uint32_t belowSince         = 0;
    static uint32_t aboveSince         = 0;
    static uint32_t lastShownSec       = 0;
    static uint32_t last10sBeepSec     = 0;
    static uint32_t last30sMelodySec   = 0;

    sessionStartedOnce = false;
    inSquat            = false;
    belowSince         = 0;
    aboveSince         = 0;
    lastShownSec       = 0;
    last10sBeepSec     = 0;
    last30sMelodySec   = 0;

    // End Button events
    lv_obj_add_event_cb(back_btn, [](lv_event_t *e) {

        Serial.println("🔙 Exit Squat Screen");

        squatMeasuring = false;
        squatActive    = false;
        noTone(BUZZER_PIN);

        if (squatTimer) {
            lv_timer_del(squatTimer);
            squatTimer = nullptr;
        }

        if (squatElapsed > 0) {
            sendResultToHub("Squat", squatElapsed, "s");
        }

        lv_scr_load(main_screen);

    }, LV_EVENT_CLICKED, NULL);

    // =========================
    // Logic timer (100ms sampling)
    // =========================
    if (squatTimer) {
        lv_timer_del(squatTimer);
        squatTimer = nullptr;
    }

    squatTimer = lv_timer_create([](lv_timer_t *t) {

        if (!squatMeasuring) return;

        float d = readUltrasonicCM();
        if (d < 0) return;

        bool nowBelow = (d < squatThresholdCM);
        uint32_t nowMs = millis();

        if (sessionStartedOnce && !inSquat) {
            // "Finished. Press End to exit"
            return;
        }

        if (!sessionStartedOnce && !inSquat) {

            if (nowBelow) {
                if (belowSince == 0) belowSince = nowMs;

                if (nowMs - belowSince >= 1000) {
                    sessionStartedOnce = true;
                    inSquat            = true;
                    squatActive        = true;

                    squatStartTime = nowMs;
                    squatElapsed   = 0;

                    lastShownSec     = 0;
                    last10sBeepSec   = 0;
                    last30sMelodySec = 0;

                    lv_async_call([](void*) {
                        char buf[32];
                        sprintf(buf, "0s / %ds", targetSquat);
                        lv_label_set_text(squat_label, buf);
                        lv_obj_set_style_text_font(squat_label, &lv_font_montserrat_32, 0);
                        lv_obj_set_style_text_color(squat_label, lv_color_hex(0xFFFFFF), 0);
                    }, NULL);

                    Serial.println("✅ Squat started (stable 1s)");
                }
            } else {
                belowSince = 0;
            }

            return;
        }

        if (inSquat) {

            if (nowBelow) {
                aboveSince = 0;

                uint32_t elapsedSec = (nowMs - squatStartTime) / 1000;
                squatElapsed = elapsedSec;

                if (elapsedSec != lastShownSec) {
                    lastShownSec = elapsedSec;

                    lv_async_call([](void* param) {
                        char *txt = (char*)param;
                        lv_label_set_text(squat_label, txt);
                        free(txt);
                    }, []() -> void* {
                        char *buf = (char*)malloc(32);
                        if (!buf) return nullptr;
                        sprintf(buf, "%lus / %ds", squatElapsed, targetSquat);
                        return buf;
                    }());
                }

                if (elapsedSec > 0 &&
                    (elapsedSec % 10 == 0) &&
                    (elapsedSec % 30 != 0) &&
                    (elapsedSec != last10sBeepSec)) {

                    last10sBeepSec = elapsedSec;
                    tone(BUZZER_PIN, 880, 200);
                    Serial.println("🔔 Beep (10s milestone)");
                }

                if (elapsedSec > 0 &&
                    (elapsedSec % 30 == 0) &&
                    (elapsedSec != last30sMelodySec)) {

                    last30sMelodySec = elapsedSec;
                    Serial.println("🎵 Playing melody (30s milestone)");

                    for (int i = 0; i < 8; i++) {
                        tone(BUZZER_PIN, melody[i], duration);
                        delay(duration * 1.3);
                    }
                    noTone(BUZZER_PIN);
                }

            } else {
                if (aboveSince == 0) aboveSince = nowMs;

                if (nowMs - aboveSince >= 300) {
                    inSquat     = false;
                    squatActive = false;
                    noTone(BUZZER_PIN);

                    squatElapsed = (nowMs - squatStartTime) / 1000;

                    // UI：Finished
                    lv_async_call([](void* param) {
                        char *txt = (char*)param;
                        lv_label_set_text(squat_label, txt);
                        lv_obj_set_style_text_font(squat_label, &lv_font_montserrat_28, 0);
                        lv_obj_set_style_text_color(squat_label, lv_color_hex(0x00FFFF), 0);
                        free(txt);
                    }, []() -> void* {
                        char *buf = (char*)malloc(48);
                        if (!buf) return nullptr;
                        sprintf(buf, "Finished %lus / %ds", squatElapsed, targetSquat);
                        return buf;
                    }());

                    Serial.printf("✅ Squat finished: %lus\n", squatElapsed);

                }
            }
        }

    }, 100, NULL);

    // =========================
    // Show page
    // =========================
    lv_scr_load(squat_screen);
}


void show_sit_screen() {

    lv_obj_t* sit_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(sit_screen, lv_color_hex(0x101010), LV_PART_MAIN);

    // Title
    lv_obj_t* title = lv_label_create(sit_screen);
    lv_label_set_text(title, "Sit-to-Stand");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 50);

    // Main status text
    sit_label = lv_label_create(sit_screen);
    char buf[32];
    sprintf(buf, "Sit and stand: 0 / %d", targetSit);
    lv_label_set_text(sit_label, buf);
    lv_obj_set_style_text_font(sit_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(sit_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(sit_label, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t* back_btn = lv_btn_create(sit_screen);
    lv_obj_set_size(back_btn, 140, 60);
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_text_font(back_btn, &lv_font_montserrat_28, 0);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_t* back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "End");
    lv_obj_center(back_label);

    lv_obj_add_event_cb(back_btn, [](lv_event_t *e) {
        Serial.println("🔙 Exit Sit Screen");
        sendResultToHub("Sit", sitCount, "");

        if (sitClient && sitClient->isConnected()) sitClient->disconnect();
        sitConnected = false;
        sitCount = 0;
        lv_scr_load(main_screen);
    }, LV_EVENT_CLICKED, NULL);

    // Show screen
    lv_scr_load(sit_screen);

    // Start connection task without blocking the UI
}


unsigned long balanceStartTime = 0;
lv_obj_t* balance_time_label = nullptr;
lv_obj_t* balance_state_label = nullptr;
lv_timer_t* balance_timer = nullptr;

void show_balance_screen() {
    Serial.println("🧍 Entering Balance Screen...");
    balanceStartTime = millis();

    stableCount = 20;
    isBalanced = true;
    avgMag = lastMagnitude;

    lv_obj_t* balance_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(balance_screen, lv_color_hex(0x101010), LV_PART_MAIN);

    lv_obj_t* title = lv_label_create(balance_screen);
    lv_label_set_text(title, "Balancing");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // Time label
    balance_time_label = lv_label_create(balance_screen);
    char buf[32];
    sprintf(buf, "Time: 0 / %ds", targetBalance);
    lv_label_set_text(balance_time_label, buf);
    lv_obj_set_style_text_font(balance_time_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(balance_time_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(balance_time_label, LV_ALIGN_CENTER, 0, -20);

    // Balance status label
    balance_state_label = lv_label_create(balance_screen);
    lv_label_set_text(balance_state_label, "State: Checking...");
    lv_obj_set_style_text_font(balance_state_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(balance_state_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(balance_state_label, LV_ALIGN_CENTER, 0, 30);

    // Back button
    lv_obj_t* back_btn = lv_btn_create(balance_screen);
    lv_obj_set_size(back_btn, 140, 60);
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_text_font(back_btn, &lv_font_montserrat_28, 0);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_t* back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "End");
    lv_obj_center(back_label);

    lv_scr_load(balance_screen);

    balance_timer = lv_timer_create([](lv_timer_t *t) {
        unsigned long elapsed = (millis() - balanceStartTime) / 1000;

        // Update time
        char timeBuf[32];
        sprintf(timeBuf, "Time: %lu / %ds", elapsed, targetBalance);
        lv_label_set_text(balance_time_label, timeBuf);

        // Update balance state
        if (isBalanced) {
            lv_label_set_text(balance_state_label, "State: Balanced");
            lv_obj_set_style_text_color(balance_state_label, lv_color_hex(0xFFFFFF), 0);
        } else {
            lv_label_set_text(balance_state_label, "State: Unstable");
            lv_obj_set_style_text_color(balance_state_label, lv_color_hex(0xFF0000), 0);
        }
    }, 500, NULL);

    // Back Button events
    lv_obj_add_event_cb(back_btn, [](lv_event_t *e) {
        Serial.println("🔙 Exit Balance Screen");

        if (balance_timer) {
            lv_timer_del(balance_timer);
            balance_timer = nullptr;
        }

        unsigned long totalTime = (millis() - balanceStartTime) / 1000;
        Serial.printf("🧍 Total balance session time: %lus\n", totalTime);
        sendResultToHub("Balance", totalTime, "s");

        lv_scr_load(main_screen);
    }, LV_EVENT_CLICKED, NULL);
}


void show_march_screen() {
    Serial.println("🚶 Entering March Step Screen...");

    if (marchTimer != NULL) {
      lv_timer_del(marchTimer);
      marchTimer = NULL;
    }

    imu_write(FUNC_CFG_ACCESS, 0x80);
    imu_write(0x64, 0x80);             // reset pedometer counter
    delay(10);
    imu_write(FUNC_CFG_ACCESS, 0x00);
    stepCountHW = 0;
    Serial.println("🔄 Step counter reset to 0");

    lv_obj_t* march_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(march_screen, lv_color_hex(0x101010), LV_PART_MAIN);

    lv_obj_t* title = lv_label_create(march_screen);
    lv_label_set_text(title, "March Steps");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    lv_obj_t* step_label = lv_label_create(march_screen);
    char buf[32];
    sprintf(buf, "Steps: 0 / %d", targetMarch);
    lv_label_set_text(step_label, buf);
    lv_obj_set_style_text_font(step_label, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(step_label, lv_color_hex(0x00FFFF), 0);
    lv_obj_align(step_label, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t* back_btn = lv_btn_create(march_screen);
    lv_obj_set_size(back_btn, 140, 60);
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_text_font(back_btn, &lv_font_montserrat_28, 0);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_t* back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "End");
    lv_obj_center(back_label);

    lv_obj_add_event_cb(back_btn, [](lv_event_t *e) {
        Serial.println("🔙 Exit March Screen");
        sendResultToHub("March", stepCountHW, "");
        lv_scr_load(main_screen);
        MODE_MARCH = false;

        if (marchTimer != NULL) {
        lv_timer_del(marchTimer);
        marchTimer = NULL;
        }
    }, LV_EVENT_CLICKED, NULL);

    lv_scr_load(march_screen);


    // Update step count every 0.5 seconds and provide audio feedback
    marchTimer = lv_timer_create([](lv_timer_t *timer) {
        static uint16_t lastStepShown = 0;
        static uint16_t lastMelodyStep = 0;

        if (stepCountHW != lastStepShown) {
            char buf[32];
            sprintf(buf, "Steps: %d / %d", stepCountHW, targetMarch);
            lv_label_set_text((lv_obj_t*)timer->user_data, buf);
            lastStepShown = stepCountHW;
        }

        // === Play melody when the target step count is reached ===
        if (stepCountHW == targetMarch && stepCountHW != lastMelodyStep && MODE_MARCH) {
            Serial.println("🎵 March milestone reached - playing melody");
            lastMelodyStep = stepCountHW;

            for (int i = 0; i < 8; i++) {
                tone(BUZZER_PIN, melody[i], duration);
                delay(duration * 1.3);
            }
            noTone(BUZZER_PIN);
        }

    }, 500, step_label);
}

void show_pose_screen_org() {
    Serial.println("🧘 Entering Pose Challenge Screen...");

    generate_pose_sequence();
    currentPoseIndex = 0;
    countdownValue = 5;
    poseActive = true;

    pose_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(pose_screen, lv_color_hex(0x101010), LV_PART_MAIN);

    lv_obj_t* title = lv_label_create(pose_screen);
    lv_label_set_text(title, "Pose Challenge");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 25);

    pose_label = lv_label_create(pose_screen);
    lv_label_set_text_fmt(pose_label, "Do %s", poseList[generatedPoses[currentPoseIndex]]);
    lv_obj_set_style_text_font(pose_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(pose_label, lv_color_hex(0x00FFFF), 0);
    lv_obj_align(pose_label, LV_ALIGN_CENTER, 0, -20);

    countdown_label = lv_label_create(pose_screen);
    lv_label_set_text(countdown_label, "5");
    lv_obj_set_style_text_font(countdown_label, &lv_font_montserrat_40, 0);
    lv_obj_set_style_text_color(countdown_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, 40);

    // Back button
    lv_obj_t* back_btn = lv_btn_create(pose_screen);
    lv_obj_set_size(back_btn, 140, 60);
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_text_font(back_btn, &lv_font_montserrat_28, 0);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_t* back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "End");
    lv_obj_center(back_label);
    lv_obj_add_event_cb(back_btn, [](lv_event_t *e) {
        Serial.println("🔙 Exit Pose Challenge");
        if (pose_timer) {
            lv_timer_del(pose_timer);
            pose_timer = nullptr;
        }
        lv_scr_load(main_screen);
    }, LV_EVENT_CLICKED, NULL);

    lv_scr_load(pose_screen);

    pose_timer = lv_timer_create([](lv_timer_t* timer) {
        if (!poseActive) return;

        countdownValue--;
        if (countdownValue >= 0) {
            char buf[8];
            sprintf(buf, "%d", countdownValue);
            lv_label_set_text(countdown_label, buf);
        }

        if (countdownValue <= 0) {
            uint8_t val = imu_read(D6D_SRC);
            uint8_t current = val & 0x3F;

            int detectedPose = -1;
            if (current & 0x20) detectedPose = 1;  // Up
            else if (current & 0x10) detectedPose = 0;  // Down
            else if ((current & 0x08) || (current & 0x04)) detectedPose = 2;  // Side
            else if (current & 0x02) detectedPose = 3;  // Tail
            else if (current & 0x01) detectedPose = 4;  // Head

            bool correct = (detectedPose == generatedPoses[currentPoseIndex]);

            if (correct) {
                play_success_beep();
                currentPoseIndex++;
                if (currentPoseIndex >= totalPoses) {
                    lv_timer_del(pose_timer);
                    pose_timer = nullptr;
                    poseActive = false;
                    show_pose_congratulations();
                    return;
                } else {
                    lv_label_set_text_fmt(pose_label, "Do %s", poseList[generatedPoses[currentPoseIndex]]);
                    countdownValue = 5;
                }
            } else {
                play_error_beep();
                lv_label_set_text_fmt(pose_label, "Retry %s", poseList[generatedPoses[currentPoseIndex]]);
                countdownValue = 5;
            }
        }
    }, 1000, NULL);
}

void show_pose_screen() {
  Serial.println("🧘 Entering Pose Challenge Screen (no 6D pose check)...");

  // Generate instruction sequence
  generate_pose_sequence();
  currentPoseIndex = 0;

  // Initialize state
  poseActive = true;
  posePhase = POSE_PREP;
  prepCountdown = 5;
  holdCountdown = 5;

  // Create a new screen
  pose_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(pose_screen, lv_color_hex(0x101010), LV_PART_MAIN);

  lv_obj_t* title = lv_label_create(pose_screen);
  lv_label_set_text(title, "Pose Challenge");
  lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 25);

  // Instruction label
  pose_label = lv_label_create(pose_screen);
  lv_label_set_text_fmt(pose_label, "Do %s", poseList[generatedPoses[currentPoseIndex]]);
  lv_obj_set_style_text_font(pose_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(pose_label, lv_color_hex(0x00FFFF), 0);
  lv_obj_align(pose_label, LV_ALIGN_CENTER, 0, -30);

  // Countdown label
  countdown_label = lv_label_create(pose_screen);
  lv_label_set_text(countdown_label, "5");
  lv_obj_set_style_text_font(countdown_label, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(countdown_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, 30);

  // Status hint label
  lv_obj_t* hint_label = lv_label_create(pose_screen);
  lv_label_set_text(hint_label, "Get ready...");
  lv_obj_set_style_text_font(hint_label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(hint_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(hint_label, LV_ALIGN_BOTTOM_MID, 0, -90);

  // End button
  lv_obj_t* back_btn = lv_btn_create(pose_screen);
  lv_obj_set_size(back_btn, 140, 60);
  lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x666666), LV_PART_MAIN);
  lv_obj_t* back_label = lv_label_create(back_btn);
  lv_label_set_text(back_label, "End");
  lv_obj_set_style_text_font(back_label, &lv_font_montserrat_28, 0);
  lv_obj_center(back_label);

  lv_obj_add_event_cb(back_btn, [](lv_event_t *e) {
    Serial.println("🔙 Exit Pose Challenge");
    poseActive = false;
    if (pose_timer) { lv_timer_del(pose_timer); pose_timer = nullptr; }
    sendResultToHub("Pose", currentPoseIndex, "");
    noTone(BUZZER_PIN);
    lv_scr_load(main_screen);
  }, LV_EVENT_CLICKED, NULL);

  lv_scr_load(pose_screen);

  // Prevent duplicate timer creation
  if (pose_timer) { lv_timer_del(pose_timer); pose_timer = nullptr; }

  // Run the state machine once per second
  pose_timer = lv_timer_create([](lv_timer_t* t) {
    if (!poseActive) return;

    // Read the current stability state maintained by update_imu()
    bool balancedNow = isBalanced;

    // Shared beep helper for relevant phases
    auto beep_tick = []() {
      tone(BUZZER_PIN, 880, 60);
    };

    // ========== Phase 1: PREP 5s ==========
    if (posePhase == POSE_PREP) {
      lv_label_set_text((lv_obj_t*)t->user_data, "Get ready...");
      lv_label_set_text(countdown_label, String(prepCountdown).c_str());

      prepCountdown--;
      if (prepCountdown < 0) {
        posePhase = POSE_WAIT_STABLE;
        holdCountdown = 5;
        lv_label_set_text((lv_obj_t*)t->user_data, "Hold still until stable");
        lv_label_set_text(countdown_label, "--");
      }
      return;
    }

    // ========== Phase 2: WAIT_STABLE ==========
    if (posePhase == POSE_WAIT_STABLE) {
      if (balancedNow) {
        posePhase = POSE_HOLD;
        holdCountdown = 5;
        lv_label_set_text((lv_obj_t*)t->user_data, "Stable, keep holding");
      } else {
        lv_label_set_text((lv_obj_t*)t->user_data, "Waiting for stable...");
        lv_label_set_text(countdown_label, "--");
      }
      return;
    }

    // ========== Phase 3: HOLD 5s ==========
    if (posePhase == POSE_HOLD) {
      // Fail immediately if stability is lost
      if (!balancedNow) {
        lv_label_set_text((lv_obj_t*)t->user_data, "Unstable, try again");
        lv_obj_set_style_text_color(countdown_label, lv_color_hex(0xFF0000), 0);
        play_fail_melody();

        // Return to WAIT_STABLE after failure without another preparation period
        lv_obj_set_style_text_color(countdown_label, lv_color_hex(0xFFFFFF), 0);
        posePhase = POSE_WAIT_STABLE;
        lv_label_set_text(countdown_label, "--");
        return;
      }

      // Continue countdown while stable
      lv_label_set_text(countdown_label, String(holdCountdown).c_str());
      beep_tick();

      holdCountdown--;
      if (holdCountdown < 0) {
        play_congrats_melody();

        posePhase = POSE_REST;
        restCountdown = 2;

        lv_label_set_text((lv_obj_t*)t->user_data, "Nice! Take a breath");
        lv_label_set_text(countdown_label, "✓");

        return;
        }


    //     currentPoseIndex++;
    //       return;

      return;
    }

    // ========== Phase 4: REST between poses ==========
    if (posePhase == POSE_REST) {
    restCountdown--;

    if (restCountdown <= 0) {

        currentPoseIndex++;

        if (currentPoseIndex >= totalPoses) {
        poseActive = false;
        lv_timer_del(pose_timer);
        pose_timer = nullptr;
        show_pose_congratulations();
        return;
        }

        // Next pose
        lv_label_set_text_fmt(
        pose_label,
        "Do %s",
        poseList[generatedPoses[currentPoseIndex]]
        );

        posePhase = POSE_PREP;
        prepCountdown = 5;
        holdCountdown = 5;

        lv_label_set_text((lv_obj_t*)t->user_data, "Get ready...");
        lv_label_set_text(countdown_label, "5");
    }

    return;
    }


  }, 1000, hint_label);  // user_data = hint_label
}

void show_pose_screen_org_v2() {
  Serial.println("🧘 Entering Pose Challenge Screen (no 6D pose check)...");

  // Generate instruction sequence
  generate_pose_sequence();
  currentPoseIndex = 0;

  // Initialize state
  poseActive = true;
  posePhase = POSE_PREP;
  prepCountdown = 5;
  holdCountdown = 5;

  // Create a new screen
  pose_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(pose_screen, lv_color_hex(0x101010), LV_PART_MAIN);

  lv_obj_t* title = lv_label_create(pose_screen);
  lv_label_set_text(title, "Pose Challenge");
  lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 25);

  // Instruction label
  pose_label = lv_label_create(pose_screen);
  lv_label_set_text_fmt(pose_label, "Do %s", poseList[generatedPoses[currentPoseIndex]]);
  lv_obj_set_style_text_font(pose_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(pose_label, lv_color_hex(0x00FFFF), 0);
  lv_obj_align(pose_label, LV_ALIGN_CENTER, 0, -30);

  // Countdown label
  countdown_label = lv_label_create(pose_screen);
  lv_label_set_text(countdown_label, "5");
  lv_obj_set_style_text_font(countdown_label, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(countdown_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, 30);

  // Status hint label
  lv_obj_t* hint_label = lv_label_create(pose_screen);
  lv_label_set_text(hint_label, "Get ready...");
  lv_obj_set_style_text_font(hint_label, &lv_font_montserrat_22, 0);
  lv_obj_set_style_text_color(hint_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(hint_label, LV_ALIGN_BOTTOM_MID, 0, -90);

  // End button
  lv_obj_t* back_btn = lv_btn_create(pose_screen);
  lv_obj_set_size(back_btn, 140, 60);
  lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x666666), LV_PART_MAIN);
  lv_obj_t* back_label = lv_label_create(back_btn);
  lv_label_set_text(back_label, "End");
  lv_obj_set_style_text_font(back_label, &lv_font_montserrat_28, 0);
  lv_obj_center(back_label);

  lv_obj_add_event_cb(back_btn, [](lv_event_t *e) {
    Serial.println("🔙 Exit Pose Challenge");
    poseActive = false;
    if (pose_timer) { lv_timer_del(pose_timer); pose_timer = nullptr; }
    noTone(BUZZER_PIN);
    lv_scr_load(main_screen);
  }, LV_EVENT_CLICKED, NULL);

  lv_scr_load(pose_screen);

  // Prevent duplicate timer creation
  if (pose_timer) { lv_timer_del(pose_timer); pose_timer = nullptr; }

  // Run the state machine once per second
  pose_timer = lv_timer_create([](lv_timer_t* t) {
    if (!poseActive) return;

    // Read the current stability state maintained by update_imu()
    bool balancedNow = isBalanced;

    // Shared beep helper for relevant phases
    auto beep_tick = []() {
      tone(BUZZER_PIN, 880, 60);
      delay(10);
      noTone(BUZZER_PIN);
    };

    // ========== Phase 1: PREP 5s ==========
    if (posePhase == POSE_PREP) {
      lv_label_set_text((lv_obj_t*)t->user_data, "Get ready...");
      lv_label_set_text(countdown_label, String(prepCountdown).c_str());
      beep_tick();

      prepCountdown--;
      if (prepCountdown < 0) {
        posePhase = POSE_WAIT_STABLE;
        holdCountdown = 5;
        lv_label_set_text((lv_obj_t*)t->user_data, "Hold still until stable");
        lv_label_set_text(countdown_label, "--");
      }
      return;
    }

    // ========== Phase 2: WAIT_STABLE ==========
    if (posePhase == POSE_WAIT_STABLE) {
      if (balancedNow) {
        posePhase = POSE_HOLD;
        holdCountdown = 5;
        lv_label_set_text((lv_obj_t*)t->user_data, "Stable, keep holding");
        lv_label_set_text(countdown_label, "5");
        beep_tick();
      } else {
        lv_label_set_text((lv_obj_t*)t->user_data, "Waiting for stable...");
        lv_label_set_text(countdown_label, "--");
      }
      return;
    }

    // ========== Phase 3: HOLD 5s ==========
    if (posePhase == POSE_HOLD) {
      // Fail immediately if stability is lost
      if (!balancedNow) {
        lv_label_set_text((lv_obj_t*)t->user_data, "Unstable, try again");
        lv_obj_set_style_text_color(countdown_label, lv_color_hex(0xFF0000), 0);
        play_fail_melody();

        // Return to WAIT_STABLE after failure without another preparation period
        lv_obj_set_style_text_color(countdown_label, lv_color_hex(0xFFFFFF), 0);
        posePhase = POSE_WAIT_STABLE;
        lv_label_set_text(countdown_label, "--");
        return;
      }

      // Continue countdown while stable
      lv_label_set_text(countdown_label, String(holdCountdown).c_str());
      beep_tick();

      holdCountdown--;
      if (holdCountdown < 0) {
        play_congrats_melody();

        currentPoseIndex++;
        if (currentPoseIndex >= totalPoses) {
          poseActive = false;
          lv_timer_del(pose_timer);
          pose_timer = nullptr;
          show_pose_congratulations();
          return;
        }

        lv_label_set_text_fmt(pose_label, "Do %s", poseList[generatedPoses[currentPoseIndex]]);
        posePhase = POSE_PREP;
        prepCountdown = 5;
        holdCountdown = 5;
        lv_label_set_text((lv_obj_t*)t->user_data, "Get ready...");
        lv_label_set_text(countdown_label, "5");
      }
      return;
    }

  }, 1000, hint_label);  // user_data = hint_label
}


void show_pose_congratulations() {
    Serial.println("🎉 Pose Challenge Completed!");

    lv_obj_clean(lv_scr_act());
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x203020), LV_PART_MAIN);

    lv_obj_t* congrats_label = lv_label_create(lv_scr_act());
    lv_label_set_text(congrats_label, "Congrats!");
    lv_obj_set_style_text_font(congrats_label, &lv_font_montserrat_40, 0);
    lv_obj_set_style_text_color(congrats_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(congrats_label, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t* again_btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(again_btn, 140, 60);
    lv_obj_align(again_btn, LV_ALIGN_CENTER, -80, 50);
    lv_obj_set_style_text_font(again_btn, &lv_font_montserrat_28, 0);
    lv_obj_set_style_bg_color(again_btn, lv_color_hex(0x00AA00), LV_PART_MAIN);
    lv_obj_t* again_label = lv_label_create(again_btn);
    lv_label_set_text(again_label, "Again");
    lv_obj_center(again_label);
    lv_obj_add_event_cb(again_btn, [](lv_event_t *e) {
        Serial.println("🔁 Restart Pose Challenge");
        show_pose_screen();
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t* back_btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(back_btn, 140, 60);
    lv_obj_align(back_btn, LV_ALIGN_CENTER, 80, 50);
    lv_obj_set_style_text_font(back_btn, &lv_font_montserrat_28, 0);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_t* back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "Back");
    lv_obj_center(back_label);
    lv_obj_add_event_cb(back_btn, [](lv_event_t *e) {
        Serial.println("🔙 Return to main screen");
        lv_scr_load(main_screen);
    }, LV_EVENT_CLICKED, NULL);

    sendResultToHub("Pose", totalPoses, "");

}


void show_motion_dialog() {

    if (dialogVisible || currentGameState != WAITING) {
        Serial.println("⚠️ Already in dialog or game, ignore motion");
        return;
    }

    if (!dialog_screen) {
        Serial.println("⚠️ dialog_screen is null, recreating...");
        create_dialog_ui();
    }

    Serial.println("💬 Showing motion dialog");

    dialogStartTime = millis();
    dialogVisible = true;
    currentGameState = DIALOG;

    lv_async_call([](void*) {
        lv_scr_load(dialog_screen);
        Serial.println("✅ Dialog screen loaded via async");
    }, NULL);
}


void show_motion_dialog2() {
    Serial.println("💬 Showing motion dialog");
    if (example_lvgl_lock(100)) {
        lv_scr_load(dialog_screen);
        example_lvgl_unlock();
        dialogStartTime = millis();
        dialogVisible = true;
        currentGameState = DIALOG;
    }
}

void hide_dialog() {
    Serial.println("💬 Hiding dialog, returning to main screen");
    if (example_lvgl_lock(100)) {
        if (main_screen) {
            lv_scr_load_anim(main_screen, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);
        } else {
            Serial.println("❌ main_screen is null!");
        }
        example_lvgl_unlock();
        dialogVisible = false;
        currentGameState = WAITING;
    }

    dialogVisible = false;
    dialogStartTime = 0;
    currentGameState = WAITING;
    motionDetected = false;

    lv_async_call([](void*){
    lv_scr_load(main_screen);
    Serial.println("✅ UI switched back to main screen via async call");
    }, NULL);

    motionDetected = false;

    if (deviceConnected && pCharacteristic) {
        pCharacteristic->setValue("timeout");
        pCharacteristic->notify();
        Serial.println("📡 Notified Arduino: timeout");
    }
}

void start_game() {
    Serial.println("🎮 Starting NFC Exercise Game!");

    generate_non_repeating_sequence(gameSequence);
    currentStep = 0;
    currentGameState = PLAYING;
    dialogVisible = false;

    Serial.print("🎲 Generated sequence: ");
    for (int i = 0; i < 4; i++) {
        Serial.print(gameSequence[i]);
        Serial.print(" ");
    }
    Serial.println();

    show_game_screen();

    if (deviceConnected && pCharacteristic) {
        pCharacteristic->setValue("game_started");
        pCharacteristic->notify();
        Serial.println("📡 Notified Arduino: game_started");
    }
}

void show_game_screen() {
    Serial.println("🎮 Switching to game screen");
    if (example_lvgl_lock(100)) {
        lv_scr_load(game_screen);
        update_sequence_display();
        example_lvgl_unlock();
        Serial.println("✅ Game screen loaded");
    } else {
        Serial.println("❌ Failed to lock LVGL for game screen");
    }
}

void update_sequence_display_direct() {
    char buf[8];

    Serial.printf("🔄 Updating sequence display - current step: %d\n", currentStep);

    for (int i = 0; i < 4; i++) {
        sprintf(buf, "%d", gameSequence[i]);
        lv_label_set_text(sequence_labels[i], buf);

        if (i < currentStep) {
            lv_obj_set_style_text_color(sequence_labels[i], lv_color_hex(0x00FF00), 0);
            Serial.printf("  Step %d: COMPLETED (green)\n", i);
        } else if (i == currentStep) {
            lv_obj_set_style_text_color(sequence_labels[i], lv_color_hex(0xFFFF00), 0);
            Serial.printf("  Step %d: CURRENT (yellow)\n", i);
        } else {
            lv_obj_set_style_text_color(sequence_labels[i], lv_color_hex(0xFFFFFF), 0);
            Serial.printf("  Step %d: PENDING (white)\n", i);
        }
    }

    if (currentStep < 4) {
        char progress_text[64];
        sprintf(progress_text, "Tag card %d", gameSequence[currentStep]);
        lv_label_set_text(progress_label, progress_text);
        Serial.printf("📝 Progress text: %s\n", progress_text);
    }
}

void update_sequence_display() {
    char buf[8];

    Serial.printf("🔄 Updating sequence display - current step: %d\n", currentStep);

    for (int i = 0; i < 4; i++) {
        sprintf(buf, "%d", gameSequence[i]);
        lv_label_set_text(sequence_labels[i], buf);

        if (i < currentStep) {
            lv_obj_set_style_text_color(sequence_labels[i], lv_color_hex(0x00FF00), 0);
            Serial.printf("  Step %d: COMPLETED (green)\n", i);
        } else if (i == currentStep) {
            lv_obj_set_style_text_color(sequence_labels[i], lv_color_hex(0xFFFF00), 0);
            Serial.printf("  Step %d: CURRENT (yellow)\n", i);
        } else {
            lv_obj_set_style_text_color(sequence_labels[i], lv_color_hex(0xFFFFFF), 0);
            Serial.printf("  Step %d: PENDING (white)\n", i);
        }
    }

    if (currentStep < 4) {
        char progress_text[64];
        sprintf(progress_text, "Tag card %d", gameSequence[currentStep]);
        lv_label_set_text(progress_label, progress_text);
        Serial.printf("📝 Progress text: %s\n", progress_text);
    }
}

void flash_red_sequence(int index) {
    if (index < 0 || index >= 4) return;

    Serial.printf("❌ Wrong card! Flashing red for index %d\n", index);

    if (example_lvgl_lock(100)) {
        lv_obj_set_style_text_color(sequence_labels[index], lv_color_hex(0xFF0000), 0);
        example_lvgl_unlock();

        vTaskDelay(pdMS_TO_TICKS(500));

        if (example_lvgl_lock(100)) {
            if (index == currentStep) {
                lv_obj_set_style_text_color(sequence_labels[index], lv_color_hex(0xFFFF00), 0);
            } else {
                lv_obj_set_style_text_color(sequence_labels[index], lv_color_hex(0xFFFFFF), 0);
            }
            example_lvgl_unlock();
        }
    }
}

void show_game_cancelled_org() {
    Serial.println("❌ Game cancelled by user");

    lv_obj_clean(lv_scr_act());

    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_obj_center(label);
    lv_label_set_text(label, "Game Cancelled");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);


    currentGameState = WAITING;
    dialogVisible = false;
    motionDetected = false;
    currentStep = 0;

    lv_timer_create([](lv_timer_t *timer) {
        Serial.println("🔁 Auto return to main screen after cancel");
        lv_scr_load(main_screen);
        lv_timer_del(timer);
    }, 3000, NULL);
}

void show_game_cancelled() {
    Serial.println("❌ Game cancelled by user");

    currentGameState = WAITING;
    currentStep = 0;
    dialogVisible = false;
    motionDetected = false;

    if (deviceConnected && pCharacteristic) {
        pCharacteristic->setValue("game_ended");
        pCharacteristic->notify();
        Serial.println("📡 Notified Arduino: game_ended");
    }

    lv_scr_load(main_screen);

    create_game_ui();
}


void show_congratulations() {
    Serial.println("🎉 Game completed! Showing congratulations");

    if (example_lvgl_lock(100)) {
        lv_obj_clean(game_screen);

        lv_obj_set_style_bg_color(game_screen, lv_color_hex(0x0F3F0F), LV_PART_MAIN);

        lv_obj_t *congrats_title = lv_label_create(game_screen);
        lv_label_set_text(congrats_title, "CONGRATULATIONS!");
        lv_obj_set_style_text_font(congrats_title, &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(congrats_title, lv_color_hex(0xFFFFFF), 0);
        lv_obj_align(congrats_title, LV_ALIGN_CENTER, 0, -40);

        lv_obj_t *congrats_sub = lv_label_create(game_screen);
        lv_label_set_text(congrats_sub, "Exercise Complete!");
        lv_obj_set_style_text_color(congrats_sub, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(congrats_sub, &lv_font_montserrat_28, 0);
        lv_obj_align(congrats_sub, LV_ALIGN_CENTER, 0, 0);

        lv_obj_t *return_btn = lv_btn_create(game_screen);
        lv_obj_set_size(return_btn, 200, 70);
        lv_obj_align(return_btn, LV_ALIGN_CENTER, 0, 60);
        lv_obj_set_style_bg_color(return_btn, lv_color_hex(0x00AA00), LV_PART_MAIN);
        lv_obj_t *return_label = lv_label_create(return_btn);
        lv_label_set_text(return_label, "Return");
        lv_obj_set_style_text_font(return_label, &lv_font_montserrat_28, 0);
        lv_obj_center(return_label);

        lv_obj_add_event_cb(return_btn, [](lv_event_t *e) {
            Serial.println("🔙 Returning to main screen");
            currentGameState = WAITING;
            currentStep = 0;

            lv_scr_load(main_screen);

            if (game_screen) {
                lv_obj_del(game_screen);
                game_screen = nullptr;
            }

            create_game_ui();

        }, LV_EVENT_CLICKED, NULL);

        example_lvgl_unlock();
    }

    currentGameState = COMPLETED;


    sendResultToHub("NFC", 1, "done");

}

void reset_game_logic() {
    currentStep = 0;

    for (int i = 0; i < 4; i++) {
        lv_label_set_text(sequence_labels[i], "?");
        lv_obj_set_style_text_color(sequence_labels[i], lv_color_white(), 0);
    }

    lv_label_set_text(progress_label, "Tag card 1");
}


void end_game() {
    Serial.println("🔚 Ending game, returning to main screen");
    currentGameState = WAITING;
    currentStep = 0;

    if (example_lvgl_lock(100)) {
        if (main_screen) {
            lv_scr_load(main_screen);
        }
        example_lvgl_unlock();
    }

    if (example_lvgl_lock(100)) {
        create_game_ui();
        example_lvgl_unlock();
    }
}


void play_success_beep() { tone(BUZZER_PIN, 1000, 100); }
void play_error_beep() {
  for (int i = 0; i < 2; i++) { tone(BUZZER_PIN, 300); delay(150); noTone(BUZZER_PIN); delay(100); }
}

void init_imu() {
  Serial.println("⚙️ Initializing LSM6DSOX IMU...");
  uint8_t whoami = imu_read(WHO_AM_I);
  Serial.printf("WHO_AM_I = 0x%02X\n", whoami);
  if (whoami != 0x6C) { Serial.println("❌ IMU not detected!"); return; }

  imu_write(CTRL1_XL, 0x60);    // 416Hz ±2g
  imu_write(TAP_CFG2, 0x80);    // 6D enable
  imu_write(TAP_THS_6D, 0x60);  // 50°
  imu_write(MD1_CFG, 0x04);     // route interrupt

  imu_write(FUNC_CFG_ACCESS, 0x80);
  imu_write(EMB_FUNC_EN_A, 0x08); // pedo_en
  imu_write(0x64, 0x80);          // reset pedo
  imu_write(0x17,0x40);
  imu_write(0x02,0x11);
  imu_write(0x08,0x84);
  imu_write(0x09,0x03);
  imu_write(0x17,0x00);
  imu_write(FUNC_CFG_ACCESS, 0x00);

  Serial.println("✅ IMU initialized for 6D orientation!");
}

//     return;

void init_nfc() {
  if (!nfc) {
    Serial.println("❌ NFC object not created");
    nfcReady = false;
    return;
  }

  nfc->begin();

  uint32_t versiondata = nfc->getFirmwareVersion();
  if (!versiondata) {
    Serial.println("❌ Didn't find PN532 NFC module");
    nfcReady = false;
    return;
  }

  Serial.printf("✅ PN532 firmware: 0x%02X\n", (versiondata >> 8) & 0xFF);

  nfc->SAMConfig();
  nfcReady = true;

  Serial.println("📡 NFC module ready");
}

void check_nfc_game() {
  if (!nfcReady || currentGameState != PLAYING) return;

  uint8_t uid[7];
  uint8_t uidLength;

  bool success = nfc->readPassiveTargetID(
    PN532_MIFARE_ISO14443A,
    uid,
    &uidLength,
    50
    );
  if (!success) return;

  uint8_t cardNumber = readNFCCardValue(uid, uidLength);
  Serial.print("🎯 NFC Card detected: ");
  for (int i = 0; i < uidLength; i++) {
    if (uid[i] < 0x10) Serial.print("0");
    Serial.print(uid[i], HEX);
    Serial.print(" ");
  }
  Serial.printf(" -> Card Number: %d\n", cardNumber);

  if (cardNumber == 0) {
    Serial.println("❓ Unknown card");
    return;
  }

  if (cardNumber == gameSequence[currentStep]) {
    Serial.printf("✅ Correct! Step %d/%d\n", currentStep + 1, 4);
    currentStep++;
    play_success_beep();

    if (currentStep >= 4) {
      Serial.println("🎉 All steps completed!");
      show_congratulations();
    } else {
      if (example_lvgl_lock(100)) {
        update_sequence_display();
        example_lvgl_unlock();
      }
    }
  } else {
    Serial.printf("❌ Wrong card! Expected %d, got %d\n", gameSequence[currentStep], cardNumber);
    play_error_beep();
    flash_red_sequence(currentStep);
  }
}


void scanWiFiNetworks() {
  Serial.println("\n📡 Scanning available Wi-Fi networks...");
  int n = WiFi.scanNetworks();
  if (n == 0) Serial.println("❌ No networks found");
  else {
    Serial.printf("✅ Found %d networks:\n", n);
    for (int i = 0; i < n; ++i) {
      Serial.printf("%2d. %s  RSSI: %d dBm  %s\n",
                    i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i),
                    (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Encrypted");
    }
  }
  WiFi.scanDelete();
}


void update_imu() {
  static unsigned long lastPrint = 0;


  int16_t ax = (int16_t)(imu_read(0x29) << 8 | imu_read(0x28));
  int16_t ay = (int16_t)(imu_read(0x2B) << 8 | imu_read(0x2A));
  int16_t az = (int16_t)(imu_read(0x2D) << 8 | imu_read(0x2C));
  float magnitude = sqrt(ax * ax + ay * ay + az * az);

  if (fabs(magnitude - lastMagnitude) > 8000 && millis() - lastStepTime > 300) {
    stepCount++;
    lastStepTime = millis();
  }
  lastMagnitude = magnitude;

  imu_write(FUNC_CFG_ACCESS, 0x80);

  uint8_t stepL = imu_read(STEP_COUNTER_L);
  uint8_t stepH = imu_read(STEP_COUNTER_H);
  stepCountHW = (uint16_t)stepH << 8 | stepL;

  imu_write(FUNC_CFG_ACCESS, 0x00);

  static uint16_t lastStepCountHW = 0;
  if (stepCountHW != lastStepCountHW) {
    Serial.printf("🚶 Step count (HW): %d\n", stepCountHW);
    lastStepCountHW = stepCountHW;
  }

  avgMag = 0.9 * avgMag + 0.1 * magnitude;
  if (fabs(magnitude - avgMag) < 2000) {
    stableCount++;
  } else {
    stableCount = 0;
  }

  if (stableCount > 10 && !isBalanced) {
    isBalanced = true;
    Serial.println("🧍 Balanced state achieved");
  } else if (stableCount == 0 && isBalanced) {
    isBalanced = false;
    Serial.println("⚠️ Unstable movement detected");
  }

}

extern "C" void* malloc_psram(size_t size) {
  if (psramFound()) return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return malloc(size);
}
extern "C" void free_psram(void* ptr) { heap_caps_free(ptr); }

// ========== Arduino Setup ==========
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("BEFORE applyDeviceConfig");

    bool ok = applyDeviceConfig(DEVICE_NO);

    Serial.println("AFTER applyDeviceConfig");

  if (!applyDeviceConfig(DEVICE_NO)) {
    while (true) {
      Serial.println("❌ Device config missing. Stop.");
      delay(1000);
    }
  }

  esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
  Serial.printf("💾 PSRAM size: %d bytes\n", ESP.getPsramSize());
  Serial.printf("💾 Free PSRAM: %d bytes\n", ESP.getFreePsram());
  Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());

  BAT_Init();
  BAT_Get_Volts();


    I2C_Init(); // Wire: SDA=11, SCL=10

    PCF85063_Init();
    TCA9554PWR_Init(0x00);
    Backlight_Init();

    scanI2CBus(Wire, "I2C0 / Wire / SDA11 SCL10");

    if (USE_EXT_I2C_FOR_SENSORS) {
    WireExt.begin(I2C2_SDA, I2C2_SCL);
    WireExt.setClock(100000);

    SensorWire = &WireExt;

    char busName[64];
    sprintf(busName, "I2C1 / WireExt / SDA%d SCL%d", I2C2_SDA, I2C2_SCL);
    scanI2CBus(WireExt, busName);
    } else {
    SensorWire = &Wire;
    Serial.println("PN532 / IMU will use onboard I2C: Wire / SDA11 SCL10");
    }

    nfc = new Adafruit_PN532(-1, 255, SensorWire);

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(US_TRIG, OUTPUT);
    pinMode(US_ECHO, INPUT);
    digitalWrite(US_TRIG, LOW);


  LCD_Init();
  Lvgl_Init();

  // 3) UI
  create_main_ui();
  create_dialog_ui();
  create_game_ui();

    if (example_lvgl_lock(-1)) {

        startup_overlay = lv_obj_create(lv_scr_act());
        lv_obj_set_size(startup_overlay, LV_HOR_RES, LV_VER_RES);
        lv_obj_set_style_bg_color(startup_overlay, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(startup_overlay, LV_OPA_COVER, 0);
        lv_obj_clear_flag(startup_overlay, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* label = lv_label_create(startup_overlay);
        lv_label_set_text(label, "Initializing...");
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_28, 0);
        lv_obj_center(label);

        example_lvgl_unlock();
    }

  // 4) Wi-Fi
  scanWiFiNetworks();
  Serial.println("📡 Connecting to Hub Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(HUB_SSID, HUB_PASSWORD);
  esp_wifi_set_max_tx_power(84);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ Connected to Hub Wi-Fi, IP: %s\n", WiFi.localIP().toString().c_str());
    if (label_hub_status) { lv_label_set_text(label_hub_status, "Hub: Wi-Fi Connected"); }
    requestTargetsFromHubOnce();

  } else {
    Serial.println("\n⚠️ Failed to connect Hub Wi-Fi");
    if (label_hub_status) { lv_label_set_text(label_hub_status, "Hub: Wi-Fi Failed"); }
  }

  prefs.begin("core", true);
  targetBalance = prefs.getInt("Balance", 60);
  targetMarch   = prefs.getInt("March", 100);
  targetPose    = prefs.getInt("Pose", 5);
  targetSit     = prefs.getInt("Sit", 10);
  targetSquat   = prefs.getInt("Squat", 60);
  targetNFC     = prefs.getInt("NFC", 3);
  prefs.end();

  prefs.begin("bleStore", true);
  sitMAC = prefs.getString("sit_mac", "");
  prefs.end();
  Serial.printf("Stored Sit MAC: %s\n", sitMAC.c_str());


  server.begin();
  Serial.println("📡 Core TCP Server Started on 8888");


  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  delay(400);
  BLEDevice::init("ESP32_NFC_Game_Controller");
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("🎯 BLE Server ready!");

  // 6) NFC / IMU
  init_nfc();
  init_imu();

  xTaskCreate([](void*) {
    int retryCount = 0;
    for(;;) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("⚠️ Wi-Fi disconnected, trying to reconnect...");
        WiFi.disconnect(); WiFi.begin(HUB_SSID, HUB_PASSWORD);
        esp_wifi_set_max_tx_power(78);
        esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
        unsigned long t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 5000) { delay(500); Serial.print("."); }
        if (WiFi.status() == WL_CONNECTED) { Serial.printf("\n✅ Reconnected, IP: %s\n", WiFi.localIP().toString().c_str()); retryCount = 0;requestTargetsFromHubOnce();
 }
        else { Serial.println("\n❌ Reconnect failed"); retryCount++; if (retryCount > 5) { WiFi.mode(WIFI_OFF); delay(300); WiFi.mode(WIFI_STA); retryCount = 0; } }
      }
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }, "wifi_reconnect_task", 4096, NULL, 1, NULL);

  xTaskCreatePinnedToCore(
      DriverTask,
      "DriverTask",         
      4096,                 
      NULL,                 
      3,                    
      NULL,                 
      0
  );


  ESP_LOGI(MAIN_TAG, "Setup completed successfully");

    systemReady = true;

if (example_lvgl_lock(-1)) {
    if (startup_overlay) {
        lv_obj_del(startup_overlay);
        startup_overlay = nullptr;
    }
    example_lvgl_unlock();
}
}

// ========== Arduino Loop ==========
void loop() {
  Lvgl_Loop();
  Touch_Loop();

  checkIncomingTarget();


  static unsigned long lastNfcCheck = 0;
  unsigned long now = millis();

  if (lv_scr_act() == game_screen && currentGameState == PLAYING) {
      if (now - lastNfcCheck > 300) {
          check_nfc_game();
          lastNfcCheck = now;
      }
  }

  update_imu();
  delay(50);
}

void DriverTask(void *parameter) {
  while(1){
    BAT_Get_Volts();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

float readUltrasonicCM() {
    // trigger pulse
    digitalWrite(US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG, LOW);

    long duration = pulseIn(US_ECHO, HIGH, 20000);

    if (duration == 0) {
        Serial.println("No echo");
        return -1;
    }

    float distance = duration * 0.0343 / 2.0;

    Serial.printf("Distance: %.2f cm\n", distance);

    return distance;
}

void checkIncomingTarget() {
    WiFiClient client = server.available();
    if (!client) return;

    if (client.connected()) {
        String line = client.readStringUntil('\n');
        line.trim();

        Serial.printf("📥 Received from Hub: %s\n", line.c_str());

        if (line.startsWith("Target:")) {

            int p1 = line.indexOf(':');
            int p2 = line.indexOf(':', p1 + 1);

            String mode  = line.substring(p1 + 1, p2);
            int value    = line.substring(p2 + 1).toInt();

            String key;
            if (mode == "Balancing") key = "Balance";
            else if (mode == "March Steps") key = "March";
            else if (mode == "Pose") key = "Pose";
            else if (mode == "Sit-to-Stand") key = "Sit";
            else if (mode == "Long Squat") key = "Squat";
            else if (mode == "NFC Game") key = "NFC";


            Serial.printf("🎯 Parsed => mode=%s  value=%d\n", mode.c_str(), value);

            prefs.begin("core", false);
            prefs.putInt(key.c_str(), value);
            prefs.end();

            if (key == "Balance") targetBalance = value;
            else if (key == "March") targetMarch = value;
            else if (key == "Pose") targetPose = value;
            else if (key == "Sit") targetSit = value;
            else if (key == "Squat") targetSquat = value;
            else if (key == "NFC") targetNFC = value;

            Serial.println("💾 Saved to NVS successfully.");
        }
    }

    client.stop();
}

void play_fail_melody() {
  int bad[] = { NOTE_E5, NOTE_DS5, NOTE_D5, NOTE_CS5, NOTE_C5 };
  for (int i = 0; i < 5; i++) {
    tone(BUZZER_PIN, bad[i], 120);
    delay(160);
  }
  noTone(BUZZER_PIN);
}

void play_congrats_melody() {
  for (int i = 0; i < 8; i++) {
    tone(BUZZER_PIN, melody[i], duration);
    delay(duration * 1.3);
  }
  noTone(BUZZER_PIN);
}

#define NFC_DATA_BLOCK 4


uint8_t readNFCCardValue(uint8_t* uid, uint8_t uidLength) {
    uint8_t keya[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    uint8_t data[16];

    if (!nfc) {
        Serial.println("❌ NFC object not created");
        return 0;
    }

    if (!nfc->mifareclassic_AuthenticateBlock(
            uid, uidLength, NFC_DATA_BLOCK, 0, keya)) {
        Serial.println("❌ NFC auth failed");
        return 0;
    }

    if (!nfc->mifareclassic_ReadDataBlock(NFC_DATA_BLOCK, data)) {
        Serial.println("❌ NFC read failed");
        return 0;
    }

    uint8_t value = data[3];
    Serial.printf("📦 NFC value read from card: %d\n", value);
    return value;
}

void requestTargetsFromHubOnce() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("⚠️ WiFi not connected, skip HELLO sync");
        return;
    }

    WiFiClient client;
    Serial.println("🤝 Connecting to Hub for HELLO sync...");

    if (!client.connect(HUB_IP, HUB_PORT)) {
        Serial.println("❌ HELLO sync connect failed");
        return;
    }

    client.println("HELLO");
    client.flush();

    uint32_t t0 = millis();
    const uint32_t timeoutMs = 1500;

    while (millis() - t0 < timeoutMs) {
        while (client.available()) {
            String line = client.readStringUntil('\n');
            line.trim();
            if (line.length() == 0) continue;

            Serial.printf("📥 [HELLO sync] %s\n", line.c_str());

            if (line.startsWith("Target:")) {
                int p1 = line.indexOf(':');
                int p2 = line.indexOf(':', p1 + 1);

                String mode  = line.substring(p1 + 1, p2);
                int value    = line.substring(p2 + 1).toInt();

                String key;
                if (mode == "Balancing") key = "Balance";
                else if (mode == "March Steps") key = "March";
                else if (mode == "Pose") key = "Pose";
                else if (mode == "Sit-to-Stand") key = "Sit";
                else if (mode == "Long Squat") key = "Squat";
                else if (mode == "NFC Game") key = "NFC";
                else {
                    Serial.printf("⚠️ Unknown mode name: %s\n", mode.c_str());
                    continue;
                }

                Serial.printf("🎯 Parsed => mode=%s  key=%s  value=%d\n", mode.c_str(), key.c_str(), value);

                prefs.begin("core", false);
                prefs.putInt(key.c_str(), value);
                prefs.end();

                if (key == "Balance") targetBalance = value;
                else if (key == "March") targetMarch = value;
                else if (key == "Pose") targetPose = value;
                else if (key == "Sit") targetSit = value;
                else if (key == "Squat") targetSquat = value;
                else if (key == "NFC") targetNFC = value;

                Serial.println("💾 Saved target from HELLO sync");
            }
        }

        if (!client.connected()) break;

        delay(10);
    }

    client.stop();
    Serial.println("✅ HELLO sync done");
}


