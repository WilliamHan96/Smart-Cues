/**
 ******************************************************************************
 * @brief    Dashboard + BLE + PIR brightness control (Waveshare ESP32-S3-Touch-LCD-5)
 * @author   Smart Cues Group
 * @date     2026-10-02
 * @note     Grey dashboard background, BLE support, and PIR-based brightness control
 ******************************************************************************
 */

#include "waveshare_sd_card.h"
#include <SD.h>
#include <SPI.h>
esp_expander::CH422G *expander = NULL;
SemaphoreHandle_t rtcMutex;

#include <Arduino.h>
#include <esp_display_panel.hpp>

// BLE libraries
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include "esp_bt.h"   // Required for esp_ble_tx_power_set()

#include <WiFi.h>
WiFiServer server(8888);
#include "esp_wifi.h"
#include "esp_log.h"

// LVGL
#include "lvgl.h"
#include "lvgl_v8_port.h"
#include <demos/lv_demos.h>

using namespace esp_panel::drivers;
using namespace esp_panel::board;

#include "waveshare_pcf85063a.h"
datetime_t now_time;
unsigned long lastClockUpdate = 0;
lv_obj_t *footer;

#include <Preferences.h>
Preferences prefs;  // NVS persistent storage

// Global state
bool popupVisible = false;
unsigned long lastPopupTime = 0;
// Shared title colour for all six cards
static const lv_color_t TITLE_COLOR = lv_color_hex(0xFFFFFF);  // Change this value to use a different title colour

// ========== BLE Configuration ==========
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define MODE_CHAR_UUID      "87654321-4321-4321-4321-cba987654321"
#define COUNTDOWN_CHAR_UUID "87654321-4321-4321-4321-cba987654322"

// ========== Global Variables ==========
static const char *MAIN_TAG = "waveshare_dashboard";
Board *board = nullptr;
// Global variables
int lastDay = -1; // -1 means not loaded yet
uint32_t lastDateKey = 0;   // Used to detect a date change

// BLE state
BLEServer* pServer = nullptr;
BLECharacteristic* pModeCharacteristic = nullptr;
BLECharacteristic* pCountdownCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool ble_initialized = false;
uint8_t current_mode = 2;
uint32_t selected_countdown = 30;

// ========== PIR & Brightness Control ==========
bool motionActive = false;
bool lowBrightness = false;
unsigned long lastMotionTime = 0;

// ========== Dashboard UI Elements ==========
lv_obj_t *cards[6];
lv_obj_t *uibars[6];
lv_obj_t *labels[6];
static void set_bar_color_by_progress(int i);
void update_footer(datetime_t now);
void update_footer_now();
static void show_target_dialog(int index);
bool safeReadRTC(datetime_t* t);
void logToSD(const char* message, const datetime_t& t);
void show_logo_from_sd();
const char *names[6] = {"Balancing","March Steps","Pose","Sit-to-Stand","Long Squat","NFC Game"};
const char *units[6] = {"s","","","","s",""};
uint16_t targets[6] = {60,100,5,10,60,3};
uint16_t currentVals[6] = {0,0,0,0,0,0};
lv_color_t colors[6] = {
  lv_color_hex(0x00CC88),
  lv_color_hex(0x0099FF),
  lv_color_hex(0xAA88FF),
  lv_color_hex(0xFFBB33),
  lv_color_hex(0xFF5577),
  lv_color_hex(0x00CCCC)
};

#include <queue>

struct UIUpdate {
  int index;   // Card index
  int value;   // Value increment
};
std::queue<UIUpdate> uiQueue;  // FIFO queue

// Log queue
std::queue<String> logQueue;  // Queued log messages
SemaphoreHandle_t logMutex = NULL;  // Mutex protecting the log queue

// ========== BLE Functions ==========
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        Serial.println("🔗 Device connected!");
    }
    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("❌ Device disconnected!");
        delay(100);
        pServer->getAdvertising()->start();
    }
};

void setup_enhanced_ble() {
    Serial.println("🔵 Setting up BLE server...");

    if (!btStart()) return;
    BLEDevice::init("ESP32_Dashboard");
    // Increase BLE advertising and connection power

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);

    pModeCharacteristic = pService->createCharacteristic(MODE_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

    pCountdownCharacteristic = pService->createCharacteristic(COUNTDOWN_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

    // Register receive callback
    class DashboardCharCallbacks : public BLECharacteristicCallbacks {
        void onWrite(BLECharacteristic *pCharacteristic) override {
            std::string val = pCharacteristic->getValue().c_str();
            if (val.empty()) return;
            Serial.printf("📩 Received from Core: %s\n", val.c_str());

            String msg = val.c_str();
            int colonIndex = msg.indexOf(':');
            if (colonIndex < 0) return;

            String type = msg.substring(0, colonIndex);
            String valuePart = msg.substring(colonIndex + 1);
            valuePart.replace("s", "");
            valuePart.replace("done", "");
            int valInt = valuePart.toInt();

            int index = -1;
            if (type == "Balance") index = 0;
            else if (type == "March") index = 1;
            else if (type == "Pose") index = 2;
            else if (type == "Sit") index = 3;
            else if (type == "Squat") index = 4;
            else if (type == "NFC") index = 5;

            if (index >= 0) {
                currentVals[index] += valInt;
                lvgl_port_lock(-1);
                lv_bar_set_value(uibars[index], currentVals[index], LV_ANIM_OFF);
                set_bar_color_by_progress(index);
                prefs.begin("dashboard", false);
                char key_c[16];
                sprintf(key_c, "current%d", index);
                prefs.putUInt(key_c, currentVals[index]);
                prefs.end();
                Serial.printf("💾 Saved progress[%d] = %d to NVS\n", index, currentVals[index]);

                lv_label_set_text_fmt(labels[index], "%d / %d%s",
                                      currentVals[index], targets[index], units[index]);

                int doneCount = 0;
                for (int i = 0; i < 6; i++) {
                    if (currentVals[i] >= targets[i]) doneCount++;
                }
                char footerText[64];
                sprintf(footerText, "Today: %d/6 done  Keep it up!", doneCount);
                lv_label_set_text(footer, footerText);
                update_footer_now();

                lvgl_port_unlock();

                Serial.printf("✅ Updated card[%d] (%s) → %d%s\n",
                              index, names[index], currentVals[index], units[index]);
            }
        }
    };

    pCountdownCharacteristic->setCallbacks(new DashboardCharCallbacks());

    pModeCharacteristic->addDescriptor(new BLE2902());
    pCountdownCharacteristic->addDescriptor(new BLE2902());
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();

    ble_initialized = true;
    Serial.println("✅ BLE initialized and advertising");
}

// ========== Dashboard UI ==========
void create_dashboard_ui() {
    Serial.println("🎨 Creating Dashboard UI...");
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x818485), LV_PART_MAIN);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Smart Cues Home Physical Activity Dashboard");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    int startX[3] = {-220, 0, 220};
    int startY[2] = {-60, 100};
    int index = 0;

    for (int row = 0; row < 2; row++) {
        for (int col = 0; col < 3; col++) {
            if (index >= 6) break;
            // Card container
            cards[index] = lv_obj_create(scr);
            lv_obj_set_size(cards[index], 200, 100);
            lv_obj_set_style_bg_color(cards[index], lv_color_hex(0x3A3A3A), LV_PART_MAIN);
            lv_obj_set_style_radius(cards[index], 10, 0);
            lv_obj_align(cards[index], LV_ALIGN_CENTER, startX[col], startY[row]);
            // Title
            lv_obj_t *label = lv_label_create(cards[index]);
            lv_label_set_text_fmt(label, "%s", names[index]);
            lv_obj_set_style_text_color(label, TITLE_COLOR, 0);
            lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
            lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 5);
            // Progress bar
            uibars[index] = lv_bar_create(cards[index]);
            lv_obj_set_size(uibars[index], 160, 15);
            lv_obj_align(uibars[index], LV_ALIGN_CENTER, 0, 10);
            lv_bar_set_range(uibars[index], 0, targets[index]);
            lv_bar_set_value(uibars[index], currentVals[index], LV_ANIM_OFF);
            lv_obj_set_style_bg_color(uibars[index], lv_color_hex(0x444444), LV_PART_MAIN);
            lv_obj_set_style_bg_color(uibars[index], colors[index], LV_PART_INDICATOR);
            set_bar_color_by_progress(index);
            // Progress label
            labels[index] = lv_label_create(cards[index]);
            lv_label_set_text_fmt(labels[index], "%d / %d%s", currentVals[index], targets[index], units[index]);
            lv_obj_set_style_text_font(labels[index], &lv_font_montserrat_16, 0);
            lv_obj_set_style_text_color(labels[index], lv_color_white(), 0);
            lv_obj_align(labels[index], LV_ALIGN_BOTTOM_MID, 0, -5);

            lv_obj_add_flag(cards[index], LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(cards[index], [](lv_event_t * e) {
                int i = (int)(intptr_t)lv_event_get_user_data(e);
                show_target_dialog(i);
            }, LV_EVENT_CLICKED, (void*)(intptr_t)index);

            index++;
        }
    }

    // Footer
    footer = lv_label_create(scr);
    lv_label_set_text(footer, "Today: 5/6 done  Keep it up!   --:--");
    lv_obj_set_style_text_color(footer, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(footer, &lv_font_montserrat_20, 0);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, -20);
    Serial.println("✅ Dashboard UI created");

    // Reset button
    lv_obj_t *reset_btn = lv_btn_create(scr);
    lv_obj_set_size(reset_btn, 120, 50);
    lv_obj_align(reset_btn, LV_ALIGN_BOTTOM_RIGHT, -15, -15);
    lv_obj_set_style_bg_color(reset_btn, lv_color_hex(0x444444), LV_PART_MAIN);
    lv_obj_set_style_text_font(reset_btn, &lv_font_montserrat_20, 0);
    lv_obj_t *reset_label = lv_label_create(reset_btn);
    lv_label_set_text(reset_label, "Reset");
    lv_obj_center(reset_label);

    // Click handler
    lv_obj_add_event_cb(reset_btn, [](lv_event_t * e) {
        Serial.println("🧹 Manual reset triggered by user!");

        char before[256];
        sprintf(before, "MANUAL RESET - Before: Balancing=%d, March=%d, Pose=%d, Sit=%d, Squat=%d, NFC=%d",
                currentVals[0], currentVals[1], currentVals[2],
                currentVals[3], currentVals[4], currentVals[5]);
        datetime_t now;
        if (safeReadRTC(&now)) {
            logToSD(before, now);
        }

        // Clear current values
        prefs.begin("dashboard", false);
        for (int i = 0; i < 6; i++) {
            currentVals[i] = 0;

            char key_c[16];
            sprintf(key_c, "current%d", i);
            prefs.putUInt(key_c, 0);
        }
        prefs.end();
        Serial.println("💾 All progress reset and saved to NVS.");
        lvgl_port_lock(-1);
        for (int i = 0; i < 6; i++) {
            lv_bar_set_value(uibars[i], 0, LV_ANIM_OFF);
            set_bar_color_by_progress(i);
            lv_label_set_text_fmt(labels[i], "0 / %d%s", targets[i], units[i]);
        }
        update_footer_now();

        lvgl_port_unlock();
    }, LV_EVENT_CLICKED, NULL);

}

void show_motion_popup() {
    popupVisible = true;
    lastPopupTime = millis();

    lvgl_port_lock(-1);

    // Semi-transparent overlay
    lv_obj_t *mask = lv_obj_create(lv_scr_act());
    lv_obj_set_style_bg_opa(mask, LV_OPA_50, 0);
    lv_obj_set_style_bg_color(mask, lv_color_black(), 0);
    lv_obj_set_size(mask, LV_PCT(100), LV_PCT(100));

    // Popup container
    lv_obj_t *popup = lv_obj_create(mask);
    lv_obj_set_size(popup, 300, 160);
    lv_obj_center(popup);
    lv_obj_set_style_bg_color(popup, lv_color_hex(0x2E2E2E), 0);
    lv_obj_set_style_radius(popup, 15, 0);

    // Title
    lv_obj_t *title = lv_label_create(popup);
    lv_label_set_text(title, "Time to Move!");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    // Message
    lv_obj_t *msg = lv_label_create(popup);
    lv_label_set_text(msg, "Take a quick break and stretch!");
    lv_obj_set_style_text_font(msg, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(msg, lv_color_hex(0x00FFFF), 0);
    lv_obj_align(msg, LV_ALIGN_CENTER, 0, -10);

    // OK button
    lv_obj_t *ok_btn = lv_btn_create(popup);
    lv_obj_set_size(ok_btn, 80, 40);
    lv_obj_align(ok_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_t *ok_label = lv_label_create(ok_btn);
    lv_label_set_text(ok_label, "OK");
    lv_obj_center(ok_label);

    // Close on click
    lv_obj_add_event_cb(ok_btn, [](lv_event_t * e) {
        lv_obj_t *btn = lv_event_get_target(e);
        lv_obj_t *popup = lv_obj_get_parent(btn);
        lv_obj_t *mask = lv_obj_get_parent(popup);
        lv_obj_del_async(mask);
        popupVisible = false;
        Serial.println("🟢 Popup manually dismissed");
    }, LV_EVENT_CLICKED, NULL);

    lvgl_port_unlock();

    Serial.println("🏃 Motion popup displayed!");
}

// ========== PIR Detection ==========
void handlePIR() {
    if (!expander) return;
    static bool lastVal = 0;
    uint8_t val = expander->digitalRead(0);


    if (val != lastVal) {
        lastVal = val;
        if (val == HIGH) {
            Serial.println("Motion start");
            motionActive = true;
            lowBrightness = false;   // Clear dimmed-state flag after motion

        } else {
            Serial.println("Motion end");
            if (motionActive) {
                motionActive = false;
                lastMotionTime = millis();
                Serial.println("✅ PIR Triggered - Brighten screen");
            }
        }
    }

}

// ========== Setup ==========
void setup() {
    Serial.begin(115200);
    Wire.begin(8, 9, 400000);  // SDA=8, SCL=9, 400kHz

    delay(1000);

    Serial.println("========================================");
    Serial.println("🚀 Waveshare ESP32-S3 Dual Mode Controller");
    Serial.println("========================================");

    // Step 1: start the Wi-Fi access point
    Serial.println("📡 Step 1: Starting Wi-Fi AP (Hub mode)...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("SmartHub_AP", "SmartCues");
    esp_wifi_set_max_tx_power(84);   // 78 × 0.25 dBm = 19.5 dBm
    esp_wifi_set_protocol(WIFI_IF_AP,
                      WIFI_PROTOCOL_LR);
    IPAddress myIP = WiFi.softAPIP();
    Serial.printf("✅ Wi-Fi AP started. SSID: SmartHub_AP  IP: %s\n", myIP.toString().c_str());
    int8_t power;
    esp_wifi_get_max_tx_power(&power);
    Serial.printf("📶 Current TX Power: %.2f dBm\n", power * 0.25);

    server.begin();
    Serial.println("✅ TCP server started on port 8888");

    // Create the log mutex and background logger task
    logMutex = xSemaphoreCreateMutex();
    rtcMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
        [](void* param) {
            while (true) {
                // Check for queued log messages
                if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(100))) {
                    if (!logQueue.empty()) {
                        String msg = logQueue.front();
                        logQueue.pop();
                        xSemaphoreGive(logMutex);

                        // Write to the SD card in the background
                        File logFile = SD.open("/activity_log.txt", FILE_APPEND);
                        if (logFile) {
                            logFile.println(msg);
                            logFile.close();
                            Serial.printf("📝 Written to SD: %s\n", msg.c_str());
                        } else {
                            Serial.println("❌ Failed to write log");
                        }
                    } else {
                        xSemaphoreGive(logMutex);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100 ms
            }
        },
        "SD_Logger",   // Task name
        4096,          // Stack size
        NULL,
        1,             // Low task priority
        NULL,
        0              // Run on Core 0
    );

    Serial.println("✅ Background SD logger task started");

    // Step 2: initialise the Waveshare board
    Serial.println("📋 Step 2: Initialize Waveshare 5inch board...");
    board = new Board();
    board->init();

    // Configure the RGB LCD
    auto lcd_panel = board->getLCD();
    lcd_panel->configFrameBufferNumber(3);  // Frame buffers

    #if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    auto lcd_bus = lcd_panel->getBus();
    if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
        static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd_panel->getFrameWidth() * 30);
    }

    #endif

    // Start the board
    if (!board->begin()) {
        Serial.println("❌ Failed to initialize board!");
        while(1) {
            delay(1000);
        }
    }
    Serial.println("✅ Waveshare 5inch board initialized successfully");

    // Print board information
    auto lcd_info = board->getLCD();
    Serial.printf("📺 LCD Resolution: %dx%d\n", lcd_info->getFrameWidth(), lcd_info->getFrameHeight());
    Serial.printf("🖥️ LCD Bus Type: %s\n",
                  (lcd_info->getBus()->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) ? "RGB" : "Other");

    // Step 3: initialise LVGL
    delay(500);
    Serial.println("🎨 Step 3: Initialize LVGL...");
    if (!lvgl_port_init(board->getLCD(), board->getTouch())) {
        Serial.println("❌ Failed to initialize LVGL!");
        while(1) {
            delay(1000);
        }
    }
    Serial.println("✅ LVGL initialized successfully");

    Serial.println("🕒 Step 4: Initialize internal PCF85063A RTC...");

    // Initialise the PCF85063A RTC
    PCF85063A_Init();
    delay(50);

    // Detect a new firmware build and sync the RTC

    char buildTimeStr[40];
    sprintf(buildTimeStr, "%s %s", __DATE__, __TIME__);

    prefs.begin("dashboard", false);
    String savedBuildTime = prefs.getString("fwBuild", "");
    prefs.end();

    Serial.printf("🧱 Current firmware build: %s\n", buildTimeStr);
    Serial.printf("🗄️ Saved firmware build: %s\n", savedBuildTime.c_str());

    if (savedBuildTime != String(buildTimeStr)) {
        Serial.println("🆕 New firmware detected → Sync RTC to compile time");

        const char* date_str = __DATE__;
        const char* time_str = __TIME__;

        char month_str[4];
        int day, year, hour, minute, second;

        sscanf(date_str, "%3s %d %d", month_str, &day, &year);
        sscanf(time_str, "%d:%d:%d", &hour, &minute, &second);

        int month = 1;
        const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
        const char* pos = strstr(months, month_str);
        if (pos) month = ((pos - months) / 3) + 1;

        datetime_t compile_time = {
            .year = (uint16_t)year,
            .month = (uint8_t)month,
            .day = (uint8_t)day,
            .dotw = 0,
            .hour = (uint8_t)hour,
            .min = (uint8_t)minute,
            .sec = (uint8_t)second
        };

        PCF85063A_Set_All(compile_time);
        Serial.println("✅ RTC synced to compile time");

        prefs.begin("dashboard", false);
        prefs.putString("fwBuild", buildTimeStr);
        prefs.end();
    }

    // Register the LVGL file system and display the SD-card logo
    // Register the SD file system with LVGL
    Serial.println("💾 Mounting SD card for LVGL...");

    static lv_fs_drv_t drv;
    lv_fs_drv_init(&drv);
    drv.letter = 'S';

    // Open file
    drv.open_cb = [](lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode) -> void* {
        const char *real_path = path;
        File *f = new File(SD.open(real_path, (mode == LV_FS_MODE_WR) ? FILE_WRITE : FILE_READ));
        if (!f || !*f) {
            delete f;
            return NULL;
        }
        return f;
    };

    // Close file
    drv.close_cb = [](lv_fs_drv_t *drv, void *file_p) -> lv_fs_res_t {
        if (!file_p) return LV_FS_RES_INV_PARAM;
        File *f = static_cast<File *>(file_p);
        f->close();
        delete f;
        return LV_FS_RES_OK;
    };

    // Read file
    drv.read_cb = [](lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br) -> lv_fs_res_t {
        if (!file_p) return LV_FS_RES_INV_PARAM;
        File *f = static_cast<File *>(file_p);
        *br = f->read((uint8_t*)buf, btr);
        return LV_FS_RES_OK;
    };

    // Seek within file
    drv.seek_cb = [](lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence) -> lv_fs_res_t {
        if (!file_p) return LV_FS_RES_INV_PARAM;
        File *f = static_cast<File *>(file_p);
        f->seek(pos);
        return LV_FS_RES_OK;
    };

    // Get the current file position
    drv.tell_cb = [](lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p) -> lv_fs_res_t {
        if (!file_p || !pos_p) return LV_FS_RES_INV_PARAM;
        File *f = static_cast<File *>(file_p);
        *pos_p = f->position();
        return LV_FS_RES_OK;
    };

    // Register file-system driver
    lv_fs_drv_register(&drv);
    Serial.println("✅ SD card registered to LVGL drive 'S:/'");

    // Display logo
    show_logo_from_sd();

    Serial.println("💾 Loading saved progress and targets...");
    prefs.begin("dashboard", false); // Open the "dashboard" namespace

    for (int i = 0; i < 6; i++) {
        // Load target, using the compiled default if missing
        char key_t[16];
        sprintf(key_t, "target%d", i);
        targets[i] = prefs.getUInt(key_t, targets[i]);

        // Load current progress, defaulting to zero
        char key_c[16];
        sprintf(key_c, "current%d", i);
        currentVals[i] = prefs.getUInt(key_c, currentVals[i]);
    }

    prefs.end();

    // Load the saved date
    prefs.begin("dashboard", false);
    lastDay = prefs.getInt("lastDay", -1);
    prefs.end();
    Serial.printf("📅 Loaded last saved day = %d\n", lastDay);

    Serial.println("✅ Loaded saved progress from NVS.");

    prefs.begin("dashboard", false);
    lastDateKey = prefs.getULong("lastDateKey", 0);
    prefs.end();

    Serial.printf("📅 Loaded lastDateKey = %lu\n", lastDateKey);

    if (lastDateKey == 0) {
        uint32_t todayKey =
            now_time.year * 10000UL +
            now_time.month * 100 +
            now_time.day;

        lastDateKey = todayKey;

        prefs.begin("dashboard", false);
        prefs.putULong("lastDateKey", lastDateKey);
        prefs.end();

        Serial.println("🆕 Initialized lastDateKey to today");
    }

    if (lastDay == -1) {
        // On first run, save the current date to avoid an accidental reset
        lastDay = now_time.day;
        prefs.begin("dashboard", false);
        prefs.putInt("lastDay", lastDay);
        prefs.end();
        Serial.printf("🆕 First boot, initialize lastDay = %d\n", lastDay);
    }

    // Display the dashboard
    lvgl_port_lock(-1);
    create_dashboard_ui();
    lvgl_port_unlock();

    lastMotionTime = millis();

    // Initialise the SD-card log file
    Serial.println("📝 Initializing activity log...");
    if (!SD.exists("/activity_log.txt")) {
        File logFile = SD.open("/activity_log.txt", FILE_WRITE);
        if (logFile) {
            logFile.println("=== Activity Log Started ===");
            logFile.close();
            Serial.println("✅ Created new activity_log.txt");
        } else {
            Serial.println("❌ Failed to create log file");
        }
    } else {
        Serial.println("✅ activity_log.txt already exists");
    }

    // Log system startup
    char bootMsg[128];
    sprintf(bootMsg, "=== SYSTEM BOOT: %04d-%02d-%02d %02d:%02d:%02d ===",
            now_time.year, now_time.month, now_time.day,
            now_time.hour, now_time.min, now_time.sec);
    datetime_t now;
    if (safeReadRTC(&now)) {
        logToSD(bootMsg, now);
    }

    Serial.println("✅ Setup completed. Dashboard running.");
}

// ========== Main Loop ==========
void loop() {
    // Receive activity data over Wi-Fi
    // ==================== Wi-Fi Data Receiver ====================
    WiFiClient client = server.available();
    if (client) {
        Serial.println("📥 Client connected");
        String msg = client.readStringUntil('\n');
        Serial.printf("📦 Received from Core: %s\n", msg.c_str());

        if (msg.startsWith("HELLO")) {
            Serial.println("🤝 Core says HELLO → sending all targets");

            for (int i = 0; i < 6; i++) {
                String tmsg = String("Target:") + names[i] + ":" + targets[i];
                client.println(tmsg);
                delay(10);  // Small gap between messages
            }

            client.stop();
            return;
        }

        // Parse messages such as "Balance:60"
        int colonIndex = msg.indexOf(':');
        if (colonIndex >= 0) {
            String type = msg.substring(0, colonIndex);
            String valuePart = msg.substring(colonIndex + 1);
            valuePart.replace("s", "");
            int valInt = valuePart.toInt();

            int index = -1;
            if (type == "Balance") index = 0;
            else if (type == "March") index = 1;
            else if (type == "Pose") index = 2;
            else if (type == "Sit") index = 3;
            else if (type == "Squat") index = 4;
            else if (type == "NFC") index = 5;

            if (index >= 0) {

                UIUpdate update = {index, valInt};
                uiQueue.push(update);   // Queue only; the UI is updated later

            }
        }
        client.stop();
    }

    if (millis() - lastClockUpdate > 1000) {
        lastClockUpdate = millis();

        update_footer_now();

    }

    datetime_t currentTime;

    if (!safeReadRTC(&currentTime)) {
        // Skip this iteration if the RTC read fails
        return;
    }

    static uint32_t pendingDateKey = 0;

    uint32_t todayKey =
        currentTime.year * 10000UL +
        currentTime.month * 100 +
        currentTime.day;

    static uint32_t debugCounter = 0;

    if (lastDateKey == 0) {
        lastDateKey = todayKey;   // First-time initialisation
    }

    if (todayKey != lastDateKey) {

        if (pendingDateKey == 0) {
            pendingDateKey = todayKey;   // First observed date change
        }
        else if (pendingDateKey == todayKey) {

            // Confirm that the date has changed
            char summary[256];
            sprintf(summary,
                "DAY SUMMARY %04d-%02d-%02d: Balancing=%d/%d, March=%d/%d, Pose=%d/%d, Sit=%d/%d, Squat=%d/%d, NFC=%d/%d",
                now_time.year, now_time.month, lastDateKey % 100,
                currentVals[0], targets[0],
                currentVals[1], targets[1],
                currentVals[2], targets[2],
                currentVals[3], targets[3],
                currentVals[4], targets[4],
                currentVals[5], targets[5]);

            int doneCount = 0;
            for (int i = 0; i < 6; i++) {
                if (currentVals[i] >= targets[i]) doneCount++;
            }

            char completion[64];
            sprintf(completion, "Completion: %d/6 tasks (%d%%)",
                    doneCount, (doneCount * 100) / 6);

            datetime_t now;
            if (safeReadRTC(&now)) {
                logToSD(summary, now);
                logToSD(completion, now);
            }

            Serial.println("📅 New day confirmed → reset all progress");

            // Reset daily progress
            prefs.begin("dashboard", false);
            for (int i = 0; i < 6; i++) {
                currentVals[i] = 0;
                char key_c[16];
                sprintf(key_c, "current%d", i);
                prefs.putUInt(key_c, 0);
            }
            prefs.end();

            lvgl_port_lock(-1);
            for (int i = 0; i < 6; i++) {
                lv_bar_set_value(uibars[i], 0, LV_ANIM_OFF);
                set_bar_color_by_progress(i);
                lv_label_set_text_fmt(labels[i], "0 / %d%s",
                                    targets[i], units[i]);
            }
            update_footer_now();
            lvgl_port_unlock();

            char newDay[64];
            sprintf(newDay, "=== NEW DAY START: %04d-%02d-%02d ===",
                    now_time.year, now_time.month, now_time.day);

            if (safeReadRTC(&now)) {
                logToSD(newDay, now);
            }

            lastDateKey = todayKey;
            pendingDateKey = 0;

            prefs.begin("dashboard", false);
            prefs.putULong("lastDateKey", lastDateKey);
            prefs.end();
        }
    }
    else {
        pendingDateKey = 0;
    }

    // Process queued UI updates
    if (!uiQueue.empty()) {

        UIUpdate task = uiQueue.front();
        uiQueue.pop();

        int i = task.index;

        // Update the stored value in one place
        currentVals[i] += task.value;

        // Save to NVS
        prefs.begin("dashboard", false);
        char key_c[16];
        sprintf(key_c, "current%d", i);
        prefs.putUInt(key_c, currentVals[i]);
        prefs.end();

        // Write activity log
        char logMsg[128];
        sprintf(logMsg, "%s: +%d (now: %d/%d%s)",
                names[i], task.value,
                currentVals[i], targets[i], units[i]);

        datetime_t now;
        if (safeReadRTC(&now)) {
            logToSD(logMsg, now);
        }

        // Update the LVGL UI in one place
        lvgl_port_lock(-1);

        lv_bar_set_value(uibars[i], currentVals[i], LV_ANIM_OFF);
        set_bar_color_by_progress(i);
        lv_label_set_text_fmt(labels[i], "%d / %d%s",
                            currentVals[i], targets[i], units[i]);

        update_footer_now();

        lvgl_port_unlock();

        Serial.printf("✅ Stable update card[%d] = %d\n", i, currentVals[i]);
    }

    delay(5);

}

void show_logo_from_sd() {
    uint32_t t0 = millis();

    Serial.println("========================================");
    Serial.println("🖼️ Displaying Logo from SD Card");
    Serial.println("========================================");

    // Step 1: obtain and configure the I/O expander
    auto base_expander = board->getExpander();
    if (!base_expander) {
        Serial.println("❌ Failed to get expander from board!");
        return;
    }

    expander = static_cast<esp_expander::CH422G*>(base_expander);
    Serial.println("✅ Expander obtained from board");

    Serial.println("✅ Expander obtained from board");

    // Step 2: configure the GPIO pins
    Serial.println("📌 Configuring expander pins for SD card...");

    // Set the required control pins
    expander->digitalWrite(SD_CS, HIGH);     // Deselect the SD card initially
    expander->digitalWrite(LCD_BL, HIGH);    // Enable backlight
    expander->digitalWrite(USB_SEL, LOW);    // Configure USB selection
    delay(10);  // Allow GPIO levels to settle

    Serial.println("✅ Expander pins configured");

    // Step 3: initialise SPI
    Serial.println("🔌 Initializing SPI bus...");

    // End any previous SPI session
    SPI.end();
    delay(10);

    // Reinitialise SPI
    SPI.setHwCs(false);
    Serial.printf("t+%lu: before SD.begin\n", millis()-t0);

    SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_SS);
    Serial.printf("t+%lu: after SD.begin (ok=%d)\n", millis()-t0);

    delay(10);

    Serial.println("✅ SPI initialized");

    // Step 4: mount the SD card
    Serial.println("💾 Mounting SD card...");

    expander->digitalWrite(SD_CS, LOW);  // Select the SD card
    delay(10);

    // Mount the SD card
    if (!SD.begin(SD_SS, SPI, 20000000)) {
        Serial.println("\n❌ SD CARD MOUNT FAILED!\n");
        Serial.println("Troubleshooting:");
        Serial.println("  1. Check SD card is properly inserted");
        Serial.println("  2. Check SD card format is FAT32");
        Serial.println("  3. Try a different SD card (≤32GB)");
        return;
    }

    Serial.println("✅ SD card mounted successfully");
    Serial.printf("t+%lu: before lv_img_set_src\n", millis()-t0);

    lvgl_port_lock(-1);

    lv_obj_t *img = lv_img_create(lv_scr_act());
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_src(img, "S:/images/test16.bin");

    Serial.println("✅ Image source set");
    Serial.printf("t+%lu: after lv_img_set_src\n", millis()-t0);

    lvgl_port_unlock();

    Serial.printf("\n💾 Free Heap: %d KB\n", ESP.getFreeHeap() / 1024);
    Serial.println("========================================\n");
    delay(3000);

}

void test_lvgl_fs(const char *path) {
    lv_fs_file_t f;
    uint8_t buf[8];
    uint32_t br = 0;
    if (lv_fs_open(&f, path, LV_FS_MODE_RD) != LV_FS_RES_OK) {
        Serial.printf("❌ lv_fs_open failed for %s\n", path);
        return;
    }
    lv_fs_read(&f, buf, 8, &br);
    lv_fs_close(&f);
    Serial.printf("✅ lv_fs_read success, first bytes: %02X %02X %02X %02X\n", buf[0], buf[1], buf[2], buf[3]);
}

static const lv_color_t BAR_PINK   = lv_color_hex(0xFF4FA3);
static const lv_color_t BAR_BLUE   = lv_color_hex(0x3FA9FF);
static const lv_color_t BAR_ORANGE = lv_color_hex(0xFFA500);
static const lv_color_t BAR_GREEN  = lv_color_hex(0x2DFF88);

static void set_bar_color_by_progress(int i) {
    if (!uibars[i] || targets[i] == 0) return;

    float p = (float)currentVals[i] / (float)targets[i];
    if (p < 0) p = 0;

    lv_color_t c;

    if (p >= 1.0f) {
        c = BAR_GREEN;          // Green when complete
    } else if (p >= 0.8f) {
        c = BAR_ORANGE;         // Orange at 80% or above
    } else if (p >= 0.5f) {
        c = BAR_BLUE;           // Mid progress
    } else {
        c = BAR_PINK;           // Early progress
    }

    lv_obj_set_style_bg_color(uibars[i], c, LV_PART_INDICATOR);
}

void update_footer(datetime_t now) {
    int doneCount = 0;
    for (int i = 0; i < 6; i++) {
        if (currentVals[i] >= targets[i]) doneCount++;
    }

    char buf[80];
    if (doneCount >= 6) {
        sprintf(buf, "🎉 Congrats! All goals achieved!  %02d:%02d:%02d",
                now.hour, now.min, now.sec);
    } else {
        sprintf(buf, "Today: %d/6 done  Keep it up!  %02d:%02d:%02d",
                doneCount, now.hour, now.min, now.sec);
    }

    lv_label_set_text(footer, buf);
}

// ========== Target Settings Dialog ==========
// Target range for each activity
uint16_t minTargets[6] = {10, 50, 3, 5, 10, 1};
uint16_t maxTargets[6] = {120, 500, 20, 30, 100, 10};

// ========== Target Settings Dialog ==========
static void show_target_dialog(int index) {
    // Semi-transparent overlay
    lv_obj_t *mask = lv_obj_create(lv_scr_act());
    lv_obj_set_style_bg_opa(mask, LV_OPA_50, 0);
    lv_obj_set_style_bg_color(mask, lv_color_black(), 0);
    lv_obj_set_size(mask, LV_PCT(100), LV_PCT(100));

    // Dialog container
    lv_obj_t *box = lv_obj_create(mask);
    lv_obj_set_size(box, 300, 220);
    lv_obj_center(box);
    lv_obj_set_style_bg_color(box, lv_color_hex(0x2E2E2E), 0);
    lv_obj_set_style_radius(box, 15, 0);

    // Title
    lv_obj_t *title = lv_label_create(box);
    lv_label_set_text_fmt(title, "Set target for %s", names[index]);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

    // Current target value
    lv_obj_t *value_label = lv_label_create(box);
    lv_label_set_text_fmt(value_label, "%d%s", targets[index], units[index]);
    lv_obj_set_style_text_font(value_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(value_label, lv_color_hex(0x00FFFF), 0);
    lv_obj_align(value_label, LV_ALIGN_CENTER, 0, -25);

    // Slider
    lv_obj_t *slider = lv_slider_create(box);
    lv_obj_set_width(slider, 240);
    lv_obj_align(slider, LV_ALIGN_CENTER, 0, 20);
    lv_slider_set_range(slider, minTargets[index], maxTargets[index]);
    lv_slider_set_value(slider, targets[index], LV_ANIM_OFF);

    // Store the value label in the slider user data
    lv_obj_set_user_data(slider, value_label);

    // Allocate a stable copy of the activity index for callbacks
    int *index_copy = (int *)lv_mem_alloc(sizeof(int));
    *index_copy = index;

    // Update the displayed value while sliding
    lv_obj_add_event_cb(slider, [](lv_event_t * e) {
        lv_obj_t *slider = lv_event_get_target(e);
        int val = lv_slider_get_value(slider);

        // Read activity index
        int *pindex = (int *)lv_event_get_user_data(e);
        int index = *pindex;

        // Read value label
        lv_obj_t *val_label = (lv_obj_t *)lv_obj_get_user_data(slider);

        // NFC uses a step size of 1
        if (index == 5) {
            lv_label_set_text_fmt(val_label, "%d", val);
        } else {
            // Other activities use steps of 5
            val = ((val + 2) / 5) * 5;
            lv_slider_set_value(slider, val, LV_ANIM_OFF);
            lv_label_set_text_fmt(val_label, "%d", val);
        }

    }, LV_EVENT_VALUE_CHANGED, index_copy);

    // OK button
    lv_obj_t *ok_btn = lv_btn_create(box);
    lv_obj_set_size(ok_btn, 80, 40);
    lv_obj_align(ok_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_t *ok_label = lv_label_create(ok_btn);
    lv_label_set_text(ok_label, "OK");
    lv_obj_center(ok_label);

    // Save the target when OK is pressed
    lv_obj_add_event_cb(ok_btn, [](lv_event_t * e) {
        lv_obj_t *btn = lv_event_get_target(e);
        lv_obj_t *box = lv_obj_get_parent(btn);
        lv_obj_t *slider = lv_obj_get_child(box, 2);  // Slider is the third child object
        int *pindex = (int *)lv_event_get_user_data(e);
        int index = *pindex;

        int new_val = lv_slider_get_value(slider);
        // Round non-NFC targets before saving
        if (index != 5) {
            new_val = ((new_val + 2) / 5) * 5;
        }

        int old_target = targets[index];
        targets[index] = new_val;

        prefs.begin("dashboard", false);
        char key_t[16];
        sprintf(key_t, "target%d", index);
        prefs.putUInt(key_t, new_val);
        prefs.end();
        Serial.printf("💾 Saved new target[%d] = %d to NVS\n", index, new_val);

        // Log the target change to the SD card
        char logMsg[128];
        sprintf(logMsg, "Target changed: %s %d→%d%s",
                names[index], old_target, new_val, units[index]);
        datetime_t now;
        if (safeReadRTC(&now)) {
            logToSD(logMsg, now);
        }

        // Send the new target to the Core over Wi-Fi
        WiFiClient targetClient;
        if (targetClient.connect("192.168.4.2", 8888)) {   // Core IP address
            String msg = String("Target:") + names[index] + ":" + new_val;
            targetClient.println(msg);
            Serial.printf("📤 Sent new target to Core: %s\n", msg.c_str());
            targetClient.stop();
        } else {
            Serial.println("⚠️ Failed to send target to Core (not connected)");
        }

        lvgl_port_lock(-1);
        lv_bar_set_range(uibars[index], 0, targets[index]);
        set_bar_color_by_progress(index);
        lv_label_set_text_fmt(labels[index], "%d / %d%s",
                              currentVals[index], targets[index], units[index]);
        lvgl_port_unlock();

        // Close the dialog
        lv_obj_del_async(lv_obj_get_parent(box));
        lv_mem_free(pindex); // Free the allocated index
        Serial.printf("🎯 %s target updated → %d%s\n",
                      names[index], targets[index], units[index]);
    }, LV_EVENT_CLICKED, index_copy);
}

void handlePIR2() {
    if (!expander) return;

    static bool lastVal = 0;
    uint8_t val;

    // Read the PIR input
    delayMicroseconds(300);
    val = expander->digitalRead(0);

    // Detect a rising edge (motion start)
    if (val != lastVal) {
        lastVal = val;
        if (val == HIGH) {
            Serial.println("👀 PIR detected motion → show popup");

            // Trigger only when no popup is visible
            if (!popupVisible) {
                show_motion_popup();
            }
        }
    }

    // Automatically close the popup
    if (popupVisible && (millis() - lastPopupTime > 5000)) {
        lvgl_port_lock(-1);
        lv_obj_t *act = lv_scr_act();
        // The popup overlay is expected to be the last screen object
        lv_obj_t *mask = lv_obj_get_child(act, -1);
        if (mask) lv_obj_del_async(mask);
        lvgl_port_unlock();
        popupVisible = false;
        Serial.println("⏰ Popup auto dismissed (5s timeout)");
    }
}

// ========== Non-blocking SD Logging ==========

void logToSD(const char* message, const datetime_t& t) {
    if (!logMutex) return;

    char fullMsg[256];
    snprintf(fullMsg, sizeof(fullMsg),
             "[%04d-%02d-%02d %02d:%02d:%02d] %s",
             t.year, t.month, t.day,
             t.hour, t.min, t.sec,
             message);

    if (xSemaphoreTake(logMutex, portMAX_DELAY)) {
        logQueue.push(String(fullMsg));
        xSemaphoreGive(logMutex);
        Serial.printf("📩 Queued log: %s\n", fullMsg);
    }
}

bool safeReadRTC(datetime_t* t) {
    if (!rtcMutex) return false;

    if (xSemaphoreTake(rtcMutex, pdMS_TO_TICKS(50)) != pdTRUE)
        return false;

    PCF85063A_Read_now(t);
    xSemaphoreGive(rtcMutex);

    // Basic RTC validity checks
    if (t->year < 2026 || t->year > 2040) return false;
    if (t->month < 1 || t->month > 12) return false;
    if (t->day < 1 || t->day > 31) return false;
    if (t->hour > 23 || t->min > 59 || t->sec > 59) return false;

    return true;
}

void update_footer_now() {
    datetime_t now;

    if (!safeReadRTC(&now)) {
        Serial.println("⚠️ RTC invalid → footer not updated");
        return;
    }

    int doneCount = 0;
    for (int i = 0; i < 6; i++) {
        if (currentVals[i] >= targets[i]) doneCount++;
    }

    char buf[80];
    if (doneCount >= 6) {
        sprintf(buf, "🎉 Congrats! All goals achieved!  %02d:%02d:%02d",
                now.hour, now.min, now.sec);
    } else {
        sprintf(buf, "Today: %d/6 done  Keep it up!  %02d:%02d:%02d",
                doneCount, now.hour, now.min, now.sec);
    }

    lv_label_set_text(footer, buf);
}

