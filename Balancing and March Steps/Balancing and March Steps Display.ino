/**
 * @brief    Smart Cues step and balance countdown controller for the larger ESP32
 * @author   Smart Cues Group
 * @note     Handles stepping and balance countdown interactions for Smart Cues.
 */


#include "waveshare_sd_card.h"
#include <SD.h>
#include <SPI.h>
esp_expander::CH422G *expander = nullptr;


#include <Arduino.h>
#include <esp_display_panel.hpp>

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "lvgl.h"
#include "lvgl_v8_port.h"
#include <demos/lv_demos.h>


using namespace esp_panel::drivers;
using namespace esp_panel::board;

// ========== BLE Configuration ==========
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define MODE_CHAR_UUID      "87654321-4321-4321-4321-cba987654321"
#define COUNTDOWN_CHAR_UUID "87654321-4321-4321-4321-cba987654322"

// ========== Global Variables ==========
static const char *MAIN_TAG = "waveshare_dual_mode";

// Board object
Board *board = nullptr;

BLEServer* pServer = nullptr;
BLECharacteristic* pModeCharacteristic = nullptr;
BLECharacteristic* pCountdownCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t current_mode = 2;
bool mode_changed = false;
uint32_t selected_countdown = 30;

// LVGL UI objects
lv_obj_t* btn_step = nullptr;
lv_obj_t* btn_balance = nullptr;
lv_obj_t* label_current_mode = nullptr;
lv_obj_t* label_status = nullptr;
lv_obj_t* countdown_slider = nullptr;
lv_obj_t* label_countdown = nullptr;
lv_obj_t* btn_start_countdown = nullptr;
lv_obj_t* label_timer_display = nullptr;

// Task synchronization
bool ble_initialized = false;

// Function declarations
void send_mode_to_arduino();
void send_countdown_to_arduino(uint32_t seconds);
void update_mode_display();

// ========== BLE Server Callbacks ==========
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        Serial.println("🔗 Arduino R4 connected!");

        send_mode_to_arduino();
    }

    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("❌ Arduino R4 disconnected!");

        delay(100);
        pServer->getAdvertising()->start();
        Serial.println("🔄 Restarted advertising");
    }
};

// ========== BLE Functions ==========
void send_mode_to_arduino() {
    if (deviceConnected && pModeCharacteristic) {
        char buffer[4];
        snprintf(buffer, sizeof(buffer), "%d", current_mode);
        pModeCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
        pModeCharacteristic->notify();
        Serial.printf("📤 Mode sent to Arduino: %d\n", current_mode);
    }
}

void send_countdown_to_arduino(uint32_t seconds) {
    if (deviceConnected && pCountdownCharacteristic) {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%d", seconds);
        pCountdownCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
        pCountdownCharacteristic->notify();
        Serial.printf("📤 Countdown sent to Arduino: %d seconds\n", seconds);
    }
}

// ========== Enhanced BLE Setup ==========
void setup_enhanced_ble() {
    Serial.println("🔵 Setting up BLE server...");

    if (!btStart()) {
        Serial.println("❌ Failed to initialize BT controller");
        return;
    }

    BLEDevice::init("ESP32_Mode_Controller");
    Serial.println("✅ BLE Device initialized");
    delay(100);

    pServer = BLEDevice::createServer();
    if (!pServer) {
        Serial.println("❌ Failed to create BLE server");
        return;
    }
    pServer->setCallbacks(new MyServerCallbacks());
    Serial.println("✅ BLE Server created");

    BLEService *pService = pServer->createService(SERVICE_UUID);
    if (!pService) {
        Serial.println("❌ Failed to create BLE service");
        return;
    }
    Serial.println("✅ BLE Service created");

    pModeCharacteristic = pService->createCharacteristic(
                         MODE_CHAR_UUID,
                         BLECharacteristic::PROPERTY_READ |
                         BLECharacteristic::PROPERTY_WRITE |
                         BLECharacteristic::PROPERTY_NOTIFY
                       );

    pCountdownCharacteristic = pService->createCharacteristic(
                         COUNTDOWN_CHAR_UUID,
                         BLECharacteristic::PROPERTY_READ |
                         BLECharacteristic::PROPERTY_WRITE |
                         BLECharacteristic::PROPERTY_NOTIFY
                       );

    if (!pModeCharacteristic || !pCountdownCharacteristic) {
        Serial.println("❌ Failed to create BLE characteristics");
        return;
    }

    pModeCharacteristic->addDescriptor(new BLE2902());
    pCountdownCharacteristic->addDescriptor(new BLE2902());
    Serial.println("✅ BLE Characteristics created");

    String initialMode = String(current_mode);
    pModeCharacteristic->setValue(initialMode.c_str());
    pCountdownCharacteristic->setValue("0");

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

    if (pServer != nullptr) {
        Serial.println("🎯 BLE Server ready!");
        Serial.println("📱 Device name: ESP32_Mode_Controller");
        Serial.println("🔗 Waiting for Arduino R4 connection...");
        ble_initialized = true;
    } else {
        Serial.println("❌ Failed to start advertising");
    }
}

void bleTask(void* parameter) {
    setup_enhanced_ble();
    vTaskDelete(NULL);
}

// ========== UI Functions ==========
void update_mode_display() {
    if (current_mode == 1) {
        lv_label_set_text(label_current_mode, "Current Mode: STEP");
        lv_obj_set_style_text_color(label_current_mode, lv_color_hex(0x00AAFF), 0);

        lv_obj_set_style_bg_color(btn_step, lv_color_hex(0x00AAFF), LV_PART_MAIN);
        lv_obj_set_style_bg_color(btn_balance, lv_color_hex(0x333333), LV_PART_MAIN);
    } else {
        lv_label_set_text(label_current_mode, "Current Mode: BALANCE");
        lv_obj_set_style_text_color(label_current_mode, lv_color_hex(0x00FF00), 0);

        lv_obj_set_style_bg_color(btn_step, lv_color_hex(0x333333), LV_PART_MAIN);
        lv_obj_set_style_bg_color(btn_balance, lv_color_hex(0x00FF00), LV_PART_MAIN);
    }
}

// ========== Enhanced UI Creation for 5inch Screen ==========
void create_dual_mode_ui_large() {
    Serial.println("🎨 Creating dual mode UI for 5inch screen...");

    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);

    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0F0F0F), LV_PART_MAIN);

    lv_obj_t *title_label = lv_label_create(scr);
    lv_label_set_text(title_label, "Dual Mode Controller");
    lv_obj_set_style_text_color(title_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_20, 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, -100, 20);

    lv_obj_t *board_info = lv_label_create(scr);
    lv_label_set_text(board_info, "Waveshare ESP32-S3-Touch-LCD-5");
    lv_obj_set_style_text_color(board_info, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(board_info, &lv_font_montserrat_14, 0);
    lv_obj_align_to(board_info, title_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    label_status = lv_label_create(scr);
    lv_label_set_text(label_status, "Status: Initializing...");
    lv_obj_set_style_text_color(label_status, lv_color_hex(0xFFFF00), 0);
    lv_obj_set_style_text_font(label_status, &lv_font_montserrat_16, 0);
    lv_obj_align_to(label_status, board_info, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    label_current_mode = lv_label_create(scr);
    lv_obj_set_style_text_font(label_current_mode, &lv_font_montserrat_18, 0);
    lv_obj_align(label_current_mode, LV_ALIGN_CENTER, -100, -80);

    btn_step = lv_btn_create(scr);
    lv_obj_set_size(btn_step, 180, 80);
    lv_obj_align(btn_step, LV_ALIGN_CENTER, -150, -10);
    lv_obj_t *label_step = lv_label_create(btn_step);
    lv_label_set_text(label_step, "STEP");
    lv_obj_set_style_text_font(label_step, &lv_font_montserrat_18, 0);
    lv_obj_center(label_step);

    // Balance Mode Button
    btn_balance = lv_btn_create(scr);
    lv_obj_set_size(btn_balance, 180, 80);
    lv_obj_align(btn_balance, LV_ALIGN_CENTER, -150, 80);
    lv_obj_t *label_balance = lv_label_create(btn_balance);
    lv_label_set_text(label_balance, "BALANCE");
    lv_obj_set_style_text_font(label_balance, &lv_font_montserrat_18, 0);
    lv_obj_center(label_balance);


    lv_obj_t *countdown_title = lv_label_create(scr);
    lv_label_set_text(countdown_title, "Countdown Timer");
    lv_obj_set_style_text_color(countdown_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(countdown_title, &lv_font_montserrat_18, 0);
    lv_obj_align(countdown_title, LV_ALIGN_TOP_MID, 180, 80);

    countdown_slider = lv_slider_create(scr);
    lv_slider_set_range(countdown_slider, 0, 60);
    lv_obj_set_width(countdown_slider, 40);
    lv_obj_set_height(countdown_slider, 250);
    lv_obj_align(countdown_slider, LV_ALIGN_CENTER, 250, -20);
    lv_slider_set_value(countdown_slider, selected_countdown, LV_ANIM_OFF);

    lv_obj_set_style_bg_color(countdown_slider, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(countdown_slider, lv_color_hex(0x0066FF), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(countdown_slider, lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_set_style_width(countdown_slider, 25, LV_PART_KNOB);
    lv_obj_set_style_height(countdown_slider, 25, LV_PART_KNOB);
    lv_obj_set_style_radius(countdown_slider, 20, LV_PART_MAIN);
    lv_obj_set_style_radius(countdown_slider, 20, LV_PART_INDICATOR);
    lv_obj_set_style_radius(countdown_slider, 15, LV_PART_KNOB);

    label_countdown = lv_label_create(scr);
    lv_label_set_text_fmt(label_countdown, "%ds", selected_countdown);
    lv_obj_set_style_text_color(label_countdown, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(label_countdown, &lv_font_montserrat_22, 0);
    lv_obj_align_to(label_countdown, countdown_slider, LV_ALIGN_OUT_LEFT_MID, -20, 0);

    btn_start_countdown = lv_btn_create(scr);
    lv_obj_set_size(btn_start_countdown, 120, 60);
    lv_obj_align_to(btn_start_countdown, countdown_slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 30);
    lv_obj_t *label_start = lv_label_create(btn_start_countdown);
    lv_label_set_text(label_start, "START");
    lv_obj_set_style_text_font(label_start, &lv_font_montserrat_16, 0);
    lv_obj_center(label_start);
    lv_obj_set_style_bg_color(btn_start_countdown, lv_color_hex(0x00AA00), LV_PART_MAIN);
    lv_obj_set_style_radius(btn_start_countdown, 10, LV_PART_MAIN);

    label_timer_display = lv_label_create(scr);
    lv_label_set_text(label_timer_display, "Ready");
    lv_obj_set_style_text_color(label_timer_display, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(label_timer_display, &lv_font_montserrat_16, 0);
    lv_obj_align_to(label_timer_display, btn_start_countdown, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);

    lv_obj_t *info_label = lv_label_create(scr);
    lv_label_set_text(info_label, "Touch buttons to switch modes | Set countdown and start timer");
    lv_obj_set_style_text_color(info_label, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_font(info_label, &lv_font_montserrat_14, 0);
    lv_obj_align(info_label, LV_ALIGN_BOTTOM_MID, 0, -60);

    lv_obj_t *connect_info = lv_label_create(scr);
    lv_label_set_text(connect_info, "Device: ESP32_Mode_Controller | Waiting for Arduino R4...");
    lv_obj_set_style_text_color(connect_info, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(connect_info, &lv_font_montserrat_12, 0);
    lv_obj_align(connect_info, LV_ALIGN_BOTTOM_MID, 0, -30);

    lv_obj_t *version_label = lv_label_create(scr);
    lv_label_set_text_fmt(version_label, "LVGL %d.%d.%d | ESP32-S3 Dual-Core | 5inch Touch LCD",
                         LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);
    lv_obj_set_style_text_color(version_label, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_font(version_label, &lv_font_montserrat_12, 0);
    lv_obj_align(version_label, LV_ALIGN_BOTTOM_MID, 0, -10);


    lv_obj_add_event_cb(btn_step, [](lv_event_t *e) {
        if (current_mode != 1) {
            current_mode = 1;
            mode_changed = true;
            send_mode_to_arduino();
            update_mode_display();
            Serial.println("🎯 Switched to STEP mode");
        }
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(btn_balance, [](lv_event_t *e) {
        if (current_mode != 2) {
            current_mode = 2;
            mode_changed = true;
            send_mode_to_arduino();
            update_mode_display();
            Serial.println("🎯 Switched to BALANCE mode");
        }
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(countdown_slider, [](lv_event_t *e) {
        selected_countdown = lv_slider_get_value(countdown_slider);
        lv_label_set_text_fmt(label_countdown, "%ds", selected_countdown);
        Serial.printf("⏱️ Countdown set to: %d seconds\n", selected_countdown);
    }, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_add_event_cb(btn_start_countdown, [](lv_event_t *e) {
        if (selected_countdown > 0) {
            send_countdown_to_arduino(selected_countdown);

            lv_label_set_text(label_timer_display, "Sent to Arduino");
            lv_obj_set_style_text_color(label_timer_display, lv_color_hex(0x00FF00), 0);

            Serial.printf("⏰ Countdown sent to Arduino: %d seconds\n", selected_countdown);
        }
    }, LV_EVENT_CLICKED, NULL);

    update_mode_display();

    Serial.println("✅ Dual Mode UI for 5inch screen created successfully");
}

// ========== Setup ==========
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("========================================");
    Serial.println("🚀 Waveshare ESP32-S3 Dual Mode Controller");
    Serial.println("========================================");

    Serial.println("🔵 Step 1: Starting BLE initialization on Core 0...");
    xTaskCreatePinnedToCore(
        bleTask,
        "BLE_Init_Task",
        8192,
        NULL,
        2,
        NULL,
        0
    );

    Serial.println("📋 Step 2: Initialize Waveshare 5inch board...");
    board = new Board();
    board->init();

    auto lcd_panel = board->getLCD();
    lcd_panel->configFrameBufferNumber(2);

    #if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    auto lcd_bus = lcd_panel->getBus();
    if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
        static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd_panel->getFrameWidth() * 4);
    }
    #endif

    if (!board->begin()) {
        Serial.println("❌ Failed to initialize board!");
        while(1) {
            delay(1000);
        }
    }
    Serial.println("✅ Waveshare 5inch board initialized successfully");

    auto lcd_info = board->getLCD();
    Serial.printf("📺 LCD Resolution: %dx%d\n", lcd_info->getFrameWidth(), lcd_info->getFrameHeight());
    Serial.printf("🖥️ LCD Bus Type: %s\n",
                  (lcd_info->getBus()->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) ? "RGB" : "Other");

    delay(500);
    Serial.println("🎨 Step 3: Initialize LVGL...");
    if (!lvgl_port_init(board->getLCD(), board->getTouch())) {
        Serial.println("❌ Failed to initialize LVGL!");
        while(1) {
            delay(1000);
        }
    }
    Serial.println("✅ LVGL initialized successfully");

    delay(300);
    Serial.println("🎨 Step 4: Creating dual mode UI for 5inch screen...");
    lvgl_port_lock(-1);
    create_dual_mode_ui_large();
    lvgl_port_unlock();
    Serial.println("✅ Dual Mode UI created successfully");

    Serial.println("🎉 Setup completed!");
    Serial.println("📱 Device ready for Arduino R4 connection");
    Serial.println("🎮 Touch buttons to control modes and countdown");
}

// ========== Loop ==========
void loop()
{
    if (deviceConnected != oldDeviceConnected) {
        if (lvgl_port_lock(50)) {
            if (deviceConnected) {
                lv_label_set_text(label_status, "Status: Connected");
                lv_obj_set_style_text_color(label_status, lv_color_hex(0x00FF00), 0);
            } else {
                lv_label_set_text(label_status, "Status: Waiting...");
                lv_obj_set_style_text_color(label_status, lv_color_hex(0xFFFF00), 0);
            }
            lvgl_port_unlock();
        }
        oldDeviceConnected = deviceConnected;
    }

    static bool ui_updated = false;
    if (ble_initialized && !ui_updated && !deviceConnected) {
        if (lvgl_port_lock(50)) {
            lv_label_set_text(label_status, "Status: Ready");
            lv_obj_set_style_text_color(label_status, lv_color_hex(0x00AAFF), 0);
            lvgl_port_unlock();
            ui_updated = true;
        }
    }

    static uint32_t last_status_print = 0;
    if (millis() - last_status_print > 10000) {
        Serial.printf("🔄 Status: BLE %s, Mode: %s, Countdown: %ds, Free Heap: %d KB\n",
                     deviceConnected ? "Connected" : (ble_initialized ? "Ready" : "Init"),
                     current_mode == 1 ? "STEP" : "BALANCE",
                     selected_countdown,
                     ESP.getFreeHeap() / 1024);
        last_status_print = millis();
    }

    delay(20);
}

