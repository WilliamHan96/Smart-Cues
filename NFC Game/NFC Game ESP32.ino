/**
 * @brief    Smart Cues NFC game controller for ESP32 with PN532
 * @author   Smart Cues Group
 * @note     Handles NFC game interaction using a compact ESP32 and PN532 configuration.
 */


#include <stdio.h>
#include "BLE2902.h"
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"

#include "lvgl.h"
#include "demos/lv_demos.h"
#include "esp_lcd_sh8601.h"
#include "touch_bsp.h"

#include <Wire.h>
#include <Adafruit_PN532.h>
#include "esp_random.h"

#define SERVICE_UUID        "a9f238b1-7c3e-4f2c-9359-91b47f8e2fcb"
#define CHARACTERISTIC_UUID "d473cf0c-3e09-4811-bd6e-8fc3d42e6b26"

#define NFC_SDA_PIN 11
#define NFC_SCL_PIN 12
#define NFC_RST_PIN 10

#define BUZZER_PIN 3

TwoWire NFC_Wire = TwoWire(1);
Adafruit_PN532 nfc(-1, NFC_RST_PIN, &NFC_Wire);

bool nfcReady = false;

static const char *MAIN_TAG = "nfc_exercise_game";
static SemaphoreHandle_t main_lvgl_mux = NULL;
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;
bool motionDetected = false;
lv_obj_t* btn_yes = nullptr;
lv_obj_t* btn_no = nullptr;
lv_obj_t* label_status = nullptr;
lv_obj_t* main_screen = nullptr;
lv_obj_t* dialog_screen = nullptr;
lv_obj_t* game_screen = nullptr;
lv_obj_t* label_ble = nullptr;
static uint32_t dialogStartTime = 0;
static bool dialogVisible = false;
static bool shouldReturnToMain = false;


enum GameState {
  WAITING,
  DIALOG,
  PLAYING,
  COMPLETED
};

GameState currentGameState = WAITING;
uint8_t gameSequence[4];
int currentStep = 0;
lv_obj_t* sequence_labels[4];
lv_obj_t* progress_label = nullptr;
lv_obj_t* instruction_label = nullptr;

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

uint8_t getNFCCardNumber(uint8_t* uid, uint8_t uidLength) {
  if (uidLength < 4) return 0;

  for (int i = 0; i < 4; i++) {
    bool match = true;
    for (int j = 0; j < 4; j++) {
      if (uid[j] != nfcCards[i].uid[j]) {
        match = false;
        break;
      }
    }
    if (match) {
      Serial.printf("🎯 Recognized card %d\n", nfcCards[i].number);
      return nfcCards[i].number;
    }
  }

  Serial.print("❓ Unknown card: ");
  for (int i = 0; i < uidLength; i++) {
    if (uid[i] < 0x10) Serial.print("0");
    Serial.print(uid[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  return 0;
}

// ========== Hardware Configuration ==========
#define LCD_HOST    SPI2_HOST
#define TOUCH_HOST  I2C_NUM_0
#define LCD_BIT_PER_PIXEL       (16)

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_LCD_CS            (GPIO_NUM_6)
#define EXAMPLE_PIN_NUM_LCD_PCLK          (GPIO_NUM_47)
#define EXAMPLE_PIN_NUM_LCD_DATA0         (GPIO_NUM_18)
#define EXAMPLE_PIN_NUM_LCD_DATA1         (GPIO_NUM_7)
#define EXAMPLE_PIN_NUM_LCD_DATA2         (GPIO_NUM_48)
#define EXAMPLE_PIN_NUM_LCD_DATA3         (GPIO_NUM_5)
#define EXAMPLE_PIN_NUM_LCD_RST           (GPIO_NUM_17)
#define EXAMPLE_PIN_NUM_BK_LIGHT          (-1)

#define EXAMPLE_LCD_H_RES              536
#define EXAMPLE_LCD_V_RES              240

#define EXAMPLE_USE_TOUCH              1
#define EXAMPLE_LVGL_BUF_HEIGHT        (EXAMPLE_LCD_V_RES/4)
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

// Function declarations
void setup_ble_server();
void show_motion_dialog();
void hide_dialog();
void start_game();
void end_game();
void check_nfc_game();
void show_game_screen();
void update_sequence_display();
void flash_red_sequence(int index);
static bool example_lvgl_lock(int timeout_ms);
static void example_lvgl_unlock(void);

// ========== BLE Client Callbacks ==========
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    deviceConnected = true;
    Serial.println("🔗 Connected to Arduino!");
  }

  void onDisconnect(BLEClient* pclient) {
    deviceConnected = false;
    Serial.println("❌ Disconnected from Arduino!");
  }
};

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                          uint8_t* pData, size_t length, bool isNotify) {
  motionDetected = true;
}

// ========== LCD Initialization Commands ==========
static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
  {0x11, (uint8_t []){0x00}, 0, 120},
  {0x36, (uint8_t []){0xF0}, 1, 0},
  {0x3A, (uint8_t []){0x55}, 1, 0},
  {0x2A, (uint8_t []){0x00,0x00,0x02,0x17}, 4, 0},
  {0x2B, (uint8_t []){0x00,0x00,0x00,0xEF}, 4, 0},
  {0x51, (uint8_t []){0x00}, 1, 10},
  {0x29, (uint8_t []){0x00}, 0, 10},
  {0x51, (uint8_t []){0xFF}, 1, 0},
};

// ========== LVGL Callback Functions ==========
static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
  lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
  lv_disp_flush_ready(disp_driver);
  return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
  const int offsetx1 = area->x1;
  const int offsetx2 = area->x2;
  const int offsety1 = area->y1;
  const int offsety2 = area->y2;

  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void example_lvgl_update_cb(lv_disp_drv_t *drv)
{
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

  switch (drv->rotated)
  {
    case LV_DISP_ROT_NONE:
      esp_lcd_panel_swap_xy(panel_handle, false);
      esp_lcd_panel_mirror(panel_handle, true, false);
      break;
    case LV_DISP_ROT_90:
      esp_lcd_panel_swap_xy(panel_handle, true);
      esp_lcd_panel_mirror(panel_handle, true, true);
      break;
    case LV_DISP_ROT_180:
      esp_lcd_panel_swap_xy(panel_handle, false);
      esp_lcd_panel_mirror(panel_handle, false, true);
      break;
    case LV_DISP_ROT_270:
      esp_lcd_panel_swap_xy(panel_handle, true);
      esp_lcd_panel_mirror(panel_handle, false, false);
      break;
  }
}

void example_lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
  uint16_t x1 = area->x1;
  uint16_t x2 = area->x2;
  uint16_t y1 = area->y1;
  uint16_t y2 = area->y2;

  area->x1 = (x1 >> 1) << 1;
  area->y1 = (y1 >> 1) << 1;
  area->x2 = ((x2 >> 1) << 1) + 1;
  area->y2 = ((y2 >> 1) << 1) + 1;
}

#if EXAMPLE_USE_TOUCH
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
  uint16_t tp_x,tp_y;
  uint8_t win;
  win = getTouch(&tp_x,&tp_y);
  if (win)
  {
    data->point.x = tp_x;
    data->point.y = tp_y;
    data->state = LV_INDEV_STATE_PRESSED;
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}
#endif

static void example_increase_lvgl_tick(void *arg)
{
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static bool example_lvgl_lock(int timeout_ms)
{
  assert(main_lvgl_mux && "display must be initialized first");
  const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  return xSemaphoreTake(main_lvgl_mux, timeout_ticks) == pdTRUE;
}

static void example_lvgl_unlock(void)
{
  assert(main_lvgl_mux && "display must be initialized first");
  xSemaphoreGive(main_lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
  ESP_LOGI(MAIN_TAG, "Starting LVGL task");
  uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
  while (1)
  {
    if (example_lvgl_lock(-1)) {
        task_delay_ms = lv_timer_handler();
        example_lvgl_unlock();
    }
    if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
        task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
        task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
    }
    vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
  }
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

// ========== UI Creation Functions ==========
void create_main_ui() {
    // Create main screen
    main_screen = lv_obj_create(NULL);
    lv_scr_load(main_screen);

    // Set background color
    lv_obj_set_style_bg_color(main_screen, lv_color_hex(0x0F0F0F), LV_PART_MAIN);

    // Title label
    lv_obj_t *title_label = lv_label_create(main_screen);
    lv_label_set_text(title_label, "NFC Exercise Game");
    lv_obj_set_style_text_color(title_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 10);


    label_ble = lv_label_create(main_screen);
    lv_label_set_text(label_ble, "Status: Waiting...");
    lv_obj_set_style_text_color(label_ble, lv_color_hex(0x00FFFF), 0);
    lv_obj_set_style_text_font(label_ble, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(label_ble, LV_ALIGN_BOTTOM_LEFT, 10, -10);

    // Info label
    lv_obj_t *info_label = lv_label_create(main_screen);
    lv_label_set_text(info_label, "Waiting for motion detection...");
    lv_obj_set_style_text_color(info_label, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_font(info_label, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(info_label, LV_ALIGN_CENTER, 0, 20);

    Serial.println("✅ Main UI created");
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
    lv_obj_t *title_label = lv_label_create(game_screen);
    lv_label_set_text(title_label, "NFC Exercise Game");
    lv_obj_set_style_text_color(title_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 10);

    // Instruction
    instruction_label = lv_label_create(game_screen);
    lv_label_set_text(instruction_label, "Tag cards in this order:");
    lv_obj_set_style_text_color(instruction_label, lv_color_hex(0xFFFF00), 0);
    lv_obj_align(instruction_label, LV_ALIGN_TOP_MID, 0, 50);

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
    lv_obj_align(progress_label, LV_ALIGN_BOTTOM_MID, 0, -40);

    // Back button
    lv_obj_t *back_btn = lv_btn_create(game_screen);
    lv_obj_set_size(back_btn, 140, 60);
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x666666), LV_PART_MAIN);
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

void show_game_cancelled() {
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
        create_game_ui();
        create_dialog_ui();
        lv_timer_del(timer);
    }, 3000, NULL);
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

            static lv_timer_t* reset_timer = nullptr;
            if (reset_timer) lv_timer_del(reset_timer);
            reset_timer = lv_timer_create([](lv_timer_t* timer) {
                create_game_ui();
                lv_timer_del(timer);
            }, 100, NULL);
        }, LV_EVENT_CLICKED, NULL);

        example_lvgl_unlock();
    }

    currentGameState = COMPLETED;

    if (deviceConnected && pCharacteristic) {
        pCharacteristic->setValue("game_completed");
        pCharacteristic->notify();
        Serial.println("📡 Notified Arduino: game_completed");
    }
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

void play_success_beep() {
  tone(BUZZER_PIN, 1000, 100);
}

void play_error_beep() {
  for (int i = 0; i < 2; i++) {
    tone(BUZZER_PIN, 300);      // 300Hz
    delay(150);
    noTone(BUZZER_PIN);
    delay(100);
  }
}


void init_nfc() {
    NFC_Wire.begin(NFC_SDA_PIN, NFC_SCL_PIN);
    nfc.begin();

    uint32_t versiondata = nfc.getFirmwareVersion();
    if (!versiondata) {
        Serial.println("❌ Didn't find PN532 NFC module");
        nfcReady = false;
        return;
    }

    Serial.print("✅ Found PN532 with firmware version: 0x");
    Serial.println((versiondata >> 16) & 0xFF, HEX);

    nfc.SAMConfig();
    nfcReady = true;
    Serial.println("📡 NFC module ready");
}

void check_nfc_game() {
    if (!nfcReady || currentGameState != PLAYING) return;

    uint8_t uid[7];
    uint8_t uidLength;

    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength)) {
        uint8_t cardNumber = getNFCCardNumber(uid, uidLength);

        Serial.print("🎯 NFC Card detected: ");
        for (int i = 0; i < uidLength; i++) {
            if (uid[i] < 0x10) Serial.print("0");
            Serial.print(uid[i], HEX); Serial.print(" ");
        }
        Serial.printf(" -> Card Number: %d\n", cardNumber);

        if (cardNumber == 0) {
            Serial.println("❓ Unknown card - please use registered cards only");
            delay(1000);
            return;
        }

        if (cardNumber == gameSequence[currentStep]) {
            Serial.printf("✅ Correct! Step %d completed (expected %d, got %d)\n",
                         currentStep + 1, gameSequence[currentStep], cardNumber);

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
            Serial.printf("❌ Wrong card! Expected %d, got %d\n",
                         gameSequence[currentStep], cardNumber);
            play_error_beep();
            flash_red_sequence(currentStep);
        }

        delay(1000);
    }
}


// ========== Arduino Setup ==========
void setup()
{
  static lv_disp_draw_buf_t disp_buf;
  static lv_disp_drv_t disp_drv;


  Serial.begin(115200);
  ESP_LOGI(MAIN_TAG, "Starting NFC Exercise Game Controller...");

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(MAIN_TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

  ESP_LOGI(MAIN_TAG, "Initialize SPI bus");
  const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK,
                                                               EXAMPLE_PIN_NUM_LCD_DATA0,
                                                               EXAMPLE_PIN_NUM_LCD_DATA1,
                                                               EXAMPLE_PIN_NUM_LCD_DATA2,
                                                               EXAMPLE_PIN_NUM_LCD_DATA3,
                                                               EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * LCD_BIT_PER_PIXEL / 8);
  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

  ESP_LOGI(MAIN_TAG, "Install panel IO");
  esp_lcd_panel_io_handle_t io_handle = NULL;
  const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS,
                                                                                example_notify_lvgl_flush_ready,
                                                                                &disp_drv);
  sh8601_vendor_config_t vendor_config = {
      .init_cmds = lcd_init_cmds,
      .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
      .flags = {
          .use_qspi_interface = 1,
      },
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

  esp_lcd_panel_handle_t panel_handle = NULL;
  const esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
      .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
      .bits_per_pixel = LCD_BIT_PER_PIXEL,
      .vendor_config = &vendor_config,
  };
  ESP_LOGI(MAIN_TAG, "Install SH8601 panel driver");
  ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if EXAMPLE_USE_TOUCH
  Touch_Init();
#endif

  ESP_LOGI(MAIN_TAG, "Initialize LVGL library");
  lv_init();
  lv_color_t *buf1 = (lv_color_t*)heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
  assert(buf1);
  lv_color_t *buf2 = (lv_color_t*)heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
  assert(buf2);
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT);

  ESP_LOGI(MAIN_TAG, "Register display driver to LVGL");
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = EXAMPLE_LCD_H_RES;
  disp_drv.ver_res = EXAMPLE_LCD_V_RES;
  disp_drv.flush_cb = example_lvgl_flush_cb;
  disp_drv.rounder_cb = example_lvgl_rounder_cb;
  disp_drv.drv_update_cb = example_lvgl_update_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;
  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

  ESP_LOGI(MAIN_TAG, "Install LVGL tick timer");
  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &example_increase_lvgl_tick,
      .name = "lvgl_tick"
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

#if EXAMPLE_USE_TOUCH
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    lv_indev_drv_register(&indev_drv);
#endif

  main_lvgl_mux = xSemaphoreCreateMutex();
  assert(main_lvgl_mux);
  xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

  setup_ble_server();

  if (example_lvgl_lock(-1)) {
    create_main_ui();
    create_dialog_ui();
    create_game_ui();
    example_lvgl_unlock();
    Serial.println("✅ All UI screens created");
  } else {
    Serial.println("❌ Failed to create UI screens");
  }

  init_nfc();

  ESP_LOGI(MAIN_TAG, "Setup completed successfully");
}

// ========== Arduino Loop ==========
void loop()
{
  static uint32_t lastStatusTime = 0;
  uint32_t currentTime = millis();

  lv_timer_handler();

  if (motionDetected) {
    motionDetected = false;
    Serial.println("📡 Motion detected! Showing dialog...");
    show_motion_dialog();
  }

  if (currentGameState == PLAYING) {
    check_nfc_game();
  }

  if (currentTime - lastStatusTime > 5000) {
    lastStatusTime = currentTime;
    Serial.print("Status: ");
    Serial.print(deviceConnected ? "Connected" : "Waiting for Arduino");
    Serial.print(", Game State: ");
    switch(currentGameState) {
      case WAITING: Serial.print("Waiting"); break;
      case DIALOG: Serial.print("Dialog"); break;
      case PLAYING: Serial.print("Playing"); break;
      case COMPLETED: Serial.print("Completed"); break;
    }
    Serial.print(", Free heap: ");
    Serial.println(ESP.getFreeHeap());
  }

  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();

    if (command == "test") {
      Serial.println("🧪 Simulating motion detection...");
      show_motion_dialog();
    } else if (command == "game") {
      Serial.println("🎮 Starting test game...");
      start_game();
    } else if (command == "cards") {
      Serial.println("📋 Registered NFC Cards:");
      for (int i = 0; i < 4; i++) {
        Serial.printf("  Card %d: ", nfcCards[i].number);
        for (int j = 0; j < 4; j++) {
          if (nfcCards[i].uid[j] < 0x10) Serial.print("0");
          Serial.print(nfcCards[i].uid[j], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
    } else if (command == "scan") {
      Serial.println("🔍 Scanning for NFC cards (any game state)...");
      uint8_t uid[7];
      uint8_t uidLength;

      if (nfcReady && nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength)) {
        uint8_t cardNumber = getNFCCardNumber(uid, uidLength);
        Serial.print("Found card: ");
        for (int i = 0; i < uidLength; i++) {
          if (uid[i] < 0x10) Serial.print("0");
          Serial.print(uid[i], HEX); Serial.print(" ");
        }
        Serial.printf(" -> Card Number: %d\n", cardNumber);
      } else {
        Serial.println("No card detected or NFC not ready");
      }
      if (currentGameState == PLAYING) {
        Serial.println("🧪 Simulating NFC card 1...");
        uint8_t testUID[] = {0x04, 0x52, 0x3A, 0x01};
        uint8_t cardNumber = gameSequence[currentStep];

        Serial.printf("🎯 Simulated NFC Card -> Number: %d\n", cardNumber);

        if (cardNumber == gameSequence[currentStep]) {
          Serial.printf("✅ Correct! Step %d completed\n", currentStep + 1);
          currentStep++;

          if (currentStep >= 4) {
            Serial.println("🎉 All steps completed!");
            show_congratulations();
          } else {
            if (example_lvgl_lock(100)) {
              update_sequence_display();
              example_lvgl_unlock();
            }
          }
        }
      } else {
        Serial.println("❌ Not in game mode. Type 'game' first.");
      }
    } else if (command == "status") {
      Serial.print("📊 Status - Connected: ");
      Serial.print(deviceConnected ? "YES" : "NO");
      Serial.print(", Game State: ");
      switch(currentGameState) {
        case WAITING: Serial.print("WAITING"); break;
        case DIALOG: Serial.print("DIALOG"); break;
        case PLAYING: Serial.print("PLAYING"); break;
        case COMPLETED: Serial.print("COMPLETED"); break;
      }
      Serial.print(", Free heap: ");
      Serial.println(ESP.getFreeHeap());
    }
  }

  if (dialogVisible && millis() - dialogStartTime > 5000) {
      Serial.println("⌛ Auto-hide dialog after 5s timeout");
      hide_dialog();
  }

  delay(100);
}
