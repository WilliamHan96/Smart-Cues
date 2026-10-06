/**
 ******************************************************************************
 * @file     LVGL_BLE_Slider.ino
 * @author   Modified for BLE Slider Control
 * @version  V1.1
 * @date     2024-12-05
 * @brief    LVGL display with BLE slider control
 * @license  MIT
 * @copyright Copyright (c) 2024, Waveshare
 ******************************************************************************
 * 
 * Experiment Objective: LVGL display with BLE slider to send data to Arduino R4 WiFi
 *
 * Hardware Resources and Pin Assignment: 
 * 1. Display Interface --> ESP32 Module (SPI/QSPI)
 * 2. Touch Interface --> ESP32 Module (I2C)
 * 3. BLE --> Send slider values to Arduino R4 WiFi
 *
 * Experiment Phenomenon:
 * 1. Display shows a slider control interface
 * 2. Touch interaction changes slider value
 * 3. BLE sends updated values to Arduino R4 WiFi
 * 
 ******************************************************************************
 */

#include <stdio.h>
// 使用ESP32经典BLE库，不是NimBLE
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

// ========== BLE Configuration ==========
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

// ========== Global Variables ==========
static const char *MAIN_TAG = "ble_slider_main";
static SemaphoreHandle_t main_lvgl_mux = NULL;

// BLE related variables
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t current_slider_value = 50;  // Default slider value (0-100)
bool value_changed = false;
uint32_t last_sent_time = 0;

// LVGL UI objects
lv_obj_t* slider_obj = nullptr;
lv_obj_t* label_value = nullptr;
lv_obj_t* label_status = nullptr;

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

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              536    
#define EXAMPLE_LCD_V_RES              240

#define EXAMPLE_USE_TOUCH              1 //Without tp ---- Touch off

#define EXAMPLE_LVGL_BUF_HEIGHT        (EXAMPLE_LCD_V_RES/4)
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

// Function declarations
void create_ble_slider_ui();
void setup_ble_server();
static bool example_lvgl_lock(int timeout_ms);
static void example_lvgl_unlock(void);

// ========== BLE Server Callbacks ==========
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("🔗 BLE Client connected!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("❌ BLE Client disconnected!");
      // 重启广播 - 经典BLE库的方式
      pServer->getAdvertising()->start();
      Serial.println("🔄 Restarted advertising");
    }
};

// ========== LCD Initialization Commands ==========
static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
  {0x11, (uint8_t []){0x00}, 0, 120},   
  {0x36, (uint8_t []){0xF0}, 1, 0},   
  {0x3A, (uint8_t []){0x55}, 1, 0},  //16bits-RGB565
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

  // copy a buffer's content to a specific area of the display
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
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

  // round the start of coordinate down to the nearest 2M number
  area->x1 = (x1 >> 1) << 1;
  area->y1 = (y1 >> 1) << 1;
  // round the end of coordinate up to the nearest 2N+1 number
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
  /* Tell LVGL how many milliseconds has elapsed */
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
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1)) {
        task_delay_ms = lv_timer_handler();
        // Release the mutex
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

// ========== BLE Setup ==========
void setup_ble_server() {
    Serial.println("🔵 Initializing BLE...");
    
    // 确保BLE配置正确
    if (!btStart()) {
        Serial.println("❌ Failed to initialize BT controller");
        return;
    }
    
    // 初始化BLE设备 - 使用更明确的方式
    BLEDevice::init("ESP32_Slider_Control");
    Serial.println("✅ BLE Device initialized");
    
    // 短暂延迟确保初始化完成
    delay(100);
    
    // 创建BLE服务器
    pServer = BLEDevice::createServer();
    if (!pServer) {
        Serial.println("❌ Failed to create BLE server");
        return;
    }
    pServer->setCallbacks(new MyServerCallbacks());
    Serial.println("✅ BLE Server created");
    
    // 创建BLE服务
    BLEService *pService = pServer->createService(SERVICE_UUID);
    if (!pService) {
        Serial.println("❌ Failed to create BLE service");
        return;
    }
    Serial.println("✅ BLE Service created");
    
    // 创建BLE特征
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
    
    // 添加描述符用于通知
    pCharacteristic->addDescriptor(new BLE2902());
    Serial.println("✅ BLE Characteristic created");
    
    // 设置初始值
    String initialValue = String(current_slider_value);
    pCharacteristic->setValue(initialValue.c_str());
    
    // 启动服务
    pService->start();
    Serial.println("✅ BLE Service started");
    
    // 开始广播
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    if (!pAdvertising) {
        Serial.println("❌ Failed to get advertising");
        return;
    }
    
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    
    // 启动广播
    if (pAdvertising->start()) {
        Serial.println("🎯 BLE Server ready!");
        Serial.println("📱 Device name: ESP32_Slider_Control");
        Serial.println("🔗 Waiting for connections...");
    } else {
        Serial.println("❌ Failed to start advertising");
    }
}

// ========== UI Creation ==========
void create_ble_slider_ui() {
    // Create new screen
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_scr_load(scr);
    
    // Set background color
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0F0F0F), LV_PART_MAIN);
    
    // Title label
    lv_obj_t *title_label = lv_label_create(scr);
    lv_label_set_text(title_label, "BLE Slider Control");
    lv_obj_set_style_text_color(title_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 10);
    
    // BLE status label
    label_status = lv_label_create(scr);
    lv_label_set_text(label_status, "BLE: Initializing...");
    lv_obj_set_style_text_color(label_status, lv_color_hex(0xFFFF00), 0);
    lv_obj_align(label_status, LV_ALIGN_TOP_MID, 0, 35);
    
    // Slider
    slider_obj = lv_slider_create(scr);
    lv_slider_set_range(slider_obj, 0, 100);
    lv_obj_set_width(slider_obj, 300);
    lv_obj_set_height(slider_obj, 20);
    lv_obj_align(slider_obj, LV_ALIGN_CENTER, 0, -10);
    lv_slider_set_value(slider_obj, current_slider_value, LV_ANIM_OFF);
    
    // Style the slider
    lv_obj_set_style_bg_color(slider_obj, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider_obj, lv_color_hex(0x00AAFF), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider_obj, lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    
    // Value display label
    label_value = lv_label_create(scr);
    lv_label_set_text_fmt(label_value, "Value: %d", current_slider_value);
    lv_obj_set_style_text_color(label_value, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align_to(label_value, slider_obj, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);
    
    // Info label
    lv_obj_t *info_label = lv_label_create(scr);
    lv_label_set_text(info_label, "Slide to send data via BLE");
    lv_obj_set_style_text_color(info_label, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(info_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    // Add slider event callback
    lv_obj_add_event_cb(slider_obj, [](lv_event_t *e) {
        lv_obj_t *slider = lv_event_get_target(e);
        current_slider_value = lv_slider_get_value(slider);
        lv_label_set_text_fmt(label_value, "Value: %d", current_slider_value);
        value_changed = true;
        last_sent_time = millis();
        ESP_LOGI(MAIN_TAG, "Slider value changed to: %d", current_slider_value);
    }, LV_EVENT_VALUE_CHANGED, NULL);
    
    ESP_LOGI(MAIN_TAG, "BLE Slider UI created successfully");
}

// ========== Arduino Setup ==========
void setup()
{
  static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
  static lv_disp_drv_t disp_drv;      // contains callback functions
  
  // Initialize serial for debugging
  Serial.begin(115200);
  ESP_LOGI(MAIN_TAG, "Starting BLE Slider Demo...");
  
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
  // Attach the LCD to the SPI bus
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
  // alloc draw buffers used by LVGL
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
    static lv_indev_drv_t indev_drv;    // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    lv_indev_drv_register(&indev_drv);
#endif

  main_lvgl_mux = xSemaphoreCreateMutex();
  assert(main_lvgl_mux);
  xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
  
  // Initialize BLE first
  setup_ble_server();
  
  // Create UI and start
  if (example_lvgl_lock(-1)) {
    create_ble_slider_ui();
    // Update initial status after BLE setup
    lv_label_set_text(label_status, "BLE: Advertising");
    lv_obj_set_style_text_color(label_status, lv_color_hex(0x00AAFF), 0);
    example_lvgl_unlock();
  }
  
  ESP_LOGI(MAIN_TAG, "Setup completed successfully");
}

// ========== Arduino Loop ==========
void loop()
{
  // Send BLE data when slider value changes
  if (value_changed && millis() - last_sent_time > 100 && deviceConnected && pCharacteristic) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%d", current_slider_value);
    pCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    pCharacteristic->notify();
    value_changed = false;
    ESP_LOGI(MAIN_TAG, "Sent BLE data: %s", buffer);
  }
  
  // Update BLE status on UI periodically
  static uint32_t last_status_update = 0;
  if (millis() - last_status_update > 5000) {  // Update every 5 seconds
    if (example_lvgl_lock(10)) {
      if (deviceConnected) {
        lv_label_set_text(label_status, "BLE: Connected");
        lv_obj_set_style_text_color(label_status, lv_color_hex(0x00FF00), 0);
      } else {
        lv_label_set_text(label_status, "BLE: Waiting...");
        lv_obj_set_style_text_color(label_status, lv_color_hex(0xFFFF00), 0);
      }
      example_lvgl_unlock();
    }
    last_status_update = millis();
  }
  
  vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent watchdog issues
}