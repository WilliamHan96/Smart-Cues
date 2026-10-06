#include <lvgl.h>
#include "esp_lcd_sh8601.h"
#include "touch_bsp.h"
#include <ArduinoBLE.h>

#define SCREEN_WIDTH  536
#define SCREEN_HEIGHT 240
#define DEFAULT_INTERVAL 1000

lv_obj_t *ta;
lv_obj_t *label;
BLEDevice central;
BLECharacteristic intervalChar;
BLEService controlService("12345678-1234-1234-1234-1234567890ab");
BLECharacteristic intervalCharacteristic("12345678-1234-1234-1234-1234567890ac", BLEWrite, 20);

// 初始化 LVGL 相关任务
void lv_port_init();
void lv_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data);
void lv_tick_handler_task(void *arg);

void send_interval_to_arduino(uint32_t val) {
  if (central && intervalCharacteristic.subscribed()) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu", val);
    intervalCharacteristic.writeValue(buf);
  }
}

void on_confirm_event(lv_event_t *e) {
  const char *text = lv_textarea_get_text(ta);
  uint32_t val = atoi(text);
  if (val > 0 && val <= 10000) {
    send_interval_to_arduino(val);
    lv_label_set_text(label, "Updated");
  } else {
    lv_label_set_text(label, "Invalid (10-10000)");
  }
}

void lv_create_ui() {
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);

  label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Set interval (ms):");
  lv_obj_set_style_text_color(label, lv_color_white(), 0);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);

  ta = lv_textarea_create(lv_scr_act());
  lv_textarea_set_one_line(ta, true);
  lv_textarea_set_placeholder_text(ta, "1000");
  lv_obj_set_width(ta, 100);
  lv_obj_align(ta, LV_ALIGN_CENTER, 0, 0);

  lv_obj_t *btn = lv_btn_create(lv_scr_act());
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, 40);
  lv_obj_add_event_cb(btn, on_confirm_event, LV_EVENT_CLICKED, NULL);

  lv_obj_t *btn_label = lv_label_create(btn);
  lv_label_set_text(btn_label, "Confirm");
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  lv_init();
  lv_port_init();
  lv_create_ui();

  BLE.begin();
  BLE.setLocalName("ESP32_Controller");
  BLE.setAdvertisedService(controlService);
  controlService.addCharacteristic(intervalCharacteristic);
  BLE.addService(controlService);
  intervalCharacteristic.setValue("1000");
  BLE.advertise();
  Serial.println("BLE ready");
}

void loop() {
  lv_timer_handler();
  delay(5);

  if (!central) {
    central = BLE.central();
    if (central) {
      Serial.println("Connected to central");
    }
  }

  if (central && !central.connected()) {
    Serial.println("Disconnected");
    central = BLEDevice();
  }
}

// ============ 屏幕和触摸相关函数 ============

#define EXAMPLE_LCD_H_RES SCREEN_WIDTH
#define EXAMPLE_LCD_V_RES SCREEN_HEIGHT

#define LCD_HOST SPI2_HOST
#define EXAMPLE_PIN_NUM_LCD_CS    (GPIO_NUM_6)
#define EXAMPLE_PIN_NUM_LCD_PCLK  (GPIO_NUM_47) 
#define EXAMPLE_PIN_NUM_LCD_DATA0 (GPIO_NUM_18)
#define EXAMPLE_PIN_NUM_LCD_DATA1 (GPIO_NUM_7)
#define EXAMPLE_PIN_NUM_LCD_DATA2 (GPIO_NUM_48)
#define EXAMPLE_PIN_NUM_LCD_DATA3 (GPIO_NUM_5)
#define EXAMPLE_PIN_NUM_LCD_RST   (GPIO_NUM_17)

esp_lcd_panel_handle_t panel_handle = NULL;
SemaphoreHandle_t lvgl_mux = NULL;

static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
  esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
  lv_disp_flush_ready(drv);
}

void lv_port_init() {
  lvgl_mux = xSemaphoreCreateMutex();

  const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK,
                      EXAMPLE_PIN_NUM_LCD_DATA0,
                      EXAMPLE_PIN_NUM_LCD_DATA1,
                      EXAMPLE_PIN_NUM_LCD_DATA2,
                      EXAMPLE_PIN_NUM_LCD_DATA3,
                      EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * 2);
  spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);

  esp_lcd_panel_io_handle_t io_handle = NULL;
  const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS, NULL, NULL);
  esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle);

  static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0x11, (uint8_t[]){0x00}, 0, 120},
    {0x36, (uint8_t[]){0x60}, 1, 0}, // 横屏（屏幕翻转）
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x2A, (uint8_t[]){0x00,0x00,0x02,0x17}, 4, 0},
    {0x2B, (uint8_t[]){0x00,0x00,0x00,0xEF}, 4, 0},
    {0x29, (uint8_t[]){0x00}, 0, 10},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
  };
  sh8601_vendor_config_t vendor_config = {
    .init_cmds = lcd_init_cmds,
    .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
    .flags = { .use_qspi_interface = 1 }
  };
  const esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
    .bits_per_pixel = 16,
    .vendor_config = &vendor_config,
  };
  esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle);
  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);
  esp_lcd_panel_disp_on_off(panel_handle, true);

  static lv_disp_draw_buf_t draw_buf;
  static lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(EXAMPLE_LCD_H_RES * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA);
  static lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(EXAMPLE_LCD_H_RES * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA);
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 40);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = EXAMPLE_LCD_H_RES;
  disp_drv.ver_res = EXAMPLE_LCD_V_RES;
  disp_drv.flush_cb = flush_cb;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.user_data = panel_handle;
  lv_disp_drv_register(&disp_drv);

  Touch_Init();

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lv_touch_read;
  lv_indev_drv_register(&indev_drv);

  xTaskCreatePinnedToCore(lv_tick_handler_task, "lv_tick", 2048, NULL, 1, NULL, 1);
}

void lv_tick_handler_task(void *arg) {
  while (1) {
    lv_tick_inc(5);
    delay(5);
  }
}

void lv_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  uint16_t x, y;
  if (getTouch(&x, &y)) {
    data->point.x = x;
    data->point.y = y;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}
