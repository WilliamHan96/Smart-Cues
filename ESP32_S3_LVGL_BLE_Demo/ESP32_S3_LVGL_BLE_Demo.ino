
#include <lvgl.h>
#include <Wire.h>
#include <SPI.h>
#include "esp_lcd_sh8601.h"
#include "touch_bsp.h"
#include <ArduinoBLE.h>

#define SCREEN_HOR_RES 536
#define SCREEN_VER_RES 240

// BLE
BLEDevice central;
BLEService controlService("180A");
BLECharacteristic intervalChar("2A6E", BLEWrite, 4);

uint32_t lastUpdate = 0;
uint32_t updateDelay = 1000;
bool sliderChanged = false;

lv_obj_t *label;
lv_obj_t *slider;

// -------------------- LVGL 相关 --------------------
void flush_cb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint16_t w = area->x2 - area->x1 + 1;
  uint16_t h = area->y2 - area->y1 + 1;
  lcd_draw_area(area->x1, area->y1, w, h, (uint16_t *)color_p);
  lv_disp_flush_ready(disp);
}

void setup_ui() {
  lv_obj_clean(lv_scr_act());

  slider = lv_slider_create(lv_scr_act());
  lv_obj_set_width(slider, 400);
  lv_obj_align(slider, LV_ALIGN_CENTER, 0, 40);
  lv_slider_set_range(slider, 100, 5000);
  lv_slider_set_value(slider, 1000, LV_ANIM_OFF);

  label = lv_label_create(lv_scr_act());
  lv_obj_align(label, LV_ALIGN_CENTER, 0, -40);
  lv_label_set_text(label, "Interval: 1000 ms");

  lv_obj_add_event_cb(slider, [](lv_event_t *e) {
    int val = lv_slider_get_value(slider);
    static char buf[32];
    snprintf(buf, sizeof(buf), "Interval: %d ms", val);
    lv_label_set_text(label, buf);
    sliderChanged = true;
    lastUpdate = millis();
  }, LV_EVENT_VALUE_CHANGED, NULL);
}

void setup() {
  Serial.begin(115200);
  lcd_init();
  touch_init();

  lv_init();
  static lv_disp_draw_buf_t draw_buf;
  static lv_color_t buf[SCREEN_HOR_RES * 40];
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, SCREEN_HOR_RES * 40);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_HOR_RES;
  disp_drv.ver_res = SCREEN_VER_RES;
  disp_drv.flush_cb = flush_cb;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touch_read;
  lv_indev_drv_register(&indev_drv);

  setup_ui();

  // BLE 初始化
  if (!BLE.begin()) {
    Serial.println("BLE init failed!");
    while (1);
  }
  BLE.setLocalName("ESP32S3-Controller");
  BLE.setAdvertisedService(controlService);
  controlService.addCharacteristic(intervalChar);
  BLE.addService(controlService);
  BLE.advertise();
  Serial.println("BLE ready");
}

void loop() {
  lv_timer_handler();
  delay(5);

  BLEDevice central = BLE.central();
  if (central) {
    while (central.connected()) {
      lv_timer_handler();
      delay(5);
      if (sliderChanged && millis() - lastUpdate > updateDelay) {
        int val = lv_slider_get_value(slider);
        uint8_t data[4];
        data[0] = val & 0xFF;
        data[1] = (val >> 8) & 0xFF;
        data[2] = 0;
        data[3] = 0;
        intervalChar.writeValue(data, 4);
        sliderChanged = false;
        Serial.print("Sent via BLE: ");
        Serial.println(val);
      }
    }
  }
}
