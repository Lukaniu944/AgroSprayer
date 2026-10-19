#include <Arduino.h>
#include "config.h"
#include "board_config.h"
#include "hw_i2c.h"
#include "io_extension.h"
#include "lcd_driver.h"
#include "gt911.h"
#include "lvgl_port.h"
#include "flowmeter.h"
#include "pressure_sensor.h"
#include "valve_test.h"
#include "esp_log.h"
#include <stdarg.h>
#include "aog.h"
#include "ui.h"

static esp_lcd_panel_handle_t panel = nullptr;
static uint32_t lastUi = 0;
// GPIO43/44 are now L298N controls; do not send runtime logs on UART0.
static int discard_runtime_log(const char *, va_list) { return 0; }

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("AgroSprayer V11 - LVGL 9.6");

    if(!hw_i2c_begin()) while(true){ Serial.println("BLAD I2C"); delay(1000); }
    if(!io_extension_begin()) while(true){ Serial.println("BLAD IOX"); delay(1000); }

    panel = lcd_begin();
    backlight_on();

    if(!gt911_begin()) while(true){ Serial.println("BLAD GT911"); delay(1000); }
    if(!lvgl_port_begin(panel)) while(true){ Serial.println("BLAD LVGL"); delay(1000); }

    if(lvgl_lock()) {
        ui_create();
        lvgl_unlock();
    }

    // Pressure/nozzle model supplies flow; pulse input is not used.
    pressure_sensor_begin();
    aog_begin();
    Serial.flush();
    Serial.end();
    esp_log_set_vprintf(discard_runtime_log);
    valve_test_begin();
}

void loop() {
    aog_update();

    pressure_sensor_update();

    uint32_t now=millis();
    if(now-lastUi>=200){
        lastUi=now;
        if(lvgl_lock(100)){
            ui_update();
            lvgl_unlock();
        }
    }
    delay(2);
}
