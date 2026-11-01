#include <Arduino.h>
#include "config.h"
#include "board_config.h"
#include "hw_i2c.h"
#include "io_extension.h"
#include "lcd_driver.h"
#include "gt911.h"
#include "lvgl_port.h"
// Flowmeter is deliberately not used: dose/consumption come from pressure and nozzle curves.
#include "pressure_sensor.h"
#include "valve_test.h"
#include "auto_regulator.h"
#include "esp_log.h"
#include <stdarg.h>
#include "aog.h"
#include "nozzle_model.h"
#include "ui.h"

static esp_lcd_panel_handle_t panel = nullptr;
static uint32_t lastUi = 0;
static uint32_t lastNozzleUpdate = 0;
static constexpr uint32_t NOZZLE_ESTIMATE_INTERVAL_MS = 10;
// GPIO43/44 are L298N IN1/IN2 and are also UART0 TX/RX on this board.
// ROM boot output can toggle TX before setup: keep L298N ENA physically
// disabled during reset/boot/flashing. Do not start Serial on these pins.
static int discard_runtime_log(const char *, va_list) { return 0; }

void setup() {
    // Establish the safe motor-input state before initializing any peripheral.
    // This also covers later init-failure loops: IN1/IN2 remain LOW.
    esp_log_set_vprintf(discard_runtime_log);
    (void)valve_test_begin();

    if(!hw_i2c_begin()) while(true){ delay(1000); }
    if(!io_extension_begin()) while(true){ delay(1000); }

    panel = lcd_begin();
    backlight_on();

    if(!gt911_begin()) while(true){ delay(1000); }
    if(!lvgl_port_begin(panel)) while(true){ delay(1000); }

    if(lvgl_lock()) {
        ui_create();
        lvgl_unlock();
    }

    // Pressure/nozzle model supplies flow; pulse input is not used.
    pressure_sensor_begin();
    aog_begin();
}

void loop() {
    aog_update();

    pressure_sensor_update();

    // Keep consumption estimation independent of the LVGL refresh/lock.
    // A 10 ms cadence limits the interval skipped on a section-mask change.
    uint32_t now=millis();
    if ((uint32_t)(now - lastNozzleUpdate) >= NOZZLE_ESTIMATE_INTERVAL_MS) {
        lastNozzleUpdate = now;
        nozzle_update();
    }

    auto_regulator_update();
    if(now-lastUi>=200){
        lastUi=now;
        if(lvgl_lock(100)){
            ui_update();
            lvgl_unlock();
        }
    }
    delay(2);
}
