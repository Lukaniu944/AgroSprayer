#include "valve_test.h"
#include "board_config.h"
#include "pressure_sensor.h"
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static bool ready = false, armed = false;
static int direction = 0;
static uint32_t started = 0, stopped = 0, pulseDuration = VALVE_TEST_PULSE_MS;
static int openDirection = 1;
static void outputsOff() {
    gpio_set_level((gpio_num_t)VALVE_IN1_PIN, 0);
    gpio_set_level((gpio_num_t)VALVE_IN2_PIN, 0);
}
static void watchdogTask(void *) {
    for (;;) {
        portENTER_CRITICAL(&mux);
        const bool pressureFault =
            !pressure_sensor_valid() || pressure_sensor_overpressure();
        if (pressureFault) {
            // A pressure fault must also disarm an idle manual test, not only
            // stop an active pulse. A fresh operator arm is required after recovery.
            if (direction) outputsOff();
            direction = 0;
            armed = false;
            stopped = millis();
        } else if (direction &&
                   (uint32_t)(millis() - started) >= pulseDuration) {
            outputsOff();
            direction = 0;
            stopped = millis();
        }
        portEXIT_CRITICAL(&mux);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
bool valve_test_begin() {
    // UART0 must have been stopped before these pins are repurposed.
    gpio_reset_pin((gpio_num_t)VALVE_IN1_PIN);
    gpio_reset_pin((gpio_num_t)VALVE_IN2_PIN);
    outputsOff();
    gpio_set_direction((gpio_num_t)VALVE_IN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)VALVE_IN2_PIN, GPIO_MODE_OUTPUT);
    outputsOff();
    stopped = millis();
    Preferences prefs;
    if(prefs.begin("agrovalve",true)) {
        openDirection = prefs.getInt("open_dir",1)==-1 ? -1 : 1;
        prefs.end();
    }
    ready = xTaskCreate(watchdogTask, "valve_timeout", 2048, nullptr, 3, nullptr) == pdPASS;
    return ready;
}
bool valve_test_ready() { return ready; }
void valve_test_arm() {
    portENTER_CRITICAL(&mux);
    if (ready) armed = true;
    portEXIT_CRITICAL(&mux);
}
void valve_test_stop() {
    portENTER_CRITICAL(&mux);
    outputsOff(); direction = 0; armed = false; stopped = millis();
    portEXIT_CRITICAL(&mux);
}
static bool startPulse(int requested, uint32_t duration, bool requireArm) {
    portENTER_CRITICAL(&mux);
    const uint32_t now = millis();
    const bool pressureSafe = pressure_sensor_valid() && !pressure_sensor_overpressure();
    const bool allowed = ready && pressureSafe && (!requireArm || armed) && !direction &&
        (requested == 1 || requested == -1) &&
        (uint32_t)(now - stopped) >= 100;
    if (allowed) {
        outputsOff();
        gpio_set_level((gpio_num_t)(requested == 1 ? VALVE_IN1_PIN : VALVE_IN2_PIN), 1);
        started = now; pulseDuration = duration; direction = requested;
    }
    portEXIT_CRITICAL(&mux);
    return allowed;
}
bool valve_test_pulse(int requested) { return startPulse(requested,VALVE_TEST_PULSE_MS,true); }
bool valve_manual_pulse(int requested) {
    if(requested!=1 && requested!=-1)return false;
    return startPulse(requested*valve_open_direction(),250,false);
}
bool valve_auto_pulse(int logicalDirection, uint32_t durationMs) {
    if ((logicalDirection != 1 && logicalDirection != -1) || durationMs < 50 || durationMs > 250) return false;
    return startPulse(logicalDirection * valve_open_direction(),durationMs,false);
}
int valve_open_direction() {
    portENTER_CRITICAL(&mux);const int value=openDirection;portEXIT_CRITICAL(&mux);return value;
}
bool valve_toggle_open_direction() {
    valve_test_stop();
    const int next=-valve_open_direction();
    Preferences prefs;
    if(!prefs.begin("agrovalve",false))return false;
    const bool saved=prefs.putInt("open_dir",next)==sizeof(int32_t);
    prefs.end();
    if(saved){portENTER_CRITICAL(&mux);openDirection=next;portEXIT_CRITICAL(&mux);}
    return saved;
}
bool valve_test_armed() {
    portENTER_CRITICAL(&mux); const bool value = armed; portEXIT_CRITICAL(&mux); return value;
}
int valve_test_direction() {
    portENTER_CRITICAL(&mux); const int value = direction; portEXIT_CRITICAL(&mux); return value;
}
