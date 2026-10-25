#include "pressure_sensor.h"
#include "board_config.h"
#include "hw_i2c.h"
#include <math.h>

static i2c_master_dev_handle_t device = nullptr;
static bool pending = false, valid = false, filtered = false;
static volatile bool overpressureTrip = false;
static constexpr float PRESSURE_TRIP_BAR = 4.9f;
static uint32_t started = 0, attempted = 0, lastGood = 0;
static uint32_t retryMs = 0;
static float bar = 0;
static int signalMv = -1;

static bool readRegister(uint8_t reg, uint16_t &value) {
    uint8_t bytes[2];
    if (!hw_i2c_write_read(device, &reg, 1, bytes, 2)) return false;
    value = ((uint16_t)bytes[0] << 8) | bytes[1];
    return true;
}
static void busError() {
    pending = valid = filtered = false;
    signalMv = -1;
    retryMs = 2000;
}
void pressure_sensor_begin() {
    pending = valid = filtered = false;
    overpressureTrip = false;
    signalMv = -1;
    retryMs = 0;
    attempted = millis() - PRESSURE_INTERVAL_MS;
    if (!device && !hw_i2c_add_device(PRESSURE_ADS_ADDR, &device)) busError();
}
void pressure_sensor_update() {
    const uint32_t now = millis();
    if (valid && (uint32_t)(now - lastGood) > 1000) valid = filtered = false;
    if (!pending) {
        const uint32_t interval = retryMs ? retryMs : PRESSURE_INTERVAL_MS;
        if ((uint32_t)(now - attempted) < interval) return;
        attempted = now;
        if (!device && !hw_i2c_add_device(PRESSURE_ADS_ADDR, &device)) {
            busError(); return;
        }
        // OS=1, A0/GND, +/-4.096 V, single-shot, 128 SPS, comparator disabled.
        const uint8_t config[] = {1, 0xC3, 0x83};
        if (!hw_i2c_write(device, config, sizeof(config))) { busError(); return; }
        started = now;
        pending = true;
        retryMs = 0;
        return;
    }
    if ((uint32_t)(now - started) < 10) return;
    uint16_t config;
    if (!readRegister(1, config)) { busError(); return; }
    if (!(config & 0x8000)) {
        if ((uint32_t)(now - started) > 100) busError();
        return;
    }
    uint16_t raw;
    if (!readRegister(0, raw)) { busError(); return; }
    pending = false;
    const float sensorV = (int16_t)raw * (4.096f / 32768.0f) * PRESSURE_DIVIDER_RATIO;
    signalMv = (int)lroundf(sensorV * 1000.0f);
    if (!isfinite(sensorV) || sensorV < PRESSURE_VALID_MIN_V || sensorV > PRESSURE_VALID_MAX_V) {
        valid = filtered = false;
        return;
    }
    float measuredBar = (sensorV - PRESSURE_ZERO_V) / (PRESSURE_FULL_V - PRESSURE_ZERO_V)
                        * PRESSURE_MAX_PSI * 0.0689475729f;
    // Trip on the unfiltered measurement so the display smoothing cannot delay shutdown.
    if (measuredBar >= PRESSURE_TRIP_BAR) overpressureTrip = true;
    const float maxBar = PRESSURE_MAX_PSI * 0.0689475729f;
    if (measuredBar < 0) measuredBar = 0;
    if (measuredBar > maxBar) measuredBar = maxBar;
    bar = filtered ? bar + 0.25f * (measuredBar - bar) : measuredBar;
    filtered = valid = true;
    lastGood = now;
}
bool pressure_sensor_valid() {
    return valid && (uint32_t)(millis() - lastGood) <= 1000;
}
bool pressure_sensor_overpressure() { return overpressureTrip; }
int pressure_sensor_bar_x10() { return (int)lroundf(bar * 10.0f); }
int pressure_sensor_signal_mv() { return signalMv; }
