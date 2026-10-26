#include "pressure_sensor.h"
#include "board_config.h"
#include "hw_i2c.h"
#include <math.h>
#include <atomic>

static i2c_master_dev_handle_t device = nullptr;
static bool pending = false, filtered = false;
static std::atomic<bool> valid{false};
static std::atomic<bool> overpressureTrip{false};
static constexpr float PRESSURE_TRIP_BAR = 4.9f;
static uint32_t started = 0, attempted = 0;
static std::atomic<uint32_t> lastGood{0};
static uint32_t retryMs = 0;
// Only pressure_sensor_update() owns the filter accumulator. Other tasks read
// the published integer snapshot below, avoiding cross-task access to a float.
static float filteredBar = 0;
static std::atomic<int> publishedBarX10{0};
static std::atomic<int> signalMv{-1};

static bool readRegister(uint8_t reg, uint16_t &value) {
    uint8_t bytes[2];
    if (!hw_i2c_write_read(device, &reg, 1, bytes, 2)) return false;
    value = ((uint16_t)bytes[0] << 8) | bytes[1];
    return true;
}
static void busError() {
    pending = false;
    valid.store(false, std::memory_order_release);
    filtered = false;
    signalMv.store(-1, std::memory_order_release);
    retryMs = 2000;
}
void pressure_sensor_begin() {
    pending = false;
    valid.store(false, std::memory_order_release);
    filtered = false;
    overpressureTrip.store(false, std::memory_order_release);
    signalMv.store(-1, std::memory_order_release);
    retryMs = 0;
    attempted = millis() - PRESSURE_INTERVAL_MS;
    if (!device && !hw_i2c_add_device(PRESSURE_ADS_ADDR, &device)) busError();
}
void pressure_sensor_update() {
    const uint32_t now = millis();
    if (valid.load(std::memory_order_acquire) &&
        (uint32_t)(now - lastGood.load(std::memory_order_acquire)) > 1000) {
        // Keep the validity timeout authoritative in pressure_sensor_valid().
        // Reset smoothing after a gap without racing the watchdog task.
        filtered = false;
    }
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
    signalMv.store((int)lroundf(sensorV * 1000.0f), std::memory_order_release);
    if (!isfinite(sensorV) || sensorV < PRESSURE_VALID_MIN_V || sensorV > PRESSURE_VALID_MAX_V) {
        valid.store(false, std::memory_order_release);
        filtered = false;
        return;
    }
    float measuredBar = (sensorV - PRESSURE_ZERO_V) / (PRESSURE_FULL_V - PRESSURE_ZERO_V)
                        * PRESSURE_MAX_PSI * 0.0689475729f;
    // Trip on the unfiltered measurement so the display smoothing cannot delay shutdown.
    if (measuredBar >= PRESSURE_TRIP_BAR) overpressureTrip.store(true, std::memory_order_release);
    const float maxBar = PRESSURE_MAX_PSI * 0.0689475729f;
    if (measuredBar < 0) measuredBar = 0;
    if (measuredBar > maxBar) measuredBar = maxBar;
    filteredBar = filtered ? filteredBar + 0.25f * (measuredBar - filteredBar) : measuredBar;
    filtered = true;
    publishedBarX10.store((int)lroundf(filteredBar * 10.0f), std::memory_order_release);
    lastGood.store(now, std::memory_order_relaxed);
    valid.store(true, std::memory_order_release);
}
bool pressure_sensor_valid() {
    if (!valid.load(std::memory_order_acquire)) return false;
    const uint32_t goodAt = lastGood.load(std::memory_order_acquire);
    return (uint32_t)(millis() - goodAt) <= 1000;
}
bool pressure_sensor_overpressure() {
    return overpressureTrip.load(std::memory_order_acquire);
}
int pressure_sensor_bar_x10() {
    return publishedBarX10.load(std::memory_order_acquire);
}
int pressure_sensor_signal_mv() {
    return signalMv.load(std::memory_order_acquire);
}
