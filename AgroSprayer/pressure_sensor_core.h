#pragma once
// Hardware-independent pressure conversion, sample freshness and trip threshold.
// Keep this math shared between firmware and native regression tests.
#include <cmath>
#include <cstdint>
#include <limits>

namespace agrosensor {
constexpr float kBarPerPsi = 0.0689475729f;
constexpr float kPressureTripBar = 4.9f;
// ADS1115 is requested every 100 ms. Allow scheduling jitter, but do not
// let the watchdog trust a pressure sample older than three sample periods.
constexpr uint32_t kPressureSampleTimeoutMs = 300;

inline bool sample_is_fresh(uint32_t now, uint32_t lastGood) {
    return static_cast<uint32_t>(now - lastGood) <= kPressureSampleTimeoutMs;
}

inline float sensor_voltage_from_ads1115_raw(int16_t raw, float lsbV,
                                              float dividerRatio) {
    if (!std::isfinite(lsbV) || !std::isfinite(dividerRatio) ||
        lsbV <= 0.0f || dividerRatio <= 0.0f) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return static_cast<float>(raw) * lsbV * dividerRatio;
}

inline bool voltage_valid(float sensorV, float minV, float maxV) {
    return std::isfinite(sensorV) && std::isfinite(minV) &&
           std::isfinite(maxV) && minV <= maxV &&
           sensorV >= minV && sensorV <= maxV;
}

inline float raw_pressure_bar_from_voltage(float sensorV, float zeroV,
                                           float fullV, float maxPsi) {
    if (!std::isfinite(sensorV) || !std::isfinite(zeroV) ||
        !std::isfinite(fullV) || !std::isfinite(maxPsi) ||
        fullV <= zeroV || maxPsi <= 0.0f) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return (sensorV - zeroV) / (fullV - zeroV) *
           maxPsi * kBarPerPsi;
}

inline bool overpressure_trip(float rawPressureBar) {
    return std::isfinite(rawPressureBar) &&
           rawPressureBar >= kPressureTripBar;
}
} // namespace agrosensor
