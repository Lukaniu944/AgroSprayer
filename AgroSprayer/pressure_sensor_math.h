#pragma once
// Pure conversion helpers for the ADS1115 pressure input.
// Kept independent of Arduino/ESP-IDF so the math can be regression-tested on desktop.
#include <cmath>
#include <cstdint>

namespace agropres {
constexpr float kAds1115FullScaleV = 4.096f;
constexpr float kPsiToBar = 0.0689475729f;

inline float sensorVoltageFromAdcRaw(int16_t raw, float dividerRatio) {
    if (!std::isfinite(dividerRatio) || dividerRatio <= 0.0f)
        return std::numeric_limits<float>::quiet_NaN();
    return static_cast<float>(raw) * (kAds1115FullScaleV / 32768.0f) * dividerRatio;
}

inline bool sensorVoltageValid(float sensorV, float minV, float maxV) {
    return std::isfinite(sensorV) && std::isfinite(minV) &&
           std::isfinite(maxV) && minV <= maxV &&
           sensorV >= minV && sensorV <= maxV;
}

inline bool sensorVoltageToBar(float sensorV, float zeroV, float fullV,
                               float maxPsi, float &bar) {
    bar = 0.0f;
    if (!std::isfinite(sensorV) || !std::isfinite(zeroV) ||
        !std::isfinite(fullV) || !std::isfinite(maxPsi) ||
        fullV <= zeroV || maxPsi <= 0.0f)
        return false;

    const float maxBar = maxPsi * kPsiToBar;
    const float fraction = (sensorV - zeroV) / (fullV - zeroV);
    bar = fraction * maxBar;
    if (!std::isfinite(bar)) {
        bar = 0.0f;
        return false;
    }
    if (bar < 0.0f) bar = 0.0f;
    if (bar > maxBar) bar = maxBar;
    return true;
}
} // namespace agropres
