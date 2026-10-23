#pragma once
// Pure C++ helpers: 3-bar nozzle calibration, independent of ESP32/LVGL.
#include <cmath>
#include <limits>
namespace agronozzle {
constexpr float kRsmTableQ3 = 1.54f;
constexpr float kV2RsmReference = 1.78f;
constexpr float kLegacyRsmReference = 1.54f;
inline float migrateLegacyRsmQ3(float legacyValue) {
    return legacyValue * (kRsmTableQ3 / kLegacyRsmReference);
}
inline float migrateV2RsmQ3(float v2Value) {
    return v2Value * (kRsmTableQ3 / kV2RsmReference);
}
inline float calibratedWaterFlow(float tableFlow, float waterQ3, int nozzle, float density) {
    const float tableQ3 = nozzle == 1 ? kRsmTableQ3 : 1.18f;
    return tableFlow * (waterQ3 / tableQ3) / std::sqrt(density);
}
// Manufacturer water-flow tables for the blue 11003 and red 04RSM tips.
// Returns NaN outside each published pressure range; no extrapolation.
inline float tableWaterFlow(float bar, int nozzle) {
    if (!std::isfinite(bar) || nozzle < 0 || nozzle > 1)
        return std::numeric_limits<float>::quiet_NaN();
    static constexpr float pressures[2][5] = {
        {2.0f, 2.5f, 3.0f, 3.5f, 4.0f},
        {2.0f, 3.0f, 4.0f, 5.0f, 6.0f}
    };
    static constexpr float flows[2][5] = {
        {0.96f, 1.08f, 1.18f, 1.27f, 1.39f},
        {1.26f, 1.54f, 1.78f, 1.99f, 2.18f}
    };
    if (bar < pressures[nozzle][0] || bar > pressures[nozzle][4])
        return std::numeric_limits<float>::quiet_NaN();
    for (int i = 1; i < 5; ++i) {
        if (bar <= pressures[nozzle][i]) {
            const float t = (bar - pressures[nozzle][i - 1]) /
                            (pressures[nozzle][i] - pressures[nozzle][i - 1]);
            const float q0 = flows[nozzle][i - 1];
            const float q1 = flows[nozzle][i];
            return std::sqrt(q0 * q0 + t * (q1 * q1 - q0 * q0));
        }
    }
    return flows[nozzle][4];
}
} // namespace agronozzle
