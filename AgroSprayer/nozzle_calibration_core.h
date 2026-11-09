#pragma once
// Pure C++ nozzle tables and calibration helpers, independent of ESP32/LVGL.
#include <cmath>
#include <cstddef>
#include <limits>

namespace agronozzle {
constexpr float kRsmTableQ3 = 1.54f;
constexpr float kV2RsmReference = 1.78f;
constexpr float kLegacyRsmReference = 1.54f;
constexpr float kWaterMaxTargetPressureBar = 4.0f;
constexpr float kRsmMaxTargetPressureBar = 4.8f;

constexpr float maxTargetPressureBar(int nozzle) {
    return nozzle == 1 ? kRsmMaxTargetPressureBar : kWaterMaxTargetPressureBar;
}

// 11003 accepts all supported liquid profiles. 04RSM is restricted to
// the explicit RSM 28/30/32 profiles; water and custom liquid are not mapped
// to this direct-RSM table.
inline bool supportedNozzleLiquid(int nozzle, int liquid) {
    if (nozzle == 0) return liquid >= 0 && liquid <= 4;
    return nozzle == 1 && liquid >= 1 && liquid <= 3;
}

// A missing NVS key means first boot and permits defaults. Any present but
// malformed or rejected profile must block flow estimation until it is saved.
inline bool storedProfileMustBlock(bool keyExists,
                                   std::size_t storedBytes,
                                   std::size_t bytesRead,
                                   std::size_t expectedBytes,
                                   bool settingsValid) {
    return keyExists &&
           (storedBytes != expectedBytes || bytesRead != expectedBytes ||
            !settingsValid);
}

// 11003 is the water table; 04RSM is the supplied RSM table.
// Interpolate Q^2 linearly against pressure (Q proportional to sqrt(bar)).
inline float nozzleTableFlow(float bar, int nozzle) {
    if (!std::isfinite(bar) || bar < 0.0f || nozzle < 0 || nozzle > 1)
        return std::numeric_limits<float>::quiet_NaN();
    if (bar <= 0.1f) return 0.0f;

    constexpr float waterPressure[5] = {2.0f, 2.5f, 3.0f, 3.5f, 4.0f};
    constexpr float waterFlow[5] = {0.96f, 1.08f, 1.18f, 1.27f, 1.39f};
    constexpr float rsmPressure[5] = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    constexpr float rsmFlow[5] = {1.26f, 1.54f, 1.78f, 1.99f, 2.18f};
    const float *pressure = nozzle == 1 ? rsmPressure : waterPressure;
    const float *flow = nozzle == 1 ? rsmFlow : waterFlow;

    if (bar < pressure[0] || bar > pressure[4])
        return std::numeric_limits<float>::quiet_NaN();
    for (int i = 1; i < 5; ++i) {
        if (bar <= pressure[i]) {
            const float t = (bar - pressure[i - 1]) /
                            (pressure[i] - pressure[i - 1]);
            const float q2 = flow[i - 1] * flow[i - 1] +
                             t * (flow[i] * flow[i] -
                                  flow[i - 1] * flow[i - 1]);
            return std::sqrt(q2);
        }
    }
    return flow[4];
}

// Calibration is expressed as the measured output at 3 bar.
// The 11003 table is water, so a selected RSM liquid needs one density
// correction. The supplied 04RSM table is already RSM: never correct it again.
inline float calibratedNozzleFlow(float tableFlow, float referenceQ3,
                                  int nozzle, float density) {
    if (!std::isfinite(tableFlow) || !std::isfinite(referenceQ3) ||
        !std::isfinite(density) || tableFlow < 0.0f ||
        referenceQ3 <= 0.0f || density <= 0.0f ||
        nozzle < 0 || nozzle > 1) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const float tableQ3 = nozzle == 1 ? kRsmTableQ3 : 1.18f;
    float result = tableFlow * (referenceQ3 / tableQ3);
    if (nozzle == 0) result /= std::sqrt(density);
    return result;
}

inline float migrateLegacyRsmQ3(float legacyValue) {
    return legacyValue * (kRsmTableQ3 / kLegacyRsmReference);
}
inline float migrateV2RsmQ3(float v2Value) {
    return v2Value * (kRsmTableQ3 / kV2RsmReference);
}

// V1-V3 stored a water-reference Q3 and applied density to both nozzle tables.
// Convert custom 04RSM calibrations to direct RSM Q3. Keep the table default
// (1.54) unchanged so the supplied RSM table remains exact.
inline float migrateRsmQ3ToDirect(float legacyWaterQ3, float density) {
    if (!std::isfinite(legacyWaterQ3) || !std::isfinite(density) ||
        legacyWaterQ3 <= 0.0f || density <= 0.0f) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    if (std::fabs(legacyWaterQ3 - kRsmTableQ3) <= 0.0001f)
        return kRsmTableQ3;
    return legacyWaterQ3 / std::sqrt(density);
}
} // namespace agronozzle
