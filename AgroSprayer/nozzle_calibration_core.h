#pragma once
// Pure C++ helpers: 3-bar nozzle calibration, independent of ESP32/LVGL.
#include <cmath>
namespace agronozzle {
constexpr float kRsmTableQ3 = 1.54f;
constexpr float kV2RsmReference = 1.78f;
constexpr float kLegacyRsmReference = 1.54f;
// Maximum automatic target pressure: the regulator itself trips above 4.8 bar.
// Keep the nozzle table's 6-bar RSM data for measured-flow estimation only.
constexpr float kWaterMaxTargetPressureBar = 4.0f;
constexpr float kRsmMaxTargetPressureBar = 4.8f;
constexpr float maxTargetPressureBar(int nozzle) {
    return nozzle == 1 ? kRsmMaxTargetPressureBar : kWaterMaxTargetPressureBar;
}
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
} // namespace agronozzle
