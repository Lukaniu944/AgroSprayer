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
// Density presets used by the production nozzle model and native regression tests.
// Preset profiles override stale saved density; WLASNA (index 4) uses the editable value.
inline float presetLiquidDensity(int liquid) {
    static constexpr float densities[4] = {1.0f, 1.28f, 1.30f, 1.32f};
    if (liquid < 0 || liquid >= 4)
        return std::numeric_limits<float>::quiet_NaN();
    return densities[liquid];
}
inline float effectiveLiquidDensity(int liquid, float customDensity) {
    if (liquid >= 0 && liquid < 4)
        return presetLiquidDensity(liquid);
    if (liquid != 4 || !std::isfinite(customDensity) ||
        customDensity < 0.8f || customDensity > 1.6f)
        return std::numeric_limits<float>::quiet_NaN();
    return customDensity;
}

inline bool validNozzleLiquidProfile(int nozzle, int liquid) {
    // Blue 11003 uses the water table (or a custom liquid derived from it).
    // Red 04RSM uses the supplied RSM table; do not accept unverified pairs.
    if (nozzle == 0) return liquid == 0 || liquid == 4;
    if (nozzle == 1) return liquid >= 1 && liquid <= 3;
    return false;
}

// Manufacturer correction factors for the red 04RSM water-flow table.
// Values follow the published table: RSM 28 /1.13, RSM 30 /1.14, RSM 32 /1.15.
// For other liquids, use the physical sqrt(relative-density) correction.
inline float liquidCorrectionFactor(int nozzle, float density) {
    if (nozzle < 0 || nozzle > 1 || !std::isfinite(density) || density <= 0.0f)
        return std::numeric_limits<float>::quiet_NaN();
    if (nozzle == 1) {
        if (std::fabs(density - 1.28f) < 0.005f) return 1.13f;
        if (std::fabs(density - 1.30f) < 0.005f) return 1.14f;
        if (std::fabs(density - 1.32f) < 0.005f) return 1.15f;
    }
    return std::sqrt(density);
}

inline float calibratedWaterFlow(float tableFlow, float waterQ3, int nozzle, float density) {
    // Reject invalid calibration inputs instead of propagating an impossible
    // flow estimate into the dose display or pressure-target calculation.
    if (!std::isfinite(tableFlow) || tableFlow < 0.0f ||
        !std::isfinite(waterQ3) || waterQ3 <= 0.0f ||
        nozzle < 0 || nozzle > 1 ||
        !std::isfinite(density) || density <= 0.0f)
        return std::numeric_limits<float>::quiet_NaN();

    const float tableQ3 = nozzle == 1 ? kRsmTableQ3 : 1.18f;
    const float correction = liquidCorrectionFactor(nozzle, density);
    if (!std::isfinite(correction) || correction <= 0.0f)
        return std::numeric_limits<float>::quiet_NaN();
    const float calibrated = tableFlow * (waterQ3 / tableQ3) / correction;
    return std::isfinite(calibrated) && calibrated >= 0.0f
        ? calibrated : std::numeric_limits<float>::quiet_NaN();
}
// Manufacturer tables are published as water flow (l/min) for the selected tip.
// RSM flow is obtained with the manufacturer's liquid correction factors.
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

/*
 * Inverse of the same calibrated nozzle curve used by the live flow model.
 * Returns NaN when the requested per-nozzle flow is outside the published
 * pressure range; callers must not silently clamp an unreachable target.
 */
inline float targetPressureForFlow(float totalFlowLpm, int nozzleCount,
                                   int nozzle, float waterQ3, float density) {
    if (!std::isfinite(totalFlowLpm) || totalFlowLpm <= 0.0f ||
        nozzleCount <= 0 || nozzle < 0 || nozzle > 1 ||
        !std::isfinite(waterQ3) || waterQ3 <= 0.0f ||
        !std::isfinite(density) || density <= 0.0f)
        return std::numeric_limits<float>::quiet_NaN();

    float low = 2.0f;
    float high = nozzle == 1 ? 6.0f : 4.0f;
    const float requested = totalFlowLpm / static_cast<float>(nozzleCount);
    const float minFlow = calibratedWaterFlow(tableWaterFlow(low, nozzle),
                                               waterQ3, nozzle, density);
    const float maxFlow = calibratedWaterFlow(tableWaterFlow(high, nozzle),
                                               waterQ3, nozzle, density);
    if (!std::isfinite(requested) || !std::isfinite(minFlow) ||
        !std::isfinite(maxFlow))
        return std::numeric_limits<float>::quiet_NaN();

    const float epsilon = maxFlow * 1.0e-6f;
    if (requested < minFlow)
        return minFlow - requested <= epsilon ? low
                                               : std::numeric_limits<float>::quiet_NaN();
    if (requested > maxFlow)
        return requested - maxFlow <= epsilon ? high
                                               : std::numeric_limits<float>::quiet_NaN();

    for (int i = 0; i < 24; ++i) {
        const float mid = (low + high) * 0.5f;
        const float flow = calibratedWaterFlow(tableWaterFlow(mid, nozzle),
                                                waterQ3, nozzle, density);
        if (!std::isfinite(flow))
            return std::numeric_limits<float>::quiet_NaN();
        if (flow < requested) low = mid;
        else high = mid;
    }
    return (low + high) * 0.5f;
}

} // namespace agronozzle
