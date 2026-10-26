#pragma once
// Hardware-independent pressure conversion and trip threshold.
// Keep this math shared between firmware and native regression tests.
#include <cmath>
#include <limits>

namespace agrosensor {
constexpr float kBarPerPsi = 0.0689475729f;
constexpr float kPressureTripBar = 4.9f;

inline bool voltage_valid(float sensorV, float minV, float maxV) {
    return std::isfinite(sensorV) && std::isfinite(minV) &&
           std::isfinite(maxV) && minV <= maxV &&
           sensorV >= minV && sensorV <= maxV;
}

// Returns the unfiltered, unclamped pressure in bar. The caller must reject
// invalid sensor voltage before using this value for control decisions.
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
