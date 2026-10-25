#pragma once
#include "nozzle_profile_compatibility.h"
#include <cmath>
#include <cstdint>
#include <limits>

namespace agronozzle {

// The manufacturer's 04RSM table is labelled "Wasser Ausbringung" (water
// output). Its footnote specifies correction factors for RSM 28/30/32:
// 1.13 / 1.14 / 1.15. Convert the water-reference flow to actual RSM flow by
// dividing by that factor. Do not use sqrt(density) as well: that would apply
// the same density correction twice. A custom liquid on 11003 uses sqrt(density).
inline float nozzle_flow_from_table(float bar, uint8_t nozzle, uint8_t liquid,
                                    float calibratedQ3, float density)
{
    const float invalid = std::numeric_limits<float>::quiet_NaN();
    if (!nozzle_profile_compatible(nozzle, liquid) ||
        !std::isfinite(bar) || bar < 0.0f ||
        !std::isfinite(calibratedQ3) || calibratedQ3 < 0.1f || calibratedQ3 > 5.0f ||
        !std::isfinite(density) || density < 0.8f || density > 1.6f) {
        return invalid;
    }
    if (bar <= 0.1f) return 0.0f;

    static constexpr float pressure[2][5] = {
        {2.0f, 2.5f, 3.0f, 3.5f, 4.0f},
        {2.0f, 3.0f, 4.0f, 5.0f, 6.0f}
    };
    static constexpr float flow[2][5] = {
        {0.96f, 1.08f, 1.18f, 1.27f, 1.39f},
        {1.26f, 1.54f, 1.78f, 1.99f, 2.18f}
    };
    const int k = nozzle;
    if (bar < pressure[k][0] || bar > pressure[k][4]) return invalid;

    float q = flow[k][4];
    for (int i = 1; i < 5; ++i) {
        if (bar <= pressure[k][i]) {
            const float t = (bar - pressure[k][i - 1]) /
                            (pressure[k][i] - pressure[k][i - 1]);
            q = std::sqrt(flow[k][i - 1] * flow[k][i - 1] +
                          t * (flow[k][i] * flow[k][i] -
                               flow[k][i - 1] * flow[k][i - 1]));
            break;
        }
    }

    const float referenceQ3 = nozzle == 0 ? 1.18f : 1.54f;
    float rsmCorrection = 1.0f;
    if (nozzle == 1) {
        if (liquid == 1) rsmCorrection = 1.13f;      // RSM 28
        else if (liquid == 2) rsmCorrection = 1.14f; // RSM 30
        else if (liquid == 3) rsmCorrection = 1.15f; // RSM 32
    }
    const float customDensityCorrection = (nozzle == 0 && liquid == 4)
        ? std::sqrt(density) : 1.0f;
    return q * calibratedQ3 / referenceQ3 /
           customDensityCorrection / rsmCorrection;
}

} // namespace agronozzle
