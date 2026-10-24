#include "../AgroSprayer/nozzle_calibration_core.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
    int checks = 0;
    auto check = [&](bool ok, const char *name) {
        if (!ok) {
            std::cerr << "FAIL: " << name << '\n';
            std::abort();
        }
        ++checks;
    };
    auto near = [](float a, float b, float tolerance = 0.005f) {
        return std::fabs(a - b) <= tolerance;
    };

    // Manufacturer table nodes: blue 11003 (nozzle 0), red 04RSM (nozzle 1).
    const float bluePressure[] = {2.0f, 2.5f, 3.0f, 3.5f, 4.0f};
    const float blueFlow[]     = {0.96f, 1.08f, 1.18f, 1.27f, 1.39f};
    const float redPressure[]  = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    const float redFlow[]      = {1.26f, 1.54f, 1.78f, 1.99f, 2.18f};
    for (int i = 0; i < 5; ++i) {
        check(near(agronozzle::tableWaterFlow(bluePressure[i], 0), blueFlow[i]),
              "blue 11003 manufacturer table node");
        check(near(agronozzle::tableWaterFlow(redPressure[i], 1), redFlow[i]),
              "red 04RSM manufacturer table node");
    }
    check(std::isnan(agronozzle::tableWaterFlow(1.99f, 0)),
          "blue pressure below published range rejected");
    check(std::isnan(agronozzle::tableWaterFlow(4.01f, 0)),
          "blue pressure above published range rejected");
    check(std::isnan(agronozzle::tableWaterFlow(1.99f, 1)),
          "red pressure below published range rejected");
    check(std::isnan(agronozzle::tableWaterFlow(6.01f, 1)),
          "red pressure above published range rejected");
    check(std::isnan(agronozzle::tableWaterFlow(NAN, 0)),
          "NaN pressure rejected");
    check(std::isnan(agronozzle::tableWaterFlow(3.0f, -1)),
          "invalid nozzle index rejected");
    check(near(agronozzle::tableWaterFlow(2.25f, 0), 1.022f, 0.01f),
          "blue interpolation between published values");
    check(agronozzle::tableWaterFlow(2.25f, 0) < agronozzle::tableWaterFlow(2.5f, 0),
          "blue interpolation is monotonic");

    // Check every published interval for both tips: interpolation must stay
    // strictly between table nodes and remain linear in pressure-space q^2.
    for (int nozzle = 0; nozzle < 2; ++nozzle) {
        const float *pressures = nozzle == 0 ? bluePressure : redPressure;
        const float *flows = nozzle == 0 ? blueFlow : redFlow;
        for (int i = 0; i < 4; ++i) {
            const float midPressure = (pressures[i] + pressures[i + 1]) * 0.5f;
            const float midFlow = agronozzle::tableWaterFlow(midPressure, nozzle);
            check(std::isfinite(midFlow) && midFlow > flows[i] &&
                  midFlow < flows[i + 1],
                  "midpoint interpolation remains between manufacturer nodes");
            check(near(midFlow * midFlow,
                       (flows[i] * flows[i] +
                        flows[i + 1] * flows[i + 1]) * 0.5f, 0.005f),
                  "midpoint interpolation preserves q-squared pressure relation");
        }
    }

    // Invert every published node, not only the 3-bar reference, for both tips.
    for (int nozzle = 0; nozzle < 2; ++nozzle) {
        const float *pressures = nozzle == 0 ? bluePressure : redPressure;
        const float *flows = nozzle == 0 ? blueFlow : redFlow;
        const float q3 = flows[nozzle == 0 ? 2 : 1];
        const float density = 1.0f;
        for (int i = 0; i < 5; ++i) {
            const float calibrated = agronozzle::calibratedWaterFlow(
                flows[i], q3, nozzle, density);
            const float recovered = agronozzle::targetPressureForFlow(
                calibrated * 6.0f, 6, nozzle, q3, density);
            check(near(recovered, pressures[i], 0.002f),
                  "inverse pressure solver round-trips every table node");
        }
    }

    // Preset density mapping is shared with production code; stale saved density
    // must never override the selected liquid, while WLASNA keeps its own value.
    check(near(agronozzle::effectiveLiquidDensity(0, 1.30f), 1.00f),
          "WODA preset overrides stale saved density");
    check(near(agronozzle::effectiveLiquidDensity(1, 1.00f), 1.28f),
          "RSM 28 preset overrides stale saved density");
    check(near(agronozzle::effectiveLiquidDensity(2, 1.00f), 1.30f),
          "RSM 30 preset overrides stale saved density");
    check(near(agronozzle::effectiveLiquidDensity(3, 1.00f), 1.32f),
          "RSM 32 preset overrides stale saved density");
    check(near(agronozzle::effectiveLiquidDensity(4, 1.36f), 1.36f),
          "WLASNA preserves editable density");
    check(std::isnan(agronozzle::effectiveLiquidDensity(-1, 1.00f)),
          "negative liquid preset rejected");
    check(std::isnan(agronozzle::effectiveLiquidDensity(5, 1.00f)),
          "unknown liquid preset rejected");
    check(std::isnan(agronozzle::effectiveLiquidDensity(4, NAN)),
          "custom liquid NaN density rejected");
    check(std::isnan(agronozzle::effectiveLiquidDensity(4, 1.61f)),
          "custom liquid density above allowed range rejected");

    // Liquid-density correction is sqrt(relative density): 1.13 for RSM 28,
    // 1.14 for RSM 30, and 1.15 for RSM 32, applied to estimated liquid flow.
    const float qWater = 1.54f;
    const float q28 = agronozzle::calibratedWaterFlow(qWater, qWater, 1, 1.28f);
    const float q30 = agronozzle::calibratedWaterFlow(qWater, qWater, 1, 1.30f);
    const float q32 = agronozzle::calibratedWaterFlow(qWater, qWater, 1, 1.32f);
    check(near(qWater / q28, 1.13f), "RSM 28 correction factor ~1.13");
    check(near(qWater / q30, 1.14f), "RSM 30 correction factor ~1.14");
    check(near(qWater / q32, 1.15f), "RSM 32 correction factor ~1.15");

    check(near(agronozzle::calibratedWaterFlow(1.18f, 1.18f, 0, 1.0f),
               1.18f), "water 11003 default calibration at 3 bar");
    check(near(agronozzle::calibratedWaterFlow(1.54f, 1.54f, 1, 1.0f),
               1.54f), "04RSM default water reference at 3 bar");

    // Invalid calibration inputs must not become plausible flow values.
    check(std::isnan(agronozzle::calibratedWaterFlow(NAN, 1.18f, 0, 1.0f)),
          "non-finite table flow rejected");
    check(std::isnan(agronozzle::calibratedWaterFlow(1.18f, NAN, 0, 1.0f)),
          "non-finite calibration reference rejected");
    check(std::isnan(agronozzle::calibratedWaterFlow(1.18f, 1.18f, 2, 1.0f)),
          "invalid nozzle index rejected by calibration");
    check(std::isnan(agronozzle::calibratedWaterFlow(1.18f, 1.18f, 0, NAN)),
          "non-finite density rejected by calibration");
    check(std::isnan(agronozzle::calibratedWaterFlow(-0.1f, 1.18f, 0, 1.0f)),
          "negative table flow rejected");
    check(std::isnan(agronozzle::calibratedWaterFlow(1.18f, 1.18f, 0, 0.0f)),
          "non-positive density rejected by calibration");

    check(near(agronozzle::migrateLegacyRsmQ3(1.54f), 1.54f),
          "legacy RSM calibration migration");
    check(near(agronozzle::migrateV2RsmQ3(1.78f), 1.54f),
          "V2 RSM calibration migration from 4-bar reference");


    // Inverse pressure solver must round-trip the same curve and density model.
    check(near(agronozzle::targetPressureForFlow(1.18f * 8.0f, 8, 0, 1.18f, 1.0f),
               3.0f, 0.001f), "blue target-pressure round trip at 3 bar");
    check(near(agronozzle::targetPressureForFlow(0.96f * 5.0f, 5, 0, 1.18f, 1.0f),
               2.0f, 0.001f), "blue target-pressure lower boundary");
    const float rsm28At4 = agronozzle::calibratedWaterFlow(1.78f, 1.54f, 1, 1.28f);
    check(near(agronozzle::targetPressureForFlow(rsm28At4 * 8.0f, 8, 1, 1.54f, 1.28f),
               4.0f, 0.001f), "RSM 28 target-pressure round trip at 4 bar");
    const float rsm32At6 = agronozzle::calibratedWaterFlow(2.18f, 1.54f, 1, 1.32f);
    check(near(agronozzle::targetPressureForFlow(rsm32At6 * 8.0f, 8, 1, 1.54f, 1.32f),
               6.0f, 0.001f), "RSM 32 target-pressure upper boundary");
    check(std::isnan(agronozzle::targetPressureForFlow(0.95f * 8.0f, 8, 0, 1.18f, 1.0f)),
          "target below minimum nozzle output rejected");
    check(std::isnan(agronozzle::targetPressureForFlow(1.40f * 8.0f, 8, 0, 1.18f, 1.0f)),
          "target above maximum nozzle output rejected");
    check(std::isnan(agronozzle::targetPressureForFlow(NAN, 8, 0, 1.18f, 1.0f)),
          "NaN target flow rejected");
    check(std::isnan(agronozzle::targetPressureForFlow(10.0f, 0, 0, 1.18f, 1.0f)),
          "zero nozzle count rejected");
    check(std::isnan(agronozzle::targetPressureForFlow(10.0f, 8, 1, 1.54f, NAN)),
          "invalid liquid density rejected");

    std::cout << "OK: " << checks << " nozzle calibration assertions passed\n";
}
