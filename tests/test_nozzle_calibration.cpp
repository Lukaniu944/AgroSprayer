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

    check(near(agronozzle::migrateLegacyRsmQ3(1.54f), 1.54f),
          "legacy RSM calibration migration");
    check(near(agronozzle::migrateV2RsmQ3(1.78f), 1.54f),
          "V2 RSM calibration migration from 4-bar reference");

    std::cout << "OK: " << checks << " nozzle calibration assertions passed\n";
}
