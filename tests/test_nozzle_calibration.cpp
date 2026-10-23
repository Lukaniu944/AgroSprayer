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

    // The liquid-density correction is the square root of relative density:
    // RSM 28: sqrt(1.28) ~= 1.13; RSM 30: sqrt(1.30) ~= 1.14;
    // RSM 32: sqrt(1.32) ~= 1.15.
    const float qWater = 1.54f;
    const float q28 = agronozzle::calibratedWaterFlow(qWater, qWater, 1, 1.28f);
    const float q30 = agronozzle::calibratedWaterFlow(qWater, qWater, 1, 1.30f);
    const float q32 = agronozzle::calibratedWaterFlow(qWater, qWater, 1, 1.32f);
    check(near(qWater / q28, 1.13f), "RSM 28 correction factor ~1.13");
    check(near(qWater / q30, 1.14f), "RSM 30 correction factor ~1.14");
    check(near(qWater / q32, 1.15f), "RSM 32 correction factor ~1.15");

    // Water calibration is applied once relative to the table's 3-bar value.
    check(near(agronozzle::calibratedWaterFlow(1.18f, 1.18f, 0, 1.0f),
               1.18f), "water 11003 default calibration at 3 bar");
    check(near(agronozzle::calibratedWaterFlow(1.54f, 1.54f, 1, 1.0f),
               1.54f), "04RSM default water reference at 3 bar");

    // Saved profiles from older firmware must retain the same 3-bar reference.
    check(near(agronozzle::migrateLegacyRsmQ3(1.54f), 1.54f),
          "legacy RSM calibration migration");
    check(near(agronozzle::migrateV2RsmQ3(1.78f), 1.54f),
          "V2 RSM calibration migration from 4-bar reference");

    std::cout << "OK: " << checks << " nozzle calibration assertions passed\n";
}
