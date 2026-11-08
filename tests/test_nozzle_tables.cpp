#include "../AgroSprayer/nozzle_calibration_core.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
    int checks = 0;
    auto expect = [&](bool ok, const char *name) {
        if (!ok) {
            std::cerr << "FAIL: " << name << "\n";
            std::abort();
        }
        ++checks;
    };
    auto near = [&](float actual, float expected, const char *name) {
        expect(std::isfinite(actual) && std::fabs(actual - expected) < 0.0001f, name);
    };

    const float waterPressure[] = {2.0f, 2.5f, 3.0f, 3.5f, 4.0f};
    const float waterFlow[] = {0.96f, 1.08f, 1.18f, 1.27f, 1.39f};
    const float rsmPressure[] = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    const float rsmFlow[] = {1.26f, 1.54f, 1.78f, 1.99f, 2.18f};

    for (int i = 0; i < 5; ++i) {
        near(agronozzle::nozzleTableFlow(waterPressure[i], 0), waterFlow[i],
             "11003 water table knot");
        near(agronozzle::nozzleTableFlow(rsmPressure[i], 1), rsmFlow[i],
             "04RSM RSM table knot");
    }

    const float waterMid = std::sqrt(0.96f * 0.96f +
        0.5f * (1.08f * 1.08f - 0.96f * 0.96f));
    near(agronozzle::nozzleTableFlow(2.25f, 0), waterMid,
         "11003 squared-flow interpolation");
    const float rsmMid = std::sqrt(1.26f * 1.26f +
        0.5f * (1.54f * 1.54f - 1.26f * 1.26f));
    near(agronozzle::nozzleTableFlow(2.5f, 1), rsmMid,
         "04RSM squared-flow interpolation");

    near(agronozzle::nozzleTableFlow(0.0f, 0), 0.0f,
         "zero pressure has zero flow");
    expect(!std::isfinite(agronozzle::nozzleTableFlow(4.01f, 0)),
           "11003 rejects pressure above its table");
    expect(!std::isfinite(agronozzle::nozzleTableFlow(6.01f, 1)),
           "04RSM rejects pressure above its table");
    expect(!std::isfinite(agronozzle::nozzleTableFlow(3.0f, 2)),
           "unknown nozzle profile is rejected");

    near(agronozzle::calibratedNozzleFlow(1.18f, 1.18f, 0, 1.0f),
         1.18f, "11003 water calibration at reference Q3");
    near(agronozzle::calibratedNozzleFlow(1.18f, 1.18f, 0, 1.28f),
         1.18f / std::sqrt(1.28f), "RSM 28 correction on water table");
    near(agronozzle::calibratedNozzleFlow(1.18f, 1.18f, 0, 1.32f),
         1.18f / std::sqrt(1.32f), "RSM 32 correction on water table");
    near(agronozzle::calibratedNozzleFlow(1.54f, 1.54f, 1, 1.28f),
         1.54f, "04RSM direct RSM table has no second density correction");
    near(agronozzle::calibratedNozzleFlow(1.54f, 1.54f, 1, 1.32f),
         1.54f, "04RSM direct RSM table is independent of density factor");

    near(std::sqrt(1.28f), 1.13137f, "RSM 28 factor is dimensionless sqrt density");
    near(std::sqrt(1.32f), 1.14891f, "RSM 32 factor is dimensionless sqrt density");
    near(agronozzle::migrateRsmQ3ToDirect(1.54f, 1.28f), 1.54f,
         "default RSM table reference is not reduced during migration");
    near(agronozzle::migrateRsmQ3ToDirect(2.0f, 1.28f),
         2.0f / std::sqrt(1.28f), "custom legacy RSM Q3 migrates to direct units");

    near(agronozzle::maxTargetPressureBar(0), 4.0f,
         "11003 automatic target remains at 4 bar maximum");
    near(agronozzle::maxTargetPressureBar(1), 4.8f,
         "04RSM automatic target remains below 4.9 bar trip");

    std::cout << "OK: " << checks << " nozzle table assertions passed\n";
    return 0;
}
