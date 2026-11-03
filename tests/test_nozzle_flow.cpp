#include "nozzle_flow_core.h"
#include "nozzle_model.h"
#include <type_traits>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

static_assert(std::is_same<decltype(nozzle_settings()), NozzleSettings>::value,
              "nozzle_settings must return a value snapshot, not a shared reference");

static bool near(float a, float b, float eps = 0.002f) {
    return std::isfinite(a) && std::fabs(a - b) <= eps;
}

int main() {
    using agronozzle::nozzle_flow_from_table;
    using agronozzle::nozzle_target_pressure_from_table;

    int checks = 0;
    auto test = [&](bool condition, const char *name) {
        if (!condition) {
            std::cerr << "FAIL: " << name << "\n";
            std::abort();
        }
        ++checks;
    };

    test(agronozzle::section_flow_state(true, false, 0) ==
             agronozzle::SectionFlowState::Simulation,
         "bench simulation has explicit status");
    test(agronozzle::section_flow_state(false, false, 0) ==
             agronozzle::SectionFlowState::NoAogData,
         "stale AOG data is not reported as intentional sections-off");
    test(agronozzle::section_flow_state(false, true, 0) ==
             agronozzle::SectionFlowState::AllSectionsOff,
         "fresh AOG with zero active nozzles is intentional sections-off");
    test(agronozzle::section_flow_state(false, true, 5) ==
             agronozzle::SectionFlowState::ActiveSections,
         "fresh AOG with active nozzles enables pressure-based estimate");

    // Manufacturer 11003 water table: exact published points.
    static constexpr float waterPressure[] = {2.0f, 2.5f, 3.0f, 3.5f, 4.0f};
    static constexpr float waterFlow[] = {0.96f, 1.08f, 1.18f, 1.27f, 1.39f};
    for (int i = 0; i < 5; ++i) {
        test(near(nozzle_flow_from_table(waterPressure[i], 0, 0, 1.18f, 1.0f),
                  waterFlow[i]), "11003 exact manufacturer table point");
    }
    test(near(nozzle_flow_from_table(3.0f, 0, 0, 1.30f, 1.0f), 1.30f),
         "11003 calibration scaling");

    // Manufacturer 04RSM water-reference table, corrected to each RSM grade.
    static constexpr float rsmPressure[] = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    static constexpr float rsmWaterFlow[] = {1.26f, 1.54f, 1.78f, 1.99f, 2.18f};
    static constexpr float correction[] = {1.13f, 1.14f, 1.15f};
    for (uint8_t liquid = 1; liquid <= 3; ++liquid) {
        for (int i = 0; i < 5; ++i) {
            const float q = nozzle_flow_from_table(
                rsmPressure[i], 1, liquid, 1.54f, 1.0f);
            test(near(q, rsmWaterFlow[i] / correction[liquid - 1]),
                 "04RSM exact table point with RSM correction");
        }
        test(!std::isfinite(nozzle_flow_from_table(
                 1.9f, 1, liquid, 1.54f, 1.0f)),
             "04RSM lower boundary rejected for each RSM grade");
        test(!std::isfinite(nozzle_flow_from_table(
                 6.1f, 1, liquid, 1.54f, 1.0f)),
             "04RSM upper boundary rejected for each RSM grade");
    }

    // Interpolation follows the square-of-flow / pressure relationship.
    const float expectedWaterMid = std::sqrt(
        1.18f * 1.18f + 0.5f * (1.27f * 1.27f - 1.18f * 1.18f));
    test(near(nozzle_flow_from_table(3.25f, 0, 0, 1.18f, 1.0f),
              expectedWaterMid), "11003 interpolation at 3.25 bar");
    const float expectedRsmMid = std::sqrt(
        1.78f * 1.78f + 0.5f * (1.99f * 1.99f - 1.78f * 1.78f)) / 1.14f;
    test(near(nozzle_flow_from_table(4.5f, 1, 2, 1.54f, 1.0f),
              expectedRsmMid), "04RSM RSM 30 interpolation at 4.5 bar");

    test(near(nozzle_flow_from_table(0.1f, 0, 0, 1.18f, 1.0f), 0.0f),
         "near-zero pressure");
    test(!std::isfinite(nozzle_flow_from_table(1.9f, 0, 0, 1.18f, 1.0f)),
         "11003 pressure below table");
    test(!std::isfinite(nozzle_flow_from_table(4.1f, 0, 0, 1.18f, 1.0f)),
         "11003 pressure above table");
    test(!std::isfinite(nozzle_flow_from_table(3.0f, 1, 4, 1.54f, 1.32f)),
         "custom liquid cannot use RSM table");
    test(!std::isfinite(nozzle_flow_from_table(3.0f, 0, 0, 1.18f, NAN)),
         "invalid density");

    test(near(nozzle_target_pressure_from_table(
                  1.18f, 1, 0, 0, 1.18f, 1.0f), 3.0f, 0.001f),
         "11003 target pressure inversion at 3 bar");
    const float rsm30At48 = nozzle_flow_from_table(4.8f, 1, 2, 1.54f, 1.30f);
    test(near(nozzle_target_pressure_from_table(
                  rsm30At48, 1, 1, 2, 1.54f, 1.30f), 4.8f, 0.001f),
         "RSM target pressure capped at 4.8 bar");
    test(!std::isfinite(nozzle_target_pressure_from_table(
                  nozzle_flow_from_table(5.0f, 1, 2, 1.54f, 1.30f),
                  1, 1, 2, 1.54f, 1.30f)),
         "RSM target beyond AUTO limit rejected");
    test(!std::isfinite(nozzle_target_pressure_from_table(
                  1.40f, 1, 0, 0, 1.18f, 1.0f)),
         "11003 target above 4 bar table rejected");
    test(!std::isfinite(nozzle_target_pressure_from_table(
                  5.0f, 0, 1, 2, 1.54f, 1.30f)),
         "zero nozzle count rejected");


    // Six-section 21 m boom: the configured geometry is 2.5/4/4/4/4/2.5 m
    // and 5/8/8/8/8/5 nozzles. At a constant application rate and speed,
    // closing any combination of sections must change total flow and nozzle
    // count proportionally, leaving the per-nozzle target pressure unchanged.
    static constexpr float sectionWidthM[] = {2.5f, 4.0f, 4.0f, 4.0f, 4.0f, 2.5f};
    static constexpr int sectionNozzles[] = {5, 8, 8, 8, 8, 5};
    constexpr float rateLHa = 200.0f;
    constexpr float speedKmh = 8.0f;
    auto targetForSectionMask = [&](uint8_t mask, uint8_t nozzle,
                                    uint8_t liquid, float calibratedQ3) {
        float activeWidthM = 0.0f;
        int activeNozzles = 0;
        for (int i = 0; i < 6; ++i) {
            if ((mask & (1u << i)) != 0) {
                activeWidthM += sectionWidthM[i];
                activeNozzles += sectionNozzles[i];
            }
        }
        const float totalLpm = rateLHa * speedKmh * activeWidthM / 600.0f;
        return nozzle_target_pressure_from_table(
            totalLpm, activeNozzles, nozzle, liquid, calibratedQ3, 1.0f);
    };

    float fullWidthM = 0.0f;
    int fullNozzles = 0;
    for (int i = 0; i < 6; ++i) {
        fullWidthM += sectionWidthM[i];
        fullNozzles += sectionNozzles[i];
    }
    test(near(fullWidthM, 21.0f) && fullNozzles == 42,
         "six-section reference configuration is 21 m and 42 nozzles");
    test(!std::isfinite(targetForSectionMask(0, 0, 0, 1.18f)),
         "all sections closed has no water pressure target");

    for (uint8_t mask = 1; mask < 64; ++mask) {
        test(near(targetForSectionMask(mask, 0, 0, 1.18f),
                  targetForSectionMask(0x3F, 0, 0, 1.18f), 0.002f),
             "11003 target pressure remains stable for every active section mask");
        for (uint8_t liquid = 1; liquid <= 3; ++liquid) {
            test(near(targetForSectionMask(mask, 1, liquid, 1.54f),
                      targetForSectionMask(0x3F, 1, liquid, 1.54f), 0.002f),
                 "04RSM target pressure remains stable for every active section mask");
        }
    }

    std::cout << "OK: " << checks << " nozzle-flow assertions passed\n";
    return 0;
}
