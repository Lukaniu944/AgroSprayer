#include "nozzle_flow_core.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

static bool near(float a, float b, float eps = 0.002f) {
    return std::isfinite(a) && std::fabs(a - b) <= eps;
}

int main() {
    using agronozzle::nozzle_flow_from_table;
    int checks = 0;
    auto test = [&](bool condition, const char *name) {
        if (!condition) { std::cerr << "FAIL: " << name << "\n"; std::abort(); }
        ++checks;
    };
    test(near(nozzle_flow_from_table(2.0f,0,0,1.18f,1.0f),0.96f),
         "11003 water table at 2 bar");
    test(near(nozzle_flow_from_table(2.5f,0,0,1.18f,1.0f),1.08f),
         "11003 water table at 2.5 bar");
    test(near(nozzle_flow_from_table(3.0f,0,0,1.18f,1.0f),1.18f),
         "11003 water table at 3 bar");
    test(near(nozzle_flow_from_table(3.0f,0,0,1.30f,1.0f),1.30f),
         "11003 calibration scaling");

    test(near(nozzle_flow_from_table(3.0f,1,1,1.54f,1.28f),1.54f/1.13f),
         "RSM 28 water table corrected by 1.13");
    test(near(nozzle_flow_from_table(3.0f,1,2,1.54f,1.30f),1.54f/1.14f),
         "RSM 30 water table corrected by 1.14");
    test(near(nozzle_flow_from_table(3.0f,1,3,1.54f,1.32f),1.54f/1.15f),
         "RSM 32 water table corrected by 1.15");
    test(near(nozzle_flow_from_table(2.0f,1,3,1.54f,1.32f),1.26f/1.15f),
         "04RSM 2 bar corrected table value");
    test(near(nozzle_flow_from_table(4.0f,1,3,1.54f,1.32f),1.78f/1.15f),
         "04RSM 4 bar corrected table value");
    test(near(nozzle_flow_from_table(5.0f,1,3,1.54f,1.32f),1.99f/1.15f),
         "04RSM 5 bar corrected table value");
    test(near(nozzle_flow_from_table(6.0f,1,3,1.54f,1.32f),2.18f/1.15f),
         "04RSM 6 bar corrected table value");

    test(near(nozzle_flow_from_table(3.0f,0,4,1.18f,1.28f),1.18f/std::sqrt(1.28f)),
         "custom liquid density correction on water table");
    test(near(nozzle_flow_from_table(0.1f,0,0,1.18f,1.0f),0.0f),
         "near-zero pressure");
    test(!std::isfinite(nozzle_flow_from_table(1.9f,0,0,1.18f,1.0f)),
         "pressure below table");
    test(!std::isfinite(nozzle_flow_from_table(6.1f,1,3,1.54f,1.32f)),
         "pressure above table");
    test(!std::isfinite(nozzle_flow_from_table(3.0f,1,4,1.54f,1.32f)),
         "custom liquid cannot use RSM table");
    test(!std::isfinite(nozzle_flow_from_table(3.0f,0,0,1.18f,NAN)),
         "invalid density");
    std::cout << "OK: " << checks << " nozzle-flow assertions passed\n";
    return 0;
}
