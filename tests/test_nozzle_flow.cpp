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
    test(near(nozzle_flow_from_table(3.0f,0,0,1.18f,1.0f),1.18f),
         "11003 water table");
    test(near(nozzle_flow_from_table(3.0f,0,0,1.30f,1.0f),1.30f),
         "11003 calibration scaling");
    test(near(nozzle_flow_from_table(3.0f,1,1,1.54f,1.28f),1.54f),
         "RSM 28 is not corrected twice");
    test(near(nozzle_flow_from_table(3.0f,1,2,1.54f,1.30f),1.54f),
         "RSM 30 is not corrected twice");
    test(near(nozzle_flow_from_table(3.0f,1,3,1.54f,1.32f),1.54f),
         "RSM 32 is not corrected twice");
    test(near(nozzle_flow_from_table(2.0f,1,3,1.54f,1.32f),1.26f),
         "04RSM 2 bar table value");
    test(near(nozzle_flow_from_table(4.0f,1,3,1.54f,1.32f),1.78f),
         "04RSM 4 bar table value");
    test(near(nozzle_flow_from_table(5.0f,1,3,1.54f,1.32f),1.99f),
         "04RSM 5 bar table value");
    test(near(nozzle_flow_from_table(6.0f,1,3,1.54f,1.32f),2.18f),
         "04RSM 6 bar table value");
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
