#include "../AgroSprayer/valve_auto_gate_core.h"
#include <cstdlib>
#include <iostream>

int main() {
    int checks = 0;
    const auto check = [&](bool condition, const char *name) {
        if (!condition) {
            std::cerr << "FAIL: " << name << "\n";
            std::abort();
        }
        ++checks;
    };

    valvegate::AutoPulsePermit permit;
    check(!permit.allows(), "AUTO pulse blocked on boot");

    const uint32_t firstGeneration = permit.generation();
    check(permit.arm(firstGeneration), "fresh explicit arm accepted");
    check(permit.allows(), "permit enabled after arm");

    // Model an action already calculated by the regulator before STOP.
    const int staleDirection = -1;
    permit.stop();
    check(staleDirection != 0, "stale action remains in caller");
    check(!permit.allows(), "STOP closes AUTO pulse permit");
    check(!permit.arm(firstGeneration), "pre-STOP generation cannot re-arm");
    check(!permit.allows(), "stale action cannot pass the closed permit");

    const uint32_t secondGeneration = permit.generation();
    check(permit.arm(secondGeneration), "new explicit arm uses fresh generation");
    check(permit.allows(), "fresh permit enabled");
    permit.stop();
    check(!permit.allows(), "second STOP closes permit");

    const uint32_t tokenBeforeAnotherStop = permit.generation();
    permit.stop();
    check(!permit.arm(tokenBeforeAnotherStop),
          "STOP between token capture and arm invalidates pending arm");

    std::cout << "OK: " << checks << " AUTO permit assertions passed\n";
    return 0;
}
