#include "valve_auto_gate_core.h"
#include <cstdint>
#include <cstdlib>
#include <iostream>

int main() {
    int checks = 0;
    auto expect = [&](bool condition, const char *name) {
        if (!condition) {
            std::cerr << "FAIL: " << name << "\n";
            std::abort();
        }
        ++checks;
    };

    valvegate::AutoPulsePermit permit;
    const uint32_t firstToken = permit.generation();

    expect(!permit.allows(), "AUTO permit starts disabled");
    expect(permit.arm(firstToken), "current generation can arm AUTO");
    expect(permit.allows(), "successful arm enables AUTO");

    permit.stop();
    expect(!permit.allows(), "STOP disables AUTO permit");
    expect(permit.generation() == firstToken + 1,
           "STOP advances generation");
    expect(!permit.arm(firstToken),
           "token captured before STOP cannot re-arm AUTO");
    expect(!permit.allows(), "stale token leaves permit disabled");

    const uint32_t secondToken = permit.generation();
    expect(permit.arm(secondToken), "fresh token can arm after STOP");
    expect(permit.allows(), "fresh arm enables AUTO");

    permit.stop();
    expect(!permit.allows(), "second STOP disables an armed permit");
    const uint32_t thirdToken = permit.generation();
    expect(!permit.arm(secondToken),
           "second STOP invalidates the previously captured token");
    expect(!permit.allows(), "invalidated token cannot enable AUTO");

    expect(permit.arm(thirdToken), "new generation can arm again");
    const uint32_t staleToken = thirdToken;
    permit.stop();
    const uint32_t currentToken = permit.generation();
    expect(permit.arm(currentToken), "current token arms after another STOP");
    expect(!permit.arm(staleToken),
           "older token is rejected while a newer permit is armed");
    expect(permit.allows(),
           "rejected stale token does not cancel the valid current permit");

    permit.stop();
    expect(!permit.allows(), "final STOP leaves AUTO disabled");

    std::cout << "OK: " << checks
              << " AUTO permit generation assertions passed\n";
    return 0;
}
