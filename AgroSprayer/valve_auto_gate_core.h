#pragma once
#include <cstdint>

// Hardware-independent state for the AUTO pulse permit and pressure watchdog.
// Production callers serialize every permit operation with valve_test.cpp's portMUX.
namespace valvegate {
class AutoPulsePermit {
    uint32_t generation_ = 0;
    bool enabled_ = false;
public:
    uint32_t generation() const { return generation_; }
    bool arm(uint32_t expectedGeneration) {
        if (expectedGeneration != generation_) return false;
        enabled_ = true;
        return true;
    }
    void stop() {
        enabled_ = false;
        ++generation_;
    }
    bool allows() const { return enabled_; }
};

struct WatchdogState {
    bool pressureFaultActive = false;
};

struct WatchdogDecision {
    bool forceOutputsOff = false;
    bool invalidateAutoPermit = false;
};

// A pressure fault must force outputs off on every watchdog tick, but the AUTO
// generation is invalidated only on the transition into the fault. Recovery
// never re-arms AUTO; a later fault is a new transition and invalidates again.
inline WatchdogDecision watchdogStep(WatchdogState &state, bool pressureFault) {
    const bool enteringFault = pressureFault && !state.pressureFaultActive;
    state.pressureFaultActive = pressureFault;
    return {pressureFault, enteringFault};
}
} // namespace valvegate
