#pragma once
#include <cstdint>

// Hardware-independent state for the AUTO pulse permit.
// Production callers serialize every operation with valve_test.cpp's portMUX.
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
} // namespace valvegate
