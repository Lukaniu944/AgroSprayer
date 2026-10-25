#pragma once
#include <cmath>
#include <cstdint>
#include <atomic>

// Native-testable accounting core used by firmware.
// Timestamped section transitions come from local button/UDP receive time;
// they estimate command changes, not measured physical valve motion.
// Call settleBeforeSectionChange at the actual, timestamped mask transition,
// then sample() immediately with the new mask and its estimated total flow.
namespace agrovolume {
// Preserve the earliest section-change timestamp until the next model tick.
// Further transitions in that interval are deliberately not extrapolated.
// UINT32_MAX is reserved as empty; an event at that exact millisecond is
// conservatively rounded back by 1 ms, including across millis() rollover.
class SectionEventLatch {
public:
    void note(uint32_t eventMs) {
        if (eventMs == UINT32_MAX) eventMs = UINT32_MAX - 1u;
        uint32_t empty = UINT32_MAX;
        firstMs_.compare_exchange_strong(empty, eventMs,
                                          std::memory_order_release,
                                          std::memory_order_relaxed);
    }
    bool take(uint32_t &eventMs) {
        const uint32_t value = firstMs_.exchange(UINT32_MAX, std::memory_order_acq_rel);
        if (value == UINT32_MAX) return false;
        eventMs = value;
        return true;
    }
private:
    std::atomic<uint32_t> firstMs_{UINT32_MAX};
};

class SectionVolumeIntegrator {
public:
    void reset() { primed_ = false; lastValid_ = false; totalLiters_ = 0; }

    void sample(uint32_t nowMs, uint8_t sectionMask, float totalFlowLpm, bool valid) {
        valid = valid && std::isfinite(totalFlowLpm) && totalFlowLpm >= 0.0f;
        sectionMask &= 0x3Fu;
        if (primed_) {
            const uint32_t elapsed = nowMs - lastMs_;
            // An unannounced mask transition has no reliable event timestamp.
            // Never invent the amount sprayed across that interval.
            if (valid && lastValid_ && sectionMask == lastMask_ && elapsed <= 2000u)
                totalLiters_ += (static_cast<double>(lastFlow_) + totalFlowLpm)
                                * 0.5 * elapsed / 60000.0;
        }
        primed_ = true;
        lastMs_ = nowMs;
        lastMask_ = sectionMask;
        lastFlow_ = valid ? totalFlowLpm : 0.0f;
        lastValid_ = valid;
    }

    void settleBeforeSectionChange(uint32_t eventMs, bool sensorStillValid) {
        if (!primed_) return;
        const uint32_t elapsed = eventMs - lastMs_;
        // Zero-order hold only up to the known section-change event.
        // This is an estimate, not a measurement of physical valve closure.
        if (lastValid_ && sensorStillValid && elapsed <= 2000u)
            totalLiters_ += static_cast<double>(lastFlow_) * elapsed / 60000.0;
        lastMs_ = eventMs;
        lastValid_ = false; // Require a fresh post-transition sample.
    }

    double liters() const { return totalLiters_; }
private:
    bool primed_ = false;
    bool lastValid_ = false;
    uint32_t lastMs_ = 0;
    uint8_t lastMask_ = 0;
    float lastFlow_ = 0.0f;
    double totalLiters_ = 0;
};
} // namespace agrovolume
