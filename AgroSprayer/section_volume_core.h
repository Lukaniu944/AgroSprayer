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
    void reset() {
        primed_ = lastValid_ = lastSprayValid_ = lastAreaValid_ = false;
        totalLiters_ = totalHectares_ = areaLiters_ = 0.0;
        sprayMs_ = 0;
    }

    // All quantities are model estimates from commanded sections, not
    // feedback from physical valves or a flowmeter.
    void sample(uint32_t nowMs, uint8_t sectionMask, float totalFlowLpm, bool valid,
                int speedX10 = 0, float activeWidthM = 0.0f) {
        valid = valid && std::isfinite(totalFlowLpm) && totalFlowLpm >= 0.0f;
        sectionMask &= 0x3Fu;
        const bool spraying = valid && sectionMask != 0 && totalFlowLpm > 0.0f;
        const bool moving = spraying && speedX10 > 0 &&
                            std::isfinite(activeWidthM) && activeWidthM > 0.0f;
        if (primed_) {
            const uint32_t elapsed = nowMs - lastMs_;
            // Unknown section transitions and stale intervals must not
            // manufacture either volume or area.
            if (valid && lastValid_ && sectionMask == lastMask_ && elapsed <= 2000u) {
                const double liters = (static_cast<double>(lastFlow_) + totalFlowLpm)
                                      * 0.5 * elapsed / 60000.0;
                totalLiters_ += liters;
                if (lastSprayValid_ && spraying) sprayMs_ += elapsed;
                if (lastAreaValid_ && moving &&
                    std::fabs(activeWidthM - lastWidthM_) < 0.001f) {
                    // speedX10 is 0.1 km/h; width is meters; dt is ms.
                    totalHectares_ += (static_cast<double>(lastSpeedX10_) + speedX10)
                                      * 0.5 * activeWidthM * elapsed / 360000000.0;
                    areaLiters_ += liters;
                }
            }
        }
        primed_ = true;
        lastMs_ = nowMs;
        lastMask_ = sectionMask;
        lastFlow_ = valid ? totalFlowLpm : 0.0f;
        lastValid_ = valid;
        lastSprayValid_ = spraying;
        lastAreaValid_ = moving;
        lastSpeedX10_ = speedX10;
        lastWidthM_ = activeWidthM;
    }

    void settleBeforeSectionChange(uint32_t eventMs, bool sensorStillValid) {
        if (!primed_) return;
        const uint32_t elapsed = eventMs - lastMs_;
        // Zero-order hold only to the first known command transition.
        if (lastValid_ && sensorStillValid && elapsed <= 2000u) {
            const double liters = static_cast<double>(lastFlow_) * elapsed / 60000.0;
            totalLiters_ += liters;
            if (lastSprayValid_) sprayMs_ += elapsed;
            if (lastAreaValid_) {
                totalHectares_ += static_cast<double>(lastSpeedX10_) * lastWidthM_
                                  * elapsed / 360000000.0;
                areaLiters_ += liters;
            }
        }
        lastMs_ = eventMs;
        lastValid_ = lastSprayValid_ = lastAreaValid_ = false;
    }

    double liters() const { return totalLiters_; }
    double hectares() const { return totalHectares_; }
    double areaLiters() const { return areaLiters_; }
    uint64_t sprayMs() const { return sprayMs_; }

private:
    bool primed_ = false;
    bool lastValid_ = false;
    bool lastSprayValid_ = false;
    bool lastAreaValid_ = false;
    uint32_t lastMs_ = 0;
    uint8_t lastMask_ = 0;
    float lastFlow_ = 0.0f;
    int lastSpeedX10_ = 0;
    float lastWidthM_ = 0.0f;
    double totalLiters_ = 0.0;
    double totalHectares_ = 0.0;
    double areaLiters_ = 0.0;
    uint64_t sprayMs_ = 0;
};
} // namespace agrovolume
