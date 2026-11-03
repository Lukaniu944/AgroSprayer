#include "../AgroSprayer/section_volume_core.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <cstdlib>

int main() {
    int checks = 0;
    auto check = [&](bool ok, const char *name) {
        if (!ok) { std::cerr << "FAIL: " << name << '\n'; std::abort(); }
        ++checks;
    };
    auto near = [](double a, double b) { return std::fabs(a - b) < 1e-8; };
    agrovolume::SectionEventLatch latch;
    uint32_t event = 0;
    check(!latch.take(event), "empty event latch");
    latch.note(1200u);
    latch.note(1300u);
    check(latch.take(event) && event == 1200u, "first transition wins");
    check(!latch.take(event), "event consumed exactly once");
    latch.note(UINT32_MAX);
    check(latch.take(event) && event == UINT32_MAX - 1u, "wrap sentinel handled");
    latch.note(1500u);
    check(latch.take(event) && event == 1500u, "latch rearmed");
    agrovolume::SectionVolumeIntegrator v;
    v.sample(1000, 0x3f, 60.0f, true);
    check(near(v.liters(), 0), "first sample is baseline");
    v.sample(2000, 0x3f, 60.0f, true);
    check(near(v.liters(), 1.0), "one liter per second at 60 lpm");
    v.settleBeforeSectionChange(2500, true);
    check(near(v.liters(), 1.5), "old section mask settled to event timestamp");
    v.sample(2500, 0x1f, 30.0f, true);
    check(near(v.liters(), 1.5), "new mask at event does not double count");
    v.sample(3500, 0x1f, 30.0f, true);
    check(near(v.liters(), 2.0), "new mask integrated from event");
    v.sample(3700, 0x0f, 20.0f, true);
    check(near(v.liters(), 2.0), "unknown section transition never invents liters");
    v.sample(4700, 0x0f, 20.0f, true);
    check(near(v.liters(), 2.0 + 1.0 / 3.0), "normal sampling resumes after transition");
    v.sample(4800, 0x0f, NAN, true);
    check(near(v.liters(), 2.0 + 1.0 / 3.0), "invalid pressure estimate does not integrate");
    v.sample(4900, 0x0f, 20.0f, true);
    check(near(v.liters(), 2.0 + 1.0 / 3.0), "first valid reading rebaselines");
    v.sample(8000, 0x0f, 20.0f, true);
    check(near(v.liters(), 2.0 + 1.0 / 3.0), "long scheduling gap not extrapolated");
    v.reset();
    check(near(v.liters(), 0), "reset clears total");
    v.sample(UINT32_MAX - 500u, 0x3f, 60.0f, true);
    v.sample(499u, 0x3f, 60.0f, true);
    check(near(v.liters(), 1.0), "millis rollover integrates 1000 ms");
    v.settleBeforeSectionChange(999u, false);
    check(near(v.liters(), 1.0), "invalid sensor at event blocks estimate");
    v.sample(999u, 0x00, 0.0f, true);
    v.sample(1999u, 0x00, 0.0f, true);
    check(near(v.liters(), 1.0), "all sections off produces zero flow");
    v.sample(1999u, 0x00, -1.0f, true);
    check(near(v.liters(), 1.0), "negative flow invalidated");

    // Rapid section changes spanning millis() rollover: keep only the first
    // timestamp, settle the old mask once, then require a fresh post-change
    // sample. This must not double-count or extrapolate through later changes.
    {
        agrovolume::SectionEventLatch rolloverLatch;
        agrovolume::SectionVolumeIntegrator rolloverVolume;
        rolloverVolume.sample(UINT32_MAX - 1500u, 0x3fu, 60.0f, true);
        rolloverVolume.sample(UINT32_MAX - 1000u, 0x3fu, 60.0f, true);
        check(near(rolloverVolume.liters(), 0.5),
              "rollover transition scenario has 0.5 L baseline");
        rolloverLatch.note(UINT32_MAX - 500u);
        rolloverLatch.note(UINT32_MAX - 200u);
        check(rolloverLatch.take(event) && event == UINT32_MAX - 500u,
              "rapid rollover transitions preserve first event timestamp");
        check(!rolloverLatch.take(event),
              "rapid rollover event is consumed once");
        rolloverVolume.settleBeforeSectionChange(event, true);
        check(near(rolloverVolume.liters(), 1.0),
              "old flow settles once to first rollover event");
        rolloverVolume.sample(200u, 0x1fu, 30.0f, true);
        check(near(rolloverVolume.liters(), 1.0),
              "post-rollover transition sample only rebaselines");
        rolloverVolume.sample(1200u, 0x1fu, 30.0f, true);
        check(near(rolloverVolume.liters(), 1.5),
              "integration resumes after fresh post-rollover sample");
    }

    // A saved nozzle profile must not blend the old flow with the new one.
    v.reset();
    v.sample(1000u, 0x3f, 60.0f, true);
    v.sample(1600u, 0x3f, 60.0f, true);
    check(near(v.liters(), 0.6), "settle old nozzle profile before saving");
    v.sample(1600u, 0x3f, 0.0f, false);
    v.sample(1600u, 0x3f, 30.0f, true);
    check(near(v.liters(), 0.6), "profile switch at same timestamp adds nothing");
    v.sample(2600u, 0x3f, 30.0f, true);
    check(near(v.liters(), 1.1), "new profile integrates without mixing");
    v.sample(3600u, 0x3f, 30.0f, false);
    v.sample(4600u, 0x3f, 30.0f, true);
    check(near(v.liters(), 1.1), "invalid sensor interval does not add liters");
    // Volume, area and spray time must share the same section-event clock.
    v.reset();
    v.sample(1000u, 0x3f, 60.0f, true, 80, 21.0f);
    v.sample(2000u, 0x3f, 60.0f, true, 80, 21.0f);
    check(near(v.liters(), 1.0), "area test volume baseline");
    check(near(v.hectares(), 80.0 * 21.0 * 1000.0 / 360000000.0), "full width area 1 s");
    check(near(v.areaLiters(), 1.0), "moving liters match full-width volume");
    check(v.sprayMs() == 1000u, "full-width spray time");
    v.settleBeforeSectionChange(2500u, true);
    check(near(v.liters(), 1.5), "transition adds old-mask liters");
    check(near(v.hectares(), 80.0 * 21.0 * 1500.0 / 360000000.0), "transition adds old-mask area");
    check(v.sprayMs() == 1500u, "transition adds old-mask spray time");
    v.sample(2500u, 0x1f, 30.0f, true, 80, 17.0f);
    v.sample(3500u, 0x1f, 30.0f, true, 80, 17.0f);
    check(near(v.liters(), 2.0), "new-mask liters after transition");
    check(near(v.hectares(), (80.0 * 21.0 * 1500.0 + 80.0 * 17.0 * 1000.0) / 360000000.0),
          "new-mask hectares use 17 m not 21 m");
    check(near(v.areaLiters(), 2.0), "area liters align with event-segmented volume");
    check(v.sprayMs() == 2500u, "spray time aligns with volume");
    v.sample(3600u, 0x0f, 20.0f, true, 80, 13.0f);
    check(near(v.hectares(), (80.0 * 21.0 * 1500.0 + 80.0 * 17.0 * 1000.0) / 360000000.0),
          "unknown mask change adds no area");
    v.reset();
    v.sample(1000u, 0x3f, 60.0f, true, 0, 21.0f);
    v.sample(2000u, 0x3f, 60.0f, true, 0, 21.0f);
    check(near(v.liters(), 1.0) && near(v.hectares(), 0.0), "stationary spray counts liters but no hectares");
    check(near(v.areaLiters(), 0.0) && v.sprayMs() == 1000u, "stationary spray time, no area liters");
    v.sample(3000u, 0x3f, 60.0f, false, 80, 21.0f);
    check(near(v.hectares(), 0.0) && v.sprayMs() == 1000u, "stale sensor adds no area or time");
    v.reset();
    v.sample(UINT32_MAX - 500u, 0x3f, 60.0f, true, 80, 21.0f);
    v.sample(499u, 0x3f, 60.0f, true, 80, 21.0f);
    check(near(v.hectares(), 80.0 * 21.0 * 1000.0 / 360000000.0), "area handles millis rollover");
    v.reset();
    check(near(v.hectares(), 0.0) && near(v.areaLiters(), 0.0) && v.sprayMs() == 0,
          "reset clears area and spray time");
    std::cout << "OK: " << checks << " section volume assertions passed\n";
}
