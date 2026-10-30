#include "job_counter.h"
#include <Preferences.h>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

static uint32_t nowMs = 0;
static double liters = 0.0, hectares = 0.0, areaLiters = 0.0;
static uint64_t sprayMs = 0;
uint32_t millis() { return nowMs; }
bool bench_test_active() { return false; }
double nozzle_total_liters() { return liters; }
double nozzle_total_hectares() { return hectares; }
double nozzle_area_liters() { return areaLiters; }
uint64_t nozzle_total_spray_ms() { return sprayMs; }

struct SavedJobSnapshot {
    uint32_t magic;
    float liters;
    float hectares;
    float areaLiters;
    uint64_t sprayMs;
};

int main() {
    Preferences::storage.clear();
    Preferences::failBegin = false;
    Preferences::failWrite = false;
    nowMs = 1000;
    liters = hectares = areaLiters = 0.0;
    sprayMs = 0;

    job_counter_begin();
    assert(!job_counter_read().exists);
    assert(job_counter_new());
    assert(job_counter_read().running);

    // Simulate estimated model totals accumulated during an active job.
    nowMs = 1500;
    liters = 12.5;
    hectares = 0.012;
    areaLiters = 11.75;
    sprayMs = 1500;

    // Pause must stop accumulation even when the immediate Preferences write fails.
    Preferences::failWrite = true;
    assert(!job_counter_pause(true));
    JobReading paused = job_counter_read();
    assert(!paused.running);
    assert(!paused.saveOk);
    assert(std::fabs(paused.liters - 12.5f) < 1e-5f);
    assert(std::fabs(paused.hectares - 0.012f) < 1e-5f);
    assert(std::fabs(paused.areaLiters - 11.75f) < 1e-5f);
    assert(paused.sprayMs == 1500u);

    // A paused job must retry after the 5 s backoff without adding new totals.
    Preferences::failWrite = false;
    nowMs = 6499;
    job_counter_update();
    assert(!job_counter_read().saveOk); // 4,999 ms: not yet due
    nowMs = 6500;
    job_counter_update();
    assert(job_counter_read().saveOk);

    assert(Preferences::storage.size() == sizeof(SavedJobSnapshot));
    SavedJobSnapshot saved{};
    std::memcpy(&saved, Preferences::storage.data(), sizeof(saved));
    assert(saved.magic == 0x4a4f4232u);
    assert(std::fabs(saved.liters - 12.5f) < 1e-5f);
    assert(std::fabs(saved.hectares - 0.012f) < 1e-5f);
    assert(std::fabs(saved.areaLiters - 11.75f) < 1e-5f);
    assert(saved.sprayMs == 1500u);

    // Repeated updates while paused must not double-count the same model totals.
    nowMs = 12000;
    liters = 20.0;
    hectares = 0.02;
    areaLiters = 18.0;
    sprayMs = 3000;
    job_counter_update();
    JobReading after = job_counter_read();
    assert(!after.running);
    assert(after.saveOk);
    assert(std::fabs(after.liters - 12.5f) < 1e-5f);
    assert(std::fabs(after.hectares - 0.012f) < 1e-5f);
    assert(after.sprayMs == 1500u);

    std::cout << "OK: paused Preferences retry and no-double-count tests passed\n";
}
