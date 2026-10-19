#pragma once
#include <Arduino.h>
struct JobReading {float liters;float hectares;float areaLiters;uint64_t sprayMs;bool exists;bool running;bool saveOk;};
void job_counter_begin();
void job_counter_update();
JobReading job_counter_read();
bool job_counter_new();
bool job_counter_pause(bool pause);
