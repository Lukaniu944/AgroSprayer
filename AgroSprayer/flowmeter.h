#pragma once
#include <Arduino.h>

void flowmeter_begin();
void flowmeter_update();

uint32_t flowmeter_get_total_pulses();

uint32_t flowmeter_get_pulses_per_liter_x100();
float flowmeter_get_pulses_per_liter();

bool flowmeter_set_pulses_per_liter_x100(uint32_t value);
