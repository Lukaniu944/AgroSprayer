#pragma once
#include <Arduino.h>
struct NozzleSettings {
    uint32_t magic=0x4e5a4c33;
    uint8_t nozzle=0, liquid=0;
    uint8_t counts[6]={5,8,8,8,8,5};
    float waterQ3[2]={1.18f,1.54f};
    float density=1.0f;
};
const NozzleSettings &nozzle_settings();
bool nozzle_profile_valid(const NozzleSettings &settings);
void nozzle_begin();
bool nozzle_save(const NozzleSettings &settings);
float nozzle_flow(float bar,const NozzleSettings &settings);
float nozzle_target_pressure(float totalLpm,int count,const NozzleSettings &settings);
void nozzle_update();
// Notify before changing any live section; eventMs is millis() at command receipt.
void nozzle_section_change_at(uint32_t eventMs);
bool nozzle_reading_valid();
double nozzle_total_liters();
double nozzle_total_hectares();
double nozzle_area_liters();
uint64_t nozzle_total_spray_ms();
const char *nozzle_name();
const char *nozzle_liquid_name();
const char *nozzle_status();
float nozzle_target_bar();
