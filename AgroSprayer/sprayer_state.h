#pragma once
#include <Arduino.h>
#include "board_config.h"

struct SprayerState {
    int speedX10 = 0;
    int rateLHa = DEFAULT_RATE_L_HA;
    int actualFlowX10 = 0;
    int totalLitersX10 = 0;
    bool autoMode = true;
    bool sections[SECTION_COUNT] = {false,false,false,false,false,false};
    bool wifiOk = false;
    bool aogOk = false;
    bool udpOk = false;
    uint32_t rxPackets = 0;
    uint32_t txPackets = 0;
    uint32_t lastAogMs = 0;
};

extern SprayerState gState;
int activeWidthDm();
int targetFlowX10();
int flowErrorPercent();
const char *regulatorText();

// Live sensor/AOG data always remain in gState. UI tests use a separate state.
bool bench_test_active();
SprayerState &sprayer_display_state();
void bench_test_begin();
void bench_test_end();
void bench_test_set_speed_x10(int value);
void bench_test_set_flow_x10(int value);

void bench_test_tick();
void bench_test_set_auto(bool automatic);
int bench_test_open_percent();

struct RegulatorSettings {
    uint32_t pulseMs=250;
    uint32_t pauseMs=750;
    uint32_t tolerancePercent=5;
};
const RegulatorSettings &regulator_settings();
void regulator_settings_load();
bool regulator_settings_save(const RegulatorSettings &settings);
