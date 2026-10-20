#pragma once
#include <stdint.h>
bool valve_test_begin();
bool valve_test_ready();
void valve_test_arm();
void valve_test_stop(); // Stops and disarms.
bool valve_test_pulse(int direction); // +1 or -1; never queues or extends a pulse.
bool valve_test_armed();
int valve_test_direction();

// Direct operator command: one 250 ms pulse, no queue and no extension.
bool valve_manual_pulse(int direction); // +1=open, -1=close.
// Automatic regulation: single pulse <=250ms; mapping follows calibrated A/B.
uint32_t valve_auto_generation(); // STOP invalidates in-flight AUTO commands.
bool valve_auto_pulse(int logicalDirection, uint32_t durationMs, uint32_t generation);
int valve_open_direction();
bool valve_toggle_open_direction(); // Stops the motor, saves the mapping.
