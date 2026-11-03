#pragma once
void aog_begin();
void aog_update();
// Returns true once if any AOG link interruption was observed since last model tick.
bool aog_take_telemetry_gap();
void aog_send_switch_now();

// Clear remote manual overrides before handing sections back to AUTO.
void aog_send_mode_now();
