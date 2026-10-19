#pragma once
bool auto_regulator_armed();
const char *auto_regulator_status();
void auto_regulator_toggle_request(); // UI requests; loop owns controller state.
void auto_regulator_stop();           // Immediately kills motor and disarms next loop.
void auto_regulator_update();         // Called from Arduino loop, not LVGL.
