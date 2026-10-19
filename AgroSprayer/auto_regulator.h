#pragma once

// Regulation is OFF after every reboot and requires explicit operator arming.
// AUTO sections in AgOpenGPS do not automatically arm the pressure regulator.
bool auto_regulator_arm();
void auto_regulator_stop();
void auto_regulator_update();
bool auto_regulator_enabled();
const char *auto_regulator_status();
