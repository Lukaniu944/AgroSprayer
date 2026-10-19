#pragma once
#include <Arduino.h>
bool io_extension_begin();
bool io_extension_write(uint8_t pin, bool value);
void backlight_on();
void backlight_off();
