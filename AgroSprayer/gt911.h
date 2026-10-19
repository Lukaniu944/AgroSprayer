#pragma once
#include <Arduino.h>
struct TouchPoint { bool pressed; uint16_t x; uint16_t y; };
bool gt911_begin();
TouchPoint gt911_read();
