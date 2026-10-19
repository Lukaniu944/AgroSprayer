#pragma once
#include <Arduino.h>
#include "esp_lcd_panel_ops.h"

bool lvgl_port_begin(esp_lcd_panel_handle_t panel);
bool lvgl_lock(uint32_t timeout_ms = 1000);
void lvgl_unlock();
