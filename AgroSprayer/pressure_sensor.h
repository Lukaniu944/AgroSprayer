#pragma once
#include <Arduino.h>
void pressure_sensor_begin();
void pressure_sensor_update();
bool pressure_sensor_valid();
int pressure_sensor_bar_x10();
int pressure_sensor_signal_mv(); // Sensor voltage before the divider, -1 on I2C error.
