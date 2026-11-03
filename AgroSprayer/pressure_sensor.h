#pragma once
#include <Arduino.h>
void pressure_sensor_begin();
void pressure_sensor_update();
bool pressure_sensor_valid();
bool pressure_sensor_overpressure(); // Latched trip at raw measured pressure >= 4.9 bar.
int pressure_sensor_bar_x10();
int pressure_sensor_signal_mv(); // Sensor voltage before the divider, -1 on I2C error.
