#pragma once
struct TankReading { float liters; float usedLiters; bool configured; bool saveOk; };
void tank_counter_begin();
void tank_counter_update();
TankReading tank_counter_read();
bool tank_counter_set(float liters, bool add);
