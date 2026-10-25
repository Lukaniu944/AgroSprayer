#pragma once
#include <stdint.h>

// Liquid profile: 0=WODA, 1=RSM 28, 2=RSM 30, 3=RSM 32, 4=WLASNA.
// Water uses the blue 11003 nozzle; RSM 28/30/32 use the red 04RSM nozzle.
// A custom liquid may use either nozzle after the operator checks its setup.
inline bool nozzle_profile_compatible(uint8_t nozzle, uint8_t liquid)
{
    if (nozzle > 1 || liquid > 4) return false;
    if (liquid == 0) return nozzle == 0;
    if (liquid >= 1 && liquid <= 3) return nozzle == 1;
    return true;
}
