#pragma once
// Preserve automatic valve pulses during the UI refresh while keeping
// emergency stops active in bench mode and unarmed AUTO mode.
#include <cstdint>
namespace agrovalve {
// Checked under the same hardware lock that STOP uses.
inline bool autoPulseStillValid(uint32_t sampledGeneration, uint32_t currentGeneration) {
    return sampledGeneration == currentGeneration;
}
inline bool shouldStopMotorForUi(bool autoMode, bool benchMode,
                                 bool regulatorArmed, bool motorMoving) {
    return motorMoving && (benchMode || (autoMode && !regulatorArmed));
}
} // namespace agrovalve
