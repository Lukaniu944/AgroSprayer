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
// A queued STOP must dominate an AUTO toggle consumed in the same loop pass.
inline bool shouldHonorAutoToggle(bool stopRequested, bool toggleRequested) {
    return toggleRequested && !stopRequested;
}
} // namespace agrovalve
