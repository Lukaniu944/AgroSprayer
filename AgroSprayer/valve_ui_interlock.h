#pragma once
// Preserve automatic valve pulses during the UI refresh while keeping
// emergency stops active in bench mode and unarmed AUTO mode.
namespace agrovalve {
inline bool shouldStopMotorForUi(bool autoMode, bool benchMode,
                                 bool regulatorArmed, bool motorMoving) {
    return motorMoving && (benchMode || (autoMode && !regulatorArmed));
}
} // namespace agrovalve
