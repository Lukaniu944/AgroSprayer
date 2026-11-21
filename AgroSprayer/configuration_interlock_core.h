#pragma once

namespace agroconfig {

struct ConfigurationEditState {
    bool benchTestActive;
    bool sectionsActive;
    bool regulatorArmed;
    bool valveBusy;
};

inline bool configurationEditAllowed(const ConfigurationEditState &state) {
    return !state.benchTestActive && !state.sectionsActive &&
           !state.regulatorArmed && !state.valveBusy;
}

} // namespace agroconfig
