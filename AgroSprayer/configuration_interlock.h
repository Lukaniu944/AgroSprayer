#pragma once

#include "auto_regulator.h"
#include "configuration_interlock_core.h"
#include "sprayer_state.h"
#include "valve_test.h"

inline bool sprayer_configuration_edit_allowed() {
    bool sectionsActive = false;
    for (int i = 0; i < SECTION_COUNT; ++i) {
        sectionsActive = sectionsActive || gState.sections[i];
    }

    const bool valveBusy = valve_test_armed() || valve_test_direction() != 0;
    const agroconfig::ConfigurationEditState state{
        bench_test_active(),
        sectionsActive,
        auto_regulator_armed(),
        valveBusy
    };
    return agroconfig::configurationEditAllowed(state);
}
