#include "auto_regulator.h"
#include "sprayer_state.h"
#include "nozzle_model.h"
#include "pressure_sensor.h"
#include "valve_test.h"
#include "board_config.h"
#include <Arduino.h>
#include <cmath>

namespace {
// Nodet-type bypass valve: CLOSING the bypass increases boom pressure.
// The valve-open mapping is calibrated using the existing A/B direction button.
// If the hydraulic arrangement is different, do not arm until verified on a bench.
constexpr int PRESSURE_RISE_LOGICAL_DIRECTION = -1;
constexpr int MAX_PRESSURE_X10 = 50;        // 5.0 bar - emergency stop
constexpr float MAX_TARGET_BAR = 4.8f;     // below 80 psi sensor limit (~5.52 bar)
constexpr uint32_t MAX_PULSE_MS = 250;     // short, non-queued motor commands
constexpr int MAX_NO_RESPONSE_PULSES = 6;
bool armed = false;
const char *state = "WYLACZONY";
uint32_t lastTick = 0;
uint32_t lastPulse = 0;
int baselineX10 = 0;
int lastExpectedSign = 0;
int unansweredPulses = 0;

void fail(const char *reason) {
    armed = false;
    valve_test_stop();
    state = reason;
    lastExpectedSign = 0;
    unansweredPulses = 0;
}
bool online() {
    return gState.wifiOk && gState.aogOk && gState.rxPackets > 0 &&
           (uint32_t)(millis() - gState.lastAogMs) <= AOG_TIMEOUT_MS;
}
} // namespace

bool auto_regulator_enabled() { return armed; }
const char *auto_regulator_status() { return state; }

void auto_regulator_stop() {
    armed = false;
    valve_test_stop();
    state = "WYLACZONY";
    lastTick = lastPulse = 0;
    lastExpectedSign = unansweredPulses = 0;
}

bool auto_regulator_arm() {
    if (bench_test_active() || !gState.autoMode || !online() ||
        !pressure_sensor_valid() || !valve_test_ready()) {
        state = "BRAK WARUNKOW DO WLACZENIA";
        return false;
    }
    if (pressure_sensor_bar_x10() >= MAX_PRESSURE_X10) {
        state = "ZBYT WYSOKIE CISNIENIE";
        return false;
    }
    // This arm is intentionally not persisted. It is a new operator action
    // for every boot and after a fault or mode change.
    valve_test_stop();
    armed = true;
    state = "CZEKAM NA OPRYSK";
    lastTick = millis();
    lastPulse = 0;
    lastExpectedSign = unansweredPulses = 0;
    return true;
}

void auto_regulator_update() {
    if (!armed) return;
    const uint32_t now = millis();
    if (lastTick && (uint32_t)(now - lastTick) > 2000) {
        fail("ZA DLUGA PRZERWA PROGRAMU");
        return;
    }
    lastTick = now;
    if (bench_test_active() || !gState.autoMode || !online()) {
        fail("UTRATA AOG / ZMIANA TRYBU");
        return;
    }
    if (!pressure_sensor_valid()) {
        fail("BRAK ODCZYTU CISNIENIA");
        return;
    }
    const int measuredX10 = pressure_sensor_bar_x10();
    if (measuredX10 >= MAX_PRESSURE_X10) {
        fail("ALARM: CISNIENIE >= 5 BAR");
        return;
    }
    if (valve_test_armed()) {
        fail("TEST RECZNY AKTYWNY");
        return;
    }
    bool any = false;
    for (int i = 0; i < SECTION_COUNT; ++i) any |= gState.sections[i];
    if (!any || gState.speedX10 <= 0) {
        valve_test_stop();
        lastPulse = 0;
        lastExpectedSign = unansweredPulses = 0;
        state = "CZEKAM NA OPRYSK";
        return;
    }

    const float target = nozzle_target_bar();
    if (!std::isfinite(target) || target < 2.0f || target > MAX_TARGET_BAR) {
        valve_test_stop();
        lastPulse = 0;
        lastExpectedSign = unansweredPulses = 0;
        state = "CEL CISNIENIA POZA ZAKRESEM";
        return;
    }
    const float error = target - measuredX10 / 10.0f;
    const float tolerance = fmaxf(0.15f, target * regulator_settings().tolerancePercent / 100.0f);
    if (fabsf(error) <= tolerance) {
        lastExpectedSign = unansweredPulses = 0;
        state = "W CELU";
        return;
    }

    // Respect the watchdog-controlled pulse and a quiet period for the
    // pressure sensor to settle. Never queue or extend motor commands.
    const uint32_t pulseMs = regulator_settings().pulseMs < MAX_PULSE_MS
        ? regulator_settings().pulseMs : MAX_PULSE_MS;
    if (valve_test_direction() != 0) {
        state = "TRWA IMPULS";
        return;
    }
    if (lastPulse && (uint32_t)(now - lastPulse) < pulseMs + regulator_settings().pauseMs) {
        state = "PRZERWA PO IMPULSIE";
        return;
    }
    if (lastExpectedSign) {
        const int change = (measuredX10 - baselineX10) * lastExpectedSign;
        if (change <= -2) {
            fail("ODWROTNA REAKCJA CISNIENIA");
            return;
        }
        if (change >= 1) {
            unansweredPulses = 0;
        } else if (++unansweredPulses >= MAX_NO_RESPONSE_PULSES) {
            fail("ZAWOR: BRAK REAKCJI CISNIENIA");
            return;
        }
    }

    // A bypass CLOSURE increases pressure; its OPENING decreases pressure.
    // valve_auto_pulse maps logical OPEN/CLOSE to the chosen A/B polarity.
    const int expectedSign = error > 0 ? 1 : -1;
    const int logicalDirection = expectedSign * PRESSURE_RISE_LOGICAL_DIRECTION;
    if (valve_auto_pulse(logicalDirection, pulseMs)) {
        baselineX10 = measuredX10;
        lastExpectedSign = expectedSign;
        lastPulse = now;
        state = expectedSign > 0 ? "PODNOSZE CISNIENIE" : "OBNIZAM CISNIENIE";
    } else {
        state = "OCZEKIWANIE NA ZAWOR";
    }
}
