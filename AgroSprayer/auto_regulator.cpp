#include "auto_regulator.h"
#include "pressure_control_core.h"
#include "nozzle_model.h"
#include "pressure_sensor.h"
#include "sprayer_state.h"
#include "valve_test.h"
#include "board_config.h"
#include <Arduino.h>
#include <atomic>
#include <cmath>

namespace {
agroreg::Controller controller;
std::atomic<bool> requestToggle{false};
std::atomic<bool> requestStop{false};
std::atomic<bool> enabled{false};
std::atomic<const char *> shownStatus{"WYLACZONY"};

agroreg::Input snapshot() {
    agroreg::Input s;
    s.now = millis();
    s.autoMode = gState.autoMode;
    s.connected = gState.wifiOk && gState.udpOk && gState.aogOk &&
                  gState.rxPackets != 0 &&
                  (uint32_t)(s.now-gState.lastAogMs) <= AOG_TIMEOUT_MS;
    s.sensorValid = pressure_sensor_valid();
    s.driverReady = valve_test_ready();
    s.benchMode = bench_test_active();
    // Manual diagnostic UNLOCK is also an interlock against automatic arming.
    s.motorMoving = valve_test_direction() != 0 || valve_test_armed();
    s.speedX10 = gState.speedX10;
    for (int i=0; i<SECTION_COUNT; ++i) s.sectionsOpen |= gState.sections[i];
    s.pressureBar = pressure_sensor_bar_x10()/10.0f;
    s.targetBar = nozzle_target_bar();
    const auto &v=regulator_settings();
    s.pulseMs=v.pulseMs;
    s.pauseMs=v.pauseMs;
    s.tolerancePercent=v.tolerancePercent;
    return s;
}
}
bool auto_regulator_armed() { return enabled.load(std::memory_order_acquire); }
const char *auto_regulator_status() { return shownStatus.load(std::memory_order_acquire); }

void auto_regulator_toggle_request() {
    requestToggle.store(true,std::memory_order_release);
}
void auto_regulator_stop() {
    requestToggle.store(false,std::memory_order_release);
    requestStop.store(true,std::memory_order_release);
    enabled.store(false,std::memory_order_release);
    valve_test_stop();
}
void auto_regulator_update() {
    // Capture the STOP epoch before processing; stale pulses are rejected by the driver.
    const uint32_t generation = valve_auto_generation();
    if (requestStop.exchange(false,std::memory_order_acq_rel)) {
        controller.disarm();
        shownStatus.store(controller.status(),std::memory_order_release);
    }
    if (requestToggle.exchange(false,std::memory_order_acq_rel)) {
        if (controller.armed()) {
            controller.disarm();
            valve_test_stop();
        } else {
            controller.arm(snapshot());
        }
    }
    const agroreg::Input s=snapshot();
    const agroreg::Action a=controller.step(s);
    if (a.stopMotor) valve_test_stop();
    if (a.direction) {
        if (!valve_auto_pulse(a.direction,a.pulseMs,generation)) {
            if (generation != valve_auto_generation()) {
                // A concurrent operator STOP already disabled the outputs.
                controller.disarm();
            } else {
                controller.driverFailed();
                valve_test_stop();
            }
        }
    }
    enabled.store(controller.armed(),std::memory_order_release);
    shownStatus.store(controller.status(),std::memory_order_release);
}
