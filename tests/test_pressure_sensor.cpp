#include "pressure_sensor_core.h"
#include "pressure_control_core.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
    int checks = 0;
    auto test = [&](bool condition, const char *name) {
        if (!condition) {
            std::cerr << "FAIL: " << name << "\n";
            std::abort();
        }
        ++checks;
    };
    auto near = [&](float actual, float expected, float tolerance, const char *name) {
        test(std::isfinite(actual) && std::fabs(actual - expected) <= tolerance, name);
    };

    test(agrosensor::voltage_valid(0.3f, 0.3f, 4.7f), "minimum sensor voltage is valid");
    test(agrosensor::voltage_valid(4.7f, 0.3f, 4.7f), "maximum sensor voltage is valid");
    test(!agrosensor::voltage_valid(0.299f, 0.3f, 4.7f), "below minimum voltage is invalid");
    test(!agrosensor::voltage_valid(4.701f, 0.3f, 4.7f), "above maximum voltage is invalid");
    test(!agrosensor::voltage_valid(NAN, 0.3f, 4.7f), "NaN sensor voltage is invalid");
    test(!agrosensor::voltage_valid(1.0f, 4.7f, 0.3f), "reversed voltage limits are invalid");

    near(agrosensor::raw_pressure_bar_from_voltage(0.5f, 0.5f, 4.5f, 80.0f),
         0.0f, 0.001f, "zero point maps to 0 bar");
    near(agrosensor::raw_pressure_bar_from_voltage(2.5f, 0.5f, 4.5f, 80.0f),
         40.0f * agrosensor::kBarPerPsi, 0.001f, "midpoint maps to 40 psi");
    near(agrosensor::raw_pressure_bar_from_voltage(4.5f, 0.5f, 4.5f, 80.0f),
         80.0f * agrosensor::kBarPerPsi, 0.001f, "full-scale maps to 80 psi");
    test(!std::isfinite(agrosensor::raw_pressure_bar_from_voltage(
             2.0f, 1.0f, 1.0f, 80.0f)), "zero calibration span is rejected");
    test(!std::isfinite(agrosensor::raw_pressure_bar_from_voltage(
             2.0f, 0.5f, 4.5f, 0.0f)), "non-positive sensor range is rejected");

    const float belowTrip = agrosensor::raw_pressure_bar_from_voltage(4.05f, 0.5f, 4.5f, 80.0f);
    const float aboveTrip = agrosensor::raw_pressure_bar_from_voltage(4.06f, 0.5f, 4.5f, 80.0f);
    test(belowTrip < agrosensor::kPressureTripBar, "4.05 V stays below 4.9 bar trip");
    test(aboveTrip >= agrosensor::kPressureTripBar, "4.06 V crosses 4.9 bar trip");
    test(!agrosensor::overpressure_trip(4.899f), "4.899 bar does not trip");
    test(agrosensor::overpressure_trip(4.9f), "exactly 4.9 bar trips");
    test(agrosensor::overpressure_trip(4.901f), "above 4.9 bar trips");
    test(!agrosensor::overpressure_trip(NAN), "NaN is not treated as a valid pressure");

    constexpr float kAds1115LsbV = 4.096f / 32768.0f;
    constexpr float kDividerCompensation = 1.5f;
    const float voltageBelow = agrosensor::sensor_voltage_from_ads1115_raw(
        21618, kAds1115LsbV, kDividerCompensation);
    const float voltageAbove = agrosensor::sensor_voltage_from_ads1115_raw(
        21619, kAds1115LsbV, kDividerCompensation);
    near(voltageBelow, 4.053375f, 0.00001f, "ADS1115 code 21618 maps below trip");
    near(voltageAbove, 4.0535625f, 0.00001f, "ADS1115 code 21619 maps above trip");
    test(agrosensor::raw_pressure_bar_from_voltage(
             voltageBelow, 0.5f, 4.5f, 80.0f) < agrosensor::kPressureTripBar,
         "last ADC code below trip remains safe");
    test(agrosensor::raw_pressure_bar_from_voltage(
             voltageAbove, 0.5f, 4.5f, 80.0f) >= agrosensor::kPressureTripBar,
         "first ADC code above trip trips");
    test(!std::isfinite(agrosensor::sensor_voltage_from_ads1115_raw(
             100, 0.0f, kDividerCompensation)), "zero ADC LSB is rejected");
    test(!std::isfinite(agrosensor::sensor_voltage_from_ads1115_raw(
             100, kAds1115LsbV, 0.0f)), "zero divider ratio is rejected");

    agroreg::Input in;
    in.now = 1000;
    in.autoMode = true;
    in.connected = true;
    in.sensorValid = true;
    in.driverReady = true;
    in.sectionsOpen = true;
    in.speedX10 = 80;
    in.pressureBar = 2.5f;
    in.targetBar = 3.0f;
    agroreg::Controller controller;
    test(controller.arm(in), "controller arms below pressure trip");
    controller.step(in);
    in.now += 50;
    in.pressureTrip = true;
    const auto trip = controller.step(in);
    test(trip.stopMotor && trip.direction == 0 && !controller.armed(),
         "latched raw-pressure trip stops and disarms regulator");


    agroreg::Input cooldown;
    cooldown.now = 2000;
    cooldown.autoMode = true;
    cooldown.connected = true;
    cooldown.sensorValid = true;
    cooldown.driverReady = true;
    cooldown.sectionsOpen = true;
    cooldown.speedX10 = 80;
    cooldown.pressureBar = 2.5f;
    cooldown.targetBar = 3.0f;
    cooldown.pulseMs = 50;
    cooldown.pauseMs = 100;
    agroreg::Controller cooldownController;
    test(cooldownController.arm(cooldown), "controller arms for valve dead-time test");
    test(cooldownController.step(cooldown).direction == -1, "first pulse starts before dead-time test");
    cooldown.now += 150;
    cooldown.pressureBar = 2.6f;
    cooldown.driverCanPulse = false;
    const auto waiting = cooldownController.step(cooldown);
    test(waiting.direction == 0 && !waiting.stopMotor && cooldownController.armed(),
         "valve dead-time waits without disarming AUTO");
    cooldown.now += 10;
    cooldown.driverCanPulse = true;
    test(cooldownController.step(cooldown).direction == -1 && cooldownController.armed(),
         "AUTO resumes after driver dead-time");

    std::cout << "OK: " << checks << " pressure sensor/interlock assertions passed\n";
}
