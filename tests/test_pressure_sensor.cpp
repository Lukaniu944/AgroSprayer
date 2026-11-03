#include "pressure_sensor_core.h"
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

    // Verify the trip boundary using discrete ADS1115 output codes, not
    // arbitrary continuous voltages. With +/-4.096 V PGA and a 1.5x divider,
    // code 21618 is below 4.9 bar and code 21619 is the first code above it.
    constexpr float kAds1115LsbV = 4.096f / 32768.0f;
    constexpr float kDividerCompensation = 1.5f;
    constexpr int16_t kCodeBelowTrip = 21618;
    constexpr int16_t kCodeAboveTrip = 21619;
    const float voltageBelow = agrosensor::sensor_voltage_from_ads1115_raw(
        kCodeBelowTrip, kAds1115LsbV, kDividerCompensation);
    const float voltageAbove = agrosensor::sensor_voltage_from_ads1115_raw(
        kCodeAboveTrip, kAds1115LsbV, kDividerCompensation);
    near(voltageBelow, 4.053375f, 0.00001f,
         "ADS1115 code 21618 maps to sensor voltage below trip");
    near(voltageAbove, 4.0535625f, 0.00001f,
         "ADS1115 code 21619 maps to sensor voltage above trip");
    test(agrosensor::raw_pressure_bar_from_voltage(
             voltageBelow, 0.5f, 4.5f, 80.0f) < agrosensor::kPressureTripBar,
         "discrete ADS1115 code immediately below trip does not trip");
    test(agrosensor::raw_pressure_bar_from_voltage(
             voltageAbove, 0.5f, 4.5f, 80.0f) >= agrosensor::kPressureTripBar,
         "first discrete ADS1115 code above trip crosses threshold");
    test(!std::isfinite(agrosensor::sensor_voltage_from_ads1115_raw(
             100, 0.0f, kDividerCompensation)),
         "zero ADC LSB is rejected");
    test(!std::isfinite(agrosensor::sensor_voltage_from_ads1115_raw(
             100, kAds1115LsbV, 0.0f)),
         "zero divider ratio is rejected");

    std::cout << "OK: " << checks << " pressure sensor assertions passed\n";
}
