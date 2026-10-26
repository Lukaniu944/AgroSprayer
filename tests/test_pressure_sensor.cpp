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

    // Verify the trip boundary against the actual ADS1115 scale and divider.
    // ADS1115 PGA +/-4.096 V gives 125 uV/count; the 1.5x divider
    // compensation converts one ADC count to 187.5 uV at the sensor output.
    constexpr float kAds1115LsbV = 4.096f / 32768.0f;
    constexpr float kDividerCompensation = 1.5f;
    const float tripVoltage = 0.5f +
        (agrosensor::kPressureTripBar / (80.0f * agrosensor::kBarPerPsi)) *
        (4.5f - 0.5f);
    const float sensorLsbV = kAds1115LsbV * kDividerCompensation;
    near(tripVoltage, 4.053f, 0.002f, "4.9 bar trip corresponds to about 4.053 V sensor output");
    near(agrosensor::raw_pressure_bar_from_voltage(tripVoltage, 0.5f, 4.5f, 80.0f),
         agrosensor::kPressureTripBar, 0.001f, "calculated trip voltage maps back to 4.9 bar");
    test(agrosensor::raw_pressure_bar_from_voltage(
             tripVoltage - sensorLsbV, 0.5f, 4.5f, 80.0f) <
             agrosensor::kPressureTripBar,
         "one ADS1115 count below trip voltage remains below threshold");
    test(agrosensor::raw_pressure_bar_from_voltage(
             tripVoltage + sensorLsbV, 0.5f, 4.5f, 80.0f) >=
             agrosensor::kPressureTripBar,
         "one ADS1115 count above trip voltage crosses threshold");

    std::cout << "OK: " << checks << " pressure sensor assertions passed\n";
}
