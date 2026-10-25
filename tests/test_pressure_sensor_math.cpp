#include "../AgroSprayer/pressure_sensor_math.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

int main() {
    int checks = 0;
    auto check = [&](bool ok, const char *name) {
        if (!ok) {
            std::cerr << "FAIL: " << name << '\n';
            std::exit(1);
        }
        ++checks;
    };
    auto near = [](float a, float b, float tol = 0.001f) {
        return std::fabs(a - b) <= tol;
    };
    const float maxBar = 80.0f * agropres::kPsiToBar;

    check(near(agropres::sensorVoltageFromAdcRaw(0, 1.5f), 0.0f),
          "zero ADC raw converts to zero volts");
    check(near(agropres::sensorVoltageFromAdcRaw(24000, 1.5f), 4.5f, 0.0002f),
          "ADC raw for 3.0 V after divider reconstructs 4.5 V sensor output");
    check(std::isnan(agropres::sensorVoltageFromAdcRaw(100, 0.0f)),
          "zero divider ratio rejected");
    check(std::isnan(agropres::sensorVoltageFromAdcRaw(100, NAN)),
          "NaN divider ratio rejected");

    check(agropres::sensorVoltageValid(0.3f, 0.3f, 4.7f),
          "minimum sensor voltage boundary accepted");
    check(agropres::sensorVoltageValid(4.7f, 0.3f, 4.7f),
          "maximum sensor voltage boundary accepted");
    check(!agropres::sensorVoltageValid(0.299f, 0.3f, 4.7f),
          "voltage below sensor range rejected");
    check(!agropres::sensorVoltageValid(NAN, 0.3f, 4.7f),
          "NaN sensor voltage rejected");
    check(!agropres::sensorVoltageValid(1.0f, 2.0f, 1.0f),
          "reversed validity bounds rejected");

    float bar = -99.0f;
    check(agropres::sensorVoltageToBar(0.5f, 0.5f, 4.5f, 80.0f, bar) &&
          near(bar, 0.0f), "zero-pressure calibration point");
    check(agropres::sensorVoltageToBar(4.5f, 0.5f, 4.5f, 80.0f, bar) &&
          near(bar, maxBar), "full-scale calibration point");
    check(agropres::sensorVoltageToBar(2.5f, 0.5f, 4.5f, 80.0f, bar) &&
          near(bar, maxBar * 0.5f), "midpoint voltage maps to half full-scale pressure");
    check(agropres::sensorVoltageToBar(0.0f, 0.5f, 4.5f, 80.0f, bar) &&
          near(bar, 0.0f), "below-zero calibration output clamps to zero");
    check(agropres::sensorVoltageToBar(5.0f, 0.5f, 4.5f, 80.0f, bar) &&
          near(bar, maxBar), "above-full calibration output clamps to maximum");
    check(!agropres::sensorVoltageToBar(NAN, 0.5f, 4.5f, 80.0f, bar) &&
          near(bar, 0.0f), "NaN voltage rejected and output reset");
    check(!agropres::sensorVoltageToBar(1.0f, 2.0f, 1.0f, 80.0f, bar),
          "reversed calibration voltages rejected");
    check(!agropres::sensorVoltageToBar(1.0f, 0.5f, 4.5f, 0.0f, bar),
          "zero pressure range rejected");
    check(!agropres::sensorVoltageToBar(1.0f, 0.5f, 4.5f, INFINITY, bar),
          "infinite pressure range rejected");

    std::cout << "OK: " << checks << " pressure sensor math assertions passed\n";
}
