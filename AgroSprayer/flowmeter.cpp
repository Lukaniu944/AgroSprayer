#include "flowmeter.h"
#include "board_config.h"
#include "sprayer_state.h"

#include <Arduino.h>
#include <Preferences.h>
#include "esp_timer.h"

static volatile uint32_t s_pulses = 0;
static portMUX_TYPE s_flowMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t s_lastCalcMs = 0;
static uint32_t s_lastPulses = 0;
static volatile uint32_t s_lastEdgeUs = 0;
static volatile bool s_haveEdge = false;
// 100 us permits > 4 kHz even at 1000 imp/L and 250 L/min.
static const uint32_t FLOW_MIN_EDGE_US = 100;
static const unsigned FLOW_WINDOW = 4;
static uint32_t s_windowPulses[FLOW_WINDOW] = {};
static uint32_t s_windowMs[FLOW_WINDOW] = {};
static unsigned s_windowIndex = 0;
static void reset_flow_window()
{
    for (unsigned i=0;i<FLOW_WINDOW;++i) {s_windowPulses[i]=0;s_windowMs[i]=0;}
    s_windowIndex=0;
}

// 12.00 imp/L = 1200
static uint32_t s_pulsesPerLiterX100 = 1200;

static Preferences s_flowPrefs;

// Calibration and total litres use all accepted edges, independently of display averaging.
// Electrical plausibility ceiling.
static const int FLOW_MAX_X10 = 2500; // 250.0 L/min

static const uint32_t FLOW_PPL_X100_MIN = 10;      // 0.10 imp/L
static const uint32_t FLOW_PPL_X100_MAX = 100000;  // 1000.00 imp/L

void IRAM_ATTR flow_isr()
{
    const uint32_t nowUs = (uint32_t)esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_flowMux);
    if (!s_haveEdge || (uint32_t)(nowUs-s_lastEdgeUs)>=FLOW_MIN_EDGE_US) {
        ++s_pulses;
        s_lastEdgeUs=nowUs;
        s_haveEdge=true;
    }
    portEXIT_CRITICAL_ISR(&s_flowMux);
}

static void flowmeter_load_calibration()
{
    s_flowPrefs.begin("agroflow", true);

    uint32_t saved =
        s_flowPrefs.getUInt(
            "pplx100",
            1200
        );

    s_flowPrefs.end();

    if (saved >= FLOW_PPL_X100_MIN &&
        saved <= FLOW_PPL_X100_MAX) {
        s_pulsesPerLiterX100 = saved;
    } else {
        s_pulsesPerLiterX100 = 1200;
    }
}

void flowmeter_begin()
{
    flowmeter_load_calibration();

    // The existing divider pulls down externally; this also biases an unplugged input.
    pinMode(FLOW_PIN, INPUT_PULLDOWN);
    portENTER_CRITICAL(&s_flowMux);
    s_pulses=0;s_lastEdgeUs=0;s_haveEdge=false;
    portEXIT_CRITICAL(&s_flowMux);
    reset_flow_window();

    attachInterrupt(
        digitalPinToInterrupt(FLOW_PIN),
        flow_isr,
        RISING
    );

    portENTER_CRITICAL(&s_flowMux);
    s_pulses = 0;
    portEXIT_CRITICAL(&s_flowMux);

    s_lastPulses = 0;
    s_lastCalcMs = millis();

    gState.actualFlowX10 = 0;
    gState.totalLitersX10 = 0;
}

void flowmeter_update()
{
    uint32_t now = millis();
    uint32_t dt = now - s_lastCalcMs;

    if (dt < FLOW_CALC_INTERVAL_MS) {
        return;
    }

    portENTER_CRITICAL(&s_flowMux);
    uint32_t pulses = s_pulses;
    portEXIT_CRITICAL(&s_flowMux);

    uint32_t delta = pulses - s_lastPulses;
    s_windowPulses[s_windowIndex]=delta;
    s_windowMs[s_windowIndex]=dt;
    s_windowIndex=(s_windowIndex+1)%FLOW_WINDOW;
    uint64_t windowPulses=0, windowMs=0;
    for(unsigned i=0;i<FLOW_WINDOW;++i) {
        windowPulses+=s_windowPulses[i];windowMs+=s_windowMs[i];
    }

    uint32_t pplX100 = s_pulsesPerLiterX100;

    if (pplX100 < FLOW_PPL_X100_MIN) {
        pplX100 = 1200;
    }

    // flowX10 =
    // pulses * 600000 / (pulses_per_liter * dt_ms)
    // pulses_per_liter = pplX100 / 100
    // => pulses * 600000 * 100 / (pplX100 * dt_ms)
    uint64_t denominator =
        (uint64_t)pplX100 *
        windowMs;

    int flowX10 = 0;

    if (denominator > 0) {
        uint64_t calc =
            windowPulses *
            600000ULL *
            100ULL /
            denominator;

        flowX10 = (int)calc;
    }

    if (flowX10 < 0 || flowX10 > FLOW_MAX_X10) {
        flowX10 = 0;
    }

    gState.actualFlowX10 = flowX10;

    // total L x10 = pulses / PPL * 10
    // PPL = pplX100 / 100
    // => pulses * 1000 / pplX100
    gState.totalLitersX10 =
        (int)(
            (uint64_t)pulses *
            1000ULL /
            (uint64_t)pplX100
        );

    s_lastPulses = pulses;
    s_lastCalcMs = now;
}

uint32_t flowmeter_get_total_pulses()
{
    portENTER_CRITICAL(&s_flowMux);
    uint32_t pulses = s_pulses;
    portEXIT_CRITICAL(&s_flowMux);

    return pulses;
}

uint32_t flowmeter_get_pulses_per_liter_x100()
{
    return s_pulsesPerLiterX100;
}

float flowmeter_get_pulses_per_liter()
{
    return
        (float)s_pulsesPerLiterX100 /
        100.0f;
}

bool flowmeter_set_pulses_per_liter_x100(uint32_t value)
{
    if (value < FLOW_PPL_X100_MIN || value > FLOW_PPL_X100_MAX) return false;
    if (!s_flowPrefs.begin("agroflow", false)) return false;
    const bool saved = s_flowPrefs.putUInt("pplx100", value) == sizeof(uint32_t);
    s_flowPrefs.end();
    if (!saved) return false;
    s_pulsesPerLiterX100 = value;

    portENTER_CRITICAL(&s_flowMux);
    s_lastPulses = s_pulses;
    portEXIT_CRITICAL(&s_flowMux);
    s_lastCalcMs = millis();
    reset_flow_window();
    gState.actualFlowX10 = 0;
    gState.totalLitersX10 = (int)((uint64_t)s_lastPulses * 1000ULL / value);
    return true;
}
