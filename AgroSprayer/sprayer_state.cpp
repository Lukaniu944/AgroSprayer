#include "sprayer_state.h"
#include "ui.h"
#include <atomic>
#include "valve_test.h"
#include <Preferences.h>
SprayerState gState;
static SprayerState s_benchState;
static std::atomic<bool> s_benchActive{false};
static float s_opening=0.2f, s_modelFlow=0;
static uint32_t s_lastTick=0, s_pulseStart=0, s_nextPulse=0;
static int s_command=0;
static RegulatorSettings s_regulator;
static bool valid_regulator_settings(const RegulatorSettings &v) {
    return v.pulseMs>=50 && v.pulseMs<=1000 && v.pauseMs>=100 && v.pauseMs<=3000 &&
           v.tolerancePercent>=1 && v.tolerancePercent<=20;
}
const RegulatorSettings &regulator_settings() { return s_regulator; }
void regulator_settings_load() {
    s_regulator=RegulatorSettings{};
    Preferences prefs;
    if(!prefs.begin("agroreg",true))return;
    RegulatorSettings candidate;
    const size_t read=prefs.getBytes("settings",&candidate,sizeof(candidate));
    prefs.end();
    if(read==sizeof(candidate) && valid_regulator_settings(candidate))s_regulator=candidate;
}
bool regulator_settings_save(const RegulatorSettings &v) {
    if(!valid_regulator_settings(v))return false;
    Preferences prefs;
    if(!prefs.begin("agroreg",false))return false;
    const size_t written=prefs.putBytes("settings",&v,sizeof(v));
    prefs.end();
    if(written!=sizeof(v))return false;
    s_regulator=v;s_command=0;s_nextPulse=millis();
    return true;
}
static float bench_max_flow() { return ui_get_active_boom_width_m()*1200.0f/21.0f; }
int bench_test_open_percent() { return (int)(s_opening*100.0f+0.5f); }
void bench_test_set_auto(bool automatic) {
    if(!bench_test_active()) return;
    s_benchState.autoMode=automatic;
    s_command=0;s_nextPulse=millis();
}
void bench_test_tick() {
    if(!bench_test_active()) return;
    // This model never commands motor GPIOs. Stop any preceding real pulse.
    valve_test_stop();
    const uint32_t now=millis(), elapsed=now-s_lastTick;
    s_lastTick=now;
    const float dt=(elapsed>200 ? 200 : elapsed)/1000.0f;
    const int target=targetFlowX10();
    const bool running=s_benchState.autoMode && s_benchState.speedX10>0 &&
                       activeWidthDm()>0 && target>=REGULATOR_MIN_TARGET_X10;
    if(!running) s_command=0;
    if(s_command) {
        const uint32_t age=now-s_pulseStart;
        const uint32_t before=age>=elapsed ? age-elapsed : 0;
        const uint32_t moving=before>=s_regulator.pulseMs ? 0 : ((age<s_regulator.pulseMs?age:s_regulator.pulseMs)-before);
        s_opening+=s_command*(float)(moving>200?200:moving)/10000.0f;
        if(s_opening<0)s_opening=0;if(s_opening>1)s_opening=1;
        if(age>=s_regulator.pulseMs) {s_command=0;s_nextPulse=now+s_regulator.pauseMs;}
    } else if(running && (int32_t)(now-s_nextPulse)>=0) {
        const int error=flowErrorPercent();
        if(error < -(int)s_regulator.tolerancePercent && s_opening<1) s_command=1;
        else if(error > (int)s_regulator.tolerancePercent && s_opening>0) s_command=-1;
        if(s_command)s_pulseStart=now;
    }
    const float desired=activeWidthDm()>0 ? s_opening*bench_max_flow() : 0;
    s_modelFlow+=(desired-s_modelFlow)*(dt/(0.4f+dt));
    s_benchState.actualFlowX10=(int)(s_modelFlow+0.5f);
}


bool bench_test_active() {
    return s_benchActive.load(std::memory_order_acquire);
}

SprayerState &sprayer_display_state() {
    return bench_test_active() ? s_benchState : gState;
}

void bench_test_begin() {
    if (bench_test_active()) return;
    s_benchState = SprayerState{};
    s_benchState.rateLHa = gState.rateLHa;
    if (s_benchState.rateLHa < 10 || s_benchState.rateLHa > 1000)
        s_benchState.rateLHa = DEFAULT_RATE_L_HA;
    s_benchState.speedX10 = 80;
    s_benchState.autoMode = true;
    for (int i = 0; i < SECTION_COUNT; ++i) s_benchState.sections[i] = true;
    valve_test_stop();
    s_opening=0.2f;s_modelFlow=0;s_command=0;
    s_lastTick=s_nextPulse=millis();
    s_benchActive.store(true, std::memory_order_release);
}

void bench_test_end() {
    // No snapshot is written back: real data continued updating during the test.
    valve_test_stop();s_command=0;
    s_benchActive.store(false, std::memory_order_release);
}

void bench_test_set_speed_x10(int value) {
    if (!bench_test_active()) return;
    s_benchState.speedX10 = value < 0 ? 0 : (value > 300 ? 300 : value);
}

void bench_test_set_flow_x10(int value) {
    if (!bench_test_active()) return;
    s_benchState.actualFlowX10 = value < 0 ? 0 : (value > 2500 ? 2500 : value);
    s_modelFlow=s_benchState.actualFlowX10;
    const float maximum=bench_max_flow();
    if(maximum>0) {s_opening=s_modelFlow/maximum;if(s_opening>1)s_opening=1;}
    s_command=0;s_nextPulse=millis()+s_regulator.pauseMs;
}

// The UI owns the editable widths and loads them from NVS at startup.
// Use its configured target everywhere so rounding and limits also agree.
int activeWidthDm() {
    return (int)(ui_get_active_boom_width_m() * 10.0f + 0.5f);
}

int targetFlowX10() {
    return ui_get_target_flow_x10();
}

int flowErrorPercent() {
    int target = targetFlowX10();
    if (target <= 0) return 0;
    return (int)(((long)sprayer_display_state().actualFlowX10 - target) * 100L / target);
}

const char *regulatorText() {
    if(bench_test_active()) {
        if(!s_benchState.autoMode) return "MAN";
        if(s_benchState.speedX10<=0 || activeWidthDm()<=0) return "POSTOJ";
        if(s_command>0)return "OTWIERAJ";
        if(s_command<0)return "ZAMYKAJ";
        if((s_opening>=1 && flowErrorPercent() < -(int)s_regulator.tolerancePercent) || (s_opening<=0 && flowErrorPercent()>(int)s_regulator.tolerancePercent)) return "LIMIT";
        return (flowErrorPercent()>=-(int)s_regulator.tolerancePercent && flowErrorPercent()<=(int)s_regulator.tolerancePercent) ? "W CELU" : "PRZERWA";
    }
    int target = targetFlowX10();
    if (target < REGULATOR_MIN_TARGET_X10 || sprayer_display_state().speedX10 <= 0 || activeWidthDm() <= 0) return "STOP";
    int e = flowErrorPercent();
    if (e < -(int)s_regulator.tolerancePercent) return "OTWIERAJ";
    if (e > (int)s_regulator.tolerancePercent) return "ZAMYKAJ";
    return "STOP";
}
