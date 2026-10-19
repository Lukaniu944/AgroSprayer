#include "tank_counter.h"
#include "nozzle_model.h"
#include "sprayer_state.h"
#include <Preferences.h>
#include <cmath>
struct StoredTank { uint32_t magic; float liters; float used; };
static StoredTank state{0,0,0};
static double lastVolume=0;
static uint32_t lastSave=0;
static bool dirty=false,saveOk=true,wasSimulation=false;
static constexpr uint32_t MAGIC=0x54414e32;
static bool persist(const StoredTank &value) {
    Preferences p;
    if(!p.begin("agrotank",false))return false;
    const size_t n=p.putBytes("state",&value,sizeof(value));p.end();
    return n==sizeof(value);
}
void tank_counter_begin() {
    state={0,0,0};Preferences p;
    if(p.begin("agrotank",true)) {
        StoredTank loaded{};const size_t n=p.getBytes("state",&loaded,sizeof(loaded));p.end();
        if(n==sizeof(loaded) && loaded.magic==MAGIC && std::isfinite(loaded.liters) &&
           loaded.liters>=0 && loaded.liters<=20000 && std::isfinite(loaded.used) && loaded.used>=0)state=loaded;
    }
    lastVolume=nozzle_total_liters();lastSave=millis();dirty=false;saveOk=true;wasSimulation=bench_test_active();
}
void tank_counter_update() {
    const double total=nozzle_total_liters(),volume=total-lastVolume;lastVolume=total;
    const bool simulation=bench_test_active();
    const bool skip=simulation || wasSimulation;wasSimulation=simulation;
    if(skip || state.magic!=MAGIC)return;
    if(volume>0) {
        state.liters=state.liters>volume?state.liters-volume:0;
        state.used+=volume;dirty=true;
    }
    if(dirty && (uint32_t)(millis()-lastSave)>=60000) {
        saveOk=persist(state);lastSave=millis();if(saveOk)dirty=false;
    }
}
TankReading tank_counter_read(){return {state.liters,state.used,state.magic==MAGIC,saveOk};}
bool tank_counter_set(float liters,bool add) {
    if(bench_test_active() || !std::isfinite(liters) || liters<0 || liters>20000)return false;
    tank_counter_update();
    if(add && state.magic!=MAGIC)return false;
    StoredTank next{MAGIC,add?state.liters+liters:liters,add?state.used:0};
    if(next.liters>20000)return false;
    saveOk=persist(next);if(!saveOk)return false;
    state=next;dirty=false;lastSave=millis();return true;
}
