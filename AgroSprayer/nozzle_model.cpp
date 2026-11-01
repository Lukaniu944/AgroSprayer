#include "nozzle_model.h"
#include "nozzle_profile_compatibility.h"
#include "nozzle_flow_core.h"
#include "sprayer_state.h"
#include "pressure_sensor.h"
#include "ui.h"
#include <Preferences.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
static NozzleSettings ns;
static bool reading=false,previousValid=false;
static double liters=0;
static float previousFlow=0;
static uint32_t tick=0;
static uint8_t previousMask=0;
static const char *status="BRAK CZUJNIKA";
static SemaphoreHandle_t nozzleMutex=nullptr;

// The estimator is called by the main loop, while nozzle_save() and its
// getters can also be reached from LVGL callbacks on the display task.
class NozzleLock {
public:
    NozzleLock() : held_(nozzleMutex &&
        xSemaphoreTake(nozzleMutex, portMAX_DELAY) == pdTRUE) {}
    ~NozzleLock() { if (held_) xSemaphoreGive(nozzleMutex); }
    explicit operator bool() const { return held_; }
private:
    bool held_;
};

static int estimated_liters_x10(double totalLiters)
{
    // The home screen uses tenths of a litre and treats values above 50,000 L
    // as invalid. Clamp here so a long-running estimate cannot overflow int
    // or turn the display back to zero.
    if (!std::isfinite(totalLiters) || totalLiters <= 0.0) return 0;
    const double scaled = totalLiters * 10.0;
    if (scaled >= 500000.0) return 500000;
    return static_cast<int>(std::lround(scaled));
}

const NozzleSettings &nozzle_settings(){return ns;}
const char *nozzle_name(){return ns.nozzle?"04RSM":"11003";}
const char *nozzle_liquid_name(){static const char *names[]={"WODA","RSM 28","RSM 30","RSM 32","WLASNA"};return names[ns.liquid];}
static bool validSettings(const NozzleSettings &s){
    if(s.magic!=0x4e5a4c31||s.nozzle>1||s.liquid>4||!nozzle_profile_compatible(s.nozzle,s.liquid)||!std::isfinite(s.density)||s.density<0.8f||s.density>1.6f)return false;
    for(auto n:s.counts)if(n<1||n>40)return false;
    for(float q:s.waterQ3)if(!std::isfinite(q)||q<0.1f||q>5)return false;
    return true;
}
void nozzle_begin(){
    if(!nozzleMutex) nozzleMutex=xSemaphoreCreateMutex();
    if(!nozzleMutex) return;
    ns=NozzleSettings{};Preferences p;
    if(p.begin("agronozzle",true)){NozzleSettings s;auto n=p.getBytes("settings",&s,sizeof(s));p.end();if(n==sizeof(s)&&validSettings(s))ns=s;}
    tick=millis();liters=0;reading=previousValid=false;previousFlow=0;
    gState.totalLitersX10=0;
}
bool nozzle_save(const NozzleSettings &s){
    if(bench_test_active()||!validSettings(s)||!nozzleMutex)return false;
    // Settle the old profile through the same serialized estimator path.
    // Keep the mutex released during NVS I/O so the 10 ms main-loop update
    // is not stalled by flash writes; the next interval is discarded at swap.
    nozzle_update();
    Preferences p;if(!p.begin("agronozzle",false))return false;
    auto n=p.putBytes("settings",&s,sizeof(s));p.end();if(n!=sizeof(s))return false;
    NozzleLock lock;
    if(!lock)return false;
    ns=s;previousValid=false;reading=false;
    return true;
}
// Both configured tables are treated as water-reference values.
// nozzle_flow_core.h applies the 04RSM RSM 28/30/32 correction exactly once
// (division by the inverse factors 1.13/1.14/1.15). Do not correct 04RSM again.
float nozzle_flow(float bar,const NozzleSettings &s){
    if(!validSettings(s)||!std::isfinite(bar)||bar<0)return NAN;
    return agronozzle::nozzle_flow_from_table(
        bar, s.nozzle, s.liquid, s.waterQ3[s.nozzle], s.density);
}
float nozzle_target_pressure(float total,int count,const NozzleSettings &s){
    if(!validSettings(s))return NAN;
    return agronozzle::nozzle_target_pressure_from_table(
        total,count,s.nozzle,s.liquid,s.waterQ3[s.nozzle],s.density);
}
float nozzle_target_bar(){
    NozzleLock lock;
    if(!lock)return NAN;
    int n=0;const auto &s=sprayer_display_state();
    for(int i=0;i<6;i++)if(s.sections[i])n+=ns.counts[i];
    return nozzle_target_pressure(targetFlowX10()/10.0f,n,ns);
}
void nozzle_update(){
    NozzleLock lock;
    if(!lock)return;
    const uint32_t now=millis(),dt=now-tick;tick=now;
    uint8_t mask=0;int count=0;for(int i=0;i<6;i++)if(gState.sections[i]){mask|=1<<i;count+=ns.counts[i];}
    const bool fresh=gState.aogOk&&gState.rxPackets>0&&(uint32_t)(now-gState.lastAogMs)<=AOG_TIMEOUT_MS;
    float q=0;reading=false;
    switch(agronozzle::section_flow_state(bench_test_active(),fresh,count)) {
        case agronozzle::SectionFlowState::Simulation:
            status="SYMULACJA";
            break;
        case agronozzle::SectionFlowState::NoAogData:
            status="BRAK DANYCH SEKCJI";
            break;
        case agronozzle::SectionFlowState::AllSectionsOff:
            reading=true;
            status="SEKCJE WYLACZONE";
            break;
        case agronozzle::SectionFlowState::ActiveSections:
            if(!pressure_sensor_valid())status="BRAK CZUJNIKA";
            else {
                float one=nozzle_flow(pressure_sensor_bar_x10()/10.0f,ns);
                if(std::isfinite(one)){q=one*count;reading=true;status="OBLICZONE Z CISNIENIA";}
                else status="CISNIENIE POZA TABELA";
            }
            break;
    }
    if(reading&&previousValid&&mask==previousMask&&dt<=2000)liters+=(previousFlow+q)*0.5*dt/60000.0;
    previousFlow=q;previousValid=reading;previousMask=mask;
    gState.actualFlowX10=reading?(int)lroundf(q*10):0;
    // Publish the estimated total to the field consumed by the main-screen UI.
    // Job accounting continues to use nozzle_total_liters() directly, so this
    // is display synchronization only and does not add litres a second time.
    gState.totalLitersX10=estimated_liters_x10(liters);
}
bool nozzle_reading_valid(){NozzleLock lock;return lock && reading;}
double nozzle_total_liters(){NozzleLock lock;return lock ? liters : 0.0;}
const char *nozzle_status(){NozzleLock lock;return lock ? status : "BRAK CZUJNIKA";}
