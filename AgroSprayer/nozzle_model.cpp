#include "nozzle_model.h"
#include "nozzle_profile_compatibility.h"
#include "nozzle_flow_core.h"
#include "sprayer_state.h"
#include "pressure_sensor.h"
#include "ui.h"
#include <Preferences.h>
#include <cmath>
static NozzleSettings ns;
static bool reading=false,previousValid=false;
static double liters=0;
static float previousFlow=0;
static uint32_t tick=0;
static uint8_t previousMask=0;
static const char *status="BRAK CZUJNIKA";
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
    ns=NozzleSettings{};Preferences p;
    if(p.begin("agronozzle",true)){NozzleSettings s;auto n=p.getBytes("settings",&s,sizeof(s));p.end();if(n==sizeof(s)&&validSettings(s))ns=s;}
    tick=millis();liters=0;reading=previousValid=false;previousFlow=0;
}
bool nozzle_save(const NozzleSettings &s){
    if(bench_test_active()||!validSettings(s))return false;
    // Settle old-profile consumption before changing calibration.
    nozzle_update();Preferences p;if(!p.begin("agronozzle",false))return false;
    auto n=p.putBytes("settings",&s,sizeof(s));p.end();if(n!=sizeof(s))return false;
    ns=s;previousValid=false;reading=false;return true;
}
// 11003 is water-reference; the supplied 04RSM table is already RSM-specific.
// Keep the two table meanings separate so RSM flow is not corrected twice.
float nozzle_flow(float bar,const NozzleSettings &s){
    if(!validSettings(s)||!std::isfinite(bar)||bar<0)return NAN;
    return agronozzle::nozzle_flow_from_table(
        bar, s.nozzle, s.liquid, s.waterQ3[s.nozzle], s.density);
}
float nozzle_target_pressure(float total,int count,const NozzleSettings &s){
    if(!std::isfinite(total)||total<=0||count<=0)return NAN;
    float low=2,high=s.nozzle?6:4,q=total/count;
    if(q<nozzle_flow(low,s)||q>nozzle_flow(high,s))return NAN;
    for(int i=0;i<24;i++){float mid=(low+high)/2;if(nozzle_flow(mid,s)<q)low=mid;else high=mid;}
    return (low+high)/2;
}
float nozzle_target_bar(){int n=0;const auto &s=sprayer_display_state();for(int i=0;i<6;i++)if(s.sections[i])n+=ns.counts[i];return nozzle_target_pressure(targetFlowX10()/10.0f,n,ns);}
void nozzle_update(){
    const uint32_t now=millis(),dt=now-tick;tick=now;
    uint8_t mask=0;int count=0;for(int i=0;i<6;i++)if(gState.sections[i]){mask|=1<<i;count+=ns.counts[i];}
    const bool fresh=gState.aogOk&&gState.rxPackets>0&&(uint32_t)(now-gState.lastAogMs)<=AOG_TIMEOUT_MS;
    float q=0;reading=false;
    if(bench_test_active())status="SYMULACJA";
    else if(!count){reading=true;status="SEKCJE WYLACZONE";}
    else if(!fresh)status="BRAK DANYCH SEKCJI";
    else if(!pressure_sensor_valid())status="BRAK CZUJNIKA";
    else {float one=nozzle_flow(pressure_sensor_bar_x10()/10.0f,ns);
        if(std::isfinite(one)){q=one*count;reading=true;status="OBLICZONE Z CISNIENIA";}
        else status="CISNIENIE POZA TABELA";
    }
    if(reading&&previousValid&&mask==previousMask&&dt<=2000)liters+=(previousFlow+q)*0.5*dt/60000.0;
    previousFlow=q;previousValid=reading;previousMask=mask;
    gState.actualFlowX10=reading?(int)lroundf(q*10):0;
}
bool nozzle_reading_valid(){return reading;}
double nozzle_total_liters(){return liters;}
const char *nozzle_status(){return status;}
