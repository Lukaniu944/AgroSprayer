#include "nozzle_model.h"
#include "nozzle_calibration_core.h"
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
    if(s.magic!=0x4e5a4c32||s.nozzle>1||s.liquid>4||!std::isfinite(s.density)||s.density<0.8f||s.density>1.6f)return false;
    for(auto n:s.counts)if(n<1||n>40)return false;
    for(float q:s.waterQ3)if(!std::isfinite(q)||q<0.1f||q>5)return false;
    return true;
}
void nozzle_begin(){
    ns=NozzleSettings{};Preferences p;
    if(p.begin("agronozzle",true)){
        NozzleSettings s;auto n=p.getBytes("settings",&s,sizeof(s));p.end();
        if(n==sizeof(s)){
            // V1 stored 04RSM calibration relative to the 2.5-bar table
            // entry (1.54), although the editor specified 3 bar. Convert
            // its value to the true 3-bar reference without changing output.
            if(s.magic==0x4e5a4c31){
                s.waterQ3[1]=agronozzle::migrateLegacyRsmQ3(s.waterQ3[1]);
                s.magic=0x4e5a4c32;
            }
            if(validSettings(s))ns=s;
        }
    }
    tick=millis();liters=0;reading=previousValid=false;previousFlow=0;
}
bool nozzle_save(const NozzleSettings &s){
    if(bench_test_active()||!validSettings(s))return false;
    // Settle old-profile consumption before changing calibration.
    nozzle_update();Preferences p;if(!p.begin("agronozzle",false))return false;
    auto n=p.putBytes("settings",&s,sizeof(s));p.end();if(n!=sizeof(s))return false;
    ns=s;previousValid=false;reading=false;return true;
}
// User-supplied water tables. Interpolate squared flow vs pressure, then
// apply per-profile water calibration and density exactly once.
float nozzle_flow(float bar,const NozzleSettings &s){
    if(!validSettings(s)||!std::isfinite(bar)||bar<0)return NAN;
    if(bar<=0.1f)return 0;
    const float ps[2][5]={{2,2.5f,3,3.5f,4},{2,3,4,5,6}};
    const float qs[2][5]={{.96f,1.08f,1.18f,1.27f,1.39f},{1.26f,1.54f,1.78f,1.99f,2.18f}};
    const int k=s.nozzle;
    if(bar<ps[k][0]||bar>ps[k][4])return NAN;
    float q=qs[k][4];
    for(int i=1;i<5;i++)if(bar<=ps[k][i]){float t=(bar-ps[k][i-1])/(ps[k][i]-ps[k][i-1]);q=sqrtf(qs[k][i-1]*qs[k][i-1]+t*(qs[k][i]*qs[k][i]-qs[k][i-1]*qs[k][i-1]));break;}
    return agronozzle::calibratedWaterFlow(q,s.waterQ3[k],k,s.density);
}
float nozzle_target_pressure(float total,int count,const NozzleSettings &s){
    if(!validSettings(s)||!std::isfinite(total)||total<=0||count<=0)return NAN;
    const float lowBar=2.0f,highBar=s.nozzle?6.0f:4.0f;
    float low=lowBar,high=highBar,q=total/count;
    const float minFlow=nozzle_flow(low,s),maxFlow=nozzle_flow(high,s);
    if(!std::isfinite(q)||!std::isfinite(minFlow)||!std::isfinite(maxFlow))return NAN;
    const float epsilon=maxFlow*1.0e-6f;
    if(q<minFlow)return minFlow-q<=epsilon?lowBar:NAN;
    if(q>maxFlow)return q-maxFlow<=epsilon?highBar:NAN;
    for(int i=0;i<24;i++){
        const float mid=(low+high)/2,flow=nozzle_flow(mid,s);
        if(!std::isfinite(flow))return NAN;
        if(flow<q)low=mid;else high=mid;
    }
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
