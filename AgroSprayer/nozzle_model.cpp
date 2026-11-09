#include "nozzle_model.h"
#include "aog.h"
#include "nozzle_calibration_core.h"
#include "section_volume_core.h"
#include "sprayer_state.h"
#include "pressure_sensor.h"
#include "ui.h"
#include <Preferences.h>
#include <cmath>
static NozzleSettings ns;
static bool reading=false;
static bool profileFault=false;
static agrovolume::SectionVolumeIntegrator volumeIntegrator;
static agrovolume::SectionEventLatch sectionEventLatch;
void nozzle_section_change_at(uint32_t eventMs) {
    if (!bench_test_active()) sectionEventLatch.note(eventMs);
}
static uint8_t liveSectionMask() {
    uint8_t mask=0;
    for(int i=0;i<SECTION_COUNT;i++) if(gState.sections[i]) mask|=static_cast<uint8_t>(1u<<i);
    return mask;
}
static const char *status="BRAK CZUJNIKA";
const NozzleSettings &nozzle_settings(){return ns;}
const char *nozzle_name(){return ns.nozzle?"04RSM":"11003";}
const char *nozzle_liquid_name(){static const char *names[]={"WODA","RSM 28","RSM 30","RSM 32","WLASNA"};return names[ns.liquid];}
static bool validSettings(const NozzleSettings &s){
    if(s.magic!=0x4e5a4c34||s.nozzle>1||s.liquid>4||
       !agronozzle::supportedNozzleLiquid(s.nozzle,s.liquid)||
       !std::isfinite(s.density)||s.density<0.8f||s.density>1.6f)return false;
    for(auto n:s.counts)if(n<1||n>40)return false;
    for(int k=0;k<2;k++){const float q=s.waterQ3[k];if(!std::isfinite(q)||q<(k==1?0.08f:0.1f)||q>5)return false;}
    return true;
}
void nozzle_begin(){
    ns=NozzleSettings{};
    profileFault=false;
    Preferences p;
    if(p.begin("agronozzle",true)){
        NozzleSettings s;
        const bool hasProfile=p.isKey("settings");
        const size_t storedBytes=p.getBytesLength("settings");
        const size_t n=storedBytes==sizeof(s)
            ? p.getBytes("settings",&s,sizeof(s)) : 0;
        p.end();
        bool valid=false;
        if(n==sizeof(s)){
            // V1/V2/V3 used the old water-reference model for 04RSM.
            // Preserve custom calibrations, but keep the table default at 1.54
            // l/min and migrate it to the direct RSM reference used by V4.
            if(s.magic==0x4e5a4c31){
                s.waterQ3[1]=agronozzle::migrateLegacyRsmQ3(s.waterQ3[1]);
            } else if(s.magic==0x4e5a4c32){
                s.waterQ3[1]=agronozzle::migrateV2RsmQ3(s.waterQ3[1]);
            }
            if(s.magic==0x4e5a4c31||s.magic==0x4e5a4c32||s.magic==0x4e5a4c33){
                if(s.nozzle==1&&s.liquid==0){s.liquid=1;s.density=1.28f;}
                s.waterQ3[1]=agronozzle::migrateRsmQ3ToDirect(s.waterQ3[1],s.density);
                s.magic=0x4e5a4c34;
            }
            valid=validSettings(s);
            if(valid)ns=s;
        }
        profileFault=agronozzle::storedProfileMustBlock(
            hasProfile,storedBytes,n,sizeof(s),valid);
    }
    volumeIntegrator.reset();
    uint32_t unusedEventMs = 0;
    sectionEventLatch.take(unusedEventMs);
    reading=false;
    status=profileFault ? "PROFIL DYSZY - WYMAGA ZAPISU" : "BRAK CZUJNIKA";
}
bool nozzle_save(const NozzleSettings &s){
    if(bench_test_active()||!validSettings(s))return false;
    // Settle old-profile consumption before changing calibration.
    nozzle_update();Preferences p;if(!p.begin("agronozzle",false))return false;
    auto n=p.putBytes("settings",&s,sizeof(s));p.end();if(n!=sizeof(s))return false;
    ns=s;
    profileFault=false;
    status="PROFIL ZAPISANY";
    // The old profile was settled above; do not blend its flow with the new one.
    volumeIntegrator.sample(millis(),liveSectionMask(),0.0f,false);
    reading=false;return true;
}
// Separate 11003 water and 04RSM RSM tables; no second RSM density correction.
float nozzle_flow(float bar,const NozzleSettings &s){
    if(!validSettings(s)||!std::isfinite(bar)||bar<0)return NAN;
    const int k=s.nozzle;
    const float tableQ=agronozzle::nozzleTableFlow(bar,k);
    if(!std::isfinite(tableQ))return NAN;
    return agronozzle::calibratedNozzleFlow(tableQ,s.waterQ3[k],k,s.density);
}
float nozzle_target_pressure(float total,int count,const NozzleSettings &s){
    if(!validSettings(s)||!std::isfinite(total)||total<=0||count<=0)return NAN;
    // The 04RSM flow table extends to 6 bar, but AUTO target pressure is
    // limited to 4.8 bar by the regulator safety interlock.
    const float lowBar=2.0f,highBar=agronozzle::maxTargetPressureBar(s.nozzle);
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
float nozzle_target_bar(){if(profileFault)return NAN;int n=0;const auto &s=sprayer_display_state();for(int i=0;i<6;i++)if(s.sections[i])n+=ns.counts[i];return nozzle_target_pressure(targetFlowX10()/10.0f,n,ns);}
void nozzle_update(){
    const uint32_t now=millis();
    const uint8_t mask=liveSectionMask();
    int count=0;for(int i=0;i<SECTION_COUNT;i++)if(gState.sections[i])count+=ns.counts[i];
    const bool fresh=gState.aogOk&&gState.rxPackets>0&&(uint32_t)(now-gState.lastAogMs)<=AOG_TIMEOUT_MS;
    float q=0;reading=false;
    if(profileFault)status="PROFIL DYSZY - WYMAGA ZAPISU";
    else if(bench_test_active())status="SYMULACJA";
    else if(!count){reading=true;status="SEKCJE WYLACZONE";}
    else if(!fresh)status="BRAK DANYCH SEKCJI";
    else if(!pressure_sensor_valid())status="BRAK CZUJNIKA";
    else {float one=nozzle_flow(pressure_sensor_bar_x10()/10.0f,ns);
        if(std::isfinite(one)){q=one*count;reading=true;status="OBLICZONE Z CISNIENIA";}
        else status="CISNIENIE POZA TABELA";
    }
    // Settle the last known flow only up to the first timestamped transition.
    // Never extrapolate through stale AOG or invalid pressure readings.
    uint32_t eventMs = 0;
    const bool hadSectionEvent = sectionEventLatch.take(eventMs);
    // An observed AOG dropout invalidates the complete unsampled interval,
    // even when a fresh packet arrived before this model tick. Do not settle
    // any queued section transition across an uncertain telemetry gap.
    if (aog_take_telemetry_gap()) {
        volumeIntegrator.sample(now, mask, 0.0f, false);
    } else if (hadSectionEvent) {
        volumeIntegrator.settleBeforeSectionChange(
            eventMs, !bench_test_active() && fresh && pressure_sensor_valid());
    }
    volumeIntegrator.sample(now,mask,q,reading,gState.speedX10,ui_get_active_boom_width_m());
    gState.actualFlowX10=reading?(int)lroundf(q*10):0;
}
bool nozzle_reading_valid(){return reading&&!profileFault;}
bool nozzle_profile_fault(){return profileFault;}
double nozzle_total_liters(){return volumeIntegrator.liters();}
double nozzle_total_hectares(){return volumeIntegrator.hectares();}
double nozzle_area_liters(){return volumeIntegrator.areaLiters();}
uint64_t nozzle_total_spray_ms(){return volumeIntegrator.sprayMs();}
const char *nozzle_status(){return status;}
