#include "nozzle_model.h"
#include "nozzle_calibration_core.h"
#include "section_volume_core.h"
#include "sprayer_state.h"
#include "pressure_sensor.h"
#include "ui.h"
#include <Preferences.h>
#include <cmath>
static NozzleSettings ns;
static bool reading=false;
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
bool nozzle_profile_valid(const NozzleSettings &s){return agronozzle::validNozzleLiquidProfile(s.nozzle,s.liquid);}
const char *nozzle_name(){return ns.nozzle?"04RSM":"11003";}
const char *nozzle_liquid_name(){static const char *names[]={"WODA","RSM 28","RSM 30","RSM 32","WLASNA"};return names[ns.liquid];}
static bool validSettings(const NozzleSettings &s){
    if(s.magic!=0x4e5a4c33||s.nozzle>1||s.liquid>4||!agronozzle::validNozzleLiquidProfile(s.nozzle,s.liquid)||!std::isfinite(s.density)||s.density<0.8f||s.density>1.6f)return false;
    for(auto n:s.counts)if(n<1||n>40)return false;
    for(int k=0;k<2;k++){const float q=s.waterQ3[k];if(!std::isfinite(q)||q<(k==1?0.08f:0.1f)||q>5)return false;}
    return true;
}
// Preset liquids are authoritative: a stale/corrupt saved density must not
// override the selected profile. The 04RSM table is water flow; RSM correction
// is applied from the selected liquid preset when calculating live flow.
static void normalizeLiquidDensity(NozzleSettings &s){
    if(s.liquid<4) s.density=agronozzle::presetLiquidDensity(s.liquid);
}
void nozzle_begin(){
    ns=NozzleSettings{};Preferences p;
    if(p.begin("agronozzle",true)){
        NozzleSettings s;auto n=p.getBytes("settings",&s,sizeof(s));p.end();
        if(n==sizeof(s)){
            // The 04RSM water-flow table is 1.54 l/min at 3 bar (1.78 at 4 bar).
            // V1 already used the 1.54 reference. V2 used 1.78; rescale
            // its saved water Q3 to V3 without changing the calibrated curve.
            if(s.magic==0x4e5a4c31){
                s.waterQ3[1]=agronozzle::migrateLegacyRsmQ3(s.waterQ3[1]);
                s.magic=0x4e5a4c33;
            } else if(s.magic==0x4e5a4c32){
                s.waterQ3[1]=agronozzle::migrateV2RsmQ3(s.waterQ3[1]);
                s.magic=0x4e5a4c33;
            }
            normalizeLiquidDensity(s);
            if(validSettings(s))ns=s;
        }
    }
    volumeIntegrator.reset();
    uint32_t unusedEventMs = 0;
    sectionEventLatch.take(unusedEventMs);
    reading=false;
}
bool nozzle_save(const NozzleSettings &s){
    if(bench_test_active())return false;
    NozzleSettings normalized=s;
    normalizeLiquidDensity(normalized);
    if(!validSettings(normalized))return false;
    // Settle old-profile consumption before changing calibration.
    nozzle_update();Preferences p;if(!p.begin("agronozzle",false))return false;
    auto n=p.putBytes("settings",&normalized,sizeof(normalized));p.end();if(n!=sizeof(normalized))return false;
    ns=normalized;
    // The old profile was settled above; do not blend its flow with the new one.
    volumeIntegrator.sample(millis(),liveSectionMask(),0.0f,false);
    reading=false;return true;
}
// Manufacturer tables are centralized in nozzle_calibration_core.h so native
// tests and the ESP32 firmware exercise exactly the same pressure/flow mapping.
// Q3 calibration is a measured water flow at 3 bar. The core then applies
// the liquid correction for the selected profile, including RSM 28/30/32.
float nozzle_flow(float bar,const NozzleSettings &s){
    if(!validSettings(s)||!std::isfinite(bar)||bar<0)return NAN;
    if(bar<=0.1f)return 0;
    const float tableFlow=agronozzle::tableWaterFlow(bar,s.nozzle);
    if(!std::isfinite(tableFlow))return NAN;
    return agronozzle::calibratedWaterFlow(tableFlow,s.waterQ3[s.nozzle],
                                           s.nozzle,agronozzle::effectiveLiquidDensity(s.liquid,s.density));
}
float nozzle_target_pressure(float total,int count,const NozzleSettings &s){
    if(!validSettings(s)||!std::isfinite(total)||total<=0||count<=0)return NAN;
    return agronozzle::targetPressureForFlow(
        total, count, s.nozzle, s.waterQ3[s.nozzle], agronozzle::effectiveLiquidDensity(s.liquid,s.density));
}
float nozzle_target_bar(){int n=0;const auto &s=sprayer_display_state();for(int i=0;i<6;i++)if(s.sections[i])n+=ns.counts[i];return nozzle_target_pressure(targetFlowX10()/10.0f,n,ns);}
void nozzle_update(){
    const uint32_t now=millis();
    const uint8_t mask=liveSectionMask();
    int count=0;for(int i=0;i<SECTION_COUNT;i++)if(gState.sections[i])count+=ns.counts[i];
    const bool fresh=gState.aogOk&&gState.rxPackets>0&&(uint32_t)(now-gState.lastAogMs)<=AOG_TIMEOUT_MS;
    float q=0;reading=false;
    if(bench_test_active())status="SYMULACJA";
    else if(!count){reading=true;status="SEKCJE WYLACZONE";}
    else if(!fresh)status="BRAK DANYCH SEKCJI";
    else if(!agronozzle::validNozzleLiquidProfile(ns.nozzle,ns.liquid))status="NIEZGODNY PROFIL DYSZY/CIECZY";
    else if(!pressure_sensor_valid())status="BRAK CZUJNIKA";
    else {float one=nozzle_flow(pressure_sensor_bar_x10()/10.0f,ns);
        if(std::isfinite(one)){q=one*count;reading=true;status="OBLICZONE Z CISNIENIA";}
        else status="CISNIENIE POZA TABELA";
    }
    // Settle the last known flow only up to the first timestamped transition.
    // Never extrapolate through stale AOG or invalid pressure readings.
    uint32_t eventMs = 0;
    if (sectionEventLatch.take(eventMs)) {
        volumeIntegrator.settleBeforeSectionChange(
            eventMs, !bench_test_active() && fresh && pressure_sensor_valid());
    }
    volumeIntegrator.sample(now,mask,q,reading);
    gState.actualFlowX10=reading?(int)lroundf(q*10):0;
}
bool nozzle_reading_valid(){return reading;}
double nozzle_total_liters(){return volumeIntegrator.liters();}
const char *nozzle_status(){return status;}
