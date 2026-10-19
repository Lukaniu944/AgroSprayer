#include "job_counter.h"
#include "nozzle_model.h"
#include "sprayer_state.h"
#include "ui.h"
#include <Preferences.h>
#include <cmath>
struct SavedJob {uint32_t magic;float liters;float hectares;float areaLiters;uint64_t sprayMs;};
static constexpr uint32_t JOB_MAGIC=0x4a4f4232;
static SavedJob job{};
static bool running=false,dirtyJob=false,jobSaveOk=true,wasBench=false;
static double lastJobVolume=0;
static uint32_t lastJobTick=0,lastJobSave=0;
static bool saveJob(const SavedJob &value){
    Preferences p;if(!p.begin("agrojob",false))return false;
    const auto n=p.putBytes("state",&value,sizeof(value));p.end();return n==sizeof(value);
}
static void baselineJob(){lastJobVolume=nozzle_total_liters();lastJobTick=millis();wasBench=bench_test_active();}
void job_counter_begin(){
    job={};running=false;dirtyJob=false;jobSaveOk=true;Preferences p;
    if(p.begin("agrojob",true)){
        SavedJob saved{};const auto n=p.getBytes("state",&saved,sizeof(saved));p.end();
        if(n==sizeof(saved)&&saved.magic==JOB_MAGIC&&std::isfinite(saved.liters)&&saved.liters>=0&&std::isfinite(saved.hectares)&&saved.hectares>=0&&std::isfinite(saved.areaLiters)&&saved.areaLiters>=0)job=saved;
    }
    baselineJob();lastJobSave=millis();
}
void job_counter_update(){
    const uint32_t now=millis(),elapsed=now-lastJobTick;
    const double total=nozzle_total_liters(),volume=total-lastJobVolume;
    lastJobTick=now;lastJobVolume=total;
    const bool bench=bench_test_active(),skip=bench||wasBench;wasBench=bench;
    if(!running||skip||job.magic!=JOB_MAGIC)return;
    if(volume>0){job.liters+=volume;dirtyJob=true;}
    // Area/time are estimates from commands, not physical section feedback.
    bool any=false;for(int i=0;i<SECTION_COUNT;i++)any|=gState.sections[i];
    if(any&&nozzle_reading_valid()&&gState.actualFlowX10>0&&volume>0&&elapsed<=2000){
        job.sprayMs+=elapsed;dirtyJob|=elapsed>0;
        if(gState.aogOk&&gState.speedX10>0){
            float width=0;for(int i=0;i<SECTION_COUNT;i++)if(gState.sections[i])width+=ui_get_section_width_m(i);
            job.areaLiters+=volume;
            job.hectares+=(double)gState.speedX10*width*elapsed/360000000.0;
        }
    }
    if(dirtyJob&&(uint32_t)(now-lastJobSave)>=60000){jobSaveOk=saveJob(job);lastJobSave=now;if(jobSaveOk)dirtyJob=false;}
}
JobReading job_counter_read(){return {job.liters,job.hectares,job.areaLiters,job.sprayMs,job.magic==JOB_MAGIC,running,jobSaveOk};}
bool job_counter_new(){
    if(bench_test_active())return false;
    SavedJob next{};next.magic=JOB_MAGIC;jobSaveOk=saveJob(next);if(!jobSaveOk)return false;
    job=next;running=true;dirtyJob=false;baselineJob();lastJobSave=millis();return true;
}
bool job_counter_pause(bool pause){
    if(bench_test_active()||job.magic!=JOB_MAGIC)return false;
    job_counter_update();
    if(pause)running=false;
    jobSaveOk=saveJob(job);if(!jobSaveOk)return false;
    running=!pause;dirtyJob=false;baselineJob();lastJobSave=millis();return true;
}
