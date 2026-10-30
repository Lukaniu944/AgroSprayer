#include "job_counter.h"
#include "nozzle_model.h"
#include "sprayer_state.h"
#include <Preferences.h>
#include <cmath>
struct SavedJob {uint32_t magic;float liters;float hectares;float areaLiters;uint64_t sprayMs;};
static constexpr uint32_t JOB_MAGIC=0x4a4f4232;
static SavedJob job{};
static bool running=false,dirtyJob=false,jobSaveOk=true,wasBench=false;
static double lastJobVolume=0,lastJobHectares=0,lastJobAreaLiters=0;
static uint64_t lastJobSprayMs=0;
static uint32_t lastJobSave=0;
static bool saveJob(const SavedJob &value){
    Preferences p;if(!p.begin("agrojob",false))return false;
    const auto n=p.putBytes("state",&value,sizeof(value));p.end();return n==sizeof(value);
}
static void baselineJob(){
    lastJobVolume=nozzle_total_liters();
    lastJobHectares=nozzle_total_hectares();
    lastJobAreaLiters=nozzle_area_liters();
    lastJobSprayMs=nozzle_total_spray_ms();
    wasBench=bench_test_active();
}
void job_counter_begin(){
    job={};running=false;dirtyJob=false;jobSaveOk=true;Preferences p;
    if(p.begin("agrojob",true)){
        SavedJob saved{};const auto n=p.getBytes("state",&saved,sizeof(saved));p.end();
        if(n==sizeof(saved)&&saved.magic==JOB_MAGIC&&std::isfinite(saved.liters)&&saved.liters>=0&&std::isfinite(saved.hectares)&&saved.hectares>=0&&std::isfinite(saved.areaLiters)&&saved.areaLiters>=0)job=saved;
    }
    baselineJob();lastJobSave=millis();
}
void job_counter_update(){
    // Consume all counters from the same event-segmented model clock.
    // The UI tick must not assign an entire interval to the final section mask.
    const double total=nozzle_total_liters(),volume=total-lastJobVolume;
    const double hectares=nozzle_total_hectares(),area=hectares-lastJobHectares;
    const double areaLiters=nozzle_area_liters(),areaVolume=areaLiters-lastJobAreaLiters;
    const uint64_t sprayMs=nozzle_total_spray_ms();
    const uint64_t sprayDelta=sprayMs>=lastJobSprayMs?sprayMs-lastJobSprayMs:0;
    lastJobVolume=total;lastJobHectares=hectares;
    lastJobAreaLiters=areaLiters;lastJobSprayMs=sprayMs;
    const bool bench=bench_test_active(),skip=bench||wasBench;wasBench=bench;
    if(running&&!skip&&job.magic==JOB_MAGIC){
        if(volume>0){job.liters+=volume;dirtyJob=true;}
        if(area>0){job.hectares+=area;dirtyJob=true;}
        if(areaVolume>0){job.areaLiters+=areaVolume;dirtyJob=true;}
        if(sprayDelta>0){job.sprayMs+=sprayDelta;dirtyJob=true;}
    }
    // A failed pause/save must still be retried while the job is stopped.
    // Back off to 5 s after a failed write instead of waiting for the normal
    // 60 s periodic checkpoint or retrying on every UI tick.
    const uint32_t now=millis();
    const uint32_t saveInterval=jobSaveOk?60000u:5000u;
    if(dirtyJob&&(uint32_t)(now-lastJobSave)>=saveInterval){
        jobSaveOk=saveJob(job);lastJobSave=now;if(jobSaveOk)dirtyJob=false;
    }
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
    jobSaveOk=saveJob(job);
    lastJobSave=millis();
    if(!jobSaveOk)return false;
    running=!pause;dirtyJob=false;baselineJob();return true;
}
