#include "../AgroSprayer/pressure_control_core.h"
#include "../AgroSprayer/valve_ui_interlock.h"
#include "../AgroSprayer/aog_link_guard.h"
#include "../AgroSprayer/nozzle_calibration_core.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>
using agroreg::Controller;
using agroreg::Input;
static Input good() {
    Input i;
    i.now = 1000;
    i.autoMode = true; i.connected = true; i.sensorValid = true;
    i.driverReady = true; i.sectionsOpen = true;
    i.speedX10 = 80; i.targetBar = 3.0f; i.pressureBar = 2.5f;
    i.pulseMs = 250; i.pauseMs = 750; i.tolerancePercent = 5;
    return i;
}
int main() {
    int checks = 0;
    auto test=[&](bool cond, const char *name) {
        if(!cond) { std::cerr << "FAIL: " << name << "\n"; std::abort(); }
        ++checks;
    };
    // 04RSM table: 3 bar = 1.54 l/min; 4 bar = 1.78 l/min.
    // Q3 is the measured water output at 3 bar. Preserve V1/V2
    // estimated output when migrating stored settings to V3.
    {
        auto eq=[](float a,float b){return std::fabs(a-b)<1e-5f;};
        test(eq(agronozzle::calibratedWaterFlow(1.54f,1.54f,1,1.0f),1.54f),
             "04RSM default Q3 matches 3-bar table");
        test(eq(agronozzle::calibratedWaterFlow(1.54f,2.0f,1,1.0f),2.0f),
             "04RSM measured 3-bar water Q3");
        test(eq(agronozzle::calibratedWaterFlow(
                 1.54f,agronozzle::migrateLegacyRsmQ3(1.54f),1,1.0f),1.54f),
             "V1 Q3 migration preserves estimated output");
        test(eq(agronozzle::calibratedWaterFlow(
                 1.54f,agronozzle::migrateV2RsmQ3(1.78f),1,1.0f),1.54f),
             "V2 default migration preserves estimated output");
        test(eq(agronozzle::calibratedWaterFlow(
                 1.78f,agronozzle::migrateV2RsmQ3(2.0f),1,1.0f),2.0f),
             "V2 user calibration preserves 4-bar estimate");
        test(eq(agronozzle::calibratedWaterFlow(1.18f,1.18f,0,1.0f),1.18f),
             "11003 Q3 unchanged");
        test(eq(agronozzle::calibratedWaterFlow(1.54f,1.54f,1,1.32f),
                1.54f/std::sqrt(1.32f)),
             "RSM density correction applied once");
        for(float q : {0.1f,0.5f,1.78f,2.0f,5.0f}) {
            for(float density : {1.0f,1.32f}) {
                const float oldFlow=1.54f*(q/1.78f)/std::sqrt(density);
                const float newFlow=agronozzle::calibratedWaterFlow(
                    1.54f,agronozzle::migrateV2RsmQ3(q),1,density);
                test(eq(oldFlow,newFlow),"V2 migration preserves water/RSM flow");
            }
        }
    }
    // PGN239 packet validation: corrupted packets must never affect sections.
    {
        uint8_t p[14] = {0x80,0x81,0x7F,0xEF,8,0,80,0,0,0,0,0x3F,0,0};
        for (int k=2; k<13; ++k) p[13] = static_cast<uint8_t>(p[13]+p[k]);
        test(agroaog::validMachinePacket(p,14),"valid PGN239");
        test(!agroaog::validMachinePacket(nullptr,14),"null PGN239");
        test(!agroaog::validMachinePacket(p,13),"truncated PGN239");
        test(!agroaog::validMachinePacket(p,15),"extended PGN239");
        p[0]^=1; test(!agroaog::validMachinePacket(p,14),"bad preamble"); p[0]^=1;
        p[2]^=1; test(!agroaog::validMachinePacket(p,14),"bad sender"); p[2]^=1;
        p[3]^=1; test(!agroaog::validMachinePacket(p,14),"bad PGN"); p[3]^=1;
        p[4]=7; test(!agroaog::validMachinePacket(p,14),"bad payload length"); p[4]=8;
        p[13]^=1; test(!agroaog::validMachinePacket(p,14),"bad checksum"); p[13]^=1;
        p[6]^=1; test(!agroaog::validMachinePacket(p,14),"bad speed"); p[6]^=1;
        p[11]^=1; test(!agroaog::validMachinePacket(p,14),"bad sections"); p[11]^=1;
        test(agroaog::validMachinePacket(p,14),"PGN239 restored");
    }
    // PGN234 must send MAN/ALL OFF while AUTO is not acknowledged.
    // Compare the exact transmitted bytes, not a duplicated model.
    {
        uint8_t p[14] = {};
        auto validCrc = [&]() {
            uint8_t sum=0;
            for (int i=2;i<13;++i) sum=static_cast<uint8_t>(sum+p[i]);
            return sum == p[13];
        };
        agroaog::makeRemoteSwitchPacket(true,false,false,0x3F,6,p);
        test(p[0]==0x80 && p[1]==0x81 && p[2]==0x77 &&
             p[3]==0xEA && p[4]==8,"PGN234 header");
        test(p[5]==0x06,"unacknowledged AUTO sends master OFF");
        test(p[6]==0 && p[9]==0 && p[10]==0x3F,
             "unacknowledged AUTO forces six OFF");
        test(p[8]==6,"PGN234 section count");
        test(validCrc(),"PGN234 blocked checksum");
        agroaog::makeRemoteSwitchPacket(true,true,true,0x3F,6,p);
        test(p[5]==0x06 && p[9]==0 && p[10]==0x3F,
             "AUTO mode reset sends all OFF");
        test(validCrc(),"PGN234 reset checksum");
        agroaog::makeRemoteSwitchPacket(true,true,false,0x3F,6,p);
        test(p[5]==0x05 && p[6]==0x3F && p[9]==0 && p[10]==0,
             "acknowledged AUTO releases six sections to AOG");
        test(validCrc(),"PGN234 AUTO checksum");
        agroaog::makeRemoteSwitchPacket(false,false,false,0x15,6,p);
        test(p[5]==0x06 && p[6]==0 && p[9]==0x15 && p[10]==0x2A,
             "MAN section ON/OFF masks");
        test(validCrc(),"PGN234 MAN checksum");
        agroaog::makeRemoteSwitchPacket(false,false,false,0xFF,6,p);
        test(p[9]==0x3F && p[10]==0,"unused section bits masked");
        agroaog::makeRemoteSwitchPacket(false,false,false,0,6,p);
        test(p[9]==0 && p[10]==0x3F,"MAN all OFF");
        agroaog::makeRemoteSwitchPacket(true,false,false,0,6,p);
        test(p[9]==0 && p[10]==0x3F,"AUTO recovery fails closed");
        agroaog::makeRemoteSwitchPacket(false,true,false,0x01,6,p);
        test(p[5]==0x06 && p[9]==0x01,"MAN cannot be overridden by AUTO latch");
    }
    // Wi-Fi can stay connected while PGN239 stops: stale AUTO sections
    // and speed must be invalidated without changing manual overrides.
    {
        agroaog::Session link;
        bool sections[6] = {true,false,true,true,false,true};
        int speed = 80;
        link.packetReceived();
        agroaog::enforceMachineLinkFailSafe(link.fresh(true,1000,1000,1500),
                                            true,speed,sections,6);
        test(speed == 80 && sections[0] && sections[2],"fresh PGN keeps machine state");
        agroaog::enforceMachineLinkFailSafe(link.fresh(true,2501,1000,1500),
                                            true,speed,sections,6);
        test(speed == 0,"AOG timeout clears speed");
        test(!sections[0] && !sections[1] && !sections[2] &&
             !sections[3] && !sections[4] && !sections[5],
             "AOG timeout closes six AUTO sections");
        speed = 90; sections[0] = true;
        agroaog::enforceMachineLinkFailSafe(false,false,speed,sections,6);
        test(speed == 0,"manual mode clears stale speed");
        test(sections[0],"manual mode preserves operator sections");
        speed = 80;
        agroaog::enforceMachineLinkFailSafe(false,true,speed,nullptr,6);
        test(speed == 0,"null sections pointer is safe");
        link.reset();
        test(!link.fresh(true,3000,3000,1500),"new UDP session requires PGN");
        link.packetReceived();
        test(link.fresh(true,3001,3001,1500),"new PGN restores fresh link");
        test(!sections[2],"reconnect does not restore old AUTO mask");
    }
    // After AOG loss, PGN239 alone cannot re-enable AUTO.
    {
        agroaog::AutoRecoveryGate gate;
        test(!gate.permitsAuto(true,true),"AUTO blocked at boot");
        gate.onModeSelection(true,false);
        test(!gate.permitsAuto(true,true),"offline AUTO press cannot pre-arm");
        gate.onModeSelection(true,true);
        test(gate.permitsAuto(true,true),"operator AUTO on healthy link enables");
        test(!gate.permitsAuto(false,true),"MAN never permits AUTO");
        gate.onLinkStatus(true);
        test(gate.permitsAuto(true,true),"fresh PGN retains acknowledgement");
        gate.onLinkStatus(false);
        test(!gate.permitsAuto(true,true),"AOG loss latches AUTO off");
        gate.onLinkStatus(true);
        test(!gate.permitsAuto(true,true),"fresh PGN cannot auto-rearm");
        gate.onModeSelection(false,true);
        test(!gate.permitsAuto(true,true),"MAN selection clears latch");
        gate.onModeSelection(true,true);
        test(gate.permitsAuto(true,true),"explicit AUTO after recovery enables");
        gate.onLinkStatus(false);
        gate.onModeSelection(true,false);
        test(!gate.permitsAuto(true,true),"AUTO during outage stays blocked");
        gate.onModeSelection(true,true);
        test(gate.permitsAuto(true,true),"second healthy AUTO press acknowledges");
    }
    test(!agrovalve::shouldStopMotorForUi(true,false,true,true),
         "AUTO regulator pulse survives UI refresh");
    test(agrovalve::shouldStopMotorForUi(true,false,false,true),
         "unarmed AUTO stops stray motor motion");
    test(agrovalve::shouldStopMotorForUi(true,true,true,true),
         "bench mode always stops motor");
    test(agrovalve::shouldStopMotorForUi(false,true,false,true),
         "bench mode stops manual motor");
    test(!agrovalve::shouldStopMotorForUi(false,false,false,true),
         "manual mode permits manual valve pulse");
    test(!agrovalve::shouldStopMotorForUi(true,false,false,false),
         "stationary motor requires no redundant stop");
    test(agrovalve::autoPulseStillValid(4u,4u),"AUTO pulse allowed without intervening STOP");
    test(!agrovalve::autoPulseStillValid(4u,5u),"STOP invalidates pending AUTO pulse");
    test(!agrovalve::autoPulseStillValid(UINT32_MAX,0u),"STOP generation wrap rejects stale pulse");
    {
        agroaog::Session session;
        test(!session.fresh(true,1000,995,1500),"no AOG packet since boot");
        session.packetReceived();
        test(session.fresh(true,1000,995,1500),"fresh packet permits AOG link");
        test(!session.fresh(false,1000,995,1500),"UDP loss invalidates link");
        session.reset();
        test(!session.fresh(true,1001,995,1500),"reconnect cannot reuse old PGN");
        session.packetReceived();
        test(session.fresh(true,1002,1002,1500),"new PGN restores fresh session");
        test(!session.fresh(true,3000,1002,1500),"stale PGN expires");
        test(session.fresh(true,15,UINT32_MAX-15,1500),"millis rollover stays fresh");
    }
    auto in=good();
    {
        Controller c;
        test(!c.armed(),"boot disabled");
        test(c.step(in).direction==0,"cannot start on boot");
        test(c.arm(in),"explicit operator arm");
        auto a=c.step(in);
        test(a.direction==-1 && a.pulseMs==250,"short pressure-increase pulse");
        in.now+=100;in.motorMoving=true;
        test(c.step(in).direction==0,"no motor overlap");
        in.motorMoving=false;in.now+=1000;in.pressureBar=2.7f;
        test(c.step(in).direction==-1,"pressure rising correct polarity");
        in.now+=1100;in.pressureBar=2.4f;
        auto alarm=c.step(in);
        test(alarm.stopMotor && !c.armed(),"reverse polarity trips");
    }
    in=good();
    {
        Controller c;test(c.arm(in),"arm before target-reached test");
        test(c.step(in).direction==-1,"start pressure increase before target");
        in.now+=100;in.pressureBar=2.9f;in.motorMoving=true;
        auto a=c.step(in);
        test(a.stopMotor && a.direction==0 && c.armed(),
             "stop running motor immediately when pressure enters tolerance");
        in.now+=50;in.motorMoving=false;
        a=c.step(in);
        test(!a.stopMotor && a.direction==0 && c.armed(),
             "remain at target without repeated stop or restart");
    }
    in=good();in.pressureBar=3.5f;
    {
        Controller c;test(c.arm(in),"arm before falling-pressure target test");
        test(c.step(in).direction==1,"start pressure decrease before target");
        in.now+=100;in.pressureBar=3.1f;in.motorMoving=true;
        auto a=c.step(in);
        test(a.stopMotor && a.direction==0 && c.armed(),
             "stop running motor on target while reducing pressure");
    }
    in=good();
    {
        Controller c;c.arm(in);c.step(in);
        in.connected=false;in.now+=50;
        test(c.step(in).stopMotor && !c.armed(),"AOG loss trips");
        in.connected=true;in.now+=50;
        test(c.step(in).direction==0 && !c.armed(),"AOG recovery never auto rearms");
        test(c.arm(in),"AOG recovery requires explicit operator arm");
    }
    in=good();
    {
        Controller c;c.arm(in);c.step(in);
        in.sensorValid=false;in.now+=50;
        test(c.step(in).stopMotor && !c.armed(),"pressure sensor loss trips");
    }
    in=good();
    {
        Controller c;c.arm(in);c.step(in);
        in.pressureBar=5.0f;in.now+=50;
        test(c.step(in).stopMotor && !c.armed(),"5 bar high pressure trips");
    }
    in=good();
    {
        Controller c;c.arm(in);
        in.sectionsOpen=false;
        in.targetBar=std::numeric_limits<float>::quiet_NaN();
        auto a=c.step(in);
        test(a.stopMotor && a.direction==0 && c.armed(),
             "undefined target with closed sections pauses without disarming");
        in.sectionsOpen=true;in.now+=1000;
        a=c.step(in);
        test(a.stopMotor && a.direction==0 && !c.armed(),
             "invalid target after reopening sections stops and disarms");
        in.targetBar=3.0f;in.now+=1000;
        test(c.step(in).direction==0 && !c.armed(),
             "valid target after fault cannot automatically rearm");
    }
    in=good();
    {
        Controller c;c.arm(in);
        in.sectionsOpen=false;
        auto a=c.step(in);
        test(a.stopMotor && a.direction==0 && c.armed(),
             "headland with valid target stops without disarming");
        in.sectionsOpen=true;in.now+=1000;
        test(c.step(in).direction==0,"reopened sections wait for driver lockout");
        in.now+=90;
        test(c.step(in).direction==0,"no early motor pulse after reopen");
        in.now+=70;
        test(c.step(in).direction==-1,"resume after 150 ms motor lockout");
    }
    in=good();in.pressureBar=3.5f;
    {
        Controller c;test(c.arm(in),"arm while pressure is high but safe");
        test(c.step(in).direction==1,"decrease pressure opens bypass");
    }
    in=good();
    {
        Controller c;c.arm(in);
        for(int j=0;j<4;++j) {
            in.now=1000+j*1100;
            test(c.step(in).direction==-1,"no-pressure-response diagnostic pulses");
        }
        in.now=5400;
        test(c.step(in).stopMotor && !c.armed(),"four missing reactions trip");
    }
    in=good();in.connected=false;
    {Controller c;test(!c.arm(in) && !c.armed(),"no AOG cannot arm");}
    in=good();in.speedX10=0;
    {Controller c;test(!c.arm(in),"no speed cannot arm");}
    in=good();in.targetBar=5.1f;
    {Controller c;test(!c.arm(in),"unachievable target cannot arm");}
    in=good();
    {
        Controller c;c.arm(in);c.step(in);
        in.now=4001;
        test(c.step(in).stopMotor && !c.armed(),"blocked loop trips");
    }
    // Exhaustive PGN234 six-section mask regression: 64 masks x 4 modes.
    // Tests packet encoding only; physical relay fail-safe needs a separate test.
    {
        uint8_t p[14] = {};
        for (unsigned mask = 0; mask < 64; ++mask) {
            for (unsigned mode = 0; mode < 4; ++mode) {
                const bool selectedAuto = mode != 0;
                const bool permittedAuto = mode >= 2;
                const bool resetting = mode == 2;
                agroaog::makeRemoteSwitchPacket(selectedAuto, permittedAuto,
                                                resetting,
                                                static_cast<uint8_t>(mask), 6, p);
                const bool activeAuto = mode == 3;
                const bool manual = mode == 0;
                const uint8_t expectedOn = manual ? static_cast<uint8_t>(mask) : 0;
                const uint8_t expectedOff = manual ?
                    static_cast<uint8_t>(0x3Fu ^ mask) :
                    (activeAuto ? 0 : 0x3Fu);
                uint8_t checksum = 0;
                for (int k = 2; k < 13; ++k)
                    checksum = static_cast<uint8_t>(checksum + p[k]);
                test(p[5] == (activeAuto ? 0x05 : 0x06) &&
                     p[6] == (activeAuto ? 0x3F : 0),
                     "PGN234 master/auto mask matrix");
                test(p[9] == expectedOn, "PGN234 ON mask matrix");
                test(p[10] == expectedOff, "PGN234 OFF mask matrix");
                test(p[13] == checksum, "PGN234 checksum matrix");
            }
        }
    }
    // Cross-check the published RSM table against the safer AUTO control ceiling.
    // The table can estimate 6-bar flow, but the live controller intentionally
    // refuses targets above 4.8 bar; the installed 80-psi sensor is not a 6-bar sensor.
    {
        const float rsmQ6 = agronozzle::calibratedWaterFlow(2.18f, 1.54f, 1, 1.32f);
        const float requestedAt6 = agronozzle::targetPressureForFlow(
            rsmQ6 * 8.0f, 8, 1, 1.54f, 1.32f);
        test(std::fabs(requestedAt6 - 6.0f) < 0.001f,
             "RSM table can calculate the published 6-bar point");
        auto maxAllowed = good();
        maxAllowed.targetBar = 4.8f;
        Controller atLimit;
        test(atLimit.arm(maxAllowed), "AUTO accepts its configured 4.8-bar ceiling");
        auto aboveLimit = good();
        aboveLimit.targetBar = requestedAt6;
        Controller above;
        test(!above.arm(aboveLimit) && !above.armed(),
             "6-bar RSM target is rejected by the safer AUTO controller limit");
        aboveLimit.targetBar = 4.8001f;
        Controller justAbove;
        test(!justAbove.arm(aboveLimit),
             "AUTO rejects target immediately above 4.8-bar ceiling");
    }
    std::cout<<"OK: "<<checks<<" test assertions passed\n";
}
