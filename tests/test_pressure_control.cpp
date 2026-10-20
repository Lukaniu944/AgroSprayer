#include "../AgroSprayer/pressure_control_core.h"
#include "../AgroSprayer/valve_ui_interlock.h"
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
        test(a.stopMotor && a.direction==0 && c.armed(),"headland: stop without disarming");
        in.sectionsOpen=true;in.targetBar=3.0f;in.now+=1000;
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
    std::cout<<"OK: "<<checks<<" test assertions passed\n";
}
