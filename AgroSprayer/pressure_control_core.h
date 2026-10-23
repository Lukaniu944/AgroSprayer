#pragma once
// Hardware-independent pressure regulator; pure C++ for desktop tests.
// The regulator never enables itself on boot or when AUTO sections are selected.
#include <cmath>
#include <cstdint>

namespace agroreg {
struct Input {
    uint32_t now = 0;
    bool autoMode = false;
    bool connected = false;
    bool sensorValid = false;
    bool driverReady = false;
    bool benchMode = false;
    bool sectionsOpen = false;
    bool motorMoving = false;
    int speedX10 = 0;
    float pressureBar = 0;
    float targetBar = 0;
    uint32_t pulseMs = 250;
    uint32_t pauseMs = 750;
    uint32_t tolerancePercent = 5;
};
struct Action {
    // Logical +1 = OPEN bypass, -1 = CLOSE bypass.
    int direction = 0;
    uint32_t pulseMs = 0;
    bool stopMotor = false;
};
class Controller {
    bool armed_ = false;
    bool resumePending_ = false, resumeDelay_ = false;
    uint32_t resumedAt_ = 0;
    uint32_t lastTick_ = 0, lastPulse_ = 0;
    bool hasLastPulse_ = false;
    float pressureBefore_ = 0;
    int expectedSign_ = 0, unanswered_ = 0;
    const char *status_ = "WYLACZONY";
    void fault(const char *reason) {
        armed_ = false;
        status_ = reason;
        expectedSign_ = unanswered_ = 0;
    }
    bool validBasic(const Input &in) const {
        return in.autoMode && in.connected && in.sensorValid &&
               in.driverReady && !in.benchMode &&
               std::isfinite(in.pressureBar) && in.pressureBar >= 0.5f &&
               in.pressureBar < 5.0f;
    }
    bool validTarget(const Input &in) const {
        return std::isfinite(in.targetBar) && in.targetBar >= 2.0f &&
               in.targetBar <= 4.8f;
    }
    bool valid(const Input &in) const {
        return validBasic(in) && validTarget(in);
    }
public:
    bool armed() const { return armed_; }
    const char *status() const { return status_; }
    bool arm(const Input &in) {
        if (!valid(in) || !in.sectionsOpen || in.speedX10 <= 0 ||
            in.motorMoving) {
            status_ = "BRAK WARUNKOW STARTU";
            return false;
        }
        armed_ = true;
        lastTick_ = in.now;
        lastPulse_ = 0;
        hasLastPulse_ = false;
        resumePending_ = resumeDelay_ = false;
        resumedAt_ = 0;
        pressureBefore_ = 0;
        expectedSign_ = unanswered_ = 0;
        status_ = "GOTOWY";
        return true;
    }
    void disarm() {
        armed_ = false;
        lastTick_ = lastPulse_ = resumedAt_ = 0;
        hasLastPulse_ = false;
        resumePending_ = resumeDelay_ = false;
        expectedSign_ = unanswered_ = 0;
        status_ = "WYLACZONY";
    }
    void driverFailed() { fault("BLAD STEROWNIKA ZAWORU"); }
    Action step(const Input &in) {
        Action out;
        if (!armed_) return out;
        if (lastTick_ && (uint32_t)(in.now-lastTick_) > 2000) {
            fault("PRZERWA PROGRAMU");
            out.stopMotor = true;
            return out;
        }
        lastTick_ = in.now;
        if (!validBasic(in)) {
            fault("STOP: AOG / CZUJNIK / TRYB / CISNIENIE");
            out.stopMotor = true;
            return out;
        }
        if (!in.sectionsOpen || in.speedX10 <= 0) {
            // An undefined target is normal when every section is closed or
            // the tractor is stationary. Pause safely without a motor pulse;
            // keep link, sensor and pressure faults fatal (checked above).
            status_ = "CZEKAM NA SEKCJE";
            resumePending_ = true;
            resumeDelay_ = false;
            lastPulse_ = 0;
            hasLastPulse_ = false;
            expectedSign_ = unanswered_ = 0;
            out.stopMotor = true;
            return out;
        }
        if (!validTarget(in)) {
            // Once spraying resumes, an invalid target is always a fault.
            fault("STOP: CEL CISNIENIA POZA ZAKRESEM");
            out.stopMotor = true;
            return out;
        }
        if (resumePending_) {
            resumePending_ = false;
            resumeDelay_ = true;
            resumedAt_ = in.now;
        }
        if (resumeDelay_) {
            if ((uint32_t)(in.now - resumedAt_) < 150) {
                status_ = "PRZERWA PO SEKCJACH";
                return out;
            }
            resumeDelay_ = false;
        }
        const float error = in.targetBar - in.pressureBar;
        const float tolerance = std::fmax(0.15f,
            in.targetBar * in.tolerancePercent / 100.0f);
        if (std::fabs(error) <= tolerance) {
            status_ = "W CELU";
            expectedSign_ = unanswered_ = 0;
            // Do not finish an in-flight pulse after pressure reaches target.
            // The motor watchdog is a fallback; stop as soon as safe pressure is seen.
            out.stopMotor = in.motorMoving;
            return out;
        }
        if (in.motorMoving) {
            status_ = "RUCH ZAWORU";
            return out;
        }
        const uint32_t pulse = in.pulseMs < 250 ? in.pulseMs : 250;
        if (hasLastPulse_ &&
            (uint32_t)(in.now-lastPulse_) < pulse + in.pauseMs) {
            status_ = "PRZERWA PO IMPULSIE";
            return out;
        }
        const int desiredSign = error > 0 ? 1 : -1;
        if (expectedSign_ != 0 && expectedSign_ == desiredSign) {
            const float change = (in.pressureBar - pressureBefore_) *
                                 expectedSign_;
            if (change <= -0.20f) {
                fault("STOP: ODWROTNY KIERUNEK");
                out.stopMotor = true;
                return out;
            }
            if (change >= 0.10f) unanswered_ = 0;
            else if (++unanswered_ >= 4) {
                fault("STOP: BRAK REAKCJI ZAWORU");
                out.stopMotor = true;
                return out;
            }
        } else {
            unanswered_ = 0;
        }
        // This motor drives the Bypass: closing raises spray pressure.
        // The existing A/B direction swap must first be checked by operator.
        out.direction = desiredSign > 0 ? -1 : 1;
        out.pulseMs = pulse >= 50 ? pulse : 50;
        lastPulse_ = in.now;
        hasLastPulse_ = true;
        pressureBefore_ = in.pressureBar;
        expectedSign_ = desiredSign;
        status_ = desiredSign > 0 ? "PODNOSZE CISNIENIE" : "OBNIZAM CISNIENIE";
        return out;
    }
};
} // namespace agroreg
