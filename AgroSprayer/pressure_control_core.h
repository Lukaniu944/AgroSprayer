#pragma once
// Hardware-independent pressure regulator; pure C++ for desktop tests.
// The regulator never enables itself on boot or when AUTO sections are selected.
#include <cmath>
#include <cstdint>

namespace agroreg {
constexpr uint32_t MIN_PULSE_MS = 50;
constexpr uint32_t MAX_PULSE_MS = 250;
constexpr uint32_t MIN_PAUSE_MS = 100;
constexpr uint32_t MAX_PAUSE_MS = 3000;
constexpr uint32_t MIN_TOLERANCE_PERCENT = 1;
constexpr uint32_t MAX_TOLERANCE_PERCENT = 20;
constexpr bool valid_settings(uint32_t pulseMs, uint32_t pauseMs, uint32_t tolerancePercent) {
 return pulseMs >= MIN_PULSE_MS && pulseMs <= MAX_PULSE_MS && pauseMs >= MIN_PAUSE_MS && pauseMs <= MAX_PAUSE_MS && tolerancePercent >= MIN_TOLERANCE_PERCENT && tolerancePercent <= MAX_TOLERANCE_PERCENT;
}
struct Input {
    uint32_t now = 0;
    bool autoMode = false;
    bool connected = false;
    bool sensorValid = false;
    bool pressureTrip = false;
    bool driverReady = false;
    bool driverCanPulse = true;
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
    int direction = 0;
    uint32_t pulseMs = 0;
    bool stopMotor = false;
};
class Controller {
    bool armed_ = false;
    bool resumePending_ = false, resumeDelay_ = false;
    uint32_t resumedAt_ = 0;
    uint32_t lastTick_ = 0, lastPulse_ = 0;
    float pressureBefore_ = 0;
    int expectedSign_ = 0, unanswered_ = 0;
    const char *status_ = "WYLACZONY";
    void fault(const char *reason) {
        armed_ = false;
        status_ = reason;
        expectedSign_ = unanswered_ = 0;
    }
    bool validBasic(const Input &in) const {
        return in.autoMode && in.connected && in.sensorValid && !in.pressureTrip &&
               in.driverReady && !in.benchMode &&
               std::isfinite(in.pressureBar) && in.pressureBar >= 0.5f &&
               in.pressureBar < 5.0f;
    }
    bool validTarget(const Input &in) const {
        return std::isfinite(in.targetBar) && in.targetBar >= 2.0f &&
               in.targetBar <= 4.8f;
    }
    bool valid(const Input &in) const {
        return validBasic(in) && validTarget(in) && valid_settings(in.pulseMs, in.pauseMs, in.tolerancePercent);
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
        if (in.pressureTrip) {
            fault("STOP: NAD CISNIENIE");
            out.stopMotor = true;
            return out;
        }
        if (!validBasic(in)) {
            fault("STOP: AOG / CZUJNIK / TRYB / CISNIENIE");
            out.stopMotor = true;
            return out;
        }
        if (!valid_settings(in.pulseMs, in.pauseMs, in.tolerancePercent)) {
            fault("STOP: USTAWIENIA REGULATORA");
            out.stopMotor = true;
            return out;
        }
        if (!in.sectionsOpen || in.speedX10 <= 0) {
            status_ = "CZEKAM NA SEKCJE";
            resumePending_ = true;
            resumeDelay_ = false;
            lastPulse_ = 0;
            expectedSign_ = unanswered_ = 0;
            out.stopMotor = true;
            return out;
        }
        if (!validTarget(in)) {
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
            out.stopMotor = in.motorMoving;
            return out;
        }
        if (in.motorMoving) {
            status_ = "RUCH ZAWORU";
            return out;
        }
        const uint32_t pulse = in.pulseMs < 250 ? in.pulseMs : 250;
        if (lastPulse_ &&
            (uint32_t)(in.now-lastPulse_) < pulse + in.pauseMs) {
            status_ = "PRZERWA PO IMPULSIE";
            return out;
        }
        if (!in.driverCanPulse) {
            status_ = "OCZEKUJE NA STEROWNIK";
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
        out.direction = desiredSign > 0 ? -1 : 1;
        out.pulseMs = pulse >= 50 ? pulse : 50;
        lastPulse_ = in.now;
        pressureBefore_ = in.pressureBar;
        expectedSign_ = desiredSign;
        status_ = desiredSign > 0 ? "PODNOSZE CISNIENIE" : "OBNIZAM CISNIENIE";
        return out;
    }
};
} // namespace agroreg
