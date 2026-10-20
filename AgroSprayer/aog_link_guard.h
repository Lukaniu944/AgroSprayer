#pragma once
// Require a fresh AgOpenGPS PGN239 in each UDP session before enabling AUTO.
// Pure C++ so reconnection behaviour can be tested without ESP32 hardware.
#include <cstdint>
#include <cstddef>

namespace agroaog {
// PGN239 (machine data) is exactly 14 bytes: 5-byte header,
// 8-byte payload and 1-byte additive checksum over bytes 2..12.
inline bool validMachinePacket(const uint8_t* p, std::size_t n) {
    if (p == nullptr || n != 14) return false;
    if (p[0] != 0x80 || p[1] != 0x81 || p[2] != 0x7F ||
        p[3] != 0xEF || p[4] != 8) return false;
    uint8_t sum = 0;
    for (std::size_t i = 2; i < 13; ++i)
        sum = static_cast<uint8_t>(sum + p[i]);
    return sum == p[13];
}
// PGN234 remote maintained switches. The explicit masks allow bench tests
// to check that link recovery cannot transmit a section ON command.
inline void makeRemoteSwitchPacket(bool autoSelected, bool autoPermitted,
                                   bool modeReset, uint8_t manualOnMask,
                                   uint8_t sectionCount, uint8_t (&p)[14]) {
    const uint8_t valid = sectionCount >= 8 ? 0xFFu :
                          static_cast<uint8_t>((1u << sectionCount) - 1u);
    const bool automatic = autoSelected && autoPermitted && !modeReset;
    const bool forceOff = autoSelected && !automatic;
    const uint8_t onMask = (forceOff || automatic) ? 0 :
                           static_cast<uint8_t>(manualOnMask & valid);
    const uint8_t offMask = forceOff ? valid : automatic ? 0 :
                            static_cast<uint8_t>(valid & ~onMask);
    const uint8_t main = static_cast<uint8_t>(0x04 | (automatic ? 0x01 : 0x02));
    const uint8_t frame[14] = {
        0x80,0x81,0x77,0xEA,0x08,main,
        static_cast<uint8_t>(automatic ? valid : 0),0,sectionCount,
        onMask,offMask,0,0,0
    };
    for (std::size_t i=0;i<14;++i) p[i] = frame[i];
    uint8_t sum=0;
    for (std::size_t i=2;i<13;++i)
        sum = static_cast<uint8_t>(sum+p[i]);
    p[13]=sum;
}
// Clear stale machine telemetry when the AgOpenGPS link expires.
// Preserve operator-selected MAN sections; AUTO sections must fail closed.
inline void enforceMachineLinkFailSafe(bool linkFresh, bool autoMode,
                                       int &speedX10, bool* sections,
                                       std::size_t sectionCount) {
    if (linkFresh) return;
    speedX10 = 0;
    if (autoMode && sections != nullptr)
        for (std::size_t i = 0; i < sectionCount; ++i)
            sections[i] = false;
}
// A new PGN239 alone cannot re-enable AUTO after a link loss.
// Operator must explicitly select AUTO while the machine link is fresh.
class AutoRecoveryGate {
    bool acknowledged_ = false;
public:
    void onLinkStatus(bool fresh) {
        if (!fresh) acknowledged_ = false;
    }
    void onModeSelection(bool autoMode, bool fresh) {
        acknowledged_ = autoMode && fresh;
    }
    bool permitsAuto(bool autoMode, bool fresh) const {
        return autoMode && fresh && acknowledged_;
    }
};
class Session {
    bool seenPacket_ = false;
public:
    void reset() { seenPacket_ = false; }
    void packetReceived() { seenPacket_ = true; }
    bool fresh(bool udpReady, uint32_t now, uint32_t lastPacketMs,
               uint32_t timeoutMs) const {
        return udpReady && seenPacket_ &&
               (uint32_t)(now - lastPacketMs) <= timeoutMs;
    }
};
} // namespace agroaog
