#pragma once
// Require a fresh AgOpenGPS PGN239 in each UDP session before enabling AUTO.
// Pure C++ so reconnection behaviour can be tested without ESP32 hardware.
#include <cstdint>

namespace agroaog {
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
