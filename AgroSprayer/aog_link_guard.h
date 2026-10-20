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
