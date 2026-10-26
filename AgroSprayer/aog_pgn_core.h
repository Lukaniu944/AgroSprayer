#pragma once
#include <cstddef>
#include <cstdint>
#include <array>

namespace agopgn {

// AgOpenGPS PGN 239 (Machine Data): 5-byte header, 8-byte payload, 1-byte
// checksum. Checksum is the low byte of the sum of source through payload.
inline bool valid_machine_data_pgn239(const uint8_t *packet, std::size_t size) {
    if (!packet || size != 14 ||
        packet[0] != 0x80 || packet[1] != 0x81 ||
        packet[2] != 0x7F || packet[3] != 0xEF ||
        packet[4] != 8) {
        return false;
    }

    uint16_t sum = 0;
    for (std::size_t i = 2; i <= 12; ++i) sum += packet[i];
    return static_cast<uint8_t>(sum & 0xFFu) == packet[13];
}


inline uint8_t switch_checksum(const std::array<uint8_t, 14> &packet) {
    uint16_t sum = 0;
    for (std::size_t i = 2; i <= 12; ++i) sum += packet[i];
    return static_cast<uint8_t>(sum & 0xFFu);
}

// Builds the current PGN234 maintained-state packet without changing its
// existing field mapping. Host tests can exercise all six section masks.
template <std::size_t SectionCount>
constexpr std::array<uint8_t, 14> build_switch_packet(
        bool autoMode, bool reset, uint8_t requestedMask) {
    static_assert(SectionCount >= 1 && SectionCount <= 8,
                  "PGN234 section mask supports 1..8 sections");
    constexpr uint8_t valid =
        static_cast<uint8_t>((1u << SectionCount) - 1u);
    const bool automatic = autoMode && !reset;
    const uint8_t onMask = (reset || automatic)
        ? 0
        : static_cast<uint8_t>(requestedMask & valid);
    const uint8_t offMask = reset
        ? valid
        : automatic ? 0 : static_cast<uint8_t>((~onMask) & valid);
    const uint8_t main = static_cast<uint8_t>(
        0x04 | (automatic ? 0x01 : 0x02));

    std::array<uint8_t, 14> packet = {
        0x80, 0x81, 0x77, 0xEA, 0x08,
        main, static_cast<uint8_t>(automatic ? valid : 0), 0x00,
        static_cast<uint8_t>(SectionCount), onMask, offMask, 0x00, 0x00, 0x00
    };
    packet[13] = switch_checksum(packet);
    return packet;
}

} // namespace agopgn
