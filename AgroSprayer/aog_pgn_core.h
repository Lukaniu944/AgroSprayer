#pragma once
#include <cstddef>
#include <cstdint>

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

} // namespace agopgn
