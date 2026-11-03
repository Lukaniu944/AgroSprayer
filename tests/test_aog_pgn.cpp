#include "aog_pgn_core.h"
#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>

static void set_checksum(std::array<uint8_t, 14> &p) {
    uint16_t sum = 0;
    for (int i = 2; i <= 12; ++i) sum += p[static_cast<std::size_t>(i)];
    p[13] = static_cast<uint8_t>(sum & 0xFFu);
}

int main() {
    int checks = 0;
    auto test = [&](bool condition, const char *name) {
        if (!condition) {
            std::cerr << "FAIL: " << name << "\n";
            std::abort();
        }
        ++checks;
    };

    test(agopgn::machine_data_is_fresh(1500, 0, 1500),
         "millis timeout boundary is fresh");
    test(!agopgn::machine_data_is_fresh(1501, 0, 1500),
         "millis timeout exceeded is stale");
    test(agopgn::machine_data_is_fresh(0x10u, 0xFFFFFFF0u, 1500),
         "millis rollover remains fresh");

    int staleSpeed = 80;
    bool staleSections[6] = {true, true, true, true, true, true};
    agopgn::clear_stale_machine_state(false, staleSpeed, staleSections, 6);
    test(staleSpeed == 0, "stale AOG data clears cached speed");
    for (bool section : staleSections)
        test(!section, "stale AOG data clears each cached section");

    int freshSpeed = 80;
    bool freshSections[6] = {true, false, true, false, true, false};
    agopgn::clear_stale_machine_state(true, freshSpeed, freshSections, 6);
    test(freshSpeed == 80, "fresh AOG data preserves speed");
    for (int i = 0; i < 6; ++i)
        test(freshSections[i] == ((i % 2) == 0),
             "fresh AOG data preserves section mask");

    std::array<uint8_t, 14> p = {
        0x80, 0x81, 0x7F, 0xEF, 0x08,
        0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00
    };
    set_checksum(p);
    test(agopgn::valid_machine_data_pgn239(p.data(), p.size()),
         "valid PGN239 frame accepted");
    test(!agopgn::valid_machine_data_pgn239(nullptr, p.size()),
         "null packet rejected");
    test(!agopgn::valid_machine_data_pgn239(p.data(), 13),
         "truncated frame rejected");
    test(!agopgn::valid_machine_data_pgn239(p.data(), 15),
         "oversized frame rejected");

    auto bad = p;
    bad[0] = 0x00;
    set_checksum(bad);
    test(!agopgn::valid_machine_data_pgn239(bad.data(), bad.size()),
         "wrong header rejected");

    bad = p;
    bad[2] = 0x7E;
    set_checksum(bad);
    test(!agopgn::valid_machine_data_pgn239(bad.data(), bad.size()),
         "wrong source rejected");

    bad = p;
    bad[3] = 0xEE;
    set_checksum(bad);
    test(!agopgn::valid_machine_data_pgn239(bad.data(), bad.size()),
         "wrong PGN rejected");

    bad = p;
    bad[4] = 7;
    set_checksum(bad);
    test(!agopgn::valid_machine_data_pgn239(bad.data(), bad.size()),
         "wrong payload length rejected");

    bad = p;
    bad[6] ^= 0x01;
    test(!agopgn::valid_machine_data_pgn239(bad.data(), bad.size()),
         "checksum mismatch rejected");


    // Outgoing PGN234 packet tests: header, AUTO/MANUAL/reset and all 64
    // possible masks for the six-section 21 m boom.
    auto validSwitchChecksum = [](const std::array<uint8_t, 14> &packet) {
        uint16_t sum = 0;
        for (int i = 2; i <= 12; ++i) sum += packet[static_cast<std::size_t>(i)];
        return static_cast<uint8_t>(sum & 0xFFu) == packet[13];
    };
    const auto autoPacket = agopgn::build_switch_packet<6>(true, false, 0x3F);
    test(autoPacket[0] == 0x80 && autoPacket[1] == 0x81 &&
         autoPacket[2] == 0x77 && autoPacket[3] == 0xEA &&
         autoPacket[4] == 0x08 && autoPacket[8] == 6,
         "PGN234 header and six-section count");
    test(autoPacket[5] == 0x05 && autoPacket[6] == 0x3F &&
         autoPacket[9] == 0 && autoPacket[10] == 0,
         "PGN234 AUTO delegates sections without manual overrides");
    test(validSwitchChecksum(autoPacket), "PGN234 AUTO checksum");

    const auto manualPacket = agopgn::build_switch_packet<6>(false, false, 0x49);
    test(manualPacket[5] == 0x06 && manualPacket[6] == 0 &&
         manualPacket[9] == 0x09 && manualPacket[10] == 0x36,
         "PGN234 MANUAL masks unsupported bits and complements OFF mask");
    test(validSwitchChecksum(manualPacket), "PGN234 MANUAL checksum");

    const auto resetPacket = agopgn::build_switch_packet<6>(true, true, 0x3F);
    test(resetPacket[5] == 0x06 && resetPacket[6] == 0 &&
         resetPacket[9] == 0 && resetPacket[10] == 0x3F,
         "PGN234 AUTO handover first requests all sections OFF");
    test(validSwitchChecksum(resetPacket), "PGN234 reset checksum");

    for (uint8_t mask = 0; mask < 64; ++mask) {
        const auto packet = agopgn::build_switch_packet<6>(false, false, mask);
        test(packet[9] == mask, "PGN234 preserves each six-section ON mask");
        test(packet[10] == static_cast<uint8_t>((~mask) & 0x3F),
             "PGN234 OFF mask is complementary within six sections");
        test(validSwitchChecksum(packet), "PGN234 checksum valid for every section mask");
    }

    std::cout << "OK: " << checks << " AgOpenGPS protocol assertions passed\n";
    return 0;
}
