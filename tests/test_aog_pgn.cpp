#include "aog_pgn_core.h"
#include <array>
#include <cassert>
#include <cstdint>
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

    std::cout << "OK: " << checks << " AgOpenGPS PGN239 assertions passed\n";
    return 0;
}
