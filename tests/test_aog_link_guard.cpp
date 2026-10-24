#include "../AgroSprayer/aog_link_guard.h"
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>

int main() {
    int checks = 0;
    auto check = [&](bool ok, const char *name) {
        if (!ok) {
            std::cerr << "FAIL: " << name << '\n';
            std::abort();
        }
        ++checks;
    };
    auto seal = [](uint8_t (&p)[14]) {
        uint8_t sum = 0;
        for (std::size_t i = 2; i < 13; ++i)
            sum = static_cast<uint8_t>(sum + p[i]);
        p[13] = sum;
    };
    auto checksumOk = [](const uint8_t (&p)[14]) {
        uint8_t sum = 0;
        for (std::size_t i = 2; i < 13; ++i)
            sum = static_cast<uint8_t>(sum + p[i]);
        return sum == p[13];
    };
    auto makeMachinePacket = [&]() {
        uint8_t p[14] = {0x80, 0x81, 0x7F, 0xEF, 0x08,
                         0, 0, 0, 0, 0, 0, 0, 0, 0};
        seal(p);
        return p;
    };

    // Reject malformed PGN239 packets before they can refresh the AOG link.
    auto packet = makeMachinePacket();
    check(agroaog::validMachinePacket(packet, sizeof(packet)),
          "valid 14-byte PGN239 accepted");
    check(!agroaog::validMachinePacket(nullptr, sizeof(packet)),
          "null packet rejected");
    check(!agroaog::validMachinePacket(packet, 13),
          "truncated packet rejected");
    check(!agroaog::validMachinePacket(packet, 15),
          "oversized packet rejected");

    auto bad = makeMachinePacket();
    bad[0] = 0;
    seal(bad);
    check(!agroaog::validMachinePacket(bad, sizeof(bad)),
          "wrong preamble rejected");
    bad = makeMachinePacket();
    bad[2] = 0x7E;
    seal(bad);
    check(!agroaog::validMachinePacket(bad, sizeof(bad)),
          "wrong PGN rejected");
    bad = makeMachinePacket();
    bad[4] = 7;
    seal(bad);
    check(!agroaog::validMachinePacket(bad, sizeof(bad)),
          "wrong payload length rejected");
    bad = makeMachinePacket();
    bad[13] ^= 1u;
    check(!agroaog::validMachinePacket(bad, sizeof(bad)),
          "bad checksum rejected");

    // A new UDP session is not fresh until a valid packet is recorded.
    agroaog::Session session;
    check(!session.fresh(true, 100, 100, 1500),
          "session without packet is not fresh");
    session.packetReceived();
    check(session.fresh(true, 100, 100, 1500),
          "received packet makes session fresh");
    check(session.fresh(true, 1600, 100, 1500),
          "packet valid at exact timeout boundary");
    check(!session.fresh(true, 1601, 100, 1500),
          "expired packet makes session stale");
    check(!session.fresh(false, 100, 100, 1500),
          "UDP socket down makes session stale");
    session.reset();
    check(!session.fresh(true, 100, 100, 1500),
          "reset requires a new packet");
    agroaog::Session rolloverSession;
    rolloverSession.packetReceived();
    check(rolloverSession.fresh(true, 499u, UINT32_MAX - 500u, 1000u),
          "freshness timeout works across millis rollover");
    check(!rolloverSession.fresh(true, 500u, UINT32_MAX - 500u, 1000u),
          "staleness detected across millis rollover");

    // AUTO must remain locked after link loss until the operator reselects it.
    agroaog::AutoRecoveryGate gate;
    check(!gate.permitsAuto(true, true),
          "AUTO initially requires operator acknowledgement");
    gate.onModeSelection(true, true);
    check(gate.permitsAuto(true, true),
          "explicit AUTO selection on fresh link permits AUTO");
    gate.onLinkStatus(false);
    check(!gate.permitsAuto(true, true),
          "link loss clears AUTO acknowledgement");
    gate.onLinkStatus(true);
    check(!gate.permitsAuto(true, true),
          "fresh telemetry alone cannot restore AUTO");
    gate.onModeSelection(false, true);
    check(!gate.permitsAuto(true, true),
          "MAN selection does not acknowledge AUTO");
    gate.onModeSelection(true, true);
    check(gate.permitsAuto(true, true),
          "operator can re-arm AUTO after fresh link");

    // Loss of link closes AUTO sections but preserves MAN operator choices.
    int speed = 125;
    bool autoSections[6] = {true, true, true, true, true, true};
    agroaog::enforceMachineLinkFailSafe(false, true, speed, autoSections, 6);
    check(speed == 0, "AUTO link loss clears stale speed");
    for (bool section : autoSections)
        check(!section, "AUTO link loss closes every section");

    speed = 80;
    bool manualSections[6] = {true, false, true, false, true, false};
    agroaog::enforceMachineLinkFailSafe(false, false, speed, manualSections, 6);
    check(speed == 0, "MAN link loss clears stale speed");
    check(manualSections[0] && !manualSections[1] && manualSections[2] &&
          !manualSections[3] && manualSections[4] && !manualSections[5],
          "MAN link loss preserves operator section choices");

    speed = 80;
    bool freshSections[6] = {true, false, true, false, true, false};
    agroaog::enforceMachineLinkFailSafe(true, true, speed, freshSections, 6);
    check(speed == 80 && freshSections[0] && !freshSections[1],
          "fresh link leaves telemetry and section state unchanged");

    // Six-section PGN234 masks: AUTO only sends ON after recovery; otherwise OFF.
    uint8_t out[14] = {};
    agroaog::makeRemoteSwitchPacket(true, true, false, 0x3F, 6, out);
    check(out[5] == 0x05 && out[6] == 0x3F,
          "permitted AUTO packet advertises six valid sections");
    check(out[9] == 0 && out[10] == 0,
          "AUTO packet does not send manual ON/OFF masks");
    check(checksumOk(out), "AUTO packet checksum is valid");

    agroaog::makeRemoteSwitchPacket(true, false, false, 0x3F, 6, out);
    check(out[5] == 0x06 && out[6] == 0,
          "unacknowledged AUTO packet is not automatic");
    check(out[9] == 0 && out[10] == 0x3F,
          "unacknowledged AUTO explicitly commands all six sections OFF");
    check(checksumOk(out), "forced-OFF packet checksum is valid");

    agroaog::makeRemoteSwitchPacket(true, true, true, 0x3F, 6, out);
    check(out[9] == 0 && out[10] == 0x3F,
          "AUTO mode-reset interval forces all six sections OFF");
    check(checksumOk(out), "mode-reset packet checksum is valid");

    agroaog::makeRemoteSwitchPacket(false, false, false, 0x49, 6, out);
    check(out[5] == 0x06 && out[9] == 0x09 && out[10] == 0x36,
          "MAN packet masks commands to the six installed sections");
    check(checksumOk(out), "MAN packet checksum is valid");

    std::cout << "OK: " << checks << " AgOpenGPS link-guard assertions passed\n";
}
