#include "aog.h"
#include "config.h"
#include "board_config.h"
#include "aog_pgn_core.h"
#include "sprayer_state.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <array>

static WiFiUDP udp;
static IPAddress broadcastIp(255,255,255,255);
static uint32_t lastTx = 0;
static bool pendingTx = false;
static bool udpStarted = false;
static bool modeReset = false;
static bool resetSent = false;
static uint32_t resetStartMs = 0;
static constexpr uint32_t MODE_RESET_MS = 500;

static_assert(SECTION_COUNT > 0 && SECTION_COUNT <= 8,
              "PGN234 section mask supports 1..8 sections in the low group");

static uint8_t sectionMask() {
    uint8_t m = 0;
    for (int i=0;i<SECTION_COUNT;i++) if (gState.sections[i]) m |= 1u << i;
    return m;
}

constexpr auto kAutoSwitchPacket =
    agopgn::build_switch_packet<SECTION_COUNT>(true, false, 0x3F);
static_assert(kAutoSwitchPacket[5] == 0x05 &&
              kAutoSwitchPacket[6] == 0x3F &&
              kAutoSwitchPacket[8] == SECTION_COUNT &&
              kAutoSwitchPacket[9] == 0 && kAutoSwitchPacket[10] == 0,
              "AUTO must delegate all six sections to AgOpenGPS");
static_assert(kAutoSwitchPacket[13] == 0xB3,
              "PGN234 AUTO packet checksum regression");

constexpr auto kManualSwitchPacket =
    agopgn::build_switch_packet<SECTION_COUNT>(false, false, 0x09);
static_assert(kManualSwitchPacket[5] == 0x06 &&
              kManualSwitchPacket[6] == 0 &&
              kManualSwitchPacket[9] == 0x09 &&
              kManualSwitchPacket[10] == 0x36,
              "MANUAL must encode requested ON bits and complementary OFF bits");
static_assert(kManualSwitchPacket[13] == 0xB4,
              "PGN234 MANUAL packet checksum regression");

constexpr auto kModeResetPacket =
    agopgn::build_switch_packet<SECTION_COUNT>(true, true, 0x3F);
static_assert(kModeResetPacket[5] == 0x06 &&
              kModeResetPacket[6] == 0 &&
              kModeResetPacket[9] == 0 &&
              kModeResetPacket[10] == 0x3F &&
              kModeResetPacket[13] == 0xB4,
              "AUTO handover reset must send all-sections-OFF first");

static void updateBroadcast() {
    IPAddress ip = WiFi.localIP();
    IPAddress mask = WiFi.subnetMask();
    for (int i=0;i<4;i++) broadcastIp[i] = ip[i] | (uint8_t)(~mask[i]);
}

void aog_begin() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void aog_send_switch_now() { if (!bench_test_active()) pendingTx = true; }

void aog_send_mode_now() {
    if (bench_test_active()) return;
    // MAN immediately sends OFF masks. AUTO first clears manual overrides.
    modeReset = gState.autoMode;
    resetSent = false;
    pendingTx = true;
}

static void sendSwitch() {
    if (bench_test_active()) return;
    if (WiFi.status() != WL_CONNECTED) return;

    const auto p = agopgn::build_switch_packet<SECTION_COUNT>(gState.autoMode, modeReset, sectionMask());

    if (udp.beginPacket(broadcastIp, AOG_TX_PORT) != 1) return;
    if (udp.write(p.data(), p.size()) != p.size()) { udp.endPacket(); return; }
    if (udp.endPacket() != 1) return;
    gState.txPackets++;
    if (modeReset && !resetSent) {
        resetStartMs = millis();
        resetSent = true;
    }
}

static void parse(uint8_t *p, int n) {
    // Ignore truncated, oversized, wrong-source and checksum-invalid datagrams:
    // they must not refresh the AOG watchdog or alter section states.
    if (!agopgn::valid_machine_data_pgn239(p, static_cast<std::size_t>(n))) return;

    gState.speedX10 = p[6];
    if (gState.autoMode && !modeReset) {
        uint8_t mask = p[11];
        for (int i=0;i<SECTION_COUNT;i++) gState.sections[i] = (mask & (1u << i)) != 0;
    }
    gState.lastAogMs = millis();
    gState.rxPackets++;
}

void aog_update() {
    gState.wifiOk = WiFi.status() == WL_CONNECTED;

    // A UDP socket from the old WLAN lease is not a valid connection.
    // Reopen it and recompute the broadcast address after reconnection.
    if (!gState.wifiOk && udpStarted) {
        udp.stop();
        udpStarted = false;
    }
    if (gState.wifiOk && !udpStarted) {
        updateBroadcast();
        udpStarted = udp.begin(AOG_RX_PORT) == 1;
    }

    gState.udpOk = gState.wifiOk && udpStarted;
    if (modeReset && !gState.autoMode) modeReset = false;
    if (modeReset && resetSent && (uint32_t)(millis()-resetStartMs)>=MODE_RESET_MS) {
        modeReset = false;
        pendingTx = true;
    }

    if (udpStarted) {
        int sz = udp.parsePacket();
        while (sz > 0) {
            uint8_t buf[128];
            int n = udp.read(buf,sizeof(buf));
            if (n > 0) parse(buf,n);
            sz = udp.parsePacket();
        }
    }

    uint32_t now = millis();
    // Require a real, fresh PGN239 packet before exposing cached machine data.
    gState.aogOk = gState.udpOk && gState.rxPackets > 0 &&
                   agopgn::machine_data_is_fresh(now, gState.lastAogMs, AOG_TIMEOUT_MS);

    // Do not keep displaying/using stale speed or section states after AOG/Wi-Fi
    // data expires. This clears local state only; it is not a hardware shutoff.
    agopgn::clear_stale_machine_state(gState.aogOk, gState.speedX10,
                                      gState.sections, SECTION_COUNT);

    if (bench_test_active()) {
        pendingTx = false;
        modeReset = resetSent = false;
        lastTx = now;
    } else if (udpStarted && (pendingTx || now - lastTx >= SWITCH_TX_INTERVAL_MS)) {
        sendSwitch();
        lastTx = now;
        pendingTx = false;
    }
}
