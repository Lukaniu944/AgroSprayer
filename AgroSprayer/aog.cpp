#include "aog.h"
#include "config.h"
#include "board_config.h"
#include "sprayer_state.h"
#include "nozzle_model.h"
#include "aog_link_guard.h"
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP udp;
static agroaog::Session aogSession;
static agroaog::AutoRecoveryGate aogRecovery;
static IPAddress broadcastIp(255,255,255,255);
static uint32_t lastTx = 0;
static bool pendingTx = false;
static bool udpStarted = false;
static bool modeReset = false;
static bool resetSent = false;
static uint32_t resetStartMs = 0;
static constexpr uint32_t MODE_RESET_MS = 500;

static uint8_t sectionMask() {
    uint8_t m = 0;
    for (int i=0;i<SECTION_COUNT;i++) if (gState.sections[i]) m |= 1u << i;
    return m;
}

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
    // Acknowledge recovery only after an explicit AUTO press on a fresh link.
    const bool fresh = aogSession.fresh(gState.udpOk, millis(),
                                        gState.lastAogMs, AOG_TIMEOUT_MS);
    aogRecovery.onModeSelection(gState.autoMode, fresh);
    modeReset = gState.autoMode;
    resetSent = false;
    pendingTx = true;
}

static void sendSwitch() {
    if (bench_test_active()) return;
    if (WiFi.status() != WL_CONNECTED) return;

    uint8_t p[14];
    agroaog::makeRemoteSwitchPacket(
        gState.autoMode,
        aogRecovery.permitsAuto(gState.autoMode, gState.aogOk),
        modeReset, sectionMask(), SECTION_COUNT, p);

    if (udp.beginPacket(broadcastIp, AOG_TX_PORT) != 1) return;
    if (udp.write(p,sizeof(p)) != sizeof(p)) { udp.endPacket(); return; }
    if (udp.endPacket() != 1) return;
    gState.txPackets++;
    if (modeReset && !resetSent) {
        resetStartMs = millis();
        resetSent = true;
    }
}

static void parse(uint8_t *p, int n) {
    // Reject malformed/truncated PGN239 before updating speed, sections
    // or link freshness. Other PGNs are ignored by this parser.
    if (n < 0 || !agroaog::validMachinePacket(p, static_cast<size_t>(n))) return;

    if (p[2] == 0x7F && p[3] == 0xEF) {
        gState.speedX10 = p[6];
        if (!modeReset && aogRecovery.permitsAuto(gState.autoMode, gState.aogOk)) {
            const uint8_t mask = p[11] & 0x3Fu;
            if (mask != sectionMask()) nozzle_section_change_at(millis());
            for (int i=0;i<SECTION_COUNT;i++) gState.sections[i] = (mask & (1u << i)) != 0;
        }
        aogSession.packetReceived();
        gState.lastAogMs = millis();
        gState.rxPackets++;
    }
}

void aog_update() {
    gState.wifiOk = WiFi.status() == WL_CONNECTED;
    if (!gState.wifiOk) {
        // A new Wi-Fi lease must not reuse a packet from the old UDP session.
        aogSession.reset();
        agroaog::enforceMachineLinkFailSafe(false, gState.autoMode,
                                            gState.speedX10, gState.sections,
                                            SECTION_COUNT);
    }

    // A UDP socket from the old WLAN lease is not a valid connection.
    // Reopen it and recompute the broadcast address after reconnection.
    if (!gState.wifiOk && udpStarted) {
        udp.stop();
        udpStarted = false;
    }
    if (gState.wifiOk && !udpStarted) {
        aogSession.reset();
        updateBroadcast();
        udpStarted = udp.begin(AOG_RX_PORT) == 1;
    }

    gState.udpOk = gState.wifiOk && udpStarted;
    // Check expiry before consuming queued PGN239 packets: a late
    // packet must not hide an expired link and reopen AUTO sections.
    aogRecovery.onLinkStatus(aogSession.fresh(gState.udpOk, millis(),
                                             gState.lastAogMs, AOG_TIMEOUT_MS));
    if (modeReset && !gState.autoMode) modeReset = false;
    if (modeReset && resetSent && (uint32_t)(millis()-resetStartMs)>=MODE_RESET_MS) {
        modeReset = false;
        pendingTx = true;
    }

    if (udpStarted) {
        // Bound UDP work per pass so a packet backlog cannot monopolize
        // the control loop ahead of pressure sampling and valve regulation.
        std::size_t packetsProcessed = 0;
        int sz = udp.parsePacket();
        while (sz > 0 &&
               agroaog::machinePacketBudgetRemaining(packetsProcessed)) {
            uint8_t buf[128];
            int n = udp.read(buf,sizeof(buf));
            if (n > 0) parse(buf,n);
            ++packetsProcessed;
            sz = udp.parsePacket();
        }
    }

    uint32_t now = millis();
    // Never show AOG as connected during the first 1.5 s after boot:
    // require at least one real PGN239 packet and a fresh timestamp.
    gState.aogOk = aogSession.fresh(gState.udpOk, now,
                                    gState.lastAogMs, AOG_TIMEOUT_MS);
    aogRecovery.onLinkStatus(gState.aogOk);
    // A UDP timeout with Wi-Fi still connected must not leave stale AUTO
    // sections open or show an old speed. Invalid PGNs never refresh this link.
    agroaog::enforceMachineLinkFailSafe(gState.aogOk, gState.autoMode,
                                        gState.speedX10, gState.sections,
                                        SECTION_COUNT);

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
