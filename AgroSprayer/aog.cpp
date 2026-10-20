#include "aog.h"
#include "config.h"
#include "board_config.h"
#include "sprayer_state.h"
#include "aog_link_guard.h"
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP udp;
static agroaog::Session aogSession;
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

static uint8_t checksum(const uint8_t *p, int a, int b) {
    uint16_t s=0;
    for (int i=a;i<=b;i++) s += p[i];
    return (uint8_t)s;
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
    modeReset = gState.autoMode;
    resetSent = false;
    pendingTx = true;
}

static void sendSwitch() {
    if (bench_test_active()) return;
    if (WiFi.status() != WL_CONNECTED) return;

    uint8_t valid = (1u << SECTION_COUNT) - 1u;
    const bool automatic = gState.autoMode && !modeReset;
    uint8_t autoMask = automatic ? valid : 0;
    uint8_t onMask = modeReset || automatic ? 0 : sectionMask();
    uint8_t offMask = modeReset ? valid : automatic ? 0 : ((~onMask) & valid);
    // PGN234 Main bit2 selects maintained switch states (not edge buttons).
    // bit0 enables the AUTO master; bit1 disables it for manual control.
    const uint8_t main = 0x04 | (automatic ? 0x01 : 0x02);

    uint8_t p[14] = {
        0x80,0x81,0x77,0xEA,0x08,
        main, autoMask,0x00, SECTION_COUNT,
        onMask,offMask,0x00,0x00,0x00
    };
    p[13] = checksum(p,2,12);

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
    if (n < 12 || p[0] != 0x80 || p[1] != 0x81) return;

    if (p[2] == 0x7F && p[3] == 0xEF) {
        gState.speedX10 = p[6];
        if (gState.autoMode && !modeReset) {
            uint8_t mask = p[11];
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
        gState.speedX10 = 0;
        if (gState.autoMode) {
            for (int i = 0; i < SECTION_COUNT; ++i) gState.sections[i] = false;
        }
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
    // Never show AOG as connected during the first 1.5 s after boot:
    // require at least one real PGN239 packet and a fresh timestamp.
    gState.aogOk = aogSession.fresh(gState.udpOk, now,
                                    gState.lastAogMs, AOG_TIMEOUT_MS);

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
