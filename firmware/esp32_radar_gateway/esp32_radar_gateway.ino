/*
 * ESP32 Radar Gateway / Remote firmware.
 *
 * Single codebase.  Pick the role at build time:
 *   arduino-cli compile --build-property "build.extra_flags=-DROLE_GATEWAY" .
 *   arduino-cli compile --build-property "build.extra_flags=-DROLE_REMOTE" .
 *
 *   ROLE_GATEWAY = ESP32 A:  UART1 = left_front, UART2 = left_rear.
 *                   Receives ESP-NOW from ESP32 B, aggregates four
 *                   radars and publishes JSON v2 over USB Serial.
 *
 *   ROLE_REMOTE  = ESP32 B:  UART1 = right_front, UART2 = right_rear.
 *                   Sends both radars to ESP32 A every ~50 ms via
 *                   ESP-NOW unicast.
 *
 * All knobs live in src/hardware_config.h.  Do NOT add GPIO numbers,
 * MAC bytes or baudrates anywhere else.
 *
 * See README.md (firmware/esp32_radar_gateway/README.md) for the full
 * Arduino / PlatformIO build + upload walkthrough and for the JSON
 * contract the Jetson expects.
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "hardware_config.h"
#include "radar_state.h"
#include "ld2410_parser.h"
#include "ld2410_uart_driver.h"
#include "espnow_protocol.h"
#include "gateway_aggregator.h"
#include "jetson_json_transport.h"
#include "diagnostics.h"
#include "mock_mode.h"

/* ------------------------------------------------------------------ */
/* Globals.  Kept small: ESP32 has ~320 KB RAM, and four radars +     */
/* ESP-NOW + JSON line leave ~200 KB for stack + TLS.                 */
/* ------------------------------------------------------------------ */
static Diagnostics       g_diag;
static GatewayAggregator g_agg;
static RadarUartDriver   g_uart_drv;
static uint32_t          g_last_tx_ms = 0u;
static uint32_t          g_last_diag_ms = 0u;
static uint8_t           g_remote_mac[6] = {
    REMOTE_MAC_BYTE_0, REMOTE_MAC_BYTE_1, REMOTE_MAC_BYTE_2,
    REMOTE_MAC_BYTE_3, REMOTE_MAC_BYTE_4, REMOTE_MAC_BYTE_5
};
static uint8_t           g_gateway_mac[6] = {
    GATEWAY_MAC_BYTE_0, GATEWAY_MAC_BYTE_1, GATEWAY_MAC_BYTE_2,
    GATEWAY_MAC_BYTE_3, GATEWAY_MAC_BYTE_4, GATEWAY_MAC_BYTE_5
};
static uint8_t           g_json_line[JETSON_JSON_LINE_MAX];

/* ------------------------------------------------------------------ */
/* Local radar sample callback.  Hands the sample to the aggregator. */
/* ------------------------------------------------------------------ */
static void on_local_sample(uint8_t slot, const RadarSample *sample, void * /*user*/) {
    gateway_aggregator_set_local(&g_agg, slot, sample, millis());
}

/* ------------------------------------------------------------------ */
/* ESP-NOW send callback.  Bumps counters; production builds should   */
/* also log send failures to the diagnostic stream.                   */
/* ------------------------------------------------------------------ */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void on_espnow_sent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    (void)info;
#else
static void on_espnow_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    (void)mac_addr;
#endif
    if (status == ESP_NOW_SEND_SUCCESS) {
        g_diag.espnow_send_ok += 1u;
    } else {
        g_diag.espnow_send_fail += 1u;
    }
}

/* ------------------------------------------------------------------ */
/* ESP-NOW receive callback (Gateway only).                           */
/* ------------------------------------------------------------------ */
static void on_espnow_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (info == nullptr || data == nullptr) return;
    /* Source MAC must match the Remote MAC.  This protects against
     * stray packets from any other ESP-NOW peer. */
    if (memcmp(info->src_addr, g_remote_mac, 6) != 0) {
        g_diag.espnow_recv_drop += 1u;
        return;
    }
    if ((uint32_t)len != ESPNOW_PAYLOAD_SIZE) {
        g_diag.espnow_recv_bad += 1u;
        return;
    }
    EspNowPacket pkt;
    int rc = espnow_decode(data, (uint32_t)len, &pkt);
    if (rc != 0) {
        g_diag.espnow_recv_bad += 1u;
        return;
    }
    /* M2 fix: accept only ESPNOW_NODE_REMOTE (ESP32 B) packets.
     * The protocol has no provision for a gateway-to-gateway packet;
     * any other node_id must be treated as noise. */
    if (pkt.node_id != ESPNOW_NODE_REMOTE) {
        g_diag.espnow_recv_drop += 1u;
        return;
    }
    if (!gateway_aggregator_accept_remote(&g_agg, &pkt, millis())) {
        /* Dropped (duplicate / older).  Don't bump recv_bad; the
         * aggregator handled it internally. */
        return;
    }
    g_diag.espnow_recv_ok += 1u;
}

/* ------------------------------------------------------------------ */
/* Common setup: diagnostics + aggregator + mock or UART begin.      */
/* ------------------------------------------------------------------ */
static void common_setup(void) {
    diagnostics_init(&g_diag);
    /* Boot ID: low 32 bits from the factory MAC so two reboots on
     * the same ESP32 produce the same boot_id (great for tests); but
     * a power cycle retains the same MAC.  We mix in millis() so a
     * genuine reboot gets a new boot_id too.
     * ESP32 Arduino core 3.x renamed getEfuseMAC() -> getEfuseMac();
     * keep both spellings working. */
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    uint32_t boot_id = (uint32_t)ESP.getEfuseMac() ^ millis();
#else
    uint32_t boot_id = (uint32_t)ESP.getEfuseMAC() ^ millis();
#endif
    gateway_aggregator_init(&g_agg, boot_id);
    radar_uart_driver_init(
        &g_uart_drv,
        RADAR_PRIMARY_NAME,
        RADAR_SECONDARY_NAME,
        on_local_sample,
        nullptr,
        &g_diag
    );
    radar_uart_driver_begin(&g_uart_drv);
}

/* ------------------------------------------------------------------ */
/* ROLE_REMOTE setup + loop.                                          */
/* ------------------------------------------------------------------ */
#ifdef ROLE_REMOTE

static uint8_t g_tx_buf[ESPNOW_PAYLOAD_SIZE];
static uint32_t g_remote_sequence = 0u;

static void remote_setup(void) {
    Serial.begin(USB_SERIAL_BAUD);
    delay(100);
    common_setup();
    /* Wi-Fi must be up before ESP-NOW. */
    WiFi.mode(WIFI_STA);
    WiFi.channel(ESPNOW_CHANNEL);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) {
        Serial.println(F("[ERR] esp_now_init failed"));
        return;
    }
    esp_now_register_send_cb(on_espnow_sent);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, g_gateway_mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println(F("[ERR] esp_now_add_peer failed"));
    }
    Serial.println(F("[OK] ROLE_REMOTE ready"));
}

static void remote_loop(void) {
    uint32_t now = millis();
    radar_uart_driver_poll(&g_uart_drv, now);
    if (now - g_last_tx_ms < TX_INTERVAL_MS) return;
    g_last_tx_ms = now;
    RadarSample snap[2] = {g_agg.remote[0], g_agg.remote[1]};
    /* The aggregator keeps the latest two Remote radars in remote[];
     * for the Remote side we directly use the two local radars. */
    RadarSample local[2];
    local[0] = g_agg.local[0];
    local[1] = g_agg.local[1];
    uint32_t written = espnow_encode(
        ESPNOW_NODE_REMOTE,
        g_agg.gateway_boot_id,
        ++g_remote_sequence,
        now,
        local,
        g_tx_buf,
        sizeof(g_tx_buf)
    );
    (void)snap;
    if (written == 0u) return;
    esp_now_send(g_gateway_mac, g_tx_buf, written);
}

#endif  /* ROLE_REMOTE */

/* ------------------------------------------------------------------ */
/* ROLE_GATEWAY setup + loop.                                         */
/* ------------------------------------------------------------------ */
#ifdef ROLE_GATEWAY

static void gateway_setup(void) {
    Serial.begin(USB_SERIAL_BAUD);
    delay(100);
    common_setup();
    WiFi.mode(WIFI_STA);
    WiFi.channel(ESPNOW_CHANNEL);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) {
        Serial.println(F("[ERR] esp_now_init failed"));
        return;
    }
    esp_now_register_send_cb(on_espnow_sent);
    esp_now_register_recv_cb(on_espnow_recv);
    /* We don't need to register the Remote as a peer for unicast
     * receive; ESP-NOW accepts callbacks for any MAC by default. */
    Serial.println(F("[OK] ROLE_GATEWAY ready"));
}

static void gateway_loop(void) {
    uint32_t now = millis();
    radar_uart_driver_poll(&g_uart_drv, now);
    if (now - g_last_tx_ms < TX_INTERVAL_MS) return;
    g_last_tx_ms = now;
    RadarSnapshot snap;
    gateway_aggregator_build_snapshot(
        &g_agg, now,
        RADAR_FRAME_TIMEOUT_MS, REMOTE_PACKET_TIMEOUT_MS,
        &snap
    );
    uint32_t n = jetson_json_encode_v2(&snap, g_json_line, sizeof(g_json_line));
    if (n == 0u) {
        g_diag.json_lines_dropped += 1u;
        return;
    }
    Serial.write(g_json_line, n);
    g_diag.json_lines_sent += 1u;
    if (now - g_last_diag_ms > 1000u) {
        g_last_diag_ms = now;
        char dbg[160];
        uint32_t dn = diagnostics_render(&g_diag, dbg, sizeof(dbg));
        if (dn > 0u) {
            /* IMPORTANT: route diagnostics over USB Serial (UART0).
             * Serial1 and Serial2 are the LD2410 radar UARTs on this
             * board; never use them for debug logs or we will
             * corrupt the radar stream.  Guard with DEBUG_LOG so
             * production builds drop the diagnostic string entirely
             * and keep the JSON line uninterrupted. */
#ifdef DEBUG_LOG
            Serial.print(F("[DBG] "));
            Serial.write((const uint8_t *)dbg, dn);
            Serial.println();
#endif
        }
    }
}

#endif  /* ROLE_GATEWAY */

void setup() {
#ifdef ROLE_GATEWAY
    gateway_setup();
#else
    remote_setup();
#endif
}

void loop() {
#ifdef ROLE_GATEWAY
    gateway_loop();
#else
    remote_loop();
#endif
}
