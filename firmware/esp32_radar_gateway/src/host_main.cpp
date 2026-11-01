/* host_main.cpp
 *
 * End-to-end smoke test for the pure C++ firmware modules.
 *
 * Builds with the host g++ script (tools/firmware_host_build.py) and
 * exercises the LD2410 mock frame builder, the ESP-NOW codec, the
 * Jetson JSON transport and the gateway aggregator against a small
 * synthetic scenario.  Mirrors the Python host tests; both must
 * agree byte-for-byte so any regression is caught in CI.
 *
 * NOT compiled by the Arduino build (the .ino file uses Serial/USB
 * instead of stdout).
 */
#include <cstdio>
#include <cstring>
#include <cassert>

#include "ld2410_parser.h"
#include "espnow_protocol.h"
#include "jetson_json_transport.h"
#include "gateway_aggregator.h"
#include "diagnostics.h"
#include "mock_mode.h"

static void step(const char *name) {
    std::printf("[STEP] %s\n", name);
}

int main() {
    /* ---- mock mode ---------------------------------------------------- */
    step("mock_radar_next");
    MockRadar mock;
    mock_radar_init(&mock, 0xA1B2C3D4u);
    RadarSample ms0, ms1;
    mock_radar_next(&mock, "left_front", 0, 1000, &ms0);
    mock_radar_next(&mock, "left_rear",  1, 1000, &ms1);
    assert(ms0.valid == 1u && ms1.valid == 1u);
    assert(ms0.detected == 1u);
    std::printf("  mock: dist=%u energy=%u\n",
                ms0.detection_distance_cm, ms0.moving_energy);

    /* ---- ld2410 mock frame round-trip -------------------------------- */
    step("ld2410 mock frame round-trip");
    uint8_t frame[LD2410_MAX_FRAME_LEN];
    uint16_t frame_len = ld2410_parser_make_mock_frame(
        TARGET_STATE_MOVING, 250, LD2410_DISTANCE_NONE,
        42, LD2410_ENERGY_NONE, frame, sizeof(frame));
    assert(frame_len > 0u);
    Ld2410Parser parser;
    ld2410_parser_init(&parser);
    RadarSample parsed;
    int surfaced = 0;
    for (uint16_t i = 0; i < frame_len; ++i) {
        if (ld2410_parser_feed(&parser, frame[i], "left_front", 1234, &parsed)) {
            surfaced++;
        }
    }
    assert(surfaced == 1);
    assert(parsed.detection_distance_cm == 250);
    assert(parsed.radar_id[0] == 'l');
    std::printf("  parsed dist=%u radar_id=%s\n",
                parsed.detection_distance_cm, parsed.radar_id);

    /* ---- espnow encode/decode round-trip ---------------------------- */
    step("espnow encode/decode");
    RadarSample pair[2] = {parsed, parsed};
    strncpy(pair[1].radar_id, "left_rear", sizeof(pair[1].radar_id) - 1);
    uint8_t tx[ESPNOW_PAYLOAD_SIZE];
    uint32_t written = espnow_encode(
        ESPNOW_NODE_REMOTE, 0xDEADBEEFu, 42u, 12345u,
        pair, tx, sizeof(tx));
    assert(written == ESPNOW_PAYLOAD_SIZE);
    EspNowPacket rx;
    int rc = espnow_decode(tx, written, &rx);
    assert(rc == 0);
    assert(rx.boot_id == 0xDEADBEEFu);
    assert(rx.sequence == 42u);
    assert(rx.timestamp_ms == 12345u);
    std::printf("  decoded: boot=%08x seq=%u radars=%u\n",
                rx.boot_id, rx.sequence, rx.radar_count);

    /* ---- aggregator stale detection ---------------------------------- */
    step("aggregator stale detection");
    GatewayAggregator agg;
    gateway_aggregator_init(&agg, 0xCAFEBABEu);
    gateway_aggregator_set_local(&agg, 0, &parsed, 1000);
    gateway_aggregator_set_local(&agg, 1, &parsed, 1000);
    gateway_aggregator_accept_remote(&agg, &rx, 1000);
    RadarSnapshot snap;
    gateway_aggregator_build_snapshot(&agg, 1000, 300, 600, &snap);
    assert(snap.gateway_ok == 1u);
    assert(snap.remote_ok == 1u);
    /* Now jump far into the future and rebuild: local radars should be
     * stale, remote pair should be stale, remote_ok should drop. */
    gateway_aggregator_build_snapshot(&agg, 5000, 300, 600, &snap);
    assert(snap.remote_ok == 0u);
    for (int i = 0; i < ADAS_MAX_RADARS; ++i) {
        assert(snap.radars[i].stale == 1u);
    }
    std::printf("  snapshot sequence=%u remote_ok=%u stale_count=4\n",
                snap.sequence, snap.remote_ok);

    /* ---- jetson json v2 line ----------------------------------------- */
    step("jetson json v2 line");
    uint8_t json[JETSON_JSON_LINE_MAX];
    uint32_t json_len = jetson_json_encode_v2(&snap, json, sizeof(json));
    /* The encoder returns 0 on overflow.  A 4-radar line with the
     * default fields fits well under the 1024-byte cap. */
    assert(json_len > 0u);
    std::printf("  json_len=%u line=%.*s",
                json_len, (int)json_len, (const char *)json);

    /* Negative case: a 64-byte buffer must trigger overflow and yield 0. */
    uint8_t tiny[64];
    uint32_t tiny_len = jetson_json_encode_v2(&snap, tiny, sizeof(tiny));
    assert(tiny_len == 0u);
    std::printf("  overflow path: tiny_len=%u (expected 0)\n", tiny_len);

    /* ---- diagnostics render ----------------------------------------- */
    step("diagnostics render");
    Diagnostics d;
    diagnostics_init(&d);
    d.espnow_send_ok = 100;
    d.espnow_send_fail = 2;
    char dbg[160];
    uint32_t dbg_len = diagnostics_render(&d, dbg, sizeof(dbg));
    std::printf("  diag(%u): %s\n", dbg_len, dbg);

    std::printf("[OK] all host smoke checks passed\n");
    return 0;
}