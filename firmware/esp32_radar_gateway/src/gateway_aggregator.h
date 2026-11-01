/*
 * gateway_aggregator.h
 *
 * State machine that turns four RadarSample objects (two local UARTs
 * on the Gateway, two from the Remote over ESP-NOW) into a single
 * RadarSnapshot that the JSON encoder then serialises.  This module is
 * pure C++ (no Arduino.h) so it can be unit-tested on a host.
 *
 * Responsibilities:
 *   - Apply RADAR_FRAME_TIMEOUT_MS to local radars (sets stale=true
 *     after the deadline).
 *   - Apply REMOTE_PACKET_TIMEOUT_MS to the Remote pair (sets stale on
 *     both right_* radars after the deadline and bumps packet_loss).
 *   - Detect boot_id changes (reboot of the Remote) and reset the
 *     sequence tracker so new packets are accepted again.
 *   - Drop duplicate / older sequences per boot_id; count the gaps as
 *     packet_loss.
 *   - Mark gateway_ok=false if the local aggregator itself never saw
 *     a frame (used by the Jetson to refuse Live mode).
 *   - Bump `sequence` (the per-Gateway monotonic counter) once per
 *     snapshot published to USB Serial.
 */
#ifndef ADAS_GATEWAY_AGGREGATOR_H
#define ADAS_GATEWAY_AGGREGATOR_H

#include <stdint.h>

#include "radar_state.h"
#include "espnow_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    RadarSample local[2];            /* left_front, left_rear            */
    RadarSample remote[2];           /* right_front, right_rear          */
    uint32_t    local_last_update_ms;
    uint32_t    remote_last_packet_ms;
    uint32_t    remote_boot_id;
    uint32_t    remote_last_sequence;
    uint8_t     remote_have_state;
    uint32_t    packet_loss;
    uint32_t    gateway_boot_id;
    uint32_t    sequence;
    uint8_t     gateway_ok;          /* set after the first local frame  */
} GatewayAggregator;

void gateway_aggregator_init(
    GatewayAggregator *agg,
    uint32_t gateway_boot_id
);

/* Ingest one ESP-NOW packet from the Remote.  Returns 1 if the packet
 * was accepted (boot_id changed OR sequence advanced), 0 if dropped. */
uint8_t gateway_aggregator_accept_remote(
    GatewayAggregator *agg,
    const EspNowPacket *pkt,
    uint32_t now_ms
);

/* Refresh a local radar sample.  `slot` is 0 (left_front) or 1
 * (left_rear). */
void gateway_aggregator_set_local(
    GatewayAggregator *agg,
    uint8_t slot,
    const RadarSample *sample,
    uint32_t now_ms
);

/* Build the RadarSnapshot that will be encoded to JSON.  Sets stale on
 * any radar whose last_update_ms is older than the corresponding
 * timeout relative to `now_ms`. */
void gateway_aggregator_build_snapshot(
    GatewayAggregator *agg,
    uint32_t now_ms,
    uint32_t radar_frame_timeout_ms,
    uint32_t remote_packet_timeout_ms,
    RadarSnapshot *out
);

#ifdef __cplusplus
}
#endif

#endif  /* ADAS_GATEWAY_AGGREGATOR_H */
