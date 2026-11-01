/*
 * gateway_aggregator.cpp
 */

#include "gateway_aggregator.h"

#include <string.h>

void gateway_aggregator_init(GatewayAggregator *agg, uint32_t gateway_boot_id) {
    if (agg == nullptr) return;
    memset(agg, 0, sizeof(*agg));
    radar_sample_reset(&agg->local[0]);
    radar_sample_reset(&agg->local[1]);
    radar_sample_reset(&agg->remote[0]);
    radar_sample_reset(&agg->remote[1]);
    /* Seed radar_id strings so the JSON encoder always has a label. */
    static const char left_front[]  = "left_front";
    static const char left_rear[]   = "left_rear";
    static const char right_front[] = "right_front";
    static const char right_rear[]  = "right_rear";
    memcpy(agg->local[0].radar_id,  left_front,  sizeof(left_front));
    memcpy(agg->local[1].radar_id,  left_rear,   sizeof(left_rear));
    memcpy(agg->remote[0].radar_id, right_front, sizeof(right_front));
    memcpy(agg->remote[1].radar_id, right_rear,  sizeof(right_rear));
    agg->local_last_update_ms  = 0u;
    agg->remote_last_packet_ms = 0u;
    agg->remote_boot_id        = 0u;
    agg->remote_last_sequence  = 0u;
    agg->remote_have_state     = 0u;
    agg->packet_loss           = 0u;
    agg->gateway_boot_id       = gateway_boot_id;
    agg->sequence              = 0u;
    agg->gateway_ok            = 0u;
}

uint8_t gateway_aggregator_accept_remote(
    GatewayAggregator *agg,
    const EspNowPacket *pkt,
    uint32_t now_ms
) {
    if (agg == nullptr || pkt == nullptr) return 0u;

    /* Reboot detection: a different boot_id means the Remote just
     * rebooted.  Accept the new packet and reset the sequence
     * tracker.  Always treat the very first packet we see as a
     * reboot too (remote_have_state==0). */
    uint8_t accepted = 0u;
    if (!agg->remote_have_state || pkt->boot_id != agg->remote_boot_id) {
        agg->remote_boot_id       = pkt->boot_id;
        agg->remote_last_sequence = pkt->sequence;
        agg->remote_have_state    = 1u;
        accepted = 1u;
    } else if (pkt->sequence == agg->remote_last_sequence) {
        /* Duplicate.  Drop silently. */
        return 0u;
    } else if (pkt->sequence > agg->remote_last_sequence) {
        /* Sequence advanced; count any gap as packet loss.  Wrapping
         * is handled by the inequality: if the Remote wraps to 0, the
         * next packet is 1 which is < current sequence, so we treat
         * it as a reboot (boot_id change required for that). */
        uint32_t gap = pkt->sequence - agg->remote_last_sequence;
        if (gap > 1u) {
            agg->packet_loss += (gap - 1u);
        }
        agg->remote_last_sequence = pkt->sequence;
        accepted = 1u;
    } else {
        /* Older sequence under the same boot_id.  Drop. */
        return 0u;
    }

    /* Copy radar payloads into our state. */
    espnow_unpack_radar(&pkt->radars[0], agg->remote[0].radar_id, now_ms,
                        &agg->remote[0]);
    espnow_unpack_radar(&pkt->radars[1], agg->remote[1].radar_id, now_ms,
                        &agg->remote[1]);
    agg->remote_last_packet_ms = now_ms;
    return accepted ? 1u : 0u;
}

void gateway_aggregator_set_local(
    GatewayAggregator *agg,
    uint8_t slot,
    const RadarSample *sample,
    uint32_t now_ms
) {
    if (agg == nullptr || sample == nullptr) return;
    if (slot > 1u) return;
    agg->local[slot] = *sample;
    agg->local_last_update_ms = now_ms;
    if (!agg->gateway_ok) agg->gateway_ok = 1u;
}

void gateway_aggregator_build_snapshot(
    GatewayAggregator *agg,
    uint32_t now_ms,
    uint32_t radar_frame_timeout_ms,
    uint32_t remote_packet_timeout_ms,
    RadarSnapshot *out
) {
    if (agg == nullptr || out == nullptr) return;
    radar_snapshot_reset(out);

    /* Local radars. */
    for (uint8_t i = 0; i < 2u; ++i) {
        RadarSample s = agg->local[i];
        uint32_t age = (agg->local_last_update_ms == 0u)
                           ? 0xFFFFFFFFu
                           : now_ms - agg->local_last_update_ms;
        if (agg->local_last_update_ms == 0u || age > radar_frame_timeout_ms) {
            s.valid = 0u;
            s.stale = 1u;
            s.error_code = RADAR_ERROR_NO_FRAME;
            s.sample_age_ms = 0xFFFFu;
        }
        out->radars[i] = s;
    }

    /* Remote radars. */
    uint32_t remote_age = (agg->remote_last_packet_ms == 0u)
                              ? 0xFFFFFFFFu
                              : now_ms - agg->remote_last_packet_ms;
    uint8_t remote_stale =
        (agg->remote_last_packet_ms == 0u ||
         remote_age > remote_packet_timeout_ms) ? 1u : 0u;
    for (uint8_t i = 0; i < 2u; ++i) {
        RadarSample s = agg->remote[i];
        if (remote_stale) {
            s.valid = 0u;
            s.stale = 1u;
            s.error_code = RADAR_ERROR_NO_FRAME;
            s.sample_age_ms = 0xFFFFu;
        }
        out->radars[2u + i] = s;
    }

    out->sequence        = agg->sequence;
    out->gateway_ok      = agg->gateway_ok;
    out->remote_ok       = remote_stale ? 0u : 1u;
    out->gateway_boot_id = agg->gateway_boot_id;
    out->remote_boot_id  = agg->remote_boot_id;
    out->packet_loss     = agg->packet_loss;

    /* Per-snapshot sequence is bumped on each call so the Jetson can
     * detect dropped JSON lines (it sees the same sequence as last
     * time, then a jump). */
    agg->sequence += 1u;
}
