/*
 * jetson_json_transport.cpp
 *
 * Streaming JSON v2 encoder.  Append-only, fixed buffer, no malloc.
 *
 * Implementation note: we don't pull in any 3rd-party JSON library so
 * the encoder fits comfortably in the Gateway's flash budget and the
 * same source compiles unmodified on a host test harness.  The Python
 * mirror in tests/test_jetson_json_transport.py follows the exact same
 * rules, byte for byte.
 */

#include "jetson_json_transport.h"

#include <string.h>

static void append_bytes(JsonWriter *w, const char *p, uint32_t n) {
    if (w == nullptr || w->overflow) return;
    if (w->len + n > w->cap) {
        w->overflow = 1u;
        return;
    }
    memcpy(w->buf + w->len, p, n);
    w->len += n;
}

static void append_byte(JsonWriter *w, char c) {
    append_bytes(w, &c, 1u);
}

void json_writer_init(JsonWriter *w, uint8_t *buf, uint32_t cap) {
    if (w == nullptr) return;
    w->buf = buf;
    w->cap = cap;
    w->len = 0u;
    w->overflow = 0u;
}

void json_writer_append(JsonWriter *w, const char *literal) {
    if (literal == nullptr) return;
    append_bytes(w, literal, (uint32_t)strlen(literal));
}

void json_writer_append_u32(JsonWriter *w, uint32_t value) {
    /* Up to 10 digits + NUL. */
    char tmp[12];
    char *p = tmp + sizeof(tmp) - 1;
    *p = 0;
    if (value == 0u) {
        *(--p) = '0';
    } else {
        while (value > 0u && p > tmp) {
            *(--p) = (char)('0' + (value % 10u));
            value /= 10u;
        }
    }
    append_bytes(w, p, (uint32_t)(tmp + sizeof(tmp) - 1 - p));
}

void json_writer_append_i32(JsonWriter *w, int32_t value) {
    char tmp[12];
    char *p = tmp + sizeof(tmp) - 1;
    *p = 0;
    uint32_t abs_val;
    if (value < 0) {
        abs_val = (uint32_t)(-value);
    } else {
        abs_val = (uint32_t)value;
    }
    if (abs_val == 0u) {
        *(--p) = '0';
    } else {
        while (abs_val > 0u && p > tmp) {
            *(--p) = (char)('0' + (abs_val % 10u));
            abs_val /= 10u;
        }
    }
    if (value < 0) *(--p) = '-';
    append_bytes(w, p, (uint32_t)(tmp + sizeof(tmp) - 1 - p));
}

/* Emit either `value` (cm) divided by 100.0 with one decimal, or `null`.
 * Always emits one decimal so the consumer can parse as float. */
static void append_distance_m_or_null(JsonWriter *w, uint16_t cm, uint8_t valid) {
    if (!valid || cm == LD2410_DISTANCE_NONE) {
        append_bytes(w, "null", 4u);
        return;
    }
    /* Render as "<a>.<b>" metres. No leading zeros: json.loads rejects them. */
    uint16_t whole = (uint16_t)(cm / 100u);
    uint16_t frac  = (uint16_t)(cm % 100u);
    char tmp[8];
    uint32_t n = 0u;
    if (whole >= 10u) {
        tmp[n++] = (char)('0' + (whole / 10u) % 10u);
    }
    tmp[n++] = (char)('0' + (whole % 10u));
    tmp[n++] = '.';
    tmp[n++] = (char)('0' + (frac / 10u) % 10u);
    tmp[n++] = (char)('0' + (frac % 10u));
    append_bytes(w, tmp, n);
}

static void append_uint_or_null(JsonWriter *w, uint8_t valid, uint8_t value) {
    if (!valid || value == LD2410_ENERGY_NONE) {
        append_bytes(w, "null", 4u);
        return;
    }
    char tmp[4];
    uint32_t n = 0u;
    if (value >= 100u) {
        tmp[n++] = (char)('0' + (value / 100u) % 10u);
    }
    if (value >= 10u) {
        tmp[n++] = (char)('0' + (value / 10u) % 10u);
    }
    tmp[n++] = (char)('0' + (value % 10u));
    append_bytes(w, tmp, n);
}

void json_writer_append_radar(JsonWriter *w, const RadarSample *s) {
    /* Open. */
    append_byte(w, '{');
    append_bytes(w, "\"detected\":", 11u);
    append_bytes(w, (s->detected ? "true" : "false"), (s->detected ? 4u : 5u));
    append_byte(w, ',');

    append_bytes(w, "\"target_state\":", 15u);
    append_byte(w, '"');
    switch (s->target_state) {
        case TARGET_STATE_MOVING:
            append_bytes(w, "moving", 6u); break;
        case TARGET_STATE_STATIONARY:
            append_bytes(w, "stationary", 10u); break;
        case TARGET_STATE_BOTH:
            append_bytes(w, "both", 4u); break;
        default:
            append_bytes(w, "none", 4u); break;
    }
    append_byte(w, '"');
    append_byte(w, ',');

    /* Distances.  We trust `valid` only -- a stale sample is still
     * emitted but its distance fields go to null. */
    uint8_t dist_valid = s->valid ? 1u : 0u;
    append_bytes(w, "\"distance_m\":", 13u);
    append_distance_m_or_null(w, s->detection_distance_cm, dist_valid);
    append_byte(w, ',');
    append_bytes(w, "\"moving_distance_m\":", 21u);
    append_distance_m_or_null(w, s->moving_distance_cm, dist_valid);
    append_byte(w, ',');
    append_bytes(w, "\"stationary_distance_m\":", 24u);
    append_distance_m_or_null(w, s->stationary_distance_cm, dist_valid);
    append_byte(w, ',');

    append_bytes(w, "\"moving_energy\":", 16u);
    append_uint_or_null(w, dist_valid, s->moving_energy);
    append_byte(w, ',');
    append_bytes(w, "\"stationary_energy\":", 20u);
    append_uint_or_null(w, dist_valid, s->stationary_energy);
    append_byte(w, ',');

    append_bytes(w, "\"valid\":", 8u);
    append_bytes(w, (s->valid ? "true" : "false"), (s->valid ? 4u : 5u));
    append_byte(w, ',');
    /* An invalid sample is implicitly stale too: the Jetson must drop
     * its distance fields regardless of timing. */
    uint8_t stale = (s->stale || !s->valid) ? 1u : 0u;
    append_bytes(w, "\"stale\":", 8u);
    append_bytes(w, (stale ? "true" : "false"), (stale ? 4u : 5u));
    append_byte(w, ',');

    append_bytes(w, "\"age_ms\":", 9u);
    json_writer_append_u32(w, s->sample_age_ms);
    append_byte(w, ',');

    append_bytes(w, "\"error\":", 8u);
    json_writer_append_u32(w, s->error_code);
    append_byte(w, '}');
}

uint32_t jetson_json_encode_v2(
    const RadarSnapshot *snap,
    uint8_t *buf,
    uint32_t buf_len
) {
    if (snap == nullptr || buf == nullptr || buf_len < 4u) return 0u;

    JsonWriter w;
    json_writer_init(&w, buf, buf_len);

    json_writer_append(&w, "{\"v\":2,\"seq\":");
    json_writer_append_u32(&w, snap->sequence);
    json_writer_append(&w, ",\"gateway_ok\":");
    json_writer_append_u32(&w, snap->gateway_ok);
    json_writer_append(&w, ",\"remote_ok\":");
    json_writer_append_u32(&w, snap->remote_ok);
    json_writer_append(&w, ",\"gateway_boot_id\":");
    json_writer_append_u32(&w, snap->gateway_boot_id);
    json_writer_append(&w, ",\"remote_boot_id\":");
    json_writer_append_u32(&w, snap->remote_boot_id);
    json_writer_append(&w, ",\"packet_loss\":");
    json_writer_append_u32(&w, snap->packet_loss);
    json_writer_append(&w, ",\"radars\":{");

    /* Always emit all four radars in the order declared by the
     * protocol.  Missing radars get valid=false, stale=true, all
     * distance fields null. */
    static const char *order[ADAS_MAX_RADARS] = {
        "left_front", "left_rear", "right_front", "right_rear"
    };
    for (uint32_t i = 0; i < ADAS_MAX_RADARS; ++i) {
        if (i > 0u) json_writer_append(&w, ",");
        json_writer_append(&w, "\"");
        json_writer_append(&w, order[i]);
        json_writer_append(&w, "\":");
        json_writer_append_radar(&w, &snap->radars[i]);
    }
    json_writer_append(&w, "}}\n");

    if (w.overflow) {
        return 0u;
    }
    return w.len;
}
