/*
 * ld2410_parser.cpp
 *
 * Implementation of the LD2410 frame parser.  No Arduino dependency;
 * compiles with plain g++ on a host for unit testing.  See
 * tests/test_ld2410_parser.py for the Python mirror used by the test
 * suite.
 *
 * Stream behaviour:
 *   - We accumulate bytes until we have read the 4-byte header.  Any
 *     garbage before the header is silently dropped (the parser
 *     recovers sync).
 *   - After the header comes a 2-byte little-endian length field.
 *     The full frame size is `len_field + 2 + 1` (length bytes +
 *     checksum byte) wrapped with header+footer = `len_field + 9`.
 *     We use that as the expected length and refuse to buffer more
 *     than LD2410_MAX_FRAME_LEN bytes.
 *   - Once we have the expected length, we wait for the remaining
 *     bytes, then verify the footer and the LD2410 internal checksum
 *     (last byte == 8-bit sum of the bytes from the first length byte
 *     through the last footer byte).
 *   - A failed verification clears `len`, sets `sync_lost`, and the
 *     next call resumes scanning for a new header.
 */

#include "ld2410_parser.h"

#include <string.h>

void ld2410_parser_init(Ld2410Parser *p) {
    if (p == nullptr) return;
    memset(p, 0, sizeof(*p));
    p->sync_lost = 1u;  /* start in resync state until first header    */
}

static uint8_t ld2410_checksum(const uint8_t *data, uint16_t n) {
    uint16_t sum = 0u;
    for (uint16_t i = 0; i < n; ++i) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFFu);
}

/* Try to consume bytes from the head of `buf[0..len]` as a complete
 * LD2410 frame.  On success returns the number of bytes consumed and
 * fills `out`.  On failure returns 0 and shifts the remaining bytes
 * down so we can keep scanning. */
static uint16_t try_parse_frame(
    const uint8_t *buf,
    uint16_t len,
    const char *radar_id,
    uint32_t now_ms,
    RadarSample *out
) {
    /* Minimum frame: 4 header + 2 length + 1 type + 2 detection +
     * 4 footer + 1 checksum = 14 bytes. */
    if (len < 14u) return 0u;

    if (buf[0] != LD2410_HEADER_0 || buf[1] != LD2410_HEADER_1 ||
        buf[2] != LD2410_HEADER_2 || buf[3] != LD2410_HEADER_3) {
        return 0u;
    }
    /* Footer. */
    if (buf[len - 5] != LD2410_FOOTER_0 ||
        buf[len - 4] != LD2410_FOOTER_1 ||
        buf[len - 3] != LD2410_FOOTER_2 ||
        buf[len - 2] != LD2410_FOOTER_3) {
        return 0u;
    }
    uint16_t length_field = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    /* Total frame size = 11 + length_field
     *   = header(4) + length(2) + payload(length_field) + footer(4) + csum(1). */
    uint16_t expected_total = (uint16_t)(11u + length_field);
    if (expected_total != len) return 0u;

    /* Internal checksum.  Covers length(2) + payload + footer. */
    uint8_t expected = ld2410_checksum(buf + 4u, (uint16_t)(length_field + 2u + 4u));
    if (expected != buf[len - 1u]) return 0u;

    /* Populate sample. */
    radar_sample_reset(out);
    /* radar_id copy, NUL terminated, max 15 chars. */
    size_t idlen = 0u;
    while (radar_id != nullptr && radar_id[idlen] != 0 && idlen < 15u) {
        out->radar_id[idlen] = radar_id[idlen];
        ++idlen;
    }
    out->radar_id[idlen] = 0;

    out->target_state = buf[LD2410_OFFSET_TARGET_STATE] & 0x03u;
    out->detection_distance_cm =
        (uint16_t)buf[LD2410_OFFSET_DETECTION_DIST_LO] |
        ((uint16_t)buf[LD2410_OFFSET_DETECTION_DIST_HI] << 8);
    if (out->detection_distance_cm == LD2410_DISTANCE_NONE) {
        out->target_state = TARGET_STATE_NONE;
    }

#if LD2410_PARSE_FULL_GATES
    /* Full-gate parsing requires verified offsets; the user must opt in
     * after cross-checking the datasheet.  These offsets are placeholders
     * derived from the LD2410 engineering report layout and are only
     * compiled when the flag is set. */
    out->moving_distance_cm =
        (uint16_t)buf[12] | ((uint16_t)buf[13] << 8);
    out->moving_energy = buf[14];
    out->stationary_distance_cm =
        (uint16_t)buf[15] | ((uint16_t)buf[16] << 8);
    out->stationary_energy = buf[17];
    /* Sanity: if a gate says no target, force its distance to the
     * sentinel so the JSON encoder emits null instead of 0. */
    if ((out->target_state & TARGET_STATE_MOVING) == 0u) {
        out->moving_distance_cm = LD2410_DISTANCE_NONE;
        out->moving_energy      = LD2410_ENERGY_NONE;
    }
    if ((out->target_state & TARGET_STATE_STATIONARY) == 0u) {
        out->stationary_distance_cm = LD2410_DISTANCE_NONE;
        out->stationary_energy      = LD2410_ENERGY_NONE;
    }
#endif

    out->detected = (out->target_state != TARGET_STATE_NONE) ? 1u : 0u;
    out->valid    = 1u;
    out->valid    = 1u;
    out->stale    = 0u;
    out->error_code = RADAR_ERROR_NONE;
    out->last_update_ms = now_ms;
    return len;
}

uint8_t ld2410_parser_feed(
    Ld2410Parser *p,
    uint8_t byte_in,
    const char *radar_id,
    uint32_t now_ms,
    RadarSample *out
) {
    if (p == nullptr || out == nullptr) return 0u;

    /* Drop overflow but keep trying to resync. */
    if (p->len >= LD2410_MAX_FRAME_LEN) {
        p->len = 0u;
        p->sync_lost = 1u;
    }

    p->buf[p->len++] = byte_in;

    /* Resync: scan forward for a header. */
    if (p->sync_lost || p->expected_len == 0u) {
        /* Look for F4 F3 F2 F1 at the start.  If not, drop the first
         * byte and keep looking. */
        while (p->len >= 4u &&
               (p->buf[0] != LD2410_HEADER_0 ||
                p->buf[1] != LD2410_HEADER_1 ||
                p->buf[2] != LD2410_HEADER_2 ||
                p->buf[3] != LD2410_HEADER_3)) {
            memmove(p->buf, p->buf + 1, p->len - 1u);
            p->len -= 1u;
        }
        if (p->len < 4u) return 0u;
        /* We have header; compute expected length if the two length
         * bytes are in. */
        if (p->len >= 6u) {
            uint16_t lf = (uint16_t)p->buf[4] | ((uint16_t)p->buf[5] << 8);
            uint16_t total = (uint16_t)(11u + lf);
            if (lf < 2u || total > LD2410_MAX_FRAME_LEN) {
                /* Junk; drop header and resync. */
                memmove(p->buf, p->buf + 1, p->len - 1u);
                p->len -= 1u;
                p->sync_lost = 1u;
                return 0u;
            }
            p->expected_len = total;
            p->sync_lost = 0u;
        } else {
            return 0u;
        }
    }

    if (p->len < p->expected_len) return 0u;

    /* We have a complete frame candidate. */
    RadarSample tmp;
    uint16_t consumed = try_parse_frame(
        p->buf, p->expected_len, radar_id, now_ms, &tmp);
    if (consumed == 0u) {
        /* Bad frame -- resync by dropping just the header byte so we
         * can find the next one. */
        if (p->len > 1u) {
            memmove(p->buf, p->buf + 1, p->len - 1u);
            p->len -= 1u;
        } else {
            p->len = 0u;
        }
        p->expected_len = 0u;
        p->sync_lost = 1u;
        radar_sample_reset(out);
        out->error_code = RADAR_ERROR_FRAME_CRC;
        return 0u;
    }

    /* Shift remaining bytes (if any frames were appended back-to-back). */
    if (p->len > consumed) {
        memmove(p->buf, p->buf + consumed, p->len - consumed);
        p->len -= consumed;
    } else {
        p->len = 0u;
    }
    p->expected_len = 0u;
    p->sync_lost = (p->len > 0u) ? 1u : 0u;
    *out = tmp;
    return 1u;
}

uint16_t ld2410_parser_make_mock_frame(
    uint8_t target_state,
    uint16_t moving_distance_cm,
    uint16_t stationary_distance_cm,
    uint8_t  moving_energy,
    uint8_t  stationary_energy,
    uint8_t *out_buf,
    uint16_t out_buf_len
) {
    if (out_buf == nullptr || out_buf_len < LD2410_MAX_FRAME_LEN) return 0u;

    /* Build a basic-mode-shaped payload: header(4) + length(2) +
     * type(1) + head(4: target_state + detection_distance) + (optional)
     * gate fields + footer(4) + checksum(1).  We always emit a minimal
     * frame so the parser still validates header/footer/length. */
    memset(out_buf, 0, LD2410_MAX_FRAME_LEN);

    uint16_t idx = 0u;
    out_buf[idx++] = LD2410_HEADER_0;
    out_buf[idx++] = LD2410_HEADER_1;
    out_buf[idx++] = LD2410_HEADER_2;
    out_buf[idx++] = LD2410_HEADER_3;

    /* length_field = number of bytes after the length field and before
     * the checksum byte.  6 for the basic-mode payload we emit here
     * (frame_type + head_marker + tail_marker + target_state +
     * 2-byte distance).  Must match what
     * ld2410_parser_py.py::make_mock_frame writes. */
    uint16_t length_field = 6u;

    out_buf[idx++] = (uint8_t)(length_field & 0xFFu);
    out_buf[idx++] = (uint8_t)((length_field >> 8) & 0xFFu);

    /* byte 6: frame type (0x01 = engineering, 0x02 = basic).  The
     * parser does not interpret this but the LD2410 emits one of them. */
    out_buf[idx++] = 0x02u;
    /* byte 7: head marker used by LD2410.  Always 0xAA. */
    out_buf[idx++] = 0xAAu;
    /* byte 8: tail marker used by LD2410.  Always 0xAA. */
    out_buf[idx++] = 0xAAu;
    /* byte 9: target_state. */
    out_buf[idx++] = (uint8_t)(target_state & 0x03u);
    /* bytes 10-11: detection_distance_cm (little-endian). */
    out_buf[idx++] = (uint8_t)(moving_distance_cm & 0xFFu);
    out_buf[idx++] = (uint8_t)((moving_distance_cm >> 8) & 0xFFu);

#if LD2410_PARSE_FULL_GATES
    /* bytes 11-12: stationary distance. */
    out_buf[idx++] = (uint8_t)(stationary_distance_cm & 0xFFu);
    out_buf[idx++] = (uint8_t)((stationary_distance_cm >> 8) & 0xFFu);
    /* byte 13: moving energy. */
    out_buf[idx++] = moving_energy;
    /* byte 14: stationary energy. */
    out_buf[idx++] = stationary_energy;
    /* byte 15: tail marker. */
    out_buf[idx++] = 0x55u;
    /* byte 16: end marker. */
    out_buf[idx++] = 0x55u;
#endif

    /* Footer. */
    out_buf[idx++] = LD2410_FOOTER_0;
    out_buf[idx++] = LD2410_FOOTER_1;
    out_buf[idx++] = LD2410_FOOTER_2;
    out_buf[idx++] = LD2410_FOOTER_3;

    /* Checksum: sum of length bytes + payload + footer. */
    uint8_t sum = 0u;
    for (uint16_t i = 4u; i < 4u + 2u + length_field + 4u; ++i) {
        sum = (uint8_t)(sum + out_buf[i]);
    }
    out_buf[idx++] = sum;

    return idx;
}
