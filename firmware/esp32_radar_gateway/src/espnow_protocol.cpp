/*
 * espnow_protocol.cpp
 *
 * Implementation of the ESP-NOW packet codec.  No Arduino dependency;
 * compiles with plain g++ on a host.
 *
 * The wire format is little-endian (ESP32 is little-endian natively).
 * We rely on a fixed-size struct + manual byte writes so we do not
 * depend on struct padding.  The Python test mirror in
 * tests/test_espnow_protocol.py replicates the exact same byte layout
 * so the test suite is authoritative.
 */

#include "espnow_protocol.h"

#include <string.h>

static inline void write_u8 (uint8_t *p, uint8_t  v)                { p[0] = v; }
static inline void write_u16(uint8_t *p, uint16_t v)                { p[0] = (uint8_t)(v & 0xFFu); p[1] = (uint8_t)((v >> 8) & 0xFFu); }
static inline void write_u32(uint8_t *p, uint32_t v)                { p[0] = (uint8_t)(v & 0xFFu); p[1] = (uint8_t)((v >> 8) & 0xFFu); p[2] = (uint8_t)((v >> 16) & 0xFFu); p[3] = (uint8_t)((v >> 24) & 0xFFu); }

static inline uint8_t  read_u8 (const uint8_t *p)                   { return p[0]; }
static inline uint16_t read_u16(const uint8_t *p)                   { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static inline uint32_t read_u32(const uint8_t *p)                   { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

uint32_t espnow_checksum(const uint8_t *data, uint32_t len) {
    /* FNV-1a 32. */
    uint32_t hash = 0x811C9DC5u;
    for (uint32_t i = 0u; i < len; ++i) {
        hash ^= (uint32_t)data[i];
        hash *= 0x01000193u;
    }
    return hash;
}

void espnow_pack_radar(const RadarSample *src, EspNowRadar *dst) {
    if (src == nullptr || dst == nullptr) return;
    dst->target_state            = src->target_state;
    dst->detected                = src->detected;
    dst->valid                   = src->valid;
    dst->stale                   = src->stale;
    dst->moving_distance_cm      = src->moving_distance_cm;
    dst->stationary_distance_cm  = src->stationary_distance_cm;
    dst->moving_energy           = src->moving_energy;
    dst->stationary_energy       = src->stationary_energy;
    dst->detection_distance_cm   = src->detection_distance_cm;
    dst->error_code              = src->error_code;
    dst->sample_age_ms           = src->sample_age_ms;
    dst->reserved[0]             = 0u;
    dst->reserved[1]             = 0u;
}

void espnow_unpack_radar(
    const EspNowRadar *src,
    const char        *radar_id,
    uint32_t           now_ms,
    RadarSample       *dst
) {
    if (src == nullptr || dst == nullptr) return;
    radar_sample_reset(dst);
    size_t idlen = 0u;
    while (radar_id != nullptr && radar_id[idlen] != 0 && idlen < 15u) {
        dst->radar_id[idlen] = radar_id[idlen];
        ++idlen;
    }
    dst->radar_id[idlen] = 0;
    dst->target_state            = src->target_state;
    dst->detected                = src->detected;
    dst->valid                   = src->valid;
    dst->stale                   = src->stale;
    dst->moving_distance_cm      = src->moving_distance_cm;
    dst->stationary_distance_cm  = src->stationary_distance_cm;
    dst->moving_energy           = src->moving_energy;
    dst->stationary_energy       = src->stationary_energy;
    dst->detection_distance_cm   = src->detection_distance_cm;
    dst->error_code              = src->error_code;
    dst->sample_age_ms           = src->sample_age_ms;
    dst->last_update_ms          = now_ms;
}

uint32_t espnow_encode(
    uint8_t        node_id,
    uint32_t       boot_id,
    uint32_t       sequence,
    uint32_t       timestamp_ms,
    const RadarSample *radars,
    uint8_t       *out_buf,
    uint32_t       out_buf_len
) {
    if (out_buf == nullptr || radars == nullptr) return 0u;
    if (out_buf_len < ESPNOW_PAYLOAD_SIZE) return 0u;

    const uint16_t total_size = (uint16_t)ESPNOW_PAYLOAD_SIZE;

    uint32_t off = 0u;
    write_u32(out_buf + off, ESPNOW_MAGIC);        off += 4u;
    write_u8 (out_buf + off, ESPNOW_PROTOCOL_VERSION); off += 1u;
    write_u8 (out_buf + off, node_id);             off += 1u;
    write_u32(out_buf + off, boot_id);             off += 4u;
    write_u32(out_buf + off, sequence);            off += 4u;
    write_u32(out_buf + off, timestamp_ms);        off += 4u;
    write_u16(out_buf + off, total_size);          off += 2u;
    write_u16(out_buf + off, ESPNOW_RADARS_PER_PACKET); off += 2u;

    for (uint16_t i = 0; i < ESPNOW_RADARS_PER_PACKET; ++i) {
        EspNowRadar r;
        espnow_pack_radar(&radars[i], &r);
        uint8_t tmp[sizeof(EspNowRadar)];
        memset(tmp, 0, sizeof(tmp));
        write_u8 (tmp + 0,  r.target_state);
        write_u8 (tmp + 1,  r.detected);
        write_u8 (tmp + 2,  r.valid);
        write_u8 (tmp + 3,  r.stale);
        write_u16(tmp + 4,  r.moving_distance_cm);
        write_u16(tmp + 6,  r.stationary_distance_cm);
        write_u8 (tmp + 8,  r.moving_energy);
        write_u8 (tmp + 9,  r.stationary_energy);
        write_u16(tmp + 10, r.detection_distance_cm);
        write_u8 (tmp + 12, r.error_code);
        write_u16(tmp + 13, r.sample_age_ms);
        tmp[15] = 0u; tmp[16] = 0u;
        memcpy(out_buf + off, tmp, sizeof(EspNowRadar));
        off += sizeof(EspNowRadar);
    }

    uint32_t checksum = espnow_checksum(out_buf, off);
    write_u32(out_buf + off, checksum);
    off += 4u;

    return off;
}

int espnow_decode(
    const uint8_t *buf,
    uint32_t       buf_len,
    EspNowPacket  *out
) {
    if (buf == nullptr || out == nullptr) return 1;
    if (buf_len < ESPNOW_PAYLOAD_SIZE) return 1;

    uint32_t off = 0u;
    out->magic            = read_u32(buf + off); off += 4u;
    if (out->magic != ESPNOW_MAGIC) return 2;
    out->protocol_version = read_u8 (buf + off); off += 1u;
    if (out->protocol_version != ESPNOW_PROTOCOL_VERSION) return 3;
    out->node_id          = read_u8 (buf + off); off += 1u;
    if (out->node_id != ESPNOW_NODE_GATEWAY && out->node_id != ESPNOW_NODE_REMOTE) return 3;
    out->boot_id          = read_u32(buf + off); off += 4u;
    out->sequence         = read_u32(buf + off); off += 4u;
    out->timestamp_ms     = read_u32(buf + off); off += 4u;
    out->packet_size      = read_u16(buf + off); off += 2u;
    out->radar_count      = read_u16(buf + off); off += 2u;
    if (out->radar_count != ESPNOW_RADARS_PER_PACKET) return 4;
    if (out->packet_size != ESPNOW_PAYLOAD_SIZE)      return 5;

    for (uint16_t i = 0; i < ESPNOW_RADARS_PER_PACKET; ++i) {
        const uint8_t *r = buf + off;
        EspNowRadar tmp;
        tmp.target_state            = read_u8 (r + 0);
        tmp.detected                = read_u8 (r + 1);
        tmp.valid                   = read_u8 (r + 2);
        tmp.stale                   = read_u8 (r + 3);
        tmp.moving_distance_cm      = read_u16(r + 4);
        tmp.stationary_distance_cm  = read_u16(r + 6);
        tmp.moving_energy           = read_u8 (r + 8);
        tmp.stationary_energy       = read_u8 (r + 9);
        tmp.detection_distance_cm   = read_u16(r + 10);
        tmp.error_code              = read_u8 (r + 12);
        tmp.sample_age_ms           = read_u16(r + 13);
        tmp.reserved[0]             = r[15];
        tmp.reserved[1]             = r[16];
        out->radars[i] = tmp;
        off += sizeof(EspNowRadar);
    }
    out->checksum = read_u32(buf + off); off += 4u;

    uint32_t expected = espnow_checksum(buf, off - 4u);
    if (expected != out->checksum) return 6;
    return 0;
}

/* Compile-time check that our layout matches the expected wire size.
 * If this fails, the encoder/decoder are inconsistent and the test
 * suite will also fail. */
typedef char _espnow_size_check_t[
    (ESPNOW_PAYLOAD_SIZE == (22u + 2u * sizeof(EspNowRadar) + 4u)) ? 1 : -1];
