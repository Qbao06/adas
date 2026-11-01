/*
 * espnow_protocol.h
 *
 * Pure C++ codec for the Remote -> Gateway ESP-NOW packet (protocol v2).
 * This header deliberately avoids Arduino.h so it can be unit-tested
 * on a host.  See tests/test_espnow_protocol.py for the Python mirror.
 *
 * Wire format (little-endian on the wire, fixed-size fields, no padding):
 *
 *   offset  size  field
 *   0       4     magic           = ESPNOW_MAGIC ("RDN1" as uint32 LE)
 *   4       1     protocol_version
 *   5       1     node_id         (1 = gateway, 2 = remote)
 *   6       4     boot_id         (random per ESP32 boot)
 *   10      4     sequence        (monotonic per boot_id)
 *   14      4     timestamp_ms    (millis() at send time)
 *   18      2     packet_size     (bytes from header[0] through checksum)
 *   20      2     radar_count     (always 2 for the Remote)
 *   22      N     radar_samples   (see RadarSample, but without the
 *                                  radar_id string to keep the struct
 *                                  packed and fixed-size; the Gateway
 *                                  re-applies the radar_id from its
 *                                  own mapping).
 *   ...     4     checksum        (FNV-1a 32 bit of all preceding bytes)
 *
 * The Gateway MUST verify:
 *   - magic
 *   - protocol_version == ESPNOW_PROTOCOL_VERSION
 *   - radar_count == 2
 *   - packet_size matches the actual received length
 *   - checksum matches the recomputed value
 *   - boot_id changes are accepted as a reboot (sequence resets)
 *   - sequence is monotonic per boot_id; duplicates / older seqs dropped
 */
#ifndef ADAS_ESPNOW_PROTOCOL_H
#define ADAS_ESPNOW_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#include "radar_state.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESPNOW_NODE_GATEWAY        1u
#define ESPNOW_NODE_REMOTE         2u

#define ESPNOW_HEADER_FIELDS_SIZE  22u   /* magic..radar_count          */
#define ESPNOW_CHECKSUM_SIZE       4u
#define ESPNOW_RADARS_PER_PACKET   2u    /* one Remote = 2 radars       */

/* Magic and protocol version live with the protocol definition, NOT
 * in hardware_config.h.  That way the pure C++ module can compile on
 * a host without pulling in the GPIO/MAC placeholders. */
#define ESPNOW_MAGIC                0x52444E31u   /* "RDN1" gateway-bound */
#define ESPNOW_PROTOCOL_VERSION     2u

/* Fixed-size radar representation over the air (no id string, no
 * last_update_ms).  The Gateway substitutes the id from its own
 * mapping so the Remote doesn't need to know the position labels. */
typedef struct {
    uint8_t  target_state;
    uint8_t  detected;
    uint8_t  valid;
    uint8_t  stale;
    uint16_t moving_distance_cm;
    uint16_t stationary_distance_cm;
    uint8_t  moving_energy;
    uint8_t  stationary_energy;
    uint16_t detection_distance_cm;
    uint8_t  error_code;
    uint16_t sample_age_ms;
    uint8_t  reserved[2];   /* explicit padding to keep size stable    */
} EspNowRadar;

/* Compute payload size in bytes for one packet. */
#define ESPNOW_PAYLOAD_SIZE (ESPNOW_HEADER_FIELDS_SIZE + \
    (sizeof(EspNowRadar) * ESPNOW_RADARS_PER_PACKET) + \
    ESPNOW_CHECKSUM_SIZE)

typedef struct {
    uint32_t       magic;
    uint8_t        protocol_version;
    uint8_t        node_id;
    uint32_t       boot_id;
    uint32_t       sequence;
    uint32_t       timestamp_ms;
    uint16_t       packet_size;
    uint16_t       radar_count;
    EspNowRadar    radars[ESPNOW_RADARS_PER_PACKET];
    uint32_t       checksum;
} EspNowPacket;

/* FNV-1a 32 bit checksum.  Matches the implementation used by the
 * Python test mirror so both sides agree byte-for-byte. */
uint32_t espnow_checksum(const uint8_t *data, uint32_t len);

/* Encode a packet from local state.  Returns total number of bytes
 * written to `out_buf`.  `out_buf` MUST be at least ESPNOW_PAYLOAD_SIZE
 * bytes. */
uint32_t espnow_encode(
    uint8_t        node_id,
    uint32_t       boot_id,
    uint32_t       sequence,
    uint32_t       timestamp_ms,
    const RadarSample *radars,    /* array of 2                          */
    uint8_t       *out_buf,
    uint32_t       out_buf_len
);

/* Decode a packet received over ESP-NOW into a local EspNowPacket.
 * Returns 0 on success, non-zero on:
 *   1  buffer too short
 *   2  bad magic
 *   3  unsupported protocol version
 *   4  radar_count != 2
 *   5  packet_size mismatch
 *   6  checksum mismatch
 */
int espnow_decode(
    const uint8_t *buf,
    uint32_t       buf_len,
    EspNowPacket  *out
);

/* Convert a host-side RadarSample into the wire EspNowRadar. */
void espnow_pack_radar(const RadarSample *src, EspNowRadar *dst);

/* Convert a wire EspNowRadar into a host-side RadarSample, attaching
 * `radar_id` as the identifier. */
void espnow_unpack_radar(
    const EspNowRadar *src,
    const char        *radar_id,
    uint32_t           now_ms,
    RadarSample       *dst
);

#ifdef __cplusplus
}
#endif

#endif  /* ADAS_ESPNOW_PROTOCOL_H */
