/*
 * jetson_json_transport.h
 *
 * Build the JSON v2 line that the Gateway sends to the Jetson over USB
 * Serial.  One JSON object per line, terminated by '\n'.  The format is
 * the protocol v2 contract (see Prompt 1):
 *
 *   {"v":2,"seq":N,"gateway_ok":1,"remote_ok":0|1,
 *    "gateway_boot_id":N,"remote_boot_id":N,"packet_loss":N,
 *    "radars":{
 *       "left_front":{...}, "left_rear":{...},
 *       "right_front":{...}, "right_rear":{...}
 *    }}
 *
 * Invalid data must be serialised as JSON `null`, never as 0 (per the
 * protocol v2 contract).  The encoder is implemented as a small
 * append-only helper so the firmware can stream the bytes to Serial
 * without allocating a JSON tree in RAM.
 *
 * This module is pure C++ (no Arduino.h) so the same encoder is used
 * by the Python host test mirror.
 */
#ifndef ADAS_JETSON_JSON_TRANSPORT_H
#define ADAS_JETSON_JSON_TRANSPORT_H

#include <stdint.h>
#include <stddef.h>

#include "radar_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum size of one JSON line.  The Gateway MUST refuse to publish
 * lines longer than this to avoid stalling the UART FIFO.  The
 * worst-case v2 line (four radars, all fields populated, extended
 * diagnostics) is under 2 KB; 4096 provides ample headroom for future
 * growth and matches the Jetson-side cap so both sides use the same
 * constant (MAX_LINE_BYTES = 4096 in config/radar_gateway.py). */
#define JETSON_JSON_LINE_MAX        4096u

/* JSON wire-protocol version.  Lives next to the transport so the
 * module compiles standalone on a host without hardware_config.h. */
#define JSON_PROTOCOL_VERSION       2u

typedef struct {
    uint8_t *buf;
    uint32_t cap;
    uint32_t len;
    uint8_t  overflow;     /* set if the line exceeded the cap */
} JsonWriter;

void json_writer_init(JsonWriter *w, uint8_t *buf, uint32_t cap);

void json_writer_append(JsonWriter *w, const char *literal);
void json_writer_append_u32(JsonWriter *w, uint32_t value);
void json_writer_append_i32 (JsonWriter *w, int32_t  value);

/* Append a quoted radar sample sub-object.  Always emits one radar
 * object, with `null` for invalid fields. */
void json_writer_append_radar(JsonWriter *w, const RadarSample *s);

/* Convenience: produce a complete v2 JSON line from a snapshot. */
uint32_t jetson_json_encode_v2(
    const RadarSnapshot *snap,
    uint8_t *buf,
    uint32_t buf_len
);

#ifdef __cplusplus
}
#endif

#endif  /* ADAS_JETSON_JSON_TRANSPORT_H */
