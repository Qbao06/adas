/*
 * ld2410_parser.h
 *
 * Pure C++ LD2410 frame parser.  This header deliberately avoids
 * Arduino.h so the parser can be unit-tested on a host build (see
 * tests/test_ld2410_parser.py).  The UART driver (ld2410_uart_driver.h)
 * is the only module that touches HardwareSerial.
 *
 * The HLK-LD2410C reports presence using 24 GHz FMCW and exposes a
 * binary UART protocol.  The header magic F4 F3 F2 F1 and footer
 * magic F8 F7 F6 F5 are part of the publicly documented protocol.
 * The internal offsets for moving/stationary gate distances and
 * energies are NOT yet validated against the official datasheet --
 * the firmware MUST treat parsed values as untrusted until they are
 * cross-checked on real hardware.
 *
 * To keep the contract safe, this parser:
 *   - Verifies header (F4 F3 F2 F1), footer (F8 F7 F6 F5), length and
 *     the LD2410 internal frame checksum (last byte == sum of bytes
 *     from length byte through footer, masked to 8 bits).
 *   - Extracts only the two bytes that the LD2410 basic report places
 *     at known fixed offsets: target_state (byte 8) and the combined
 *     detection distance (bytes 9-10 little-endian).
 *   - Leaves moving/stationary gate distance+energy at the "no target"
 *     sentinel (LD2410_DISTANCE_NONE / LD2410_ENERGY_NONE) unless the
 *     firmware has been built with LD2410_PARSE_FULL_GATES=1 AND the
 *     exact gate layout has been validated against the datasheet.
 *   - Never fabricates a non-zero distance on a malformed frame.
 *
 * Once the user verifies the gate layout against their datasheet, they
 * can enable LD2410_PARSE_FULL_GATES in their build flags.  The host
 * tests cover both modes.
 *
 * Output: a RadarSample (see radar_state.h).
 */
#ifndef ADAS_LD2410_PARSER_H
#define ADAS_LD2410_PARSER_H

#include <stdint.h>
#include <stddef.h>

#include "radar_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum payload size we accept.  LD2410 frames are <= ~30 bytes for
 * the basic report; we round up generously to also catch truncated
 * frames.  Frames longer than this are dropped. */
#define LD2410_MAX_FRAME_LEN        64

/* Header magic (LD2410 official). */
#define LD2410_HEADER_0             0xF4u
#define LD2410_HEADER_1             0xF3u
#define LD2410_HEADER_2             0xF2u
#define LD2410_HEADER_3             0xF1u

/* Footer magic (LD2410 official). */
#define LD2410_FOOTER_0             0xF8u
#define LD2410_FOOTER_1             0xF7u
#define LD2410_FOOTER_2             0xF6u
#define LD2410_FOOTER_3             0xF5u

/* Offsets used by the basic report frame.  Layout (basic mode):
 *   header(4) + length(2) + payload(lf) + footer(4) + checksum(1)
 *   payload bytes:
 *     +0  frame_type  (0x02 = basic, 0x01 = engineering)
 *     +1  head_marker (0xAA)
 *     +2  tail_marker (0xAA)
 *     +3  target_state
 *     +4..5  detection_distance_cm (little-endian)
 * So in the wire buffer:
 *     target_state at index 6 + 3 = 9
 *     distance_lo at index 10
 *     distance_hi at index 11
 */
#define LD2410_OFFSET_TARGET_STATE          9
#define LD2410_OFFSET_DETECTION_DIST_LO    10
#define LD2410_OFFSET_DETECTION_DIST_HI    11
#define LD2410_BASIC_PAYLOAD_LEN             6u

/* Set to 1 by build flags after the user has verified the gate layout
 * against their HLK-LD2410C datasheet.  Defaults to 0 to keep the
 * contract safe. */
#ifndef LD2410_PARSE_FULL_GATES
#define LD2410_PARSE_FULL_GATES     0
#endif

typedef struct {
    /* Accumulator byte buffer.  Sized at LD2410_MAX_FRAME_LEN so the
     * parser can refuse to grow beyond this limit (prevents RAM blowup
     * if the radar starts streaming garbage). */
    uint8_t  buf[LD2410_MAX_FRAME_LEN];
    uint16_t len;
    uint16_t expected_len;     /* 0 == unknown, otherwise length field */
    uint8_t  sync_lost;        /* set to 1 after an error so the next
                                * bytes are re-scanned for a header    */
} Ld2410Parser;

void ld2410_parser_init(Ld2410Parser *p);

/* Feed one byte from UART into the parser.  Returns 1 when a complete
 * valid frame was consumed and `out` was populated; 0 otherwise.
 *
 * `radar_id` is a NUL-terminated string copied into the sample.
 *
 * `now_ms` is the local monotonic millis() value used to set
 * RadarSample::last_update_ms.  Pass 0 when the caller doesn't have
 * millis() (host test harness uses an explicit clock instead).
 */
uint8_t ld2410_parser_feed(
    Ld2410Parser *p,
    uint8_t byte_in,
    const char *radar_id,
    uint32_t now_ms,
    RadarSample *out
);

/* Encode one minimal "engineering" mock frame into `out_buf` so the
 * firmware self-test / mock mode can inject frames without a real
 * radar.  Returns the number of bytes written.  The buffer must be
 * at least LD2410_MAX_FRAME_LEN long. */
uint16_t ld2410_parser_make_mock_frame(
    uint8_t target_state,
    uint16_t moving_distance_cm,
    uint16_t stationary_distance_cm,
    uint8_t  moving_energy,
    uint8_t  stationary_energy,
    uint8_t *out_buf,
    uint16_t out_buf_len
);

#ifdef __cplusplus
}
#endif

#endif  /* ADAS_LD2410_PARSER_H */
