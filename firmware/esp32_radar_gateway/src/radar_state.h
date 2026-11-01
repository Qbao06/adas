/*
 * radar_state.h
 *
 * Pure C/C++ struct definitions for radar samples and aggregated radar
 * state.  This header deliberately avoids Arduino.h so it can be unit
 * tested on a host build (see tests/test_ld2410_parser.py and
 * tests/test_espnow_protocol.py).
 *
 * Each radar contributes one RadarSample (latest parse result + age).
 * The Gateway aggregator collects four RadarSample objects (its own
 * left_front/left_rear plus the Remote's right_front/right_rear
 * received over ESP-NOW) and packages them into RadarSnapshot for the
 * Jetson JSON serializer.
 *
 * All distances are centimetres (the LD2410 unit).  Distances are
 * 0xFFFF (uint16) when no target is reported for the corresponding
 * gate.  Energy values are 0xFF when not present.
 *
 * Fields follow the protocol v2 spec from Prompt 1:
 *   target_state, detected, valid, stale,
 *   moving_distance_cm, moving_energy,
 *   stationary_distance_cm, stationary_energy,
 *   detection_distance_cm,
 *   error_code, sample_age_ms, radar_id.
 */
#ifndef ADAS_RADAR_STATE_H
#define ADAS_RADAR_STATE_H

#include <stdint.h>

/* Maximum number of radars aggregated by the Gateway.  Four for the
 * current architecture (left_front/left_rear/right_front/right_rear). */
#define ADAS_MAX_RADARS             4

/* Distance sentinel used by LD2410 when no target is present.  0xFFFF
 * is the documented "no target" value. */
#define LD2410_DISTANCE_NONE        0xFFFFu
#define LD2410_ENERGY_NONE          0xFFu

/* target_state bitfield.  Bit 0 = moving target present, bit 1 =
 * stationary target present, bit 2 = combined detection distance
 * reported. */
#define TARGET_STATE_NONE           0x00u
#define TARGET_STATE_MOVING         0x01u
#define TARGET_STATE_STATIONARY     0x02u
#define TARGET_STATE_BOTH           0x03u

/* Error codes (single byte, 0 == OK). */
#define RADAR_ERROR_NONE            0x00u
#define RADAR_ERROR_NO_FRAME        0x01u
#define RADAR_ERROR_FRAME_CRC       0x02u
#define RADAR_ERROR_LENGTH_MISMATCH 0x03u
#define RADAR_ERROR_BAD_HEADER      0x04u
#define RADAR_ERROR_BAD_FOOTER      0x05u
#define RADAR_ERROR_DROP            0x06u  /* UART buffer overflow       */

typedef struct {
    /* Identity of the radar (string literal, NUL-terminated, <=15 chars).
     * One of: "left_front", "left_rear", "right_front", "right_rear". */
    char     radar_id[16];

    /* Parsed target state.  See TARGET_STATE_* above. */
    uint8_t  target_state;

    /* True if a target is present in either gate. */
    uint8_t  detected;

    /* True if the latest frame passed all parser checks.  Independent
     * of `stale`. */
    uint8_t  valid;

    /* True if `RADAR_FRAME_TIMEOUT_MS` have elapsed since the last
     * valid frame.  The Gateway keeps the previous sample but flags
     * it as stale so the Jetson can drop it. */
    uint8_t  stale;

    /* Per-gate distance in centimetres, LD2410_DISTANCE_NONE if absent. */
    uint16_t moving_distance_cm;
    uint16_t stationary_distance_cm;

    /* Per-gate energy (0..100), LD2410_ENERGY_NONE if absent. */
    uint8_t  moving_energy;
    uint8_t  stationary_energy;

    /* Combined detection distance reported by LD2410 (the closest of
     * the two gates when a target is present). */
    uint16_t detection_distance_cm;

    /* See RADAR_ERROR_* above.  0 means healthy. */
    uint8_t  error_code;

    /* Milliseconds elapsed between the previous valid frame and the
     * current one.  Saturates at 0xFFFF.  Used by the Jetson to detect
     * a stalled UART without trusting ESP32 wall clock. */
    uint16_t sample_age_ms;

    /* Local monotonic timestamp (millis) when this sample was last
     * refreshed.  The aggregator / gateway compare it against
     * RADAR_FRAME_TIMEOUT_MS to set `stale`. */
    uint32_t last_update_ms;
} RadarSample;

/* Aggregated snapshot used by the Gateway JSON serializer.  All four
 * radars are always present; missing or stale radars are flagged via
 * `valid`/`stale` and serialised with null distance fields (per the
 * protocol v2 contract: invalid data MUST be null, never 0). */
typedef struct {
    uint32_t     sequence;          /* monotonically increasing       */
    uint8_t      gateway_ok;        /* always 1 on the Gateway side    */
    uint8_t      remote_ok;         /* 1 if ESP-NOW packet is fresh   */
    uint32_t     gateway_boot_id;   /* stable per Gateway boot        */
    uint32_t     remote_boot_id;    /* 0 if remote never seen         */
    uint32_t     packet_loss;       /* sequence gaps accumulated      */
    RadarSample  radars[ADAS_MAX_RADARS];
} RadarSnapshot;

/* Helpers used by parser + aggregator.  All pure, no Arduino deps. */
static inline void radar_sample_reset(RadarSample *s) {
    s->target_state            = TARGET_STATE_NONE;
    s->detected                = 0u;
    s->valid                   = 0u;
    s->stale                   = 1u;
    s->moving_distance_cm      = LD2410_DISTANCE_NONE;
    s->stationary_distance_cm  = LD2410_DISTANCE_NONE;
    s->moving_energy           = LD2410_ENERGY_NONE;
    s->stationary_energy       = LD2410_ENERGY_NONE;
    s->detection_distance_cm   = LD2410_DISTANCE_NONE;
    s->error_code              = RADAR_ERROR_NO_FRAME;
    s->sample_age_ms           = 0xFFFFu;
    s->last_update_ms          = 0u;
}

static inline void radar_snapshot_reset(RadarSnapshot *snap) {
    snap->sequence       = 0u;
    snap->gateway_ok     = 0u;
    snap->remote_ok      = 0u;
    snap->gateway_boot_id = 0u;
    snap->remote_boot_id  = 0u;
    snap->packet_loss     = 0u;
    radar_sample_reset(&snap->radars[0]); /* callers must overwrite */
    radar_sample_reset(&snap->radars[1]);
    radar_sample_reset(&snap->radars[2]);
    radar_sample_reset(&snap->radars[3]);
}

/* Distance in metres (float).  Returns NaN when invalid (caller is
 * expected to skip the sample).  Provided here so both firmware and
 * Python tests share the same semantics. */
#define RADAR_INVALID_DISTANCE_M   (-1.0f)

#endif  /* ADAS_RADAR_STATE_H */
