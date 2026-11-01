/*
 * mock_mode.h
 *
 * When MOCK_MODE=1 (defined in hardware_config.h) the firmware skips
 * real UART reads and uses a synthesised RadarSample that drifts over
 * time.  This lets the Jetson pipeline exercise the protocol without
 * any hardware, which is required by Prompt 2 step 9 ("Bật mock mode").
 *
 * The mock is also used by tests/test_jetson_json_transport.py to
 * feed a deterministic sequence of radar samples through the encoder.
 */
#ifndef ADAS_MOCK_MODE_H
#define ADAS_MOCK_MODE_H

#include <stdint.h>

#include "radar_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t seed;
    uint8_t  enabled;
} MockRadar;

void mock_radar_init(MockRadar *m, uint32_t seed);

/* Generate the next synthetic radar sample at time `now_ms`.  Always
 * produces a valid sample (never stale), with a sinusoidal distance
 * sweep between 30 cm and 500 cm to mimic a person walking past the
 * sensor.  This is a deterministic function of `now_ms` and `seed`. */
void mock_radar_next(
    MockRadar *m,
    const char *radar_id,
    uint8_t    slot,
    uint32_t   now_ms,
    RadarSample *out
);

#ifdef __cplusplus
}
#endif

#endif  /* ADAS_MOCK_MODE_H */
