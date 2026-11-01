/*
 * mock_mode.cpp
 */

#include "mock_mode.h"

#include <string.h>

void mock_radar_init(MockRadar *m, uint32_t seed) {
    if (m == nullptr) return;
    m->seed    = seed;
    m->enabled = 1u;
}

static uint32_t mix(uint32_t x) {
    /* xorshift32. */
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    return x;
}

void mock_radar_next(
    MockRadar *m,
    const char *radar_id,
    uint8_t    slot,
    uint32_t   now_ms,
    RadarSample *out
) {
    if (m == nullptr || out == nullptr) return;
    radar_sample_reset(out);
    /* Copy radar_id. */
    size_t idlen = 0u;
    while (radar_id != nullptr && radar_id[idlen] != 0 && idlen < 15u) {
        out->radar_id[idlen] = radar_id[idlen];
        ++idlen;
    }
    out->radar_id[idlen] = 0;

    /* Phase shift per slot so the four radars don't move in lockstep. */
    uint32_t phase = (uint32_t)slot * 7000u + (m->seed & 0xFFFFu);
    uint32_t t = now_ms + phase;
    /* Distance sweep 30 cm..500 cm over a 6-second period. */
    uint32_t period = 6000u;
    uint32_t pos    = t % period;
    uint16_t cm = (uint16_t)(30u + (pos * 470u) / period);

    /* Slow random energy between 20 and 60. */
    uint32_t r = mix(t ^ ((uint32_t)slot * 0x9E3779B9u) ^ m->seed);
    uint8_t energy = (uint8_t)(20u + (r % 40u));

    out->target_state          = TARGET_STATE_MOVING;
    out->detected              = 1u;
    out->valid                 = 1u;
    out->stale                 = 0u;
    out->moving_distance_cm    = cm;
    out->moving_energy         = energy;
    out->detection_distance_cm = cm;
    out->error_code            = RADAR_ERROR_NONE;
    out->sample_age_ms         = 0u;
    out->last_update_ms        = now_ms;
}
