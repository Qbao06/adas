/*
 * diagnostics.h
 *
 * Counters for firmware health.  Pure C++; the ESP-NOW send callback
 * and the UART reader bump counters; the gateway loop logs them once
 * per second over a dedicated `Serial` instance (Serial1 on the
 * Remote, the same Serial on the Gateway) so debug counters never get
 * mixed into the production JSON stream.
 */
#ifndef ADAS_DIAGNOSTICS_H
#define ADAS_DIAGNOSTICS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t radar_frames_ok;
    uint32_t radar_frames_bad;
    uint32_t espnow_send_ok;
    uint32_t espnow_send_fail;
    uint32_t espnow_recv_ok;
    uint32_t espnow_recv_bad;
    uint32_t espnow_recv_drop;
    uint32_t json_lines_sent;
    uint32_t json_lines_dropped;
} Diagnostics;

void diagnostics_init(Diagnostics *d);
void diagnostics_reset(Diagnostics *d);

/* Render a single line of "key=value key=value ..." counters into the
 * provided buffer.  Always NUL-terminates if buf_len >= 1.  Returns
 * the number of bytes written (excluding the NUL). */
uint32_t diagnostics_render(
    const Diagnostics *d,
    char *buf,
    uint32_t buf_len
);

#ifdef __cplusplus
}
#endif

#endif  /* ADAS_DIAGNOSTICS_H */
