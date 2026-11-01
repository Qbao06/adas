/*
 * diagnostics.cpp
 */

#include "diagnostics.h"

#include <string.h>

void diagnostics_init(Diagnostics *d) {
    if (d == nullptr) return;
    diagnostics_reset(d);
}

void diagnostics_reset(Diagnostics *d) {
    if (d == nullptr) return;
    memset(d, 0, sizeof(*d));
}

static void append_kv(char **p, char *end, const char *k, uint32_t v) {
    if (*p >= end) return;
    uint32_t n = 0u;
    /* key. */
    while (*k && *p < end) {
        *(*p)++ = *k++;
        ++n;
    }
    if (*p < end) { *(*p)++ = '='; ++n; }
    /* value (decimal, no leading zeros). */
    char tmp[12];
    uint32_t idx = sizeof(tmp);
    if (v == 0u) tmp[--idx] = '0';
    while (v > 0u && idx > 0u) {
        tmp[--idx] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (idx < sizeof(tmp) && *p < end) {
        *(*p)++ = tmp[idx++];
        ++n;
    }
    if (*p < end) { *(*p)++ = ' '; ++n; }
}

uint32_t diagnostics_render(const Diagnostics *d, char *buf, uint32_t buf_len) {
    if (d == nullptr || buf == nullptr || buf_len == 0u) return 0u;
    buf[buf_len - 1u] = 0;
    char *p   = buf;
    char *end = buf + buf_len - 1u;
    append_kv(&p, end, "radar_ok",     d->radar_frames_ok);
    append_kv(&p, end, "radar_bad",    d->radar_frames_bad);
    append_kv(&p, end, "send_ok",      d->espnow_send_ok);
    append_kv(&p, end, "send_fail",    d->espnow_send_fail);
    append_kv(&p, end, "recv_ok",      d->espnow_recv_ok);
    append_kv(&p, end, "recv_bad",     d->espnow_recv_bad);
    append_kv(&p, end, "recv_drop",    d->espnow_recv_drop);
    append_kv(&p, end, "json_sent",    d->json_lines_sent);
    append_kv(&p, end, "json_dropped", d->json_lines_dropped);
    if (p > buf && *(p - 1) == ' ') {
        p -= 1;
    }
    if (p < end) { *p = 0; }
    return (uint32_t)(p - buf);
}
