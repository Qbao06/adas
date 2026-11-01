/*
 * ld2410_uart_driver.cpp
 *
 * Implementation: pulls bytes from HardwareSerial and feeds them to
 * Ld2410Parser.  On the host (unit tests) this file is excluded from
 * the build so the pure parser can be exercised directly.
 */

#ifdef ARDUINO

#include "ld2410_uart_driver.h"

#include <Arduino.h>

#include "hardware_config.h"
#include "mock_mode.h"

#if MOCK_MODE
static MockRadar s_mock_primary;
static MockRadar s_mock_secondary;
#endif

void radar_uart_driver_init(
    RadarUartDriver *drv,
    const char *primary_radar_id,
    const char *secondary_radar_id,
    RadarSampleCallback on_sample,
    void *user,
    Diagnostics *diag
) {
    if (drv == nullptr) return;
    ld2410_parser_init(&drv->parser_primary);
    ld2410_parser_init(&drv->parser_secondary);
    drv->primary_radar_id   = primary_radar_id;
    drv->secondary_radar_id = secondary_radar_id;
    drv->on_sample          = on_sample;
    drv->user               = user;
    drv->diag               = diag;
}

void radar_uart_driver_begin(RadarUartDriver *drv) {
    (void)drv;
#if !MOCK_MODE
    Serial1.begin(RADAR_BAUD, SERIAL_8N1, UART1_RX_GPIO, UART1_TX_GPIO);
    Serial2.begin(RADAR_BAUD, SERIAL_8N1, UART2_RX_GPIO, UART2_TX_GPIO);
#else
    mock_radar_init(&s_mock_primary,   0xA1B2C3D4u);
    mock_radar_init(&s_mock_secondary, 0x12345678u);
#endif
}

/* Maximum bytes drained from a UART port per poll call.
 * The ESP32 task watchdog triggers if one poll() call blocks for >64 bytes
 * at 256000 baud (each byte ≈ 39 µs, 64 bytes ≈ 2.5 ms). */
#define UART_DRAIN_MAX_BYTES  64u

static void drain_one(
    HardwareSerial &port,
    Ld2410Parser *parser,
    const char *radar_id,
    uint8_t slot,
    RadarSampleCallback on_sample,
    void *user,
    Diagnostics *diag,
    uint32_t now_ms
) {
    /* M1 fix: drain at most UART_DRAIN_MAX_BYTES per poll to avoid the
     * ESP32 task watchdog.  Also count parser misses as bad frames so the
     * diagnostic counter reflects real UART corruption. */
    uint16_t drained = 0u;
    while (port.available() > 0 && drained < UART_DRAIN_MAX_BYTES) {
        int c = port.read();
        if (c < 0) break;
        drained++;
        RadarSample sample;
        uint8_t got = ld2410_parser_feed(
            parser, (uint8_t)c, radar_id, now_ms, &sample);
        if (got) {
            if (diag) diag->radar_frames_ok += 1u;
            if (on_sample) on_sample(slot, &sample, user);
        } else {
            if (diag) diag->radar_frames_bad += 1u;
        }
    }
}

void radar_uart_driver_poll(RadarUartDriver *drv, uint32_t now_ms) {
    if (drv == nullptr) return;
#if MOCK_MODE
    RadarSample s0, s1;
    mock_radar_next(&s_mock_primary,   drv->primary_radar_id,   0u, now_ms, &s0);
    mock_radar_next(&s_mock_secondary, drv->secondary_radar_id, 1u, now_ms, &s1);
    if (drv->on_sample) {
        drv->on_sample(0u, &s0, drv->user);
        drv->on_sample(1u, &s1, drv->user);
    }
    if (drv->diag) {
        drv->diag->radar_frames_ok += 2u;
    }
#else
    drain_one(Serial1, &drv->parser_primary,
              drv->primary_radar_id, 0u,
              drv->on_sample, drv->user, drv->diag, now_ms);
    drain_one(Serial2, &drv->parser_secondary,
              drv->secondary_radar_id, 1u,
              drv->on_sample, drv->user, drv->diag, now_ms);
#endif
}

#endif  /* ARDUINO */
