/*
 * ld2410_uart_driver.h
 *
 * Glue between HardwareSerial (UART1/UART2) and the pure Ld2410Parser.
 *
 * Responsibilities:
 *   - Bind one Ld2410Parser per UART.
 *   - Drain both UARTs every loop without blocking on either (one UART
 *     going silent must not stall the other).
 *   - Forward the parsed RadarSample to a caller-supplied callback.
 *   - Bump Diagnostics counters so the Gateway can report UART health.
 *   - Drive the mock-mode fallback when MOCK_MODE=1.
 *
 * No dynamic allocation, no delay(), no SoftwareSerial.
 */
#ifndef ADAS_LD2410_UART_DRIVER_H
#define ADAS_LD2410_UART_DRIVER_H

#include <stdint.h>

#include "ld2410_parser.h"
#include "diagnostics.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*RadarSampleCallback)(
    uint8_t slot,                 /* 0 = primary, 1 = secondary        */
    const RadarSample *sample,
    void *user
);

typedef struct {
    Ld2410Parser   parser_primary;
    Ld2410Parser   parser_secondary;
    const char    *primary_radar_id;
    const char    *secondary_radar_id;
    RadarSampleCallback on_sample;
    void          *user;
    Diagnostics   *diag;
} RadarUartDriver;

void radar_uart_driver_init(
    RadarUartDriver *drv,
    const char *primary_radar_id,
    const char *secondary_radar_id,
    RadarSampleCallback on_sample,
    void *user,
    Diagnostics *diag
);

/* Begin the two HardwareSerial ports.  Must be called from setup().
 * Pulls GPIO numbers and baudrate from hardware_config.h. */
void radar_uart_driver_begin(RadarUartDriver *drv);

/* Drain both UARTs.  Safe to call from loop() every iteration.  When
 * MOCK_MODE=1 this synthesises samples via mock_radar_next() instead
 * of reading the serial ports. */
void radar_uart_driver_poll(
    RadarUartDriver *drv,
    uint32_t now_ms
);

#ifdef __cplusplus
}
#endif

#endif  /* ADAS_LD2410_UART_DRIVER_H */
