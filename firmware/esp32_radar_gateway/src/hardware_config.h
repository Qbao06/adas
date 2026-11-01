/*
 * Central hardware + protocol configuration for ESP32 Radar Gateway.
 *
 * This header is the ONLY place that contains GPIO pins, MAC addresses,
 * baudrates, ESP-NOW channel and radar-to-port mapping.  Other firmware
 * modules MUST include this header to read configuration.  Do not sprinkle
 * GPIO numbers, MAC bytes or magic constants anywhere else.
 *
 * Build-time role selection:
 *   - ROLE_GATEWAY (ESP32 A, connects to Jetson over USB)
 *   - ROLE_REMOTE  (ESP32 B, forwards its radars to the gateway)
 *
 * Pick the role by passing -DROLE_GATEWAY or -DROLE_REMOTE on the build
 * command line.  Exactly one of the two must be defined.  See build.sh.
 *
 * User-supplied values:
 *   - BOARD_MODEL                  (free-form string, informational)
 *   - UART1_RX_GPIO / UART1_TX_GPIO
 *   - UART2_RX_GPIO / UART2_TX_GPIO
 *   - RADAR_BAUD (default 256000)
 *   - USB_SERIAL_BAUD (default 115200)
 *   - GATEWAY_MAC, REMOTE_MAC      (six 0x.. bytes each)
 *   - ESPNOW_CHANNEL               (1..13, both ESP32 must match)
 *
 * All placeholders are filled with sentinel values that the firmware
 * preflight script (tools/firmware_preflight.py) detects before upload.
 * Production builds additionally abort compilation if any sentinel is
 * still present (define ADAS_PRODUCTION_BUILD=1 to enforce).
 */
#ifndef ADAS_HARDWARE_CONFIG_H
#define ADAS_HARDWARE_CONFIG_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* 0. Sentinel placeholder values.  Detect with firmware_preflight.   */
/*    NEVER reuse these sentinels for real configuration.             */
/* ------------------------------------------------------------------ */
#define GPIO_PLACEHOLDER            0xFFu
#define MAC_PLACEHOLDER             0xFFu
#define CHANNEL_PLACEHOLDER         0u

/* ------------------------------------------------------------------ */
/* 1. Role selection.  Exactly one of these MUST be defined.          */
/* ------------------------------------------------------------------ */
#if defined(ROLE_GATEWAY) && defined(ROLE_REMOTE)
#error "Define exactly one of ROLE_GATEWAY or ROLE_REMOTE"
#endif
#if !defined(ROLE_GATEWAY) && !defined(ROLE_REMOTE)
#error "Define ROLE_GATEWAY (ESP32 A) or ROLE_REMOTE (ESP32 B)"
#endif

/* ------------------------------------------------------------------ */
/* 2. Board / UART GPIO selection.                                    */
/*    Replace the sentinels with the actual pins wired on your ESP32. */
/*    Common ESP32-DevKitC defaults:                                   */
/*      UART1 RX=16 TX=17   UART2 RX=18 TX=19                         */
/* ------------------------------------------------------------------ */
#ifndef BOARD_MODEL
#define BOARD_MODEL                "PLACEHOLDER_BOARD_MODEL"
#endif

#ifndef UART1_RX_GPIO
#define UART1_RX_GPIO               GPIO_PLACEHOLDER
#endif
#ifndef UART1_TX_GPIO
#define UART1_TX_GPIO               GPIO_PLACEHOLDER
#endif
#ifndef UART2_RX_GPIO
#define UART2_RX_GPIO               GPIO_PLACEHOLDER
#endif
#ifndef UART2_TX_GPIO
#define UART2_TX_GPIO               GPIO_PLACEHOLDER
#endif

/* ------------------------------------------------------------------ */
/* 3. UART baudrates.                                                */
/* ------------------------------------------------------------------ */
#ifndef RADAR_BAUD
#define RADAR_BAUD                  256000UL
#endif
#ifndef USB_SERIAL_BAUD
#define USB_SERIAL_BAUD             115200UL
#endif

/* ------------------------------------------------------------------ */
/* 4. ESP-NOW configuration.                                          */
/*    Set both peers' MAC addresses after reading them with the       */
/*    sketch from README.md step 3.  Channel must be identical on     */
/*    both boards.                                                    */
/* ------------------------------------------------------------------ */
#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL              CHANNEL_PLACEHOLDER
#endif

/* Gateway MAC (ESP32 A).  Replace every 0xFF with the bytes printed by
 * the helper sketch (e.g. AA:BB:CC:DD:EE:01). */
#ifndef GATEWAY_MAC_BYTE_0
#define GATEWAY_MAC_BYTE_0          MAC_PLACEHOLDER
#define GATEWAY_MAC_BYTE_1          MAC_PLACEHOLDER
#define GATEWAY_MAC_BYTE_2          MAC_PLACEHOLDER
#define GATEWAY_MAC_BYTE_3          MAC_PLACEHOLDER
#define GATEWAY_MAC_BYTE_4          MAC_PLACEHOLDER
#define GATEWAY_MAC_BYTE_5          MAC_PLACEHOLDER
#endif

/* Remote MAC (ESP32 B). */
#ifndef REMOTE_MAC_BYTE_0
#define REMOTE_MAC_BYTE_0           MAC_PLACEHOLDER
#define REMOTE_MAC_BYTE_1           MAC_PLACEHOLDER
#define REMOTE_MAC_BYTE_2           MAC_PLACEHOLDER
#define REMOTE_MAC_BYTE_3           MAC_PLACEHOLDER
#define REMOTE_MAC_BYTE_4           MAC_PLACEHOLDER
#define REMOTE_MAC_BYTE_5           MAC_PLACEHOLDER
#endif

/* ------------------------------------------------------------------ */
/* 5. Radar->role mapping.                                            */
/*    The Gateway handles the left side, the Remote the right side.   */
/*    UART numbers are 1-based and must match the GPIO above.         */
/* ------------------------------------------------------------------ */
#ifdef ROLE_GATEWAY
#define RADAR_PRIMARY_UART          1   /* UART1 = left_front           */
#define RADAR_PRIMARY_NAME          "left_front"
#define RADAR_SECONDARY_UART        2   /* UART2 = left_rear            */
#define RADAR_SECONDARY_NAME        "left_rear"
#define REMOTE_PRIMARY_NAME         "right_front"
#define REMOTE_SECONDARY_NAME       "right_rear"
#endif

#ifdef ROLE_REMOTE
#define RADAR_PRIMARY_UART          1   /* UART1 = right_front          */
#define RADAR_PRIMARY_NAME          "right_front"
#define RADAR_SECONDARY_UART        2   /* UART2 = right_rear           */
#define RADAR_SECONDARY_NAME        "right_rear"
#endif

/* ------------------------------------------------------------------ */
/* 6. Timing constants (milliseconds).                                */
/*    Radar stale after 300 ms, Remote stale after 600 ms.            */
/* ------------------------------------------------------------------ */
#ifndef RADAR_FRAME_TIMEOUT_MS
#define RADAR_FRAME_TIMEOUT_MS      300UL
#endif
#ifndef REMOTE_PACKET_TIMEOUT_MS
#define REMOTE_PACKET_TIMEOUT_MS    600UL
#endif
#ifndef TX_INTERVAL_MS
#define TX_INTERVAL_MS              50UL  /* 20 Hz target                */
#endif

/* ------------------------------------------------------------------ */
/* 7. Mock mode (development only).                                   */
/*    Set MOCK_MODE=1 to skip real UART reads and synthesise frames.  */
/*    Production builds must leave this 0.                            */
/* ------------------------------------------------------------------ */
#ifndef MOCK_MODE
#define MOCK_MODE                   0
#endif

/* ------------------------------------------------------------------ */
/* 8. Production-build guard.  If ADAS_PRODUCTION_BUILD is set and    */
/*    any placeholder sentinel survives, compilation aborts.          */
/* ------------------------------------------------------------------ */
#ifdef ADAS_PRODUCTION_BUILD
#if UART1_RX_GPIO == GPIO_PLACEHOLDER || UART1_TX_GPIO == GPIO_PLACEHOLDER || \
    UART2_RX_GPIO == GPIO_PLACEHOLDER || UART2_TX_GPIO == GPIO_PLACEHOLDER
#error "Production build: replace GPIO_PLACEHOLDER with real pins in hardware_config.h"
#endif
#if ESPNOW_CHANNEL == CHANNEL_PLACEHOLDER
#error "Production build: replace ESPNOW_CHANNEL placeholder"
#endif
#if GATEWAY_MAC_BYTE_0 == MAC_PLACEHOLDER || REMOTE_MAC_BYTE_0 == MAC_PLACEHOLDER
#error "Production build: replace MAC placeholders with real peer MAC bytes"
#endif
#endif

/* ------------------------------------------------------------------ */
/* 9. JSON wire-protocol version (kept here so dashboards and tests  */
/*    that include only hardware_config.h still see the contract).   */
/* ------------------------------------------------------------------ */
#define JSON_PROTOCOL_VERSION       2u

#endif  /* ADAS_HARDWARE_CONFIG_H */
