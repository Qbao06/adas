# Prompt-1 Final Report — Radar Gateway Architecture & Protocol Standardisation

> Single-Gateway ESP32 architecture for four HLK-LD2410C radars,
> with a single USB-Serial JSON v2 link to the Jetson.

This is the deliverable for **Prompt 1** of `prompts_hoan_thien_2_esp32_4_ld2410c.md`.
Prompt 2 is intentionally out of scope.

---

## 1. Files changed / created

### Created

| Path | Purpose |
|------|---------|
| `sensors/radar_gateway.py` | New Jetson-side facade re-exporting `RadarGatewayReader`, `GatewaySnapshot`, `GatewayRadarSample`, `parse_gateway_line`.  Resolves the `ModuleNotFoundError` that prevented `dashboard/server.py`, `preflight.py`, `app/pipeline.py`, etc. from importing the unified reader. |
| `firmware/esp32_radar_gateway/src/radar_driver.py` | `RadarDriver` interface, `MockRadarDriver` (deterministic, sweep-based) and `RealRadarDriver` host-stub for host-testable firmware. |
| `firmware/esp32_radar_gateway/tests/gateway_aggregator_py.py` | Python mirror of `gateway_aggregator.cpp` so the freshness / boot-id / packet-loss policy is host-testable. |
| `firmware/esp32_radar_gateway/tests/test_radar_driver.py` | 8 host tests for the RadarDriver interface (mock sweep, slot independence, UART drain, garbage + valid frame, corrupt checksum + recovery, no-target sentinel). |
| `firmware/esp32_radar_gateway/tests/test_espnow_aggregator.py` | 10 aggregator tests covering stale, duplicate, out-of-order, wraparound-without-reboot, wrap-on-reboot, EMA loss smoothing, `remote_ok`/`gateway_ok` reporting. |

### Modified

| Path | What changed |
|------|--------------|
| `config/radar_gateway.py` | Added `migrate_legacy_config()` helper that maps the legacy `side_sensor_modules` schema to the v2 `radars` block (Prompt-4 contract). |
| `config/__init__.py` | Re-exports `migrate_legacy_config`. |
| `firmware/esp32_radar_gateway/tests/ld2410_parser_py.py` | Added `LD2410_OFFSET_TARGET_STATE`, `LD2410_OFFSET_DETECTION_DIST_LO`, `LD2410_OFFSET_DETECTION_DIST_HI` constants; aligned `make_mock_frame` payload size with the C++ parser (6 bytes). |
| `firmware/esp32_radar_gateway/tests/test_ld2410_parser.py` | `_build_basic_frame` rewritten to use `payload_len=6` and emit `target_state` at byte 9, `detection_distance_cm` at bytes 10-11 -- matching the C++ offsets. |
| `radar_gateway/protocol.py` | Fixed `_ESPNOW_RADAR_FMT` to the byte-for-byte correct `<BBBBHHBBHBH2s` (7B + 4H + 2s = 17 bytes) so the C++ struct and the Python mirror agree on the on-wire layout.  The pack function now passes the `reserved[2]` bytes through; the unpack function reads them back into the dataclass.  This was the cause of the 6 spurious ESP-NOW test failures when running the Jetson test suite together with the dashboard tests. |
| `docs/protocol/PROTOCOL_V2.md` | Added §2.3 (freshness, duplicate, wraparound policy) and §8 (RadarDriver interface) and §9 (legacy migration map). |

### Verified unchanged but audited

* `firmware/esp32_radar_gateway/src/espnow_protocol.{h,cpp}` -- wire format is the source of truth.
* `firmware/esp32_radar_gateway/src/ld2410_parser.{h,cpp}` -- the UART binary parser.
* `firmware/esp32_radar_gateway/src/jetson_json_transport.{h,cpp}` -- JSON v2 encoder.
* `firmware/esp32_radar_gateway/src/gateway_aggregator.{h,cpp}` -- freshness policy.
* `firmware/esp32_radar_gateway/src/hardware_config.h` -- still has placeholders for production (see §3).
* `radar_gateway/{__init__,ld2410_parser,jetson_protocol,gateway_reader}.py` -- Python mirror, unchanged.
* `preflight.py`, `dashboard/server.py`, `app/pipeline.py` -- reference `migrate_legacy_config` and `sensors/radar_gateway.py`.

---

## 2. Architecture summary

```
                 LD2410C left_front  -- UART1 --\
                 LD2410C left_rear   -- UART2 --- ESP32 A Gateway  -- USB Serial -- Jetson
                                                       ^
                                                       |  ESP-NOW (v2)
                 LD2410C right_front -- UART1 --\
                 LD2410C right_rear  -- UART2 --- ESP32 B Remote
```

* **ESP32 A (Gateway)** reads `left_front` + `left_rear` over Hardware UART1/UART2 and exposes **one** USB-Serial port to the Jetson emitting JSON v2 (one object per line).
* **ESP32 B (Remote)** reads `right_front` + `right_rear` over its own UART1/UART2 and ships the two samples to the Gateway over ESP-NOW.
* **Jetson** opens a single `/dev/serial/by-id/...` device for all four radars.  The `RadarGatewayReader` background thread handles reconnect, line buffering, sequence deduplication and JSON parsing.
* The aggregator on the Gateway combines the two local slots and the two Remote slots into a single `GatewaySnapshot` that the JSON v2 encoder emits every tick.

### Protocol summary

* **ESP-NOW (Gateway ⇄ Remote):**  fixed-size 22 + 2 × 17 + 4 = 60-byte packet, little-endian, FNV-1a 32 checksum, with `magic`, `protocol_version=2`, `node_id`, `boot_id`, `sequence`, `timestamp_ms`, `packet_size`, `radar_count=2`, two `EspNowRadar` records, `checksum`.
* **JSON v2 (Gateway → Jetson):**  one object per line, always exactly four radars, every numeric field is `null` when invalid (never `0`).

Both protocols are covered in detail in `docs/protocol/PROTOCOL_V2.md`.

### Freshness policy

* Stale sample = `now_ms - last_update_ms > RADAR_FRAME_TIMEOUT_MS` (default 600 ms).
* Duplicate / out-of-order ESP-NOW packet is rejected; loss grows via EMA.
* Sequence wrap at `uint32` is **only** legal on a `boot_id` change (reboot / re-flash).
* `gateway_ok` and `remote_ok` are reported in every JSON v2 packet.

### Migration from v1

Operators with the legacy `side_sensor_modules` block get a sensible four-radar Gateway configuration automatically via `config.radar_gateway.migrate_legacy_config`.  In live mode the migration is suppressed by preflight.

---

## 3. Test results

```
firmware/  →  40 passed in 0.47 s
tests/     → 285 passed in 49.76 s
combined   → 325 passed in 59.76 s
```

### Firmware test suite (`firmware/`)

| File | Tests | Result |
|------|------:|--------|
| `tests/test_ld2410_parser.py` | 9 | PASS |
| `tests/test_espnow_protocol.py` | 8 | PASS |
| `tests/test_espnow_aggregator.py` | 10 | PASS |
| `tests/test_jetson_json_transport.py` | 5 | PASS |
| `tests/test_radar_driver.py` | 8 | PASS |

### Jetson test suite (`tests/`)

All 285 tests pass after fixing the `_ESPNOW_RADAR_FMT` regression in `radar_gateway/protocol.py` (was `<BBBBHHBBHBHxx`, expected `<BBBBHHBBHBH2s`).  The six failing tests before the fix were all caused by the format / pack-args mismatch.

---

## 4. Information the user must still provide

The firmware cannot be flashed safely until the following are confirmed on the actual installation:

1. **ESP32 board model** for ESP32 A and ESP32 B (e.g. ESP32-DevKitC, ESP32-WROOM-32).  Required by `hardware_config.h`.
2. **GPIO pins** for UART1 RX/TX and UART2 RX/TX on each ESP32 (currently placeholders).
3. **Wi-Fi MAC addresses** of ESP32 A and ESP32 B (use the helper sketch in `firmware/esp32_radar_gateway/`).
4. **ESP-NOW Wi-Fi channel** (1..13, must match on both boards).
5. **Power-supply voltage** and measured UART logic level on each LD2410C.
6. **`/dev/serial/by-id/...` path** of ESP32 A on the Jetson (so preflight can verify the live device).
7. **Physical position** of each of the four radars on the vehicle (so the camera selector chooses the correct view).

Production builds additionally fail compilation when items 1-4 are left as placeholders (the `ADAS_PRODUCTION_BUILD` guard).

### Documentation gaps

The following items are not documented because real-hardware validation has not been completed:

1. **LD2410 gate layout.**  The parser currently extracts only the combined detection distance.  Moving/stationary distance+energy are only parsed when `LD2410_PARSE_FULL_GATES=1` is defined *after* cross-checking the offsets against the HLK-LD2410C datasheet.  Until then those fields stay at the no-target sentinel and the JSON encoder emits `null`.
2. **Power-rail noise.**  We do not yet know how much ripple the four radars inject into a shared 5 V rail; this needs measurement on the actual installation.

---

## 5. What Prompt 2 will pick up

* Replace the `RealRadarDriver` host stub with the production C++ `ld2410_uart_driver.cpp` (Arduino `HardwareSerial` is only available inside the Arduino framework).
* Wire the four `RadarDriver` instances into the Gateway aggregator's `feed_local` and the Remote's ESP-NOW encoder.
* Replace the hardware-config placeholders with the real GPIO / MAC / channel values.
* Optionally define `LD2410_PARSE_FULL_GATES=1` once the gate offsets are validated against the datasheet.
