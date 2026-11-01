# Architecture Summary — 4 × HLK-LD2410C, 2 × ESP32, 1 × Jetson

This file is the closing deliverable for prompt 1 (firmware review and
protocol standardisation for the four-LD2410C / two-ESP32 / single-Jetson
setup).  It lists the files that were created or modified, explains the
resulting architecture and protocol, calls out the open items that still
need the user's input, and records the test pass/fail state.

> ⚠️ **Safety** — the LD2410C is a 24 GHz presence sensor, not an
> automotive radar.  Range is ~5 m with 0.75 m zones.  Radar data must
> **not** be used for autonomous braking or vehicle control.  The pipeline
> treats radar data as advisory only; the YOLO camera remains the
> authoritative source.

---

## 1. Resulting architecture

```
LD2410C left_front   -- UART1 --\
LD2410C left_rear    -- UART2 --- ESP32 A Gateway -- USB Serial (JSON v2) --> Jetson
                                              ^                                     |
                                              | ESP-NOW v2 (binary, 60 B)          v
LD2410C right_front  -- UART1 --\        |                              sensors/radar_gateway.py
LD2410C right_rear   -- UART2 --- ESP32 B Remote                       radar_gateway_selector.py
                                                                          app/pipeline.py
```

* **One** USB Serial port on the Jetson handles **all four** radars.  The
  firmware on ESP32 A emits one JSON v2 object per cycle.
* **ESP-NOW v2** between ESP32 A (Gateway) and ESP32 B (Remote) is a
  fixed-size binary packet (60 bytes) with magic, version, node_id,
  boot_id, sequence, timestamp, packet_size, radar_count, two
  `EspNowRadar` records (17 bytes each) and a 32-bit FNV-1a checksum.
* All numeric fields are explicitly sized; **no Strings on the wire**
  inside the ESP-NOW frame; the wire format has a `static_assert` for
  payload size (60 bytes).
* **JSON v2** lines on USB Serial always contain **all four** radars
  (`left_front`, `left_rear`, `right_front`, `right_rear`).  Invalid
  numeric fields become JSON `null` — the firmware never fabricates
  zeros for missing data.

---

## 2. Files created or modified

### 2.1 Python codec package (host-side mirror of the firmware)

These were either missing source files in `radar_gateway/` or rewritten
to remove byte-level mismatches with the C++ codec:

* `radar_gateway/__init__.py` — exposes the four sub-modules.
* `radar_gateway/protocol.py` — Python mirror of
  `firmware/esp32_radar_gateway/src/espnow_protocol.{h,cpp}`.  Includes
  `RadarSample`, `EspNowRadar`, `EspNowPacket`, `espnow_checksum`,
  `espnow_pack_radar`, `espnow_unpack_radar`, `espnow_encode`,
  `espnow_decode`, `detect_sequence_wrap`.
* `radar_gateway/ld2410_parser.py` — Python mirror of
  `firmware/esp32_radar_gateway/src/ld2410_parser.{h,cpp}`.  Incremental
  byte parser, checksum, target-state and detection-distance
  extraction; `make_mock_frame` builds well-formed frames for tests.
* `radar_gateway/jetson_protocol.py` — Python mirror of
  `firmware/esp32_radar_gateway/src/jetson_json_transport.{h,cpp}`.
  `RadarSample`, `RadarSnapshot`, `jetson_json_encode_v2`,
  `jetson_json_encode_v2_bytes`.
* `radar_gateway/gateway_reader.py` — POSIX UART reader (`termios`)
  used by the CLI monitor and by the host tests; yields validated JSON
  lines and reports frame errors via `on_error`.

### 2.2 Jetson-side integration

* `sensors/radar_gateway.py` — production wrapper around the codec:
  `GatewayRadarSample`, `GatewaySnapshot`, `RadarGatewayReader`,
  `parse_gateway_line`.  Background reader exposes `read()` to the
  pipeline, never invents detections, and reports `gateway_ok` /
  `remote_ok` / `packet_loss` per snapshot.

### 2.3 Documentation

* `docs/protocol/PROTOCOL_V2.md` — frozen wire format (ESP-NOW binary
  and JSON v2), safety contract, and pointers to the code that owns each
  part of the spec.
* `docs/RADAR_GATEWAY_MIGRATION.md` — already exists; lists the YAML
  schema for the new `radar_gateway` block and the safety rules.

### 2.4 Tests

* `tests/test_radar_gateway.py` — host-pure tests covering:
  * ESP-NOW encode/decode round-trip
  * bad magic / bad version / bad radar_count / bad size / corrupt
    checksum
  * stale / duplicate / wrap-around sequence handling
  * LD2410 parser: well-formed frame, two back-to-back frames, no-target
    frame (does not invent a distance), mock frame round-trip
  * JSON v2: emits all four radars in protocol order, emits `null` for
    invalid samples
  * `RadarGatewayReader`: skips invalid JSON frames, raises on
    unsupported protocol version, reports monotonic timestamps.

The firmware-side test mirrors under
`firmware/esp32_radar_gateway/tests/*_py.py` were already aligned to the
C++ codec; no edits were needed there for this prompt.

---

## 3. What the firmware review surfaced

The legacy firmware `firmware/esp32_dual_radar_uart/esp32_dual_radar_uart.ino`
assumed the radars exposed a digital DETECT output.  That code path is
now obsolete for the LD2410C because the sensor speaks a binary UART
protocol at 256000 8N1 (configurable).  The new firmware under
`firmware/esp32_radar_gateway/` reads each radar through a Hardware
UART (`ld2410_uart_driver`), feeds frames into the `ld2410_parser`
module, and converts each parsed sample to the wire-stable
`EspNowRadar` struct.  The previous dual-radar Arduino sketch is
retained for reference but **is no longer wired into the pipeline**;
the Jetson opens a single serial port backed by `RadarGatewayReader`.

The Jetson Python side previously expected **two** UARTs (one per side)
through `sensors/esp32_radar.py`.  Both `side_modules` and `side_radar`
blocks in `config/jetson_live.yaml` are still available for backward
compatibility but are disabled by default; the new `radar_gateway`
block is the single source of truth.

---

## 4. Information still needed from the user

The implementation is ready to compile and run, but the following
hardware-specific items must still be supplied by the operator before
the system can be deployed to a real vehicle:

1. **ESP32 A USB device path.**  Set `radar_gateway.device` in
   `config/jetson_live.yaml` to the real `/dev/serial/by-id/...` path
   for the Gateway ESP32.
2. **ESP-NOW peer MAC for ESP32 B.**  Edit
   `firmware/esp32_radar_gateway/src/hardware_config.h` with the real
   MAC of ESP32 B (currently a placeholder).
3. **LD2410C baudrate per radar.**  The defaults are 256000 8N1; if any
   radar is flashed to a different rate, update `RADAR_BAUD`.
4. **Camera-to-radar mapping.**  The `radars` block in
   `config/jetson_live.yaml` is wired to placeholder camera sources
   (`left_1`, `left_2`, `right_1`, `right_2`); replace with the real
   camera IDs once the cameras are mounted.
5. **Real LD2410C byte-protocol documentation.**  This prompt
   intentionally did not invent a wire layout that is not already in
   `firmware/esp32_radar_gateway/src/ld2410_parser.h`.  The parser
   handles basic-mode frames only; engineering-mode (zone-by-zone)
   frames should be documented by the user before the parser is
   extended to surface per-zone energy.  A `RealRadarDriver` is left as
   a stub and a `MockRadarDriver` is used by the host tests today.

---

## 5. Test results

Run with `PYTHONPATH="" .venv_test/bin/python -m pytest tests/` from the
project root.  All test files pass after the corrections made in this
prompt:

| Test file                                | Tests |
|------------------------------------------|------:|
| `tests/test_adas.py`                     | 9 |
| `tests/test_async_reader.py`             | 2 |
| `tests/test_config_validation.py`        | 4 |
| `tests/test_core.py`                     | 8 |
| `tests/test_diagnostics.py`              | 1 |
| `tests/test_distance_warning.py`         | 8 |
| `tests/test_edge_cases.py`               | 19 |
| `tests/test_esp32_radar.py`              | 14 |
| `tests/test_fusion.py`                   | 10 |
| `tests/test_lidar.py`                    | 11 |
| `tests/test_pipeline.py`                 | 15 |
| `tests/test_radar_gateway.py` (new)      | 26 |
| `tests/test_radar_gateway_dashboard.py`  | 62 |
| `tests/test_radar_gateway_monitor.py`    | 22 |
| `tests/test_radar_gateway_prompt4.py`    | 43 |
| `tests/test_side_modules.py`             | 3 |
| `tests/test_synchronization.py`          | 6 |
| `tests/test_vision.py`                   | 8 |
| `tests/test_web_dashboard.py`            | 14 |
| **Total**                                | **285** |

Firmware-side Python tests under
`firmware/esp32_radar_gateway/tests/` (40 tests) also all pass.

```
PYTHONPATH="" .venv_test/bin/python -m pytest tests/ -q
........................................................................ [ 25%]
........................................................................ [ 50%]
........................................................................ [ 75%]
.....................................................................    [100%]
```

No failing tests.  The 26 new tests in `tests/test_radar_gateway.py`
cover the ESP-NOW protocol invariants (checksum, magic, version,
size, stale / duplicate / wrap sequence handling), the LD2410 parser
(well-formed frame, two back-to-back frames, no-target sentinel,
mock frame round-trip) and the JSON v2 transport (all four radars
present in protocol order, `null` for invalid samples).

---

## 6. Prompt 2

Per the prompt's explicit instruction, prompt 2 was **not** executed.
