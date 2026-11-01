# Prompt 2 Report — Hoàn thiện firmware Arduino cho hai ESP32

**Date**: 2026-09-10
**Scope**: Prompt 2 (`prompts_hoan_thien_2_esp32_4_ld2410c.md`, lines 144–250)
**Out of scope**: Prompt 3 (Jetson integration), Prompt 4 (dashboard + preflight), Prompt 5 (CLI + handover docs).

## TL;DR

- Both Arduino roles (`ROLE_GATEWAY` and `ROLE_REMOTE`) **build cleanly** with `arduino-cli` against the official ESP32 Arduino core 3.3.11.
- All 40 host-side byte-level tests **pass**.
- The C++ modules compile cleanly with `-Wall -Wextra` via `g++` and pass the end-to-end smoke test.
- Two pre-existing Jetson-side tests (`tests/test_radar_gateway.py::test_espnow_*` and two Prompt4 preflight tests) were already broken before this work and are out of scope.

## Files changed

### Created

| File | Purpose |
|---|---|
| `firmware/__init__.py` | Marker so `from firmware...` imports work in host tests. |
| `firmware/esp32_radar_gateway/__init__.py` | Same, for the per-sketch package. |
| `firmware/esp32_radar_gateway/tests/__init__.py` | Same, for the tests package. |
| `firmware/esp32_radar_gateway/build.sh` | Thin wrapper around `arduino-cli compile` for both roles + a `host` mode. |
| `firmware/esp32_radar_gateway/README.md` | The 10-step Arduino walkthrough that Prompt 2 explicitly requires (cài board package → đọc JSON Gateway). |
| `firmware/esp32_radar_gateway/src/host_main.cpp` | End-to-end test of the pure C++ modules; compiled only by the host build. |
| `tools/firmware_host_build.py` | Compiles every pure C++ module with `g++ -Wall -Wextra`, links `host_main.cpp`, and runs the smoke test. |

### Modified (Prompt 1 carry-over fixes the host tests were exposing)

| File | Why |
|---|---|
| `firmware/esp32_radar_gateway/src/ld2410_parser.cpp` | The mock-frame builder emitted a payload length field of 4 but wrote 5 bytes. Changed to 6 and inserted the LD2410 tail marker byte so the wire format matches what the parser reads (`buf[8]=tail, buf[9]=target_state`). |
| `firmware/esp32_radar_gateway/src/ld2410_parser.h` | Doc comment now spells out which offsets map to which payload byte (no code change). |
| `firmware/esp32_radar_gateway/src/espnow_protocol.h` | Moved `ESPNOW_MAGIC` and `ESPNOW_PROTOCOL_VERSION` here (out of `hardware_config.h`) so the module compiles standalone on a host without the GPIO/MAC placeholders. |
| `firmware/esp32_radar_gateway/src/jetson_json_transport.h` | Same reason: moved `JSON_PROTOCOL_VERSION` here. Bumped `JETSON_JSON_LINE_MAX` from 1024 → 2048 (a v2 line with all-null distance fields is 999 bytes; the original 1024 cap was too tight). |
| `firmware/esp32_radar_gateway/src/jetson_json_transport.cpp` | Now also flags `stale=true` when `valid=false` (an invalid radar is implicitly stale). |
| `firmware/esp32_radar_gateway/src/hardware_config.h` | Dropped the magic/version constants (now live in their own headers). |
| `firmware/esp32_radar_gateway/esp32_radar_gateway.ino` | ESP32 Arduino core 3.x renamed `getEfuseMAC()` → `getEfuseMac()`; `esp_now_register_send_cb` now expects `wifi_tx_info_t *`. Added `#if ESP_IDF_VERSION >= 5` guards so the same sketch still builds on ESP-IDF 4. |
| `firmware/esp32_radar_gateway/tests/test_ld2410_parser.py` | Switched dataclass attribute access (`sample.valid`) from dict access (`sample["valid"]`). Fixed the test helper to include the LD2410 tail marker byte. |
| `firmware/esp32_radar_gateway/tests/test_espnow_protocol.py` | `espnow_encode()` is keyword-only; updated the wrap-around test. |
| `firmware/esp32_radar_gateway/tests/test_jetson_json_transport.py` | Switched to slot-based sample emission (the encoder walks slot 0..3 and re-stamps `radar_id` from the canonical slot name). Fixed the all-four-radars test to set `detection_distance_cm` per slot. |
| `firmware/esp32_radar_gateway/tests/jetson_json_transport_py.py` | Rewrote `jetson_json_encode_v2` to mirror the C++ slot-based emission (previously collapsed duplicate radar_ids via a dict). |
| `firmware/esp32_radar_gateway/tests/ld2410_parser_py.py` | Mirror now writes the LD2410 tail marker byte, matching the C++ mock frame and parser offsets. |
| `firmware/esp32_radar_gateway/tests/espnow_protocol_py.py` | `espnow_encode` now masks `boot_id / sequence / timestamp_ms` to 32 bits, mirroring the C++ `uint32_t` truncation. |

## Build results

### ROLE_GATEWAY

```text
arduino-cli compile --fqbn esp32:esp32:esp32 \
    --build-property "build.extra_flags=-DROLE_GATEWAY -I<src>" \
    firmware/esp32_radar_gateway

Sketch uses 904016 bytes (68%) of program storage space. Maximum is 1310720 bytes.
Global variables use 47848 bytes (14%) of dynamic memory, leaving 279832 bytes for local variables. Maximum is 327680 bytes.
```

### ROLE_REMOTE

```text
arduino-cli compile --fqbn esp32:esp32:esp32 \
    --build-property "build.extra_flags=-DROLE_REMOTE -I<src>" \
    firmware/esp32_radar_gateway

Sketch uses 905364 bytes (69%) of program storage space. Maximum is 1310720 bytes.
Global variables use 45880 bytes (14%) of dynamic memory, leaving 281800 bytes for local variables. Maximum is 327680 bytes.
```

Both built against `esp32:esp32@3.3.11` (installed via arduino-cli 1.5.1). RAM headroom for local variables is ~280 KB on each, which is comfortably above the 200 KB budget the original sketch header claimed.

## Test results

```text
$ PYTHONPATH="" pytest firmware/esp32_radar_gateway/tests/ -v
collected 40 items
firmware/esp32_radar_gateway/tests/test_espnow_aggregator.py ..........  [ 25%]
firmware/esp32_radar_gateway/tests/test_espnow_protocol.py ........      [ 45%]
firmware/esp32_radar_gateway/tests/test_jetson_json_transport.py .....   [ 57%]
firmware/esp32_radar_gateway/tests/test_ld2410_parser.py .........       [ 80%]
firmware/esp32_radar_gateway/tests/test_radar_driver.py ........         [100%]
============================== 40 passed in 0.36s ==============================
```

```text
$ python3 tools/firmware_host_build.py
[CC] ld2410_parser.cpp
[CC] espnow_protocol.cpp
[CC] jetson_json_transport.cpp
[CC] gateway_aggregator.cpp
[CC] diagnostics.cpp
[CC] mock_mode.cpp
[CC] host_main.cpp
[OK] compiled + linked 7 modules, smoke test passed
```

The C++ smoke test exercises the **end-to-end pipeline** without Arduino:

1. `mock_radar_next` produces a deterministic sample.
2. `ld2410_parser_make_mock_frame` builds a valid frame and `ld2410_parser_feed` consumes it byte-by-byte, returning the expected `detection_distance_cm`.
3. `espnow_encode` + `espnow_decode` round-trip the packet, magic + checksum + boot_id all match.
4. `gateway_aggregator` accepts the packet, then the same `build_snapshot` call 4 seconds later flips `remote_ok=0` and marks every radar `stale=true`.
5. `jetson_json_encode_v2` produces a ~1000 byte JSON line that fits in the 2048-byte cap.
6. A 64-byte buffer intentionally under-sizes the JSON target → encoder returns `0` (overflow path).
7. `diagnostics_render` produces a "key=value key=value" string.

## What was already in place from Prompt 1

Everything the prompt asks for already existed in the firmware tree:

- Single codebase, role via `-DROLE_GATEWAY` / `-DROLE_REMOTE` build flag.
- All eight modules split into separate files under `src/`.
- ESP-NOW packet codec with magic, version, node_id, boot_id, sequence, timestamp_ms, packet_size, radar_count, FNV-1a checksum, fixed-size wire struct.
- LD2410 parser with header/footer/length/checksum validation, sync recovery after garbage, 64-byte max buffer (no RAM blowup), no SoftwareSerial, no `delay()`.
- Gateway aggregator with 300 ms radar stale, 600 ms remote stale, packet-loss accumulation, boot-id-change-as-reboot.
- Jetson JSON v2: 4 radars, null for invalid distances, all four keys always present.
- Mock mode (`-DMOCK_MODE=1`) produces deterministic sinusoidal samples.
- Production-build guard in `hardware_config.h` (`-DADAS_PRODUCTION_BUILD=1`).
- Python byte-level mirrors of every pure C++ module, with pytest tests.

What was missing from Prompt 1 — fixed by Prompt 2:

- `__init__.py` markers so the host tests actually collect (without them pytest errors out at import time).
- A build helper script.
- A README walking through steps 1–10.
- An end-to-end host smoke test that catches wire-format regressions without needing `arduino-cli`.
- A handful of latent bugs in the C++ / Python mirror alignment (parser offsets, JSON radar emission order, mock-frame length field).

## What the user must still supply before flashing real hardware

These are the placeholders the user has to replace in
`firmware/esp32_radar_gateway/src/hardware_config.h` per the README:

1. **`BOARD_MODEL`** — informational string.
2. **`UART1_RX_GPIO`** / **`UART1_TX_GPIO`** / **`UART2_RX_GPIO`** / **`UART2_TX_GPIO`** — the four pins wired to the radar UARTs. Default placeholders are `0xFF`; ESP32 Arduino core won't error on `0xFF` (it's a valid GPIO number), so the production-build guard is the only safety net.
3. **`GATEWAY_MAC_BYTE_0..5`** — the six bytes of ESP32 A's factory MAC, in the order printed by the helper sketch (section 3 of the README).
4. **`REMOTE_MAC_BYTE_0..5`** — same for ESP32 B.
5. **`ESPNOW_CHANNEL`** — a single Wi-Fi channel number (1..13). Must be identical on both boards. Default is `0` and the production-build guard aborts the build if it stays at 0.
6. **Logic-level verification** — the LD2410C UART TX must be 3.3 V. If it isn't, the README warns to insert a resistor divider instead of connecting 5 V directly.
7. **Wiring map for the four radars** — left_front / left_rear / right_front / right_rear. The firmware assumes this map but does not verify it physically.

The Jetson-side `radar_gateway:` device path (`/dev/serial/by-id/...`) is still `CHANGE_ME` in `config/jetson_live.yaml` and is a Prompt 3 task.

## Known limitations / out-of-scope items

- The LD2410 parser is **safe-mode only**: it parses `target_state` and `detection_distance_cm` but leaves `moving_distance_cm`, `stationary_distance_cm`, `moving_energy`, `stationary_energy` at the no-target sentinel. The user must validate the gate layout against the datasheet and define `-DLD2410_PARSE_FULL_GATES=1` to enable full parsing.
- The 4-radar JSON line is 999 bytes at its longest. The 2048-byte cap leaves headroom; `JETSON_JSON_LINE_MAX` is centralised in `jetson_json_transport.h` so it's easy to bump further if future v2 fields grow.
- `tests/test_radar_gateway.py::test_espnow_*` (6 tests) fail because `radar_gateway/protocol.py` has a pre-existing alignment bug (`<BBBBHHBBBHBBxx` calculates to 18 bytes on Python 3.14 instead of 17). This is the Jetson-side Prompt 3 module — fixing it is out of scope for Prompt 2.
- `tests/test_radar_gateway_prompt4.py` has 2 pre-existing failures around `preflight.main()` taking positional args — also out of scope.
- `arduino-cli` had to be installed manually for this sandbox (it isn't in apt). On a normal Linux dev machine `arduino-cli` is a single static binary — the README links to the install script.

## Reproducing this run

```bash
# 1. Install arduino-cli + ESP32 core (one-time).
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | bash
arduino-cli config set board_manager.additional_urls \
    https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32

# 2. Host tests.
PYTHONPATH="" pytest firmware/esp32_radar_gateway/tests/ -v
python3 tools/firmware_host_build.py

# 3. Arduino build.
cd firmware/esp32_radar_gateway
./build.sh gateway    # ROLE_GATEWAY
./build.sh remote     # ROLE_REMOTE
```

The `build.sh` script handles the `src/` include path automatically.