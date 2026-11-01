# ADAS Radar Gateway Protocol v2

> Single source of truth for the binary and JSON wire format used by the
> ESP32 A Gateway, ESP32 B Remote and the Jetson pipeline.

This document is the prompt-1 deliverable.  It freezes the architecture
and wire format so the firmware and the Jetson-side readers cannot drift
out of sync.  Any change here MUST be reflected in both
``firmware/esp32_radar_gateway/src/`` (C++) and ``radar_gateway/`` (Python).

---

## 1. Hardware map

```
LD2410C left_front   -- UART1 --\
LD2410C left_rear    -- UART2 --- ESP32 A Gateway -- USB Serial -- Jetson
                                          ^
                                          | ESP-NOW
LD2410C right_front  -- UART1 --\        |
LD2410C right_rear   -- UART2 --- ESP32 B Remote
```

* **ESP32 A (Gateway)** owns the USB Serial link to the Jetson and the
  ESP-NOW peer to ESP32 B.  GPIO numbers, baudrates and MAC bytes live
  exclusively in ``firmware/esp32_radar_gateway/src/hardware_config.h``.
* **ESP32 B (Remote)** only feeds two radars to the Gateway over ESP-NOW.
* Each radar uses a dedicated HardwareSerial UART (UART1, UART2) at
  ``RADAR_BAUD`` (default ``256000`` 8N1).  ``SoftwareSerial`` is
  forbidden.
* The Jetson opens **exactly one** serial port for **all four** radars.

The default radar mapping is:

| ESP32   | UART1        | UART2        |
|---------|--------------|--------------|
| Gateway | left_front   | left_rear    |
| Remote  | right_front  | right_rear   |

---

## 2. ESP-NOW wire format (binary)

Used only between ESP32 A and ESP32 B.  Not exposed to the Jetson.

| Offset | Size | Field            | Notes                                  |
|-------:|-----:|------------------|----------------------------------------|
|   0    |   4  | magic            | ``0x52444E31`` (``"RDN1"`` little-endian) |
|   4    |   1  | protocol_version | Always ``2``                            |
|   5    |   1  | node_id          | ``1`` = Gateway, ``2`` = Remote        |
|   6    |   4  | boot_id          | Stable per ESP32 boot                    |
|  10    |   4  | sequence         | Monotonic per boot_id                    |
|  14    |   4  | timestamp_ms     | ``millis()`` at send time                |
|  18    |   2  | packet_size      | Bytes from header[0] through checksum    |
|  20    |   2  | radar_count      | Always ``2`` for the Remote              |
|  22    |  N   | radar_samples    | See :ref:`espnow-radar` below             |
|  ...   |   4  | checksum         | FNV-1a 32 over all preceding bytes        |

Fixed-size, little-endian, no padding, no ``String`` objects.

### 2.1. ``EspNowRadar`` (17 bytes, packed)

| Offset | Size | Field                  |
|-------:|-----:|------------------------|
|   0    |   1  | target_state           |
|   1    |   1  | detected               |
|   2    |   1  | valid                  |
|   3    |   1  | stale                  |
|   4    |   2  | moving_distance_cm     |
|   6    |   2  | stationary_distance_cm |
|   8    |   1  | moving_energy          |
|   9    |   1  | stationary_energy      |
|  10    |   2  | detection_distance_cm  |
|  12    |   1  | error_code             |
|  13    |   2  | sample_age_ms          |
|  15    |   2  | reserved               |

The Gateway substitutes the radar identifier from its own mapping; the
wire format does not carry ``radar_id`` strings (a packed struct only).

The Python mirror in ``radar_gateway/protocol.py`` packs the trailing
``reserved[2]`` as a 2-byte string (``2s`` in ``struct`` syntax) so the
on-wire 17 bytes match the C++ struct byte-for-byte.

### 2.2. Decoder contract

The Gateway **MUST** reject any packet where:

* the magic is wrong,
* ``protocol_version != 2``,
* ``node_id`` is neither ``1`` nor ``2``,
* ``radar_count != 2``,
* ``packet_size`` does not match the actual received length,
* the FNV-1a 32 checksum does not match.

A ``boot_id`` change is accepted as a clean reboot (the sequence is
allowed to restart at zero); otherwise the sequence must be strictly
monotonic per ``boot_id``.

The static-assert baked into the C++ codec and the Python mirror
ensures ``sizeof(EspNowPacket) == ESPNOW_PAYLOAD_SIZE``.

### 2.3. Freshness, duplicate and wraparound policy

The Gateway's ``GatewayAggregator`` (see
``firmware/esp32_radar_gateway/src/gateway_aggregator.{h,cpp}`` and its
Python mirror ``firmware/esp32_radar_gateway/tests/gateway_aggregator_py.py``)
implements the following policy on top of the wire decoder:

* **Stale sample.**  A radar whose ``last_update_ms`` is older than
  ``RADAR_FRAME_TIMEOUT_MS`` (default 600 ms) is marked
  ``stale = true`` on the next tick.  The consumer MUST treat
  ``valid == true && stale == true`` as no-data.
* **Duplicate / out-of-order packet.**  An incoming packet whose
  ``sequence`` is ``<=`` the last accepted sequence (same ``boot_id``)
  is dropped without overwriting the latest sample.  The detected gap
  contributes to ``packet_loss`` via an exponential moving average
  (``alpha = 0.2`` by default).
* **32-bit sequence wraparound.**  The on-wire ``sequence`` is a
  ``uint32``; when it wraps from ``0xFFFFFFFF`` to ``0`` the aggregator
  treats the wrapped packet as a duplicate (loss).  A legitimate wrap
  must be accompanied by a ``boot_id`` change (the Remote rebooted or
  re-flashed).  The aggregator detects reboot through ``boot_id``
  alone; sequence wrap is *never* trusted as a reboot signal.
* **Boot-id reset.**  When the Remote changes ``boot_id`` the
  aggregator resets ``sequence``, ``packet_loss`` and the Remote-side
  ``stale`` flags.  This is the only legal way for the Remote's
  sequence to restart from zero.

These rules are enforced by the test suite:

* ``firmware/esp32_radar_gateway/tests/test_espnow_protocol.py`` --
  codec-level checks (magic, version, packet_size, radar_count,
  checksum, 32-bit sequence wrap).
* ``firmware/esp32_radar_gateway/tests/test_espnow_aggregator.py`` --
  aggregator-level checks (stale marking, duplicate rejection,
  out-of-order loss, wraparound-without-reboot, wrap-on-reboot,
  EMA loss smoothing, ``remote_ok`` and ``gateway_ok`` reporting).

---

## 3. JSON v2 wire format (USB Serial)

Used between ESP32 A Gateway and the Jetson.  One JSON object per line,
terminated by ``'\n'``.  Example:

```json
{"v":2,"seq":42,"gateway_ok":1,"remote_ok":1,
 "gateway_boot_id":12345678,"remote_boot_id":98765432,"packet_loss":0,
 "radars":{
   "left_front":{"detected":true,"target_state":"moving","distance_m":1.80,
                 "moving_distance_m":1.80,"stationary_distance_m":null,
                 "moving_energy":55,"stationary_energy":null,
                 "valid":true,"stale":false,"age_ms":20,"error":0},
   "left_rear":{...}, "right_front":{...}, "right_rear":{...}
 }}
```

### 3.1. Top-level keys

| Key                | Type    | Description                                  |
|--------------------|---------|----------------------------------------------|
| ``v``              | int     | Protocol version.  Always ``2``.              |
| ``seq``            | int     | Monotonic per Gateway boot (``uint32``).      |
| ``gateway_ok``     | int 0/1 | ``1`` iff the Gateway itself is online.       |
| ``remote_ok``      | int 0/1 | ``1`` iff the Remote has sent a packet recently. |
| ``gateway_boot_id``| int     | Stable per Gateway boot.                     |
| ``remote_boot_id`` | int     | Last seen Remote ``boot_id`` (``0`` if never). |
| ``packet_loss``    | int     | Accumulated sequence gaps (Remote).          |
| ``radars``         | object  | Always exactly four entries, see below.       |

### 3.2. ``radars`` keys

The order is fixed:

1. ``left_front``
2. ``left_rear``
3. ``right_front``
4. ``right_rear``

### 3.3. Per-radar object

| Key                    | Type      | Description                                  |
|------------------------|-----------|----------------------------------------------|
| ``detected``           | bool      | True if either gate reported a target.       |
| ``target_state``       | string    | ``"none"``, ``"moving"``, ``"stationary"``, ``"both"``. |
| ``distance_m``         | float/null| Combined detection distance in metres.       |
| ``moving_distance_m``  | float/null| Moving-target gate distance (metres).        |
| ``stationary_distance_m`` | float/null | Stationary-target gate distance (metres).|
| ``moving_energy``      | int/null  | Moving-target gate energy (0..100).          |
| ``stationary_energy``  | int/null  | Stationary-target gate energy (0..100).      |
| ``valid``              | bool      | True when the latest parser run was OK.      |
| ``stale``              | bool      | True if no fresh frame for ``RADAR_FRAME_TIMEOUT_MS``. |
| ``age_ms``             | int       | Milliseconds since the previous valid frame. |
| ``error``              | int       | LD2410 ``RADAR_ERROR_*`` code.               |

### 3.4. Mandatory null-encoding rules

Invalid data **must** be serialised as JSON ``null``.  In particular:

* A sample with ``valid == false`` must have ``null`` for every numeric
  field.  The encoder never substitutes ``0``.
* A sample with ``detection_distance_cm == 0xFFFF`` (``LD2410_DISTANCE_NONE``)
  must be serialised as ``null`` even when ``valid == true``.
* A radar not yet seen on the wire (e.g. during startup) must appear in
  the JSON object with ``valid == false``, ``stale == true`` and
  ``null`` numeric fields.

### 3.5. Line length

The Jetson reader caps each line at ``MAX_LINE_BYTES`` (default 4096
bytes).  Lines exceeding this cap are dropped and counted as JSON errors.

---

## 4. Safety contract

The architecture does **not** attempt to make the system safe to control
a real vehicle.  In particular:

* HLK-LD2410C is a 24 GHz FMCW presence sensor, not a certified
  automotive blind-spot radar.  It must not be used for automatic
  braking or steering.
* ``detected == true`` is only valid if ``valid == true`` and the
  underlying distance is non-null.  All consumers must reject
  detections that violate this rule.
* A Remote loss affects **only** ``right_front`` and ``right_rear``.
  A Gateway loss puts **all** radars into ``stale = true``.
* Mock radars, ``CHANGE_ME`` device paths, and the legacy
  ``side_sensor_modules`` block are **forbidden** in live preflight.

---

## 5. What is **not** in this document

The following are intentionally out of scope; they require explicit user
input before the firmware can be flashed safely:

1. ESP32 board model (e.g. ESP32-DevKitC, ESP32-WROOM-32).
2. GPIO pins for UART1 RX/TX and UART2 RX/TX on each ESP32.
3. Wi-Fi MAC addresses of ESP32 A and ESP32 B (read with the helper
   sketch, see ``firmware/esp32_radar_gateway/README.md``).
4. ESP-NOW Wi-Fi channel (1..13, must match on both boards).
5. Power supply voltage and measured UART logic level on each LD2410C.
6. ``/dev/serial/by-id/...`` path of ESP32 A on the Jetson.
7. Physical position of each of the four radars on the vehicle.

Production builds additionally fail compilation if any of items 1-4 are
left as placeholders (see ``hardware_config.h`` /
``ADAS_PRODUCTION_BUILD`` guard).

---

## 6. Outstanding documentation

The following items are not yet documented because the corresponding
real-hardware validation has not been completed:

1. **LD2410 gate layout.**  The parser currently extracts only the
   combined detection distance.  Moving/stationary distance+energy are
   only parsed when the user defines ``LD2410_PARSE_FULL_GATES=1``
   **after** cross-checking the offsets against the HLK-LD2410C
   datasheet.  Until then, those fields stay at the no-target
   sentinel and the JSON encoder emits ``null``.
2. **Power-rail noise.**  We do not yet know how much ripple the four
   radars inject into a shared 5 V rail; this needs measurement on the
   actual installation.

These gaps are flagged so that the firmware does not silently invent
fields it cannot justify.

---

## 7. Where the code lives

| Layer              | Path                                                                  |
|--------------------|-----------------------------------------------------------------------|
| ESP-NOW codec      | ``firmware/esp32_radar_gateway/src/espnow_protocol.{h,cpp}``         |
| LD2410 parser      | ``firmware/esp32_radar_gateway/src/ld2410_parser.{h,cpp}``            |
| JSON v2 transport  | ``firmware/esp32_radar_gateway/src/jetson_json_transport.{h,cpp}``    |
| Host-test mirror   | ``firmware/esp32_radar_gateway/tests/{espnow,ld2410_parser,jetson_json_transport}_py.py`` |
| Public Python pkg  | ``radar_gateway/{__init__,protocol,ld2410_parser,jetson_protocol,gateway_reader}.py`` |
| Jetson reader      | ``sensors/radar_gateway.py``                                          |
| CLI monitor        | ``tools/radar_gateway_monitor.py``                                    |
| Preflight          | ``preflight.py``                                                      |
| Dashboard          | ``dashboard/server.py``                                               |

All host tests live in ``firmware/esp32_radar_gateway/tests/`` (use a
C++ harness via ``gcc`` + Python mirror) and ``tests/`` (Jetson-side).

---

## 8. RadarDriver interface

The firmware talks to each LD2410C through a small ``RadarDriver``
contract so the parser, MockRadarDriver and RealRadarDriver can swap
without touching the rest of the firmware.  See
``firmware/esp32_radar_gateway/src/radar_driver.py`` (host stub) and
the C++ implementation in ``firmware/esp32_radar_gateway/src/ld2410_uart_driver.cpp``
(Prompt 2 will pick that up; the host stub is the contract source).

```
class RadarDriver:
    def begin(self, baudrate: int) -> None: ...
    def read(self, *, now_ms: int) -> RadarSample: ...
```

* ``MockRadarDriver`` -- deterministic, used by host tests and
  ``MOCK_MODE=1`` builds.  Sweeps each slot through 30-500 cm with a
  sinusoidal phase shift, independent per slot.
* ``RealRadarDriver`` -- production UART reader.  The class exists
  today as a host stub that consumes a fake ``HardwareSerial``-like
  port and feeds bytes through ``Ld2410Parser``.  The Arduino-side
  production implementation lives in ``ld2410_uart_driver.cpp`` and
  pulls bytes from ``HardwareSerial``.

The driver contract is exercised by
``firmware/esp32_radar_gateway/tests/test_radar_driver.py`` which
covers:

* MockRadarDriver deterministic sweep and per-slot phase independence.
* RealRadarDriver drains bytes from a fake UART port and exposes the
  first valid sample via the ``on_sample`` callback.
* Corrupt checksum + recovery (the parser rejects the bad frame and
  the next good frame survives).
* No-target state (target_state=0) keeps ``distance_m`` at the
  ``LD2410_DISTANCE_NONE`` sentinel; the driver never fabricates a 0.

---

## 9. Migration from the legacy v1 protocol

Operators with an existing ``side_sensor_modules`` configuration can
let the runtime build a sensible four-radar Gateway block on the fly
via :func:`config.radar_gateway.migrate_legacy_config`.  This helper
maps the legacy ``left_1`` / ``left_2`` / ``right_1`` / ``right_2``
modules to the v2 ``radars`` block:

| Legacy module | v2 radar slot |
|---------------|----------------|
| ``left_1``    | ``left_front`` |
| ``left_2``    | ``left_rear``  |
| ``right_1``   | ``right_front``|
| ``right_2``   | ``right_rear`` |

The migration is enabled by default in ``preflight.py``; in live mode
it is a no-op because legacy ``side_sensor_modules`` blocks are
forbidden.
