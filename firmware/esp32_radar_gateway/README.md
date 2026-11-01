# ESP32 Radar Gateway firmware

Single-codebase Arduino sketch that drives **two ESP32 boards** (one
Gateway, one Remote) wired to **four HLK-LD2410C** 24 GHz presence
radars and streams the aggregated readings to a Jetson over USB
Serial.  See `../ESP32_RADAR_SETUP.md` for the wiring diagram; this
document covers only the Arduino side.

```
LD2410C left_front  ── UART1 ─┐
                              ├─ ESP32 A Gateway ── USB Serial ── Jetson
LD2410C left_rear   ── UART2 ─┘          ▲
                                         │ ESP-NOW
LD2410C right_front ── UART1 ─┐
                              ├─ ESP32 B Remote
LD2410C right_rear  ── UART2 ─┘
```

The wire protocol is fixed by Prompt 1 of the original spec — see
`tests/test_espnow_protocol.py` and
`tests/test_jetson_json_transport.py` for the byte-level contract.
The Prompt 2 task added the build chain, the production guard and
this README.

---

## 1. Cài ESP32 board package (Arduino CLI)

```bash
# Install arduino-cli if you haven't already.
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | bash

# Add the official Espressif board URL and install the core.
arduino-cli config set board_manager.additional_urls \
    https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

If you prefer the Arduino IDE instead of `arduino-cli`:

1. **File → Preferences → Additional Boards URLs** →
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. **Tools → Board → Boards Manager → esp32 → Install**

## 2. Chọn đúng board

For both roles the FQBN is **`esp32:esp32:esp32`** (the generic "ESP32
Dev Module").  When uploading, pick the serial port that matches
your board (e.g. `/dev/ttyUSB0` on Linux, `COM5` on Windows,
`/dev/cu.usbserial-1410` on macOS).

If you use a non-generic board, change the FQBN in `build.sh` and
`arduino-cli` commands below; the firmware itself does not depend on
any board-specific peripheral.

## 3. Lấy MAC của từng ESP32

Upload the **read-only** MAC sketch to each board (Arduino IDE →
**File → Examples → esp32 → ChipId → GetChipId** works, or run the
short snippet below).  Record the printed MACs — you will need them
in step 4.

```cpp
#include <Arduino.h>
void setup() {
    Serial.begin(115200);
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    Serial.printf("MAC = %s\n", ESP.MacAddress().c_str());
#else
    Serial.printf("MAC = %s\n", ESP.getEfuseMACString().c_str());
#endif
}
void loop() {}
```

Power-cycle each board and copy the printed address verbatim — the
firmware compares the **byte-level** MAC, so even a single typo will
silently drop every packet.

## 4. Điền MAC peer

Open `src/hardware_config.h` and replace the six placeholders for
**both** boards:

```c
/* Gateway MAC (ESP32 A). */
#define GATEWAY_MAC_BYTE_0          0xAA   /* <- your ESP32 A bytes */
#define GATEWAY_MAC_BYTE_1          0xBB
#define GATEWAY_MAC_BYTE_2          0xCC
#define GATEWAY_MAC_BYTE_3          0xDD
#define GATEWAY_MAC_BYTE_4          0xEE
#define GATEWAY_MAC_BYTE_5          0x01

/* Remote MAC (ESP32 B). */
#define REMOTE_MAC_BYTE_0           0xAA   /* <- your ESP32 B bytes */
#define REMOTE_MAC_BYTE_1           0xBB
#define REMOTE_MAC_BYTE_2           0xCC
#define REMOTE_MAC_BYTE_3           0xDD
#define REMOTE_MAC_BYTE_4           0xEE
#define REMOTE_MAC_BYTE_5           0x02
```

Both boards need the **peer** MAC.  Upload the file once per board.

## 5. Điền GPIO UART1 và UART2

The four GPIOs are board-specific.  In `src/hardware_config.h`:

```c
#ifndef UART1_RX_GPIO
#define UART1_RX_GPIO               16     /* <- your wiring */
#endif
#ifndef UART1_TX_GPIO
#define UART1_TX_GPIO               17
#endif
#ifndef UART2_RX_GPIO
#define UART2_RX_GPIO               18
#endif
#ifndef UART2_TX_GPIO
#define UART2_TX_GPIO               19
#endif
```

For ESP32-DevKitC, the safe default mapping is `UART1 = GPIO16/17`
and `UART2 = GPIO18/19`.  **Do not use GPIO6–11** (they touch the
integrated SPI flash) or any GPIO already pulled up by the board
(GPIO0, GPIO2, GPIO12, GPIO15).

**Logic level warning**: HLK-LD2410C UART TX is 3.3 V.  Most ESP32
boards accept 3.3 V on their RX pins directly.  If your radar board
uses 5 V TX (older modules), insert a resistor divider — *do not*
connect 5 V directly to the ESP32.

## 6. Build và upload ROLE_GATEWAY (ESP32 A)

```bash
cd firmware/esp32_radar_gateway
./build.sh gateway                # compile only
arduino-cli upload -p /dev/ttyUSB0 \
    --fqbn esp32:esp32:esp32 \
    --build-property "build.extra_flags=-DROLE_GATEWAY -I$(pwd)/src" \
    firmware/esp32_radar_gateway
```

Replace `/dev/ttyUSB0` with the actual port (Windows: `COM5`, macOS:
`/dev/cu.usbserial-1410`).

## 7. Build và upload ROLE_REMOTE (ESP32 B)

```bash
./build.sh remote                 # compile only
arduino-cli upload -p /dev/ttyUSB1 \
    --fqbn esp32:esp32:esp32 \
    --build-property "build.extra_flags=-DROLE_REMOTE -I$(pwd)/src" \
    firmware/esp32_radar_gateway
```

The two boards look identical externally — label them so you don't
flash the wrong role to the wrong ESP32.  Both roles must agree on
`ESPNOW_CHANNEL` (default `1`; pick a channel that has no Wi-Fi
network on either test site).

## 8. Mở Serial Monitor

Open a serial monitor at **115200 baud** on each board.  You should
see:

```
[OK] ROLE_GATEWAY ready
{"v":2,"seq":1,"gateway_ok":1,"remote_ok":0,...
{"v":2,"seq":2,"gateway_ok":1,"remote_ok":0,...
```

One JSON line per 50 ms.  On the Remote board you will only see the
boot banner — its production output is the ESP-NOW packet stream, not
JSON.

To distinguish debug output from production JSON the Gateway writes
its `[DBG] ...` counters to **Serial1** (a second hardware UART).
Either leave Serial1 unconnected, or open a second monitor on the
matching pins if your board exposes them.

## 9. Bật mock mode (chưa có phần cứng thật)

If you don't have any LD2410C yet, build with `-DMOCK_MODE=1` to make
both boards synthesise samples:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 \
    --build-property "build.extra_flags=-DROLE_GATEWAY -DMOCK_MODE=1 -I$(pwd)/src" \
    firmware/esp32_radar_gateway
```

Mock mode produces a sinusoidal distance sweep between 30 cm and
500 cm so the Jetson pipeline can be exercised end-to-end before the
real radars arrive.

## 10. Kiểm tra JSON từ Gateway

On the host (or Jetson) open the USB serial at 115200 baud:

```bash
# Linux: /dev/serial/by-id/<your ESP32 A>
stty -F /dev/ttyUSB0 115200 cs8 -cstopb -parenb -ixon
cat /dev/ttyUSB0
```

You should see one JSON line per 50 ms (20 Hz) with four radar
sub-objects.  When everything is wired correctly:

```json
{"v":2,"seq":42,"gateway_ok":1,"remote_ok":1,"gateway_boot_id":305419896,
 "remote_boot_id":3735928559,"packet_loss":0,
 "radars":{
   "left_front":{"detected":true,"target_state":"moving","distance_m":1.20,
     "moving_distance_m":1.20,"stationary_distance_m":null,
     "moving_energy":45,"stationary_energy":null,
     "valid":true,"stale":false,"age_ms":22,"error":0},
   "left_rear":{...}, "right_front":{...}, "right_rear":{...}
 }}
```

`remote_ok=0` means the Gateway has not yet received an ESP-NOW
packet from the Remote in the last 600 ms; check MAC and channel.
`gateway_ok=0` means the Gateway itself has not parsed any radar
frame yet; check UART wiring and baudrate.

---

## Production-build guard

When you are ready to flash for real, build with the extra flag
`-DADAS_PRODUCTION_BUILD=1`.  This causes the compiler to abort if
any of the placeholders in `src/hardware_config.h` (GPIOs, MACs,
channel) is still set to `0xFF`:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 \
    --build-property "build.extra_flags=-DROLE_GATEWAY -DADAS_PRODUCTION_BUILD=1 -I$(pwd)/src" \
    firmware/esp32_radar_gateway
```

Expected error if a placeholder survives:

```
src/hardware_config.h:175:2: error: "Production build: replace GPIO_PLACEHOLDER ..."
```

`tools/firmware_preflight.py` (added in Prompt 3) does the same check
at runtime before the Jetson allows `live` mode.

## Test pipeline

Run the host-side tests without any Arduino toolchain:

```bash
# Pure-Python byte-level tests (mirror the C++ protocol code).
PYTHONPATH="" pytest firmware/esp32_radar_gateway/tests/ -v

# g++ smoke build of the C++ modules + end-to-end pipeline test.
python3 tools/firmware_host_build.py
```

Both must pass before any Arduino build is attempted.

## Files

| Path | Purpose |
|---|---|
| `esp32_radar_gateway.ino` | Top-level Arduino sketch. `#include`s the modules below. |
| `src/hardware_config.h` | **Sole** source of GPIO, MAC, channel, baudrate, role mapping. |
| `src/radar_state.h` | Shared `RadarSample` / `RadarSnapshot` structs. |
| `src/ld2410_parser.{h,cpp}` | Pure-C++ LD2410 frame parser. No Arduino dependency. |
| `src/ld2410_uart_driver.{h,cpp}` | Glue: HardwareSerial → parser → callback. |
| `src/espnow_protocol.{h,cpp}` | Pure-C++ Remote → Gateway packet codec (FNV-1a checksum). |
| `src/gateway_aggregator.{h,cpp}` | 4-radar snapshot + stale tracking + boot_id/sequence guard. |
| `src/jetson_json_transport.{h,cpp}` | Pure-C++ streaming JSON v2 encoder. |
| `src/diagnostics.{h,cpp}` | Bump-counter diagnostics. |
| `src/mock_mode.{h,cpp}` | Synthetic radar for `-DMOCK_MODE=1`. |
| `src/host_main.cpp` | g++-only smoke test; NOT compiled by Arduino. |
| `tests/*.py` | Python byte-level mirrors of every protocol module. |
| `build.sh` | Thin wrapper around `arduino-cli compile`. |