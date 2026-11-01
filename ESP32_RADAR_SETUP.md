# ESP32 + 4× HLK LD2410C + Jetson (Gateway v2)

> Sơ đồ chân đầy đủ cho hai ESP32 nằm tại
> `docs/ESP32_2BOARD_4RADAR_WIRING_VI.md`.

Hệ thống radar mới dùng **ESP32 A (Gateway)** đọc 4 radar qua UART (2 cổng UART
song song) và gửi một batch JSON v2 qua USB Serial tới Jetson mỗi ~50 ms.
**ESP32 B (Remote)** chỉ bridge 2 radar bên phải tới Gateway qua ESP-NOW.
Live **không bao giờ** fallback sang mock radar; một radar lỗi chỉ làm radar đó
stale mà không ảnh hưởng 3 radar còn lại.

Schema, validation, mapping camera và migration từ UART cũ nằm trong
[`docs/RADAR_GATEWAY_MIGRATION.md`](docs/RADAR_GATEWAY_MIGRATION.md).
Tài liệu này tập trung vào phần cứng và firmware.

## Luồng hoạt động

```
left_front  ──── UART1 ──── ESP32 A (Gateway) ──── USB ──── Jetson
left_rear   ──── UART2 ────          ↑ ESP-NOW ──── ESP32 B (Remote)
right_front ──── UART1 ──── ESP32 B (Remote) ──── bridge ──┘
right_rear  ──── UART2 ──── ESP32 B (Remote)
```

**ESP32 A (Gateway):**
- UART1 = left_front, UART2 = left_rear
- Nhận ESP-NOW từ ESP32 B → tổng hợp 4 radar → JSON v2 qua USB Serial

**ESP32 B (Remote):**
- UART1 = right_front, UART2 = right_rear
- Gửi 2 radar qua ESP-NOW tới Gateway mỗi ~50 ms

Camera selector trên Jetson:
1. Radar trái có xe → camera trái
2. Radar phải có xe → camera phải
3. Hai bên cùng có → chọn bên gần hơn
4. Hết phát hiện → giữ camera 1,2 s rồi về camera trước
5. Mất UART / >0,6 s không có gói → radar stale → camera trước

## Đấu dây quan trọng

### Nguồn
- Tất cả GND chung: 2 ESP32 + 4 radar + Jetson.
- Nguồn radar: **5 V @ ≥500 mA** mỗi con (không lấy từ chân ESP32).
- 4 radar đặt gần nhau có thể gây nhiễu UART → dùng cáp xoắn đôi,
  tách xa nguồn công suất.

### UART (LD2410C)
- LD2410C dùng **UART 256000 baud 8N1**, mức logic 3,3 V.
- **KHÔNG** dùng chân `DETECT` digital (không cần thiết — firmware đọc
  khoảng cách qua UART).
- Nếu chân `DETECT` radar là 5 V → dùng level shifter 5↔3,3 V.
- Cáp UART ≤ 3 m để tránh lỗi bit.

### ESP-NOW
- Hai ESP32 phải trên **cùng một kênh Wi-Fi** (1..13) và trong
  phạm vi phủ sóng (~100 m LOS).
- Đọc MAC thật bằng sketch helper trong README firmware.
- Trên xe chuyển động, kiểm tra ESP-NOW ổn định trước khi lái thử.

## Nạp firmware

Firmware v2 nằm trong `firmware/esp32_radar_gateway/`.

### 1. Điền cấu hình phần cứng

Mở `firmware/esp32_radar_gateway/src/hardware_config.h` và thay:

| Macro | Ví dụ | Ghi chú |
|---|---|---|
| `BOARD_MODEL` | `"ESP32-DevKitC-V1"` | Tên board để tiện debug |
| `UART1_RX_GPIO` / `UART1_TX_GPIO` | 16 / 17 | GPIO ESP32 A nối radar left |
| `UART2_RX_GPIO` / `UART2_TX_GPIO` | 18 / 19 | GPIO ESP32 A nối radar left rear |
| `ESPNOW_CHANNEL` | 6 | Kênh Wi-Fi, trùng 2 ESP32 |
| `GATEWAY_MAC_BYTE_*` | `0xAA,0xBB,...` | MAC ESP32 A (đọc bằng sketch) |
| `REMOTE_MAC_BYTE_*` | `0xCC,0xDD,...` | MAC ESP32 B |

Chạy `python3 tools/firmware_preflight.py` để kiểm tra chưa còn placeholder.

### 2. Build ESP32 A (Gateway)

```bash
cd firmware/esp32_radar_gateway
./build.sh gateway
# Nạp qua arduino-cli hoặc Arduino IDE (chọn FQBN esp32:esp32:esp32)
```

### 3. Build ESP32 B (Remote)

```bash
./build.sh remote
# Nạp qua arduino-cli hoặc Arduino IDE
```

### 4. Kiểm tra

Mở Serial Monitor (115200 baud) trên **ESP32 A (Gateway)** — thấy dòng
`[OK] ROLE_GATEWAY ready` là firmware đã chạy đúng.

## Cấu hình Jetson

Sửa `config/jetson_live.yaml` với schema mới:

```yaml
camera:
  sources:
    front: /dev/v4l/by-id/...-index0
    left_1: /dev/v4l/by-id/...-index0
    left_2: /dev/v4l/by-id/...-index0
    right_1: /dev/v4l/by-id/...-index0
    right_2: /dev/v4l/by-id/...-index0

radar_gateway:
  enabled: true
  device: /dev/serial/by-id/usb-...   # ESP32 A Gateway, không để CHANGE_ME
  baudrate: 115200                    # USB Serial (radar UART trên ESP32 là 256000)
  protocol_version: 2
  stale_after_seconds: 0.6
```

Không có `config/jetson_demo.yaml`. Production launcher chỉ chạy `live` / `camera-only`.

Tài khoản chạy ADAS cần quyền đọc UART và camera:

```bash
sudo usermod -aG dialout,video $USER
# Đăng xuất / đăng nhập lại
```

Preflight:

```bash
.venv/bin/python preflight.py --config config/jetson_live.yaml
```

Kiểm tra camera `front` và phải nhận JSON v2 hợp lệ từ Gateway trước khi Live chạy.
