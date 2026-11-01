# Migration: UART per-side radar → single ESP32 A Gateway (v2)

Tài liệu này hướng dẫn chuyển từ schema cũ ``side_sensor_modules`` (mỗi radar có
một UART riêng) sang schema mới ``radar_gateway`` (một ESP32 A Gateway duy nhất
đọc bốn radar HLK-LD2410C qua JSON v2).

> **Nguyên tắc an toàn**
>
> * Live tuyệt đối không fallback sang mock radar. Nếu Gateway USB bị rút,
>   toàn bộ radar bị đánh dấu `stale=true, valid=false` và camera trở về
>   `front` sau khi hết thời gian `camera_hold_seconds`.
> * Một radar lỗi chỉ làm radar đó mất dữ liệu; ba radar còn lại vẫn hoạt
>   động bình thường.
> * ESP32 B (Remote bridge) mất → chỉ `right_front` và `right_rear` offline;
>   `left_front` và `left_rear` vẫn chạy.

## 1. Tổng quan kiến trúc

```
LD2410C left_front  --UART-->  ESP32 A Gateway  --USB Serial JSON v2-->  Jetson
LD2410C left_rear   --UART-->        (FW: firmware/esp32_radar_gateway)        |
                                                                               |
ESP32 B Remote bridge <---ESP-NOW---ESP32 A                                      |
       |                                                                       |
LD2410C right_front ---UART--- ESP32 B                                          |
LD2410C right_rear  ---UART--- ESP32 B  ----------------------------------------+
```

* **ESP32 A Gateway** chạy firmware trong `firmware/esp32_radar_gateway.ino`.
  Nó gửi mỗi radar batch thành một dòng JSON v2 (xem `radar_gateway/jetson_protocol.py`
  để biết schema đầy đủ).
* **ESP32 B Remote bridge** đọc hai radar phải qua UART, gói thành
  ESP-NOW v2 (xem `firmware/esp32_radar_gateway/src/espnow_protocol.cpp`).
* **Jetson** mở `/dev/serial/by-id/usb-ESP32_A_Gateway-if00`, parse JSON v2
  và điều khiển selector camera (xem `sensors/radar_gateway.py`,
  `sensors/radar_gateway_selector.py`).

## 2. Schema mới (`radar_gateway`)

Mở `config/jetson_live.yaml` và thay block `side_sensor_modules` cũ bằng:

```yaml
radar_gateway:
  enabled: true
  device: /dev/serial/by-id/CHANGE_ME   # đổi sang ESP32 A Gateway
  baudrate: 115200
  protocol_version: 2                   # JSON v2 — production
  stale_after_seconds: 0.6
  reconnect_seconds: 2.0
  camera_hold_seconds: 1.2
  max_line_bytes: 4096
  priority:
    - left_front
    - left_rear
    - right_front
    - right_rear
  radars:
    left_front:
      enabled: true
      camera: left_1
      warning_distance_m: 5.0            # tối đa 5.0 m — giới hạn phần cứng LD2410C
    left_rear:
      enabled: true
      camera: left_2
      warning_distance_m: 5.0
    right_front:
      enabled: true
      camera: right_1
      warning_distance_m: 5.0
    right_rear:
      enabled: true
      camera: right_2
      warning_distance_m: 5.0
```

`side_sensor_modules` (nếu vẫn còn) sẽ bị bỏ qua khi `radar_gateway.enabled=true`.
Nếu bạn muốn tắt hẳn Gateway để chạy schema cũ, đặt `radar_gateway.enabled=false`
và bật `side_sensor_modules.enabled=true`.

## 3. Migration tự động

`config.radar_gateway.migrate_legacy_config()` sinh block `radar_gateway`
từ `side_sensor_modules` cũ, giữ nguyên mapping `module_id → camera` và
`warning_distance_m`. Đây là cách nhanh nhất để chuyển đổi:

```python
from config.radar_gateway import migrate_legacy_config, save_radar_gateway

legacy_yaml = {
    "side_sensor_modules": {
        "enabled": True,
        "modules": {
            "left_1":  {"enabled": True, "side": "left",  "position": "front",
                        "camera": {"device": "/dev/video1"},
                        "radar":  {"enabled": True, "device": "/dev/ttyACM0",
                                   "warning_distance_m": 4.0}},
            # …
        },
    }
}
new_block = migrate_legacy_config(legacy_yaml)
save_radar_gateway("config/jetson_live.yaml", new_block)
```

Sau khi lưu, file YAML chỉ thay đổi duy nhất block `radar_gateway`; toàn bộ
phần `mode`, `camera`, `lidar`, `calibration`, … được giữ nguyên nhờ cơ chế
atomic save (`.tmp` + `os.replace()`).

## 4. Nạp firmware

1. Mở `firmware/esp32_radar_gateway/esp32_radar_gateway.ino` trong Arduino IDE.
2. Sửa `HARDWARE_CONFIG` cho phù hợp (chân UART, baudrate LD2410C).
3. Chọn board ESP32, Upload. ESP32 A sẽ in `Gateway ready` và bắt đầu phát
   JSON v2 qua Serial USB.
4. Trên Jetson, chạy `python tools/radar_gateway_monitor.py --device
   /dev/serial/by-id/usb-ESP32_A_Gateway-if00` để xác nhận frame hợp lệ.

## 5. Validation

* `config/validation.py:validate_config()` sẽ chạy `validate_radar_gateway`
  trên block của bạn. Lỗi thường gặp:
  * `radar_gateway.device còn placeholder CHANGE_ME` — phải đổi sang đường
    dẫn `/dev/serial/by-id/...` thật.
  * `radar_gateway.radars.<id>.warning_distance_m=… vượt quá 5.0 m` — LD2410C
    chỉ phát hiện đến 5 m.
  * `radar_gateway.protocol_version=1` — chỉ chấp nhận `2` (v2 là bản
    production).
* `preflight.py` mở UART thật và đợi một gói JSON v2 hợp lệ trong 2 s
  (dùng `probe_gateway` với `FakeGatewaySource` cho CI).

## 6. Camera selector

`sensors/radar_gateway_selector.GatewayCameraSelector` tuân thủ các quy tắc
trong Prompt 4:

1. **Khoảng cách nhỏ nhất thắng** — khi nhiều radar cùng `detected=true` và
   có `distance_m` hợp lệ, chọn radar gần nhất. Bằng nhau thì ưu tiên
   theo `priority` trong config (không dựa vào thứ tự dict).
2. **`detected=true` nhưng `distance=null`** — giữ camera hiện tại nếu nó
   thuộc radar đang detected; nếu không thì chuyển sang radar detected đầu
   tiên theo priority.
3. **`camera_hold_seconds`** — giữ camera đã chọn trong khoảng
   `1.2 s` (mặc định) sau khi mất phát hiện để tránh nhấp nháy.
4. **Mất Gateway (`link_ok=false`)** — sau khi hết hold, trở về `front`.
5. **Mất Remote (`remote_ok=false`)** — chỉ `right_front` / `right_rear`
   offline; `left_front` / `left_rear` vẫn chọn camera bình thường.
6. **Reverse gear** — selector ưu tiên `rear` bất kể radar.

Camera side chỉ phục vụ hiển thị; camera trước vẫn dùng cho LiDAR
calibration. Pipeline chỉ fusion ở `front`; các camera khác chỉ được
chọn làm viewport cho driver.

## 7. Live mode không fallback mock

Khi `radar_gateway.enabled=true` và `mode=live`:
* Nếu ESP32 A Gateway không mở được → `RuntimeError` ngay tại `_open_inputs`.
* Không bao giờ tự động dùng `side_sensor_modules` để bù đắp.
* Nếu operator quên đổi `CHANGE_ME` → preflight chặn với thông báo
  `radar_gateway.device còn placeholder CHANGE_ME`.
* Nếu operator bật `radar_gateway.enabled=true` mà đồng thời bật cả
  `side_sensor_modules.enabled=true` → preflight chặn với
  `radar_gateway.enabled=false nhưng radar giữa xe đã được bật qua
  side_sensor_modules hoặc radar block khác`.

## 8. Mapping ID → camera cố định

| Radar ID    | Camera (mặc định) | Bên xe  | Vai trò             |
|-------------|-------------------|---------|---------------------|
| left_front  | `left_1`          | Trái    | Quan sát điểm mù   |
| left_rear   | `left_2`          | Trái    | Lùi xe             |
| right_front | `right_1`         | Phải    | Quan sát điểm mù   |
| right_rear  | `right_2`         | Phải    | Lùi xe             |

Nếu bạn đảo cấu hình camera (ví dụ `right_2` thuộc radar trái), chỉ cần
đổi trường `camera` trong `radar_gateway.radars.<id>.camera`. Selector
vẫn dùng đúng `radar_id` để quyết định gọi camera nào.

## 9. Health & diagnostics

* `system/health.py` thêm heartbeat `radar_gateway` mỗi khi nhận được gói
  hợp lệ. Khi mất heartbeat quá `safety.heartbeat_timeout` → system `DEGRADED`.
* Khi `gateway_ok=false` mà `radar_gateway.enabled=true` → system `ERROR`.
  Live phải dừng để đảm bảo an toàn.
* `system/live_snapshot.py` xuất `radar_gateway` payload (4 radar, link
  status, sequence, packet loss, json errors) kèm `active_camera` để
  dashboard vẽ bảng trạng thái.
* `tools/radar_gateway_monitor.py` cung cấp CLI replay file JSONL, fault
  injection (`esp32_b_power_loss`, `one_ld2410_uart_loss`, `esp32_a_reset`,
  `gateway_usb_unplug_replug`, `corrupt_json`) để smoke-test pipeline.

## 10. Kiểm tra cuối cùng trước khi lên xe

```bash
# 1. Validation config
python -c "from config import validate_config; import yaml; print(validate_config(yaml.safe_load(open('config/jetson_live.yaml'))))"

# 2. Preflight (sẽ mở UART thật ~2s)
python preflight.py --config config/jetson_live.yaml

# 3. Soak test 5 phút
python tools/radar_gateway_monitor.py --soak --duration 5m \
    --device /dev/serial/by-id/usb-ESP32_A_Gateway-if00 \
    --csv logs/soak.csv --json-report logs/soak.json

# 4. Live demo
./start_jetson.sh
```

Nếu preflight PASS và soak không thấy `json_errors > 0` hay `packet_loss >
0.05`, hệ thống đã sẵn sàng cho production.
