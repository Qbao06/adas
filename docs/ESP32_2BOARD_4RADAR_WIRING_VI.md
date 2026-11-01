# Đấu dây 2 ESP32 + 4 radar HLK-LD2410C

Áp dụng cho **ESP32-WROOM-32 DevKit / ESP32 Dev Module**. Nếu dùng ESP32-S3,
ESP32-C3 hoặc WROVER có PSRAM, phải kiểm tra lại pinout trước khi đấu.

## Kiến trúc

```text
LD2410C trái trước ─ UART1 ─┐
                            ├─ ESP32 A GATEWAY ─ USB ─ Máy tính/Jetson
LD2410C trái sau   ─ UART2 ─┘          ▲
                                       │ ESP-NOW (không dây)
LD2410C phải trước ─ UART1 ─┐          │
                            ├─ ESP32 B REMOTE
LD2410C phải sau   ─ UART2 ─┘
```

ESP32 A đọc hai radar bên trái, nhận hai radar bên phải qua ESP-NOW rồi xuất
JSON v2 qua USB-UART0. ESP32 B đọc hai radar bên phải và gửi ESP-NOW cho A.
Không nối TX/RX giữa hai ESP32. Khi vận hành chỉ A bắt buộc nối USB với máy
tính/Jetson; USB của B chỉ dùng để nạp firmware hoặc xem log khởi động.

## ESP32 A — Gateway

| Thiết bị | Chân | Nối ESP32 A |
|---|---|---|
| `left_front` | VCC / GND | 5 V ngoài / GND chung |
| `left_front` | TX / RX | GPIO16 (RX1) / GPIO17 (TX1) |
| `left_rear` | VCC / GND | 5 V ngoài / GND chung |
| `left_rear` | TX / RX | GPIO18 (RX2) / GPIO19 (TX2) |
| Máy tính/Jetson | USB | Cổng USB ESP32 A |

## ESP32 B — Remote

| Thiết bị | Chân | Nối ESP32 B |
|---|---|---|
| `right_front` | VCC / GND | 5 V ngoài / GND chung |
| `right_front` | TX / RX | GPIO16 (RX1) / GPIO17 (TX1) |
| `right_rear` | VCC / GND | 5 V ngoài / GND chung |
| `right_rear` | TX / RX | GPIO18 (RX2) / GPIO19 (TX2) |

Quy tắc bắt buộc: **TX radar → RX ESP32; RX radar → TX ESP32**. Không nối
TX–TX hoặc RX–RX.

## Nguồn và mức logic

- Cấp radar bằng nguồn 5 V ổn định; không cấp từ chân 3V3 ESP32.
- GND ESP32 và hai radar trong cùng cụm phải nối chung. Nếu hai cụm dùng chung
  DC-DC thì nối mass nguồn chung.
- UART ESP32 là 3,3 V. Nếu board radar trung gian xuất TX 5 V, phải dùng level
  shifter hoặc chia áp trước chân RX ESP32.
- Không dùng chân `OUT/DETECT`; firmware đọc UART 256000 baud, 8N1.
- Không dùng GPIO0, GPIO2, GPIO12, GPIO15 hoặc GPIO6–11 trong sơ đồ này.

## Cấu hình firmware

Trong `firmware/esp32_radar_gateway/src/hardware_config.h`:

```c
#define BOARD_MODEL                 "ESP32-WROOM-32-DEVKIT"
#define UART1_RX_GPIO               16
#define UART1_TX_GPIO               17
#define UART2_RX_GPIO               18
#define UART2_TX_GPIO               19
#define ESPNOW_CHANNEL              6  // ví dụ; A và B phải giống nhau
```

Đọc **Wi-Fi station MAC thật** của từng ESP32 rồi điền sáu byte
`GATEWAY_MAC_BYTE_*` và `REMOTE_MAC_BYTE_*`. Không dùng MAC ví dụ. Hai board
phải build từ cùng cấu hình MAC và cùng `ESPNOW_CHANNEL`.

## Trình tự kiểm tra

1. Chưa nối radar; cấp nguồn từng ESP32 và đọc MAC Wi-Fi station.
2. Điền GPIO, channel và hai MAC vào `hardware_config.h`.
3. Tắt nguồn rồi mới đấu VCC, GND, TX và RX radar.
4. Nạp A với `ROLE_GATEWAY`, nạp B với `ROLE_REMOTE`.
5. Cấp nguồn B rồi A; mở Serial Monitor của A ở 115200 baud.
6. Xác nhận JSON có `gateway_ok=1`, `remote_ok=1` và đủ bốn radar.
7. Dùng `/dev/serial/by-id/...` của ESP32 A trong `config/jetson_live.yaml`.

`gateway_ok=0`: kiểm tra UART hai radar bên A. `remote_ok=0`: kiểm tra MAC,
kênh ESP-NOW, nguồn B và khoảng cách/anten giữa hai board.

## Checklist

- [ ] Không có dây 5 V nối vào GPIO ESP32.
- [ ] TX radar đi vào RX ESP32; RX radar đi vào TX ESP32.
- [ ] GND radar và ESP32 nối chung.
- [ ] Hai ESP32 dùng cùng ESP-NOW channel.
- [ ] Hai MAC là MAC thật.
- [ ] ESP32 A nối USB máy tính/Jetson; ESP32 B không cần UART sang A.
