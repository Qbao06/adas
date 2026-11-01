# Đấu dây HLK-LD2420 V2.0 với 2 ESP32

> Tài liệu này thay thế sơ đồ LD2410C khi phần cứng thực tế là module có in
> `HLK-LD2420-V2.0` và hàng chân `3V3 GND OT1 RX OT2`.

## Chức năng chân LD2420 V2.0

| Chân module | Chức năng |
|---|---|
| `3V3` | Nguồn 3,0–3,6 V; điển hình 3,3 V |
| `GND` | Mass |
| `OT1` | UART TX, mức logic 0–3,3 V |
| `RX` | UART RX, mức logic 0–3,3 V |
| `OT2` | Ngõ ra phát hiện: HIGH có người, LOW không người |

UART mặc định của firmware LD2420 V2.0 là **115200 baud, 8N1**. `OT2` không
cần nối khi hệ thống đọc dữ liệu khoảng cách qua UART.

## ESP32 A — Gateway, hai radar bên trái

| LD2420 | ESP32 A |
|---|---|
| `left_front.3V3` | Nguồn 3,3 V ổn định |
| `left_front.GND` | GND chung |
| `left_front.OT1` | GPIO16 — RX UART1 |
| `left_front.RX` | GPIO17 — TX UART1 |
| `left_front.OT2` | Không nối |
| `left_rear.3V3` | Nguồn 3,3 V ổn định |
| `left_rear.GND` | GND chung |
| `left_rear.OT1` | GPIO18 — RX UART2 |
| `left_rear.RX` | GPIO19 — TX UART2 |
| `left_rear.OT2` | Không nối |

USB của ESP32 A nối với máy tính/Jetson để gửi JSON tổng hợp bốn radar.

## ESP32 B — Remote, hai radar bên phải

| LD2420 | ESP32 B |
|---|---|
| `right_front.3V3` | Nguồn 3,3 V ổn định |
| `right_front.GND` | GND chung |
| `right_front.OT1` | GPIO16 — RX UART1 |
| `right_front.RX` | GPIO17 — TX UART1 |
| `right_front.OT2` | Không nối |
| `right_rear.3V3` | Nguồn 3,3 V ổn định |
| `right_rear.GND` | GND chung |
| `right_rear.OT1` | GPIO18 — RX UART2 |
| `right_rear.RX` | GPIO19 — TX UART2 |
| `right_rear.OT2` | Không nối |

ESP32 B gửi dữ liệu sang A bằng ESP-NOW. Không nối UART giữa A và B.

## Cảnh báo nguồn

- **Không cấp 5 V vào chân `3V3` của LD2420.**
- Hai module tiêu thụ điển hình khoảng 100 mA tổng. Nên dùng rail 3,3 V ổn
  định có dự phòng dòng và tụ decoupling gần từng radar.
- GND radar và ESP32 phải chung.
- Tắt nguồn trước khi đổi dây.

## Thay đổi firmware bắt buộc

Firmware hiện tại dùng parser LD2410 và `RADAR_BAUD=256000`; không thể chỉ đấu
dây rồi chạy. Trước khi nạp thật cần:

1. đổi `RADAR_BAUD` thành `115200`;
2. thêm parser frame báo cáo HLK-LD2420;
3. cập nhật test protocol/parser;
4. đổi giới hạn khoảng cách phù hợp LD2420;
5. build lại cả `ROLE_GATEWAY` và `ROLE_REMOTE`.

