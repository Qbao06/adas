# Checklist tự kiểm tra phần cứng (User Self-Test)

Tài liệu này hướng dẫn người dùng **tự kiểm tra nhanh** trước khi chạy live ADAS,
giúp phát hiện sớm các lỗi đấu dây / cấu hình trước khi cần đến dev hỗ trợ.
Mỗi mục có **cách làm**, **kết quả kỳ vọng**, và **cách khắc phục** nếu sai.

> Thời gian dự kiến: 30–45 phút. Chuẩn bị sẵn đồng hồ vạn năng / USB UART
> debug, cáp USB-C, và 1 bản copy cấu hình YAML đang chạy.

---

## 0. Trước khi bắt đầu

- [ ] Đọc qua `ESP32_RADAR_SETUP.md` để biết chân GPIO và sơ đồ UART.
- [ ] In bản copy của file này, đánh dấu vào từng mục khi xong.
- [ ] Tắt nguồn toàn hệ thống trước khi thao tác phần cứng.

---

## 1. Cấp nguồn & logic level

| # | Việc cần làm | Kết quả kỳ vọng | Nếu sai |
|---|---|---|---|
| 1.1 | Đo VCC tại chân **5V** của từng radar khi chưa cắm ESP32 | 5,0 ± 0,25 V DC, dòng ≥ 100 mA | Nguồn yếu/ripple cao → kiểm tra adapter. |
| 1.2 | Đo VCC tại chân **3V3** của ESP32 | 3,30 ± 0,10 V DC | ESP32 cấp sai, có thể cháy; dừng ngay. |
| 1.3 | Đo điện áp trên chân **TX** của radar khi idle (không cắm ESP32) | Cao ≥ 3,0 V (logic HIGH) | Nếu thấp hơn: cáp quá dài hoặc có short; thay cáp. |
| 1.4 | Xác nhận GND của **2 ESP32 + 4 radar + Jetson** đều nối chung | Đo trở kháng GND-GND < 0,5 Ω | Phải hàn nối lại; đây là lỗi phổ biến nhất. |

## 2. Đấu UART ESP32 ↔ Radar

| # | Việc cần làm | Kết quả kỳ vọng | Nếu sai |
|---|---|---|---|
| 2.1 | Đo điện áp chân **UART_RX của ESP32** khi radar **không phát hiện** | ≥ 3,0 V (HIGH = no-data) | ESP32 RX nối nhầm vào 5V → cháy ESP32, dừng. |
| 2.2 | Quan sát Serial Monitor ESP32 A (115200 baud), chạy radar_init | Thấy `[OK] ROLE_GATEWAY ready` | Nếu spam reboot/reset → xem mục 5 (ESP-NOW). |
| 2.3 | Đặt tay trước từng radar trong 5 s, quan sát log `[DBG]` | Radar tương ứng tăng `radar_frames_ok` ≥ 50 | Radar đó có thể hỏng hoặc cáp RX/TX lỏng. |
| 2.4 | Rút cáp UART của 1 radar, đợi >0,6 s | Radar đó `valid=false, stale=true`, các radar khác vẫn OK | Nếu cả 4 radar cùng `stale` → nguồn/GND chung. |

## 3. Mapping vật lý radar ↔ camera

| # | Việc cần làm | Kết quả kỳ vọng | Nếu sai |
|---|---|---|---|
| 3.1 | Mở `tools/radar_gateway_monitor.py --device /dev/ttyACM0 --raw` | Thấy 4 radar `left_front, left_rear, right_front, right_rear` | Nếu thiếu → firmware chưa đọc đúng radar. |
| 3.2 | Dùng tay di chuyển từ trước ra sau cách xe 1 m | Radar `left_front` báo distance_m ≈ 1,0 (±0,2) | Sai vị trí → swap radar trong `apply_radar_gateway`. |
| 3.3 | Tương tự bên phải → `right_front` | Radar phải báo distance hợp lý | Ngược trái/phải → đảo mapping camera. |

## 4. Tầm phát hiện & vùng chết

| # | Việc cần làm | Kết quả kỳ vọng | Nếu sai |
|---|---|---|---|
| 4.1 | Đặt vật ở 0,75 m trước radar | `distance_m` ≈ 0,75 (±0,1) | LD2410C có vùng chết dưới 20 cm; bình thường. |
| 4.2 | Lùi ra 5 m | `distance_m` ≈ 5,0 (±0,2) | Nếu thấp hơn nhiều → cáp dài quá / nhiễu. |
| 4.3 | Lùi ra 6 m | `valid=false, distance=null` | Hết tầm LD2410C. |
| 4.4 | Đặt người đi bộ từ xa đến gần (5 → 1 m, mỗi giây 1 m) | `distance_m` giảm đều, `target_state=moving` | Đứng yên → `target_state=stationary`. |

## 5. ESP-NOW giữa 2 ESP32 (Gateway ↔ Remote)

| # | Việc cần làm | Kết quả kỳ vọng | Nếu sai |
|---|---|---|---|
| 5.1 | Đặt 2 ESP32 cách 2 m, cùng phòng | Gateway in `recv_ok` tăng mỗi giây | MAC sai → sửa `hardware_config.h`. |
| 5.2 | Che 1 mảng kim loại giữa 2 ESP32 | `recv_ok` tụt nhưng `recv_drop` KHÔNG tăng đột biến | Nếu `recv_drop` tăng đột biến → kênh Wi-Fi trùng với router. |
| 5.3 | Rút nguồn ESP32 B | Gateway báo `remote_ok=false`, 2 radar phải `stale`, 2 radar trái vẫn OK | Gateway mất cả 4 radar → bug firmware, mở issue. |
| 5.4 | Đặt ESP32 B lên xe, di chuyển 50 m LOS | Trong 10 s mất tối đa 1-2 gói | Qúa 50 % loss → kiểm tra antenna đối xứng, kênh Wi-Fi trống. |

## 6. Nhiễu khi 4 radar đặt gần nhau

| # | Việc cần làm | Kết quả kỳ vọng | Nếu sai |
|---|---|---|---|
| 6.1 | Đặt 4 radar trong phạm vi 0,3 m (đặt gần nhau) | Không có radar báo "DETECT" ma | Nếu có → cáp xoắn đôi, tách xa nguồn công suất. |
| 6.2 | Che từng radar bằng vải dày | Radar đó `valid=false` ngay | Nếu vẫn "moving" → radar vẫn nhìn xuyên vải (kỳ vọng); không phải bug. |
| 6.3 | Che radar bằng tấm kim loại | Radar đó `valid=false` ngay (~100 ms) | Nếu vẫn "moving" → bug, kiểm tra kết nối UART. |

## 7. Vật liệu che

| Vật liệu | Kỳ vọng | Ghi chú |
|---|---|---|
| Nhựa / gỗ khô | Xuyên qua, phát hiện OK | LD2410C dùng sóng 24 GHz, bị hấp thụ bởi nước. |
| Kính cường lực | Xuyên qua (có thể suy hao) | |
| Kim loại đặc | Phản xạ / che hoàn toàn | |
| Nước / cơ thể người | Hấp thụ mạnh, giảm khoảng cách | |
| Mưa nhỏ | Tăng nhiễu ~10 % packet loss | Chấp nhận được, không cần đổi firmware. |

## 8. Nguồn/nhiệt khi chạy nhiều giờ

| # | Việc cần làm | Kết quả kỳ vọng | Nếu sai |
|---|---|---|---|
| 8.1 | Để toàn hệ thống chạy 4 h trong nhà 25 °C | Không có ESP32 reset / radar mất frame | Reset → nguồn yếu hoặc ESP32 quá nóng. |
| 8.2 | Đo nhiệt ESP32 bằng camera ảnh nhiệt | ESP32-A: ≤ 50 °C; ESP32-B: ≤ 55 °C | Vượt quá → thêm tản nhiệt. |
| 8.3 | Đo VCC tại 5V sau 4 giờ | Vẫn ≥ 4,85 V | Adapter yếu, thay adapter ≥ 3 A. |
| 8.4 | Chạy `--soak` 30 phút | `packet_loss` trung bình < 2 % | Loss cao → check 5.4. |

## 9. Kiểm tra cuối

- [ ] Preflight live pass: `python3 preflight.py --live --config config/jetson_live.yaml`
- [ ] Dashboard `radar/status` trả `gateway_ok=true, remote_ok=true`
- [ ] Thử lái thử trong bãi đỗ xe 5 phút: camera selector chuyển đúng hướng.

---

## Liên hệ hỗ trợ

Sau khi hoàn thành checklist và **vẫn còn lỗi**, mở issue kèm:
1. Output của `python3 tools/radar_gateway_monitor.py --device /dev/ttyACM0 --raw --duration 30`
2. Log Serial từ ESP32 A (115200 baud) trong 30 giây.
3. Sơ đồ đấu dây với điện áp đo được ở chân 5V/3V3/GND.
4. Ảnh chụp ESP32 + 4 radar thực tế.
