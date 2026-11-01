# Bộ 5 prompt hoàn thiện hệ thống 2 ESP32 và 4 radar HLK LD2410C

## Kiến trúc phần cứng đã chốt

```text
LD2410C left_front  -- UART1 --\
LD2410C left_rear   -- UART2 --- ESP32 A Gateway -- USB Serial -- Jetson
                                      ^
                                      | ESP-NOW
LD2410C right_front -- UART1 --\      |
LD2410C right_rear  -- UART2 --- ESP32 B Remote
```

- Firmware ESP32 được viết bằng Arduino framework.
- Mỗi ESP32 đọc hai radar bằng Hardware UART1 và Hardware UART2.
- ESP32 B gửi hai radar bên phải đến ESP32 A bằng ESP-NOW.
- ESP32 A gộp bốn radar và gửi một luồng JSON qua USB Serial cho Jetson.
- HLK-LD2410C là radar hiện diện người 24 GHz, tầm phát hiện khoảng 5 m, chia vùng khoảng 0,75 m.
- LD2410C không phải radar điểm mù ô tô chuyên dụng; chỉ dùng cho prototype, thử nghiệm tốc độ thấp và hỗ trợ chuyển camera.
- Chạy các prompt theo thứ tự 1 đến 5. Chỉ chuyển prompt tiếp theo khi prompt hiện tại build và test đạt.

## Thông tin người dùng phải tự cung cấp

- Model board ESP32 cụ thể.
- GPIO RX và TX của UART1, UART2 trên từng ESP32.
- MAC Wi-Fi của ESP32 A và ESP32 B.
- Điện áp nguồn và mức logic UART đo được trên board LD2410C thực tế.
- Đường dẫn `/dev/serial/by-id/...` của ESP32 A trên Jetson.
- Mapping vị trí thật của bốn radar.

---

# Prompt 1  Kiểm tra dự án và chuẩn hóa protocol

```text
Đọc toàn bộ workspace dự án ADAS trước khi sửa, bao gồm README, config, sensors,
firmware, app/pipeline.py, dashboard, preflight.py, tests, drivers và các báo cáo
review. Không xóa code đang hoạt động và không tạo lại dự án từ đầu.

Mục tiêu của task này là chuẩn hóa kiến trúc và protocol cho hệ thống:

- 4 cảm biến HLK-LD2410C.
- 2 ESP32 dùng Arduino framework.
- ESP32 A là Gateway.
- ESP32 B là Remote.
- Mỗi ESP32 đọc hai LD2410C bằng Hardware UART1 và Hardware UART2.
- ESP32 B gửi dữ liệu sang ESP32 A bằng ESP-NOW.
- ESP32 A gửi dữ liệu bốn radar cho Jetson bằng USB Serial UART0.
- Jetson chỉ mở một cổng serial cho bốn radar.

Mapping mặc định:

- ESP32 A UART1: left_front.
- ESP32 A UART2: left_rear.
- ESP32 B UART1: right_front.
- ESP32 B UART2: right_rear.

Thông tin cảm biến:

- HLK-LD2410C là cảm biến hiện diện người 24 GHz.
- Tầm phát hiện sử dụng trong dự án khoảng 5 m.
- Khoảng cách được chia thành các vùng khoảng 0,75 m.
- UART radar mặc định thường là 256000 baud, 8N1; phải cho phép cấu hình.
- Không được coi LD2410C là radar ô tô chuyên dụng.
- Không dùng dữ liệu radar này để tự động phanh hoặc điều khiển xe.

Hãy kiểm tra firmware hiện tại và xác định:

1. Phần nào có thể giữ lại.
2. Phần nào đang giả định radar chỉ có chân OUT digital.
3. Phần nào phải chuyển sang parser UART nhị phân LD2410C.
4. Code Jetson hiện đang yêu cầu bao nhiêu cổng UART.
5. Những nơi phải đổi sang một Radar Gateway duy nhất.

Tạo đặc tả protocol ESP-NOW dùng chung. Packet phải có:

- magic number;
- protocol_version;
- node_id;
- boot_id hoặc session_id;
- sequence;
- timestamp_ms;
- trạng thái hai radar;
- checksum hoặc CRC.

Mỗi radar trong packet phải có:

- radar_id;
- target_state;
- detected;
- valid;
- stale;
- moving_distance_cm;
- moving_energy;
- stationary_distance_cm;
- stationary_energy;
- detection_distance_cm;
- error_code;
- sample_age_ms.

Dùng kiểu integer có kích thước rõ ràng. Không dùng String trong packet ESP-NOW.
Không phụ thuộc padding struct. Thêm static_assert kiểm tra kích thước packet.

Chuẩn hóa JSON ESP32 A gửi Jetson theo protocol v2. Mỗi object nằm trên một dòng
và kết thúc bằng newline. JSON luôn có đủ bốn radar và gồm:

- v;
- seq;
- gateway_ok;
- remote_ok;
- gateway_boot_id;
- remote_boot_id;
- packet_loss;
- radars.left_front;
- radars.left_rear;
- radars.right_front;
- radars.right_rear.

Mỗi radar JSON có detected, target_state, distance_m, moving_distance_m,
moving_energy, stationary_distance_m, stationary_energy, valid, stale, age_ms
và error. Dữ liệu không hợp lệ phải là null, không được tạo số 0 giả.

Tạo hoặc cập nhật tài liệu protocol. Nếu workspace chưa có tài liệu byte protocol
LD2410C chính xác, không tự sáng tác cấu trúc frame. Hãy tạo interface parser,
MockRadarDriver và stub RealRadarDriver, đồng thời ghi rõ tài liệu còn thiếu.

Thêm test cho protocol ESP-NOW: checksum, sai magic, sai version, sai kích thước,
packet cũ, sequence lặp và sequence wraparound.

Cuối task:

- Liệt kê file đã tạo hoặc sửa.
- Giải thích kiến trúc vừa chuẩn hóa.
- Liệt kê thông tin người dùng còn phải cung cấp.
- Chạy test liên quan và báo pass/fail.
- Không thực hiện Prompt 2.
```

---

# Prompt 2  Hoàn thiện firmware Arduino cho hai ESP32

```text
Đọc toàn bộ workspace và đặc tả protocol sau Prompt 1. Không thay đổi protocol
nếu không có lý do kỹ thuật rõ ràng.

Mục tiêu: hoàn thiện firmware Arduino cho ESP32 A Gateway và ESP32 B Remote.
Ưu tiên một codebase dùng cấu hình build để chọn ROLE_GATEWAY hoặc ROLE_REMOTE.

Tổ chức firmware thành các module rõ ràng:

- hardware_config;
- ld2410_uart_driver;
- ld2410_parser;
- radar_state;
- espnow_protocol;
- gateway_aggregator;
- jetson_json_transport;
- diagnostics;
- mock_mode.

Không viết toàn bộ logic trong một file .ino lớn.

Yêu cầu driver LD2410C:

- Dùng HardwareSerial cho UART1 và UART2.
- Baudrate mặc định 256000, cho phép đổi trong config.
- Mỗi UART có parser và buffer độc lập.
- Xử lý frame bị chia nhỏ, nhiều frame dính nhau và byte rác.
- Kiểm tra header, footer và độ dài frame.
- Tự tìm lại header sau frame lỗi.
- Giới hạn kích thước buffer để tránh tràn RAM.
- Không dùng SoftwareSerial.
- Không dùng delay dài.
- Không để một radar lỗi làm radar còn lại ngừng hoạt động.
- Radar không có frame mới quá 300 ms phải valid=false và stale=true.

Nếu tài liệu protocol đã có, parse ít nhất:

- target state;
- moving target distance;
- moving target energy;
- stationary target distance;
- stationary target energy;
- detection distance.

Không tự tạo khoảng cách nếu frame lỗi. Không coi distance=0 là dữ liệu hợp lệ
nếu protocol thể hiện không có mục tiêu.

ROLE_REMOTE trên ESP32 B:

- UART1 đọc right_front.
- UART2 đọc right_rear.
- Gửi packet ESP-NOW unicast đến đúng MAC của ESP32 A ở khoảng 20 Hz.
- Có sequence, boot_id, timestamp và checksum.
- Xác nhận kết quả gửi bằng callback.
- Theo dõi send_ok, send_fail và packet_drop.
- Có watchdog và xử lý millis wraparound.
- Một radar offline không làm radar còn lại offline.

ROLE_GATEWAY trên ESP32 A:

- UART1 đọc left_front.
- UART2 đọc left_rear.
- Nhận ESP-NOW từ đúng MAC ESP32 B.
- Kiểm tra MAC nguồn, magic, version, length, checksum, boot_id và sequence.
- Bỏ packet lặp hoặc packet cũ.
- Chấp nhận sequence mới khi ESP32 B reboot với boot_id mới.
- Remote mất quá 600 ms thì chỉ right_front và right_rear thành stale.
- Gộp bốn radar và gửi JSON protocol v2 qua USB Serial.
- Phát JSON khoảng 20 Hz mà không block UART reader.
- Debug log không được trộn vào luồng JSON production.

Các giá trị sau phải nằm trong file config tập trung:

- GPIO UART1 RX/TX;
- GPIO UART2 RX/TX;
- baudrate hai radar;
- USB Serial baudrate;
- MAC Gateway;
- MAC Remote;
- ESP-NOW Wi-Fi channel;
- radar mapping;
- radar timeout;
- remote timeout;
- send rate.

Nếu MAC hoặc GPIO chưa được người dùng cung cấp, dùng placeholder rõ ràng.
Production build phải báo lỗi hoặc preflight firmware phải cảnh báo nếu còn
placeholder. Không rải placeholder trong nhiều file.

Tạo README hướng dẫn Arduino:

1. Cài ESP32 board package.
2. Chọn đúng board.
3. Lấy MAC của từng ESP32.
4. Điền MAC peer.
5. Điền GPIO UART1 và UART2.
6. Build và upload ROLE_GATEWAY.
7. Build và upload ROLE_REMOTE.
8. Mở Serial Monitor.
9. Bật mock mode.
10. Kiểm tra JSON từ Gateway.

Thêm test cho parser LD2410C và packet ESP-NOW. Nếu Arduino framework hiện tại
không có test runner, tách parser thành code C++ thuần có thể test trên host.

Build cả ROLE_GATEWAY và ROLE_REMOTE. Không báo hoàn thành nếu một role chưa
build. Cuối task báo file đã sửa, kết quả build, test pass/fail và placeholder
người dùng cần điền. Không thực hiện Prompt 3.
```

---

# Prompt 3  Tích hợp Radar Gateway vào chương trình Jetson

```text
Đọc toàn bộ dự án ADAS, firmware và protocol v2 sau Prompt 2.

Mục tiêu: sửa chương trình Jetson để nhận bốn LD2410C qua một cổng USB Serial
của ESP32 A Gateway. Không thay đổi luồng camera trước và LiDAR fusion.

Hiện dự án có thể đang dùng side_sensor_modules theo kiểu mỗi radar có một UART.
Hãy bổ sung schema mới radar_gateway và giữ compatibility mode cho cấu hình cũ.

Schema đề xuất:

radar_gateway:
  enabled: true
  device: /dev/serial/by-id/CHANGE_ME
  baudrate: 115200
  protocol_version: 2
  stale_after_seconds: 0.6
  reconnect_seconds: 2.0
  camera_hold_seconds: 1.2
  radars:
    left_front:
      enabled: true
      camera: left_1
      warning_distance_m: 5.0
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

Tạo RadarGatewayReader:

- Chạy trong background thread.
- Không block pipeline.
- Dùng buffer giới hạn.
- Giới hạn mỗi dòng JSON tối đa 4096 byte.
- Validate protocol version và kiểu dữ liệu từng field.
- Reject NaN, Infinity, khoảng cách âm và JSON quá dài.
- Một dòng JSON lỗi không làm reader dừng.
- Theo dõi sequence, packet lặp, packet mất và Gateway reboot.
- Tự reconnect khi ESP32 reset hoặc rút USB.
- Không dùng dữ liệu stale.
- Một radar lỗi không làm mất ba radar còn lại.
- remote_ok=false chỉ làm right_front và right_rear offline.
- gateway_ok=false phải báo health error.
- Live tuyệt đối không fallback sang mock radar.

Cập nhật camera selector:

- Radar detected=true thì chuyển sang camera đã mapping.
- Nếu nhiều radar cùng detected và có distance hợp lệ, chọn radar gần nhất.
- Nếu distance đều null, giữ camera hiện tại nếu nó vẫn thuộc radar đang detected.
- Nếu phải chọn mới, dùng danh sách priority trong config, không dựa vào thứ tự dict.
- Giữ camera theo camera_hold_seconds để tránh nhấp nháy.
- Khi hết phát hiện hoặc mất Gateway, trở về camera front.
- Mất Remote không ảnh hưởng radar và camera bên Gateway.
- Camera side chỉ dùng để hiển thị; camera trước vẫn dùng cho LiDAR calibration.

Cập nhật:

- config loader;
- config validation;
- pipeline;
- health monitor;
- diagnostics;
- live snapshot;
- camera selector;
- tài liệu migration từ cấu hình UART cũ.

Validation phải kiểm tra:

- device không trống khi enabled;
- baudrate hợp lệ;
- protocol production là v2;
- timeout lớn hơn 0;
- radar ID hợp lệ;
- camera mapping tồn tại;
- warning_distance_m lớn hơn 0 và không vượt quá khoảng 5 m cấu hình cho LD2410C;
- không ghi đè các phần YAML không liên quan.

Thêm test:

- JSON v2 hợp lệ;
- JSON lỗi cú pháp;
- JSON quá dài;
- thiếu radar;
- sai field type;
- version sai;
- distance âm;
- NaN và Infinity;
- sequence lặp;
- packet loss;
- Gateway reboot;
- Gateway stale;
- Remote stale;
- một radar stale;
- detected=true nhưng distance=null;
- chọn camera gần nhất;
- camera hold;
- mất Gateway trở về front;
- reconnect sau reset;
- migration config cũ;
- Live không fallback mock.

Chạy toàn bộ test dự án. Không sửa test để che lỗi. Cuối task báo danh sách file,
kết quả test và blocker. Không thực hiện Prompt 4.
```

---

# Prompt 4  Hoàn thiện dashboard cấu hình cổng và preflight

```text
Đọc toàn bộ workspace sau Prompt 3.

Mục tiêu: cập nhật dashboard và preflight cho kiến trúc một ESP32 Gateway quản
lý bốn radar HLK-LD2410C.

Dashboard phải cho phép:

- bật hoặc tắt Radar Gateway;
- chọn device Gateway;
- nhập baudrate USB Serial;
- chọn protocol version;
- chỉnh stale timeout;
- chỉnh camera hold time;
- mapping bốn radar sang camera;
- bật hoặc tắt từng radar;
- đặt warning distance không vượt quá khoảng 5 m;
- lưu cấu hình atomically;
- thông báo restart_required sau khi lưu.

Dashboard phải hiển thị:

- Gateway online/offline;
- Remote online/offline;
- Gateway boot_id;
- Remote boot_id;
- sequence;
- packet rate;
- packet loss;
- JSON hoặc CRC errors;
- tuổi packet;
- detected, distance, moving energy, stationary energy, valid, stale và error
  của từng radar.

Thêm API read-only để liệt kê:

- /dev/serial/by-id/*;
- /dev/ttyACM*;
- /dev/ttyUSB*;
- /dev/v4l/by-id/*;
- /dev/video*.

Yêu cầu bảo mật:

- Chỉ chấp nhận device trong allowlist /dev.
- Chống path traversal.
- Không cho đọc file tùy ý.
- Không thực thi shell command từ dữ liệu client.
- Validate toàn bộ payload.
- Demo không được truy cập phần cứng thật.
- Không ghi đè phần YAML không liên quan.

Nâng cấp nút Kiểm tra thiết bị:

- Thực sự mở camera và đọc ít nhất một frame.
- Mở UART Gateway và chờ ít nhất một JSON protocol v2 hợp lệ.
- Xác nhận sequence đang tăng.
- Xác nhận đủ bốn radar enabled xuất hiện trong JSON.
- Phân biệt Gateway offline và Remote offline.
- Phân biệt lỗi device không tồn tại, thiếu quyền, cổng bận, sai baudrate,
  timeout, JSON lỗi và version sai.
- Đóng UART và camera sạch sẽ sau kiểm tra.

Cập nhật preflight.py:

- Luôn yêu cầu camera front trong Live.
- Chỉ yêu cầu camera side đang enabled.
- Kiểm tra cổng Gateway tồn tại và có quyền đọc/ghi.
- Nhận ít nhất một JSON v2 hợp lệ.
- Kiểm tra sequence tăng.
- Gateway bắt buộc online.
- Remote bắt buộc online nếu radar bên Remote đang enabled.
- Không cho Live pass nếu còn device CHANGE_ME.
- Không cho Live dùng dữ liệu radar mock.
- Hiển thị lỗi tiếng Việt rõ ràng.

Không làm yếu các kiểm tra hiện có đối với CUDA, TensorRT model, camera, LiDAR
driver thật, calibration thật và chính sách cấm mock trong Live.

Nếu model TensorRT, driver LiDAR hoặc calibration thật chưa tồn tại, báo blocker;
không tạo file giả để preflight pass.

Thêm test dashboard API, config validation, path traversal, YAML atomic save,
UART test giả lập và preflight. Chạy toàn bộ test rồi báo pass/fail. Không thực
hiện Prompt 5.
```

---

# Prompt 5  Kiểm thử tích hợp và hoàn thiện bàn giao

```text
Đọc toàn bộ workspace sau Prompt 4. Không thay đổi kiến trúc đã chốt.

Mục tiêu: hoàn thiện công cụ kiểm thử, tài liệu và báo cáo bàn giao. Không được
tuyên bố hệ thống an toàn cho xe chỉ dựa trên mock test.

Tạo công cụ CLI:

python tools/radar_gateway_monitor.py \
  --device /dev/serial/by-id/... \
  --baudrate 115200

Công cụ phải hiển thị:

- Gateway và Remote online/offline;
- bốn radar;
- sequence;
- packet rate;
- packet loss;
- stale time;
- target state;
- moving distance và energy;
- stationary distance và energy;
- detection distance;
- JSON errors;
- reconnect count.

Hỗ trợ tham số:

- --duration;
- --csv;
- --json-report;
- --raw;
- --no-color.

Tạo chế độ mapping test:

1. Yêu cầu người dùng kích hoạt từng radar theo thứ tự.
2. Hiển thị radar ID đang phản ứng.
3. Người dùng tự xác nhận vị trí vật lý.
4. Không tự kết luận mapping nếu chưa được xác nhận.

Tạo soak test theo thời lượng CLI:

- theo dõi packet rate và packet loss;
- theo dõi reconnect;
- theo dõi CPU và RAM;
- theo dõi nhiệt độ Jetson nếu có;
- xuất CSV và JSON report;
- không giữ log vô hạn trong RAM.

Tạo fault-injection test hoặc hướng dẫn test cho:

- ESP32 B mất nguồn;
- ESP32 A reset;
- một LD2410C mất UART;
- ESP-NOW mất packet;
- JSON bị hỏng;
- camera side bị rút;
- Gateway USB bị rút và cắm lại.

Kết quả an toàn mong đợi:

- Không tạo detection giả.
- Radar lỗi phải valid=false và stale=true.
- Mất Remote chỉ ảnh hưởng hai radar Remote.
- Mất Gateway làm camera trở về front.
- Jetson tự reconnect khi Gateway xuất hiện lại.
- Pipeline không crash vì một radar lỗi.

Cập nhật tài liệu triển khai Arduino và Jetson:

- sơ đồ đấu nối;
- cảnh báo UART ESP32 chỉ chịu logic 3,3 V;
- cách cấp nguồn bốn radar;
- cách lấy MAC hai ESP32;
- cấu hình ESP-NOW peer và channel;
- GPIO UART1 UART2;
- build và upload từng role trong Arduino IDE hoặc PlatformIO;
- Serial Monitor;
- xác định /dev/serial/by-id;
- thêm user vào nhóm dialout và video;
- chạy radar monitor;
- chạy preflight;
- chạy ADAS Live headless;
- xử lý Gateway hoặc Remote offline.

Tạo checklist người dùng phải tự làm:

- đo mức logic TX của radar;
- nối chung GND;
- cấp nguồn đúng datasheet;
- xác nhận mapping bốn radar;
- đo tầm phát hiện thực tế;
- xác nhận vùng khoảng 0,75 m;
- kiểm tra góc quét;
- kiểm tra nhiễu giữa bốn radar;
- kiểm tra vật liệu vỏ che radar;
- kiểm tra ESP-NOW khi lắp trên xe;
- kiểm tra nguồn, nhiệt và mất gói nhiều giờ.

Chạy cuối cùng:

1. Syntax/static checks.
2. Toàn bộ unit test Python.
3. Integration test không cần phần cứng.
4. Build ROLE_GATEWAY.
5. Build ROLE_REMOTE.
6. Kiểm tra config không còn placeholder rải rác.

Báo cáo cuối phải có:

- kiến trúc sau khi sửa;
- file tạo mới;
- file đã sửa;
- kết quả build hai firmware;
- tổng test pass/fail;
- lệnh build và upload;
- lệnh lấy MAC;
- lệnh kiểm tra UART;
- lệnh radar monitor;
- lệnh preflight;
- lệnh chạy Live;
- placeholder người dùng phải điền;
- blocker TensorRT, LiDAR và calibration;
- việc người dùng phải kiểm tra trên phần cứng;
- hạn chế của HLK-LD2410C.

Chỉ tuyên bố hoàn thành phần code khi hai firmware build thành công, toàn bộ test
phần mềm đạt, dashboard và preflight cùng dùng schema mới, Live không fallback
mock và Jetson chỉ cần một UART Gateway.

Không tuyên bố hệ thống hoàn toàn sẵn sàng chạy trên phương tiện khi chưa kiểm
tra phần cứng, nguồn, ESP-NOW, camera, LiDAR, model TensorRT và calibration thật.
```

## Thứ tự sử dụng

1. Sao lưu hoặc commit dự án.
2. Chạy Prompt 1 và kiểm tra báo cáo.
3. Chạy Prompt 2, nạp thử hai ESP32.
4. Chạy Prompt 3, kiểm tra unit test Jetson.
5. Chạy Prompt 4, kiểm tra dashboard và preflight.
6. Chạy Prompt 5, sau đó mới kiểm thử phần cứng toàn hệ thống.

