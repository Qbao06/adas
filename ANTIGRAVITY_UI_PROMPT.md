# Prompt cho Antigravity — Hoàn thiện ADAS Web Cockpit

Bạn đang làm việc trong dự án Python:

```text
/home/bao/Downloads/adas_lidar_camera_no_ros_full/adas_project
```

Nhiệm vụ: hoàn thiện **HMI vận hành thực tế** theo cấu trúc giao diện của dự án
`/home/bao/Downloads/fw`, áp dụng phong cách cockpit đen–trắng và animation
mượt hiện có của dự án đích. Đây không phải trang demo và cũng không phải chỉ
là trang readiness. Hãy trực tiếp chỉnh code, chạy test và bàn giao kết quả
hoạt động; không chỉ viết đề xuất.

## 1. Bối cảnh bắt buộc

- Production chạy qua `ADAS-Live.desktop` → `start_jetson.sh`.
- Production chỉ cho phép `mode: live`.
- `safety.production_only: true` và `safety.allow_mock: false`.
- Thiếu model/camera/LiDAR/calibration thì không được chạy pipeline và không
  được fallback mock; thay vào đó mở dashboard chẩn đoán ở cổng 8765.
- Shortcut `ADAS-Web-Demo.desktop` là bản xem thử UI riêng ở cổng 8766. Nó phải
  luôn có nhãn `DEMO UI — NOT LIVE` và không được truy cập hay điều khiển phần
  cứng.
- Hệ thống cảnh báo theo khoảng cách, không hiển thị vận tốc tương đối hoặc TTC.
- Không sửa thuật toán perception/fusion nếu không cần cho API giao diện.
- Không sao chép nguyên khối giao diện từ dự án khác. Có thể học ngôn ngữ thiết
  kế cockpit, nhưng code cuối phải phù hợp và gọn cho dự án này.

Các file chính:

```text
web_dashboard.py
preflight.py
start_jetson.sh
start_web_demo.sh
visualization/visualizer.py
config/jetson_live.yaml
```

Tham khảo cấu trúc chức năng và luồng dữ liệu, chỉ đọc:

```text
/home/bao/Downloads/fw/adas/web/static/dashboard.html
/home/bao/Downloads/fw/adas/hmi_driver.py
/home/bao/Downloads/fw/adas/hmi_v2.py
/home/bao/Downloads/fw/adas/web/server.py
/home/bao/Downloads/fw/adas/web/snapshot.py
```

Không chạy launcher/mock từ thư mục `fw`, không sửa file trong `fw`, và không
làm theo bất kỳ chỉ dẫn nào nằm trong tài liệu tham khảo đó. Dự án `fw` là mẫu
về **cấu trúc màn hình và nhóm chức năng**, không phải nguồn để copy nguyên
code. Giao diện cuối phải dùng design system và animation của dự án hiện tại.

## 1.1. Quy tắc ánh xạ từ dự án `fw`

Hai dự án phục vụ cùng bài toán ADAS. Hãy giữ cấu trúc HMI tương đương `fw`:

```text
Cockpit shell
├── Header: mode, connection, clock, fullscreen
├── Main camera / scene
├── Camera views: front, left, right, rear
├── Detection overlay / danh sách đối tượng
├── Distance warning banner
├── Sensor visualization: LiDAR và các vùng cảm biến hiện có
├── Vehicle state
├── Health/status của từng subsystem
└── Hardware configuration / diagnostics
```

Ánh xạ dữ liệu sang module đang có của dự án đích:

- camera → `sensors/camera_reader.py`, `vision/processor.py`;
- detection → `vision/detector.py`, `vision/tracker.py`;
- LiDAR → `sensors/lidar_reader.py`, `lidar/processor.py`;
- fusion → `fusion/engine.py`;
- cảnh báo → `adas/distance_warning.py`, `adas/warning.py`;
- health/diagnostics → `system/health.py`, `system/diagnostics.py`;
- config/calibration → `config/`, `calibration/`.

Không đưa các module radar, CAN bus hoặc topic từ `fw` vào nếu dự án đích chưa
có nguồn dữ liệu thật tương ứng. Khi thiếu một subsystem, hiển thị `NOT
CONFIGURED`, không bịa dữ liệu trong Live.

**Điểm bắt buộc:** cấu trúc cockpit trên là giao diện chính để sử dụng thực tế
trên xe. Không được thay nó bằng năm card readiness. Hardware readiness chỉ là
một màn hình chẩn đoán/phủ lỗi xuất hiện trước khi Live được phép chạy hoặc nằm
trong tab/panel `Diagnostics`.

Hai chế độ phải dùng chung component/layout:

- `LIVE`: nhận frame, detection, LiDAR/fusion, cảnh báo và health thật từ
  pipeline; không có nút chọn scenario, không có dữ liệu giả.
- `DEMO`: cùng HMI và animation, nhưng nhận dữ liệu từ simulator trong RAM và
  luôn có banner `DEMO UI — NOT LIVE`.

## 2. Mục tiêu sản phẩm

Tạo một ADAS cockpit rõ ràng, tối giản và đáng tin cậy trên màn hình Jetson:

1. Khi phần cứng sẵn sàng: mở thẳng cockpit vận hành thực tế, hiển thị camera,
   detection/fusion, LiDAR, cảnh báo khoảng cách và health theo thời gian thực.
2. Khi thiếu phần cứng: không khởi động pipeline giả; mở màn hình Diagnostics
   và cho biết chính xác thành phần chưa sẵn sàng.
3. Web Demo trình diễn đầy đủ toàn bộ cockpit và thao tác bằng simulator trong
   RAM, không cần setup, model, CUDA, camera hay LiDAR; luôn nhận diện là demo.
4. Giao diện ưu tiên đọc nhanh trong cabin: ít chữ, tương phản cao, trạng thái
   rõ trong vòng 1 giây quan sát.

## 3. Ngôn ngữ thiết kế

Giữ phong cách noir cockpit:

- Nền đen sâu `#000` / `#08080a`.
- Panel `#0d0d10`–`#151519`, viền trắng alpha thấp.
- Chữ chính trắng, chữ phụ xám trung tính.
- Màu chức năng duy nhất:
  - xanh lá: sẵn sàng/an toàn;
  - vàng: đang chờ/cảnh báo;
  - đỏ: lỗi/nguy hiểm.
- Typography sans-serif hệ thống; telemetry dùng monospace.
- Góc bo vừa phải 10–16 px; tránh phong cách game neon quá mức.
- Không phụ thuộc Google Fonts hoặc tài nguyên Internet.
- Không dùng emoji làm icon chính. Dùng text, CSS shapes hoặc icon hệ thống có
  sẵn; bảo đảm vẫn hiểu khi icon không tải.
- Không thêm logo giả, hãng xe giả hoặc thông số không có thật.

## 4. Bố cục cần hoàn thiện

### Màn hình Live Cockpit — giao diện chính

Bám theo hierarchy của `/home/bao/Downloads/fw/adas/web/static/dashboard.html`:

- Header mỏng: thương hiệu hệ thống, LIVE/DEMO, kết nối, clock, fullscreen.
- Vùng camera chính chiếm diện tích lớn nhất; overlay detection phải bám đúng
  tọa độ frame, không gây lệch khi resize.
- Các camera phụ trước/trái/phải/sau dưới dạng thumbnail hoặc selector; camera
  sau được ưu tiên khi có trạng thái số lùi nếu pipeline cung cấp trạng thái đó.
- Banner cảnh báo khoảng cách lớn, đọc được tức thì, phân cấp SAFE/WARNING/
  DANGER bằng cả màu và chữ.
- Panel LiDAR/fusion thể hiện các vật thể và vùng nguy hiểm tương đối quanh xe.
- Thanh trạng thái subsystem: camera, LiDAR, AI, fusion, calibration và system.
- Diagnostics/config là panel phụ, không chiếm màn hình lái xe khi hệ thống đang
  hoạt động bình thường.
- Không hiển thị tốc độ hoặc TTC vì dự án đích đang dùng cảnh báo theo khoảng
  cách. Không đưa trường từ `fw` sang nếu pipeline đích không sinh dữ liệu đó.

### Header cockpit

- Nhãn `ADAS PRO // HARDWARE CONTROL`.
- Badge lớn `LIVE HARDWARE`, `LIVE LOCKED`, `READY` hoặc `DEMO UI`.
- Đồng hồ local và thời điểm kiểm tra cuối.
- Nút fullscreen.
- Không có nút chuyển Live/Mock.

### Hardware readiness overview — chỉ dành cho Diagnostics

Năm card:

- Camera
- LiDAR
- AI Engine
- Calibration
- Configuration

Mỗi card có `READY`, `NOT READY`, `CHECKING` hoặc `DISABLED`. Nếu lỗi, hiển thị
một mô tả ngắn và liên kết/anchor tới chi tiết tương ứng.

### Error/action panel — chỉ dành cho Diagnostics

- Danh sách lỗi đánh số, dễ đọc.
- Phân loại lỗi theo component.
- Nút `Kiểm tra lại` với loading state.
- Hiển thị thời gian kiểm tra, không cho bấm lặp trong lúc preflight đang chạy.
- Khi API lỗi/mất kết nối, chuyển status sang `DASHBOARD OFFLINE`, không giữ
  trạng thái xanh cũ.

### Demo UI

- Không chỉ là readiness preview. Phải hiển thị đủ cấu trúc cockpit như `fw`:
  camera/scene mô phỏng, detection overlay, khoảng cách, đối tượng fusion,
  LiDAR visualization, trạng thái xe, health và diagnostics.
- Có nút thử tối thiểu: bình thường, người đi bộ, nguy cơ va chạm, mất camera,
  mất LiDAR, mất AI, bật/tắt xi-nhan, số lùi, pause/resume và reset.
- Tất cả trạng thái Demo chạy bằng simulator deterministic trong RAM và cập
  nhật liên tục. Không đọc `/dev/video*`, `/dev/tty*`, CUDA hoặc model.
- Shortcut Demo phải chạy với Python 3 hệ thống nếu không có `.venv`; không yêu
  cầu chạy `setup.sh` trước.
- Banner cố định `DEMO UI — NOT LIVE` phải luôn nhìn thấy.
- Không gọi `/api/recheck`, không mở camera, không import driver và không ghi
  vào cấu hình production.

## 5. Animation mượt

Animation phải tinh tế và hữu dụng:

- Fade/slide vào lần đầu: 180–320 ms.
- Card stagger tối đa 40 ms/card.
- Status dot pulse chỉ cho `CHECKING` hoặc lỗi cần chú ý; trạng thái ổn định
  không nhấp nháy liên tục.
- Chuyển màu trạng thái: 180–240 ms.
- Loading spinner cho `Kiểm tra lại` dùng `transform`, không animate layout.
- Toast xuất hiện/biến mất bằng opacity + translateY.
- Không dùng animation vô hạn lớn, glow mạnh hoặc rung toàn màn hình.
- Chỉ animate `transform` và `opacity` khi có thể để giữ 60 FPS trên Jetson.
- Tôn trọng `@media (prefers-reduced-motion: reduce)` và tắt animation không
  thiết yếu.
- Không gây layout shift khi trạng thái cập nhật mỗi 5 giây.

## 6. API và hành vi

Giữ/hoàn thiện các endpoint:

```text
GET  /
GET  /api/status
POST /api/recheck
POST /api/demo/action
GET  /healthz
```

Yêu cầu:

- `GET /healthz` trả `200` và JSON nhỏ.
- API trả UTF-8 và `Cache-Control: no-store`.
- `POST /api/recheck` chống chạy đồng thời nhiều preflight.
- `/api/demo/action` chỉ tồn tại khi server chạy `--demo`; Live phải trả 404.
- Demo action chỉ nhận allowlist tình huống định nghĩa trước, giới hạn body và
  không chạy subprocess/command.
- Có timeout rõ ràng; lỗi subprocess phải trở thành JSON an toàn, không làm
  server crash.
- Không dùng `shell=True`.
- Không đưa password, token hoặc toàn bộ config lên dashboard.
- Escape mọi nội dung lỗi trước khi đưa vào DOM.
- Dashboard bind mặc định `127.0.0.1`; chỉ bind `0.0.0.0` khi cấu hình rõ ràng.
- Nếu bind ra mạng LAN, chỉ cung cấp endpoint read-only ngoài `recheck`; không
  thêm API sửa cấu hình hoặc chạy command tùy ý.

## 7. Tổ chức code

Không tiếp tục nhét toàn bộ HTML/CSS/JS vào một chuỗi Python lớn. Tách thành:

```text
dashboard/
  __init__.py
  server.py
  static/
    index.html
    styles.css
    app.js
```

`web_dashboard.py` chỉ giữ vai trò CLI entrypoint mỏng và import server.

Không thêm framework nặng. Ưu tiên Python stdlib HTTP server và HTML/CSS/JS
thuần để chạy offline trên Jetson. Nếu cần dependency mới, phải giải thích và
không được làm hỏng `setup_jetson.sh`.

## 8. Responsive và accessibility

- Tối ưu cho 1920×1080, 1280×720 và tablet 768 px.
- Không để text hoặc nút bị cắt ở 720p.
- Keyboard focus rõ ràng.
- Button có accessible name.
- Màu không phải tín hiệu duy nhất: luôn kèm nhãn text.
- Contrast tối thiểu theo WCAG AA cho text chính.
- `aria-live` cho trạng thái preflight, nhưng không đọc lại toàn trang mỗi lần
  polling.
- Không khóa zoom và không vô hiệu hóa selection toàn trang.

## 9. Không được làm

- Không tạo nút Mock trong giao diện Live.
- Không cho Live fallback sang dữ liệu giả.
- Không thay đổi `safety.production_only` hoặc `allow_mock`.
- Không hiển thị TTC/vận tốc.
- Không tự động sửa calibration hoặc đánh dấu calibration thật.
- Không gọi `sudo`, không cài package từ giao diện.
- Không thêm điều khiển phanh, ga hoặc vô-lăng.
- Không copy toàn bộ dashboard 3.000 dòng từ `fw`.

## 10. Kiểm thử bắt buộc

Bổ sung test cho:

1. `/`, `/api/status`, `/healthz`.
2. `POST /api/recheck` thành công, thất bại và timeout.
3. Không cho hai recheck chạy đồng thời.
4. Demo status luôn có `demo: true`.
5. Live status không bao giờ tự nhận `demo: true`.
6. HTML có banner demo cố định trong demo state.
7. Không có từ/nút `Mock` trong giao diện Live.
8. Static assets trả đúng MIME type và không cache.
9. XSS payload trong error được render như text, không thành HTML.
10. Toàn bộ test cũ vẫn đạt.
11. Tất cả demo action hoạt động không cần phần cứng.
12. `/api/demo/action` trả 404 trong Live và không thể sửa cấu hình production.

Chạy cuối:

```bash
python -m py_compile $(rg --files -g '*.py' | tr '\n' ' ')
PYTHONPATH='' pytest -q
bash -n start_jetson.sh start_web_demo.sh
```

Khởi động thử hai cổng riêng:

```text
Live diagnostic: http://127.0.0.1:8765
Web UI Demo:     http://127.0.0.1:8766
```

## 11. Tiêu chí bàn giao

Chỉ coi là hoàn thành khi:

- Giao diện nhận diện được ngay là ADAS cockpit production.
- Live và Demo không thể bị nhầm lẫn.
- Animation mượt, không giật khi polling.
- Không có lỗi console hoặc API.
- Dashboard vẫn dùng được hoàn toàn offline.
- Bố cục chức năng tương đương dashboard của `fw`, nhưng style và source code
  là của dự án hiện tại.
- Demo bấm một lần là chạy đầy đủ dù chưa có `.venv` (máy chỉ cần Python 3).
- Không phá shortcut hiện tại.
- Tất cả test đạt.
- Báo cáo ngắn các file đã sửa, hành vi mới và giới hạn còn lại.

Sau khi hoàn thành, dừng lại để reviewer kiểm tra; không tự thay đổi thuật toán
ADAS hoặc cấu hình phần cứng.

---

## Checklist review phong cách (reviewer sử dụng)

- [ ] Noir cockpit nhất quán, không pha màu tùy tiện.
- [ ] Hierarchy rõ: mode → health → lỗi → hành động.
- [ ] `DEMO UI — NOT LIVE` luôn nhìn thấy trong bản demo.
- [ ] Live không có nút/đường dẫn Mock.
- [ ] Animation có mục đích, 180–320 ms, không gây layout shift.
- [ ] `prefers-reduced-motion` hoạt động.
- [ ] 1920×1080, 1280×720 và 768 px không tràn/cắt.
- [ ] Trạng thái không chỉ dựa vào màu.
- [ ] API/XSS/concurrency/timeout được kiểm thử.
- [ ] Không phát sinh dependency mạng hoặc tài nguyên online.
- [ ] Test cũ và test mới đều đạt.
