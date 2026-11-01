# BÁO CÁO KIỂM DUYỆ MÃ NGUỒN

**Dự án:** ADAS LiDAR–Camera không ROS  
**Ngày kiểm duyệt:** 2026-08-27  
**Phạm vi:** kiến trúc pipeline, cấu hình, camera/LiDAR I/O, đồng bộ, fusion, cảnh báo, health monitoring, visualization, đóng gói và test.  
**Kết luận:** **Đạt cho demo/playback; chưa đạt điều kiện chạy live trên phương tiện.**

## 1. Bằng chứng kiểm tra

- Toàn bộ test hiện tại: **86/86 đạt**.
- Smoke test `demo --headless`: chạy và dừng đúng giới hạn frame.
- Không có `best.pt` hoặc driver LiDAR phần cứng trong mã nguồn tại thời điểm kiểm duyệt.
- Test hiện thiên về unit/integration trên mock; chưa có hardware-in-the-loop, fault injection hay acceptance test cho `live`.

## 2. Phát hiện theo mức độ ưu tiên

### P0 — Health monitor có thể báo khỏe khi cảm biến không tạo dữ liệu

**Vị trí:** `app/pipeline.py:431-434`, `app/pipeline.py:454-457`.

Khi `read()` trả `None`, pipeline vẫn phát heartbeat bằng `time.monotonic()`. Ngoài ra cảm biến bị tắt (`None`) được hiển thị là `ok`. Điều này che giấu lỗi camera/LiDAR và làm trạng thái hệ thống không đáng tin cậy.

**Yêu cầu sửa:** chỉ heartbeat khi có frame thật; phân biệt `ok`, `lost`, `disabled`, `stale`; thêm test mất tín hiệu kéo dài và reconnect.

### P0 — Chế độ live chưa fail-safe và có thể dùng dữ liệu mock

**Vị trí:** `app/pipeline.py:239-268`, `app/pipeline.py:270-304`, `vision/detector.py`.

Fallback mock phù hợp với demo nhưng không phù hợp với xe thật. Model hoặc driver lỗi phải làm `live` từ chối khởi động hoặc chuyển trạng thái lỗi, không được tạo dữ liệu giả.

**Yêu cầu sửa:** thêm policy theo mode; cấm `MockCameraReader`, `MockLidarReader`, `MockDetector` trong `live`; xác nhận tên detector/driver trong startup log và health telemetry.

### P0 — Calibration mô phỏng không bị chặn trong live

**Vị trí:** `app/pipeline.py:194-203`, `config/loader.py` (`safety.is_demo_calibration`).

Cờ `is_demo_calibration` tồn tại nhưng không được kiểm tra. Pipeline có thể fusion bằng ma trận mô phỏng.

**Yêu cầu sửa:** `live` phải từ chối khởi động nếu calibration còn là demo, sai độ phân giải, thiếu metadata/version hoặc vượt sai số validation.

### P1 — Distortion được nạp nhưng không được dùng khi projection

**Vị trí:** `calibration/projection.py:22-52`.

`self.distortion` được load nhưng projection chỉ nhân `K[R|t]`. Với camera có méo ống kính, điểm LiDAR sẽ lệch khỏi vật thể, đặc biệt ở rìa ảnh.

**Yêu cầu sửa:** thống nhất ảnh đã undistort và camera matrix mới, hoặc dùng projection có distortion; thêm test với distortion khác 0.

### P1 — Kết quả DBSCAN bị bỏ khỏi pipeline

**Vị trí:** `app/pipeline.py:319-339`.

`LidarProcessor` trả `points, clusters`, nhưng pipeline đặt tên `_clusters` rồi fusion toàn bộ point thô. Điều này mâu thuẫn với kiến trúc README và làm clustering trở thành phép tính không có consumer.

**Yêu cầu sửa:** quyết định rõ point-level hay cluster-level fusion. Nếu giữ cluster, thêm ground removal, depth gating và association theo cluster. Nếu bỏ cluster, xóa bước DBSCAN khỏi hot path và cập nhật tài liệu.

### P1 — Driver registry có fallback không an toàn và có thể lỗi tham số

**Vị trí:** `sensors/lidar_reader.py:246-255`, `app/pipeline.py:291-295`.

Driver không tồn tại được đổi thành `mock`, nhưng kwargs `path` dành cho playback/driver thật được truyền sang `MockLidarReader`, class này không nhận `path`; vì vậy fallback được mô tả là “không crash” nhưng có thể phát sinh `TypeError`.

**Yêu cầu sửa:** factory phải validate driver và schema tham số; `live` raise lỗi rõ ràng; chỉ demo mới được tạo mock bằng đúng kwargs.

### P1 — Đọc camera và LiDAR tuần tự làm tăng độ trễ

**Vị trí:** `app/pipeline.py:403-430`.

Synchronizer đã ghép timestamp nhưng hai hàm `read()` vẫn chạy tuần tự trong cùng thread. Với driver blocking, chu kỳ pipeline bằng tổng thời gian chờ camera + LiDAR, buffer không phản ánh hai stream độc lập.

**Yêu cầu sửa:** reader thread/process riêng, queue có giới hạn, timestamp từ thiết bị, backpressure/drop policy và metric về sync offset.

### P1 — Motion/TTC chưa dùng ego-motion của xe

**Vị trí:** `adas/motion.py`.

Vận tốc tương đối được suy ra từ sai phân khoảng cách theo track 2D. Rung, đổi track, xe rẽ hoặc thay đổi vận tốc có thể tạo TTC sai.

**Yêu cầu sửa:** tích hợp vehicle speed/yaw rate/IMU; bù ego-motion; lọc outlier tốc độ; định nghĩa uncertainty và điều kiện TTC không hợp lệ.

### P2 — Cấu hình công khai có khóa không được tiêu thụ

**Vị trí:** `config/adas.yaml:40-65`.

- `fusion.motion_*` không được `MotionEstimator` đọc; code đọc nhóm `adas.*`.
- `visualization.record_fps` không được dùng; Visualizer đọc `video_fps`.
- `visualization.enabled` không được kiểm tra trong `draw()`.
- `record_video` trong `ADASPipeline.run()` không được sử dụng.

**Yêu cầu sửa:** tạo schema cấu hình duy nhất, từ chối unknown/deprecated keys và thêm test chứng minh mỗi option thay đổi hành vi.

### P2 — Các mode chưa đúng cam kết

- `lidar-only` xử lý cluster nhưng không xuất/hiển thị object 3D.
- `fusion-test` gần như giống `demo`, chưa có kịch bản TTC/cảnh báo/mất cảm biến.
- `benchmark.measure_frames` không tự giới hạn số frame.

**Yêu cầu sửa:** định nghĩa contract và acceptance test riêng cho từng mode.

### P2 — CLI không hỗ trợ driver tùy chỉnh

**Vị trí:** `run.py:66-70`.

README mô tả registry mở rộng nhưng CLI giới hạn `mock|playback`. Driver thật chỉ có thể cấu hình gián tiếp qua YAML.

**Yêu cầu sửa:** lấy danh sách từ registry hoặc chấp nhận chuỗi và validate sau khi plugin driver được load.

### P2 — Đóng gói entry point cần kiểm chứng

**Vị trí:** `pyproject.toml:35-53`.

Entry point là `run:main`, nhưng cấu hình setuptools chỉ tìm package directories và chưa khai báo rõ top-level module `run`. Chạy trực tiếp thành công không chứng minh wheel đã cài có lệnh `adas` hoạt động.

**Yêu cầu sửa:** khai báo `py-modules`, hoặc chuyển launcher vào package; build wheel trong môi trường sạch và chạy smoke test từ thư mục ngoài source tree.

## 3. Khoảng trống kiểm thử

Trước khi nghiệm thu `live`, cần bổ sung:

1. Startup phải fail khi model, camera, LiDAR hoặc calibration thật bị thiếu.
2. Camera/LiDAR mất tín hiệu, stale timestamp, reconnect và packet loss.
3. Không có mock object/detection nào trong `live`.
4. Projection với distortion và calibration ground truth.
5. Sync offset, jitter và frame-drop policy.
6. Class names của `best.pt` không khớp `allowed_classes`.
7. Track-ID switch, vật thể che khuất và TTC outlier.
8. Soak test nhiều giờ: RAM, GPU memory, nhiệt độ và log rotation.
9. Wheel/install/CLI smoke test trong môi trường sạch.
10. Hardware-in-the-loop với camera và model LiDAR mục tiêu.

## 4. Điều kiện nghiệm thu đề xuất

### Nghiệm thu demo/playback

- 86/86 test đạt: **Đạt**.
- Chạy headless và dừng sạch: **Đạt**.
- Không yêu cầu cảm biến/model thật: **Đạt theo thiết kế demo**.

### Nghiệm thu hardware/live

- Driver LiDAR thật: **Chưa có**.
- `best.pt` và mapping class đã kiểm chứng: **Chưa có bằng chứng**.
- Calibration thật và validation: **Chưa có**.
- Fail-safe/health chính xác: **Không đạt**.
- Hardware timestamp/parallel acquisition: **Chưa đạt**.
- HIL/road-test evidence: **Chưa có**.

**Quyết định kiểm duyệt:** chưa cho phép dùng để điều khiển phương tiện hoặc đưa ra cảnh báo an toàn chính thức. Chỉ nên tiếp tục ở chế độ nghiên cứu, playback hoặc thử nghiệm có giám sát trong khu vực đóng.

## 4A. Cập nhật sau kiểm duyệt

Các mục sau đã được triển khai sau báo cáo gốc: live cấm mock, calibration gate,
heartbeat theo thời gian nhận local, trạng thái disabled/lost, projection có
distortion, acquisition camera–LiDAR đa luồng, timeout riêng từng cảm biến và
sync, cảnh báo distance-only, cluster filtering tùy chọn, Jetson preflight và
launcher. Bộ test hiện có **96 trường hợp pytest được thực thi và đều đạt**
(93 test functions, gồm một test tham số hóa tạo thêm 3 trường hợp).

Driver LiDAR, model TensorRT và calibration thật vẫn là đầu vào phần cứng chưa
có, nên quyết định chưa nghiệm thu live vẫn giữ nguyên.

## 5. Kế hoạch sửa theo thứ tự

1. Sửa P0: live policy, health state và calibration gate.
2. Tích hợp driver LiDAR + `best.pt`, xác nhận class mapping.
3. Chuyển acquisition sang hai luồng và đo sync/latency.
4. Sửa projection distortion và hoàn thiện calibration tool.
5. Chốt kiến trúc cluster/point fusion và bổ sung ego-motion.
6. Chuẩn hóa schema cấu hình/mode/packaging.
7. Bổ sung fault-injection, HIL và soak test trước nghiệm thu.
