# TÀI LIỆU FULL DỰ ÁN ADAS — LiDAR + Camera (no ROS)

> Ngày rà soát: 18/10/2026. Toàn bộ 273 test đều pass sau khi fix.

## 1. Tổng quan

Dự án là pipeline hỗ trợ lái xe nâng cao (ADAS) chạy **không ROS**, bằng Python thuần:

- Camera phát hiện vật thể bằng YOLOv8 (Ultralytics), bám theo bằng IoU tracker.
- LiDAR lọc điểm, gom cụm DBSCAN, chiếu điểm 3D lên ảnh 2D qua hiệu chuẩn camera.
- Fusion gán điểm LiDAR vào từng bounding box, ước lượng khoảng cách, lọc mượt theo thời gian (EMA).
- Tính tốc độ tương đối, thời gian va chạm (TTC), phân loại cảnh báo SAFE / CAUTION / DANGER / EMERGENCY.
- Radar phụ trợ qua ESP32 (LD2410 / LD2420), kiến trúc mới là **một ESP32 Gateway duy nhất** đọc 4 radar rồi gửi JSON về Jetson qua serial.
- Dashboard web xem camera trực tiếp, snapshot, trạng thái sức khỏe hệ thống.
- Chạy được 7 chế độ: demo, live, playback, camera-only, lidar-only, fusion-test, benchmark.

Entry point chính là `run.py` (`main.py` chỉ forward sang `run.py`).

```text
Camera ──> YOLO detect ──> IoU track ──┐
                                       ├─> project LiDAR→ảnh ──> gán điểm vào bbox
LiDAR ──> filter ──> cluster ──────────┘                              │
                    Radar ESP32 Gateway ──> chọn camera ──────────────┤
                                                                      ▼
                                                              khoảng cách → EMA
                                                              → TTC → cảnh báo
                                                              → vẽ overlay → dashboard
```

## 2. Cấu trúc thư mục

| Thư mục / file | Vai trò |
|---|---|
| `run.py` | Launcher chính, parse CLI, nạp config, chạy pipeline |
| `main.py` | Tương thích ngược, forward sang `run.py` |
| `preflight.py` | Kiểm tra fail-fast trước khi chạy live trên Jetson |
| `web_dashboard.py` | Entrypoint dashboard, gọi `dashboard/server.py` |
| `app/pipeline.py` | `ADASPipeline`: điều phối toàn bộ luồng xử lý |
| `core/` | `types.py` (Detection, FusedObject, LidarCluster, CameraFrame, LidarFrame, ProjectedPoints), `enums.py` (WarningLevel, FusionStatus, SystemStatus) |
| `config/` | Nạp YAML (`loader.py`), validate (`validation.py`), schema radar gateway (`radar_gateway.py`), module phụ (`side_modules.py`) |
| `config/adas.yaml` | Config demo/mô phỏng |
| `config/jetson_live.yaml` | Config live trên Jetson (camera thật + gateway) |
| `config/calibration.yaml` | Ma trận K, distortion, R, t của hiệu chuẩn LiDAR-camera |
| `calibration/` | `loader.py` đọc calibration, `projection.py` chiếu LiDAR lên ảnh |
| `vision/` | `detector.py` (YOLO + Mock), `tracker.py` (IoU), `processor.py` (nối detect+track) |
| `lidar/` | `filters.py` (normalize, filter, reject_outliers), `clustering.py` (DBSCAN), `processor.py` (nối filter+cluster) |
| `fusion/` | `association.py` (gán điểm vào bbox), `distance.py` (ước lượng khoảng cách), `engine.py` (tạo FusedObject), `temporal_filter.py` (EMA theo track) |
| `adas/` | `warning.py` (CollisionWarning theo TTC), `distance_warning.py` (theo khoảng cách), `motion.py` (vận tốc tương đối + TTC) |
| `synchronization/` | `SensorSynchronizer`: ghép cặp frame camera-LiDAR theo timestamp |
| `sensors/` | Đọc camera, LiDAR (mock/playback/real qua factory), radar ESP32, gateway manager, camera selector |
| `radar_gateway/` | Parser giao thức gateway: `protocol.py`, `jetson_protocol.py`, `ld2410_parser.py`, `gateway_reader.py` |
| `firmware/` | Code ESP32: `esp32_radar_gateway` (ESP32 A Gateway + ESP32 B Remote, 4 radar) |
| `dashboard/` | `server.py` (dashboard cockpit), `camera_live.py`, `simulator.py`, `static/` |
| `system/` | `health.py` (heartbeat), `logging.py`, `diagnostics.py`, `live_snapshot.py` |
| `visualization/` | `Visualizer`: vẽ bbox, điểm LiDAR, cảnh báo, ghi video |
| `tools/` | `check_projection.py`, `export_tensorrt.py`, `radar_gateway_monitor.py`, `firmware_preflight.py`, `firmware_host_build.py` |
| `tests/` | 20 file test, 273 test case |
| `docs/` | Hướng dẫn đấu dây ESP32, checklist selftest, migration gateway |

## 3. Luồng dữ liệu chi tiết

1. **Đọc cảm biến.** `CameraReader` / `MockCameraReader` / `MultiCameraReader` cho camera; `MockLidarReader` / `PlaybackLidarReader` / driver thật qua `LidarReaderFactory` cho LiDAR. Chế độ live dùng `AsyncFrameReader` chạy thread riêng.
2. **Đồng bộ.** `SensorSynchronizer` buffer hai phía, ghép cặp có chênh lệch timestamp nhỏ nhất trong `max_difference_ms`.
3. **Vision.** `VisionProcessor` chạy detector rồi tracker. Thiếu model hoặc thiếu ultralytics thì dùng `MockDetector` sinh 2 bbox giả lập (chỉ cho demo, live từ chối mock).
4. **LiDAR.** `LidarProcessor` chuẩn hóa (N,3), loại NaN/Inf, cắt theo range và ROI, loại outlier, gom cụm DBSCAN.
5. **Projection.** `LidarCameraProjector.project`: `p_cam = R @ p_lidar + t`, rồi `cv2.projectPoints` (hoặc công thức tay `K @ p_cam` nếu thiếu OpenCV), loại điểm sau camera và ngoài khung hình.
6. **Fusion.** `resolve_overlapping` gán điểm theo confidence giảm dần, che pixel đã claim. `estimate_distance` tính khoảng cách theo mode (adaptive = median khi ít điểm, q1 khi nhiều). `FusionEngine._estimate` chọn điểm có norm gần giá trị thống kê nhất làm `position_3d`. `TemporalFilter` làm mượt EMA theo track_id.
7. **Motion + Warning.** `MotionEstimator` tính vận tốc tương đối và TTC. Chế độ `warning.mode=distance` dùng `DistanceWarning`; còn lại dùng `CollisionWarning` theo ngưỡng TTC + hysteresis chống chập chờn.
8. **Render + health.** `Visualizer` vẽ overlay. `HealthMonitor` nhận heartbeat từng cảm biến. `LiveSnapshotPublisher` đẩy snapshot cho dashboard web.

## 4. Từng module

### 4.1 core

- `types.py`: các dataclass bất biến `Detection`, `FusedObject`, `LidarCluster`, `CameraFrame`, `LidarFrame`, `ProjectedPoints`.
- `enums.py`: `WarningLevel` (UNKNOWN/SAFE/CAUTION/DANGER/EMERGENCY), `FusionStatus` (FUSED/NO_POINTS/INSUFFICIENT_POINTS/INVALID_DISTANCE), `SystemStatus`.

### 4.2 config

- `loader.py`: `load_config`, `coerce_config`, `resolve_path` (mọi path tương đối đều resolve theo project root).
- `validation.py`: `validate_config`, kiểm tra mode, range, class cho phép.
- `radar_gateway.py`: schema gateway mới (device, baudrate, protocol_version=2, 4 radar id, priority camera).
- `side_modules.py`: schema module phụ cũ (đã deprecated, live mặc định tắt).

### 4.3 calibration

- `loader.py`: đọc K (3x3), distortion (pad lên 5), R (3x3), t (3,).
- `projection.py`: như mục luồng bước 5. Dùng `cv2.projectPoints` bằng trick truyền camera-coords làm objectPoints với pose identity — hợp lệ nhưng dễ gây nhầm lẫn khi đọc code.

### 4.4 vision

- `detector.py`: `YoloDetector` (không throw khi thiếu model, báo `is_real=False` để pipeline quyết định), `MockDetector` sinh bbox theo sin để demo chuyển động.
- `tracker.py`: `IoUTracker` ghép IoU tham lam, `min_hits=3` mặc định nên 2 frame đầu chưa có track ổn định (chủ ý, không phải bug).
- `processor.py`: nối detect rồi update tracker.

### 4.5 lidar

- `filters.py`: `normalize_points` (pad z=0 nếu chỉ có XY, cắt bỏ cột thừa), `filter_points` (range + ROI), `reject_outliers` (IQR/MAD/none).
- `clustering.py`: DBSCAN sklearn, fallback NumPy O(n²) khi thiếu sklearn (chỉ chịu được cloud nhỏ).
- `processor.py`: nối filter + cluster, lỗi config thì log warning và trả rỗng.

### 4.6 fusion

- `association.py`: `points_for_bbox` (thu bbox 0.85 để tránh lấn), `resolve_overlapping` (tham lam theo confidence).
- `distance.py`: các mode nearest/mean/median/q1/trimmed_mean/adaptive + lọc outlier IQR trước khi thống kê.
- `engine.py`: tạo `FusedObject` cho từng detection, gắn status tương ứng.
- `temporal_filter.py`: EMA `alpha`, giữ track tối đa `max_hold` frame, quét stale sau `stale_seconds`.

### 4.7 adas

- `motion.py`: vận tốc tương đối theo EMA, TTC = distance / closing_speed, có chặn chia 0 và giá trị không hữu hạn.
- `warning.py`: `CollisionWarning` theo ngưỡng TTC + hysteresis (leo thang cần đủ chênh lệch level và duy trì đủ lâu; hạ thang chấp nhận ngay).
- `distance_warning.py`: cảnh báo theo ngưỡng khoảng cách, dùng khi `warning.mode=distance`.

### 4.8 sensors

- `camera_reader.py`: CameraReader (OpenCV, tự reconnect), MockCameraReader, MultiCameraReader (5 camera front/left/right, lệnh `select`).
- `lidar_reader.py`: Mock, Playback (npy/npz/csv), factory cho driver thật.
- `esp32_radar.py`: đọc radar đơn qua UART (legacy).
- `radar_gateway*.py`, `side_module_manager.py`, `camera_selector.py`: đọc gateway JSON, chọn camera theo radar phát hiện gần nhất + hold.
- `probes.py`: `probe_camera_managed`, `probe_gateway` dùng cho preflight.
- `async_reader.py`: thread đọc không chặn cho live.

### 4.9 radar_gateway + firmware

- Giao thức JSON protocol v2 qua serial 115200: gateway ESP32 gom 4 radar HLK-LD2420 V2 rồi gửi một dòng JSON mỗi chu kỳ.
- `protocol.py` định nghĩa packet, `jetson_protocol.py` phía Jetson, `ld2410_parser.py` parse radar đơn.
- Firmware hiện tại là `esp32_radar_gateway` (ESP32 A Gateway + ESP32 B Remote). Pipeline từ chối chạy khi bật đồng thời gateway mới và schema radar cũ.

### 4.10 dashboard + system + visualization

- `dashboard/server.py`: dashboard cockpit (trạng thái preflight, snapshot, gateway stats).
- `system/health.py`: heartbeat từng kênh, chuyển DEGRADED/ERROR khi mất heartbeat.
- `system/live_snapshot.py`: xuất JPEG theo tần số `snapshot_hz` cho web.
- `visualization/visualizer.py`: vẽ bbox màu theo warning level, điểm LiDAR, chữ FPS/latency, ghi video mp4.

## 5. Config tham khảo

`config/adas.yaml` (demo): camera mock 1280x720, LiDAR mock 1 vật ở 12 m, YOLO `yolov8n.pt` conf 0.45, `warning.mode=distance` (12/6/3 m), `allow_mock=true`.

`config/jetson_live.yaml` (live): 5 camera `/dev/video*`, `radar_gateway.enabled=true`, `warning.mode` theo TTC, `allow_mock=false`, `is_demo_calibration` phải tắt sau khi calibrate thật.

Ngưỡng TTC mặc định khi config thiếu (vừa fix): emergency 1.0 s, danger 2.0 s, caution 3.5 s.

## 6. Cách chạy

```bash
python run.py --mode live            # chạy full ADAS
python run.py --mode camera-only     # chỉ camera thật, không AI
python main.py                       # tương đương run.py
python preflight.py --mode live      # kiểm tra phần cứng trước khi live
python web_dashboard.py              # mở dashboard
python -m pytest tests/ -q           # chạy 273 test
```

## 7. Lỗi đã tìm và đã fix (debug toàn bộ)

| # | File | Lỗi | Cách fix |
|---|---|---|---|
| 1 | `adas/warning.py:55-57` | `c["emergency_ttc"]` truy cập trực tiếp, thiếu key là crash mỗi frame. Đây là crash-path chắc chắn nhất | Đổi sang `c.get` với default 1.0 / 2.0 / 3.5 s |
| 2 | `calibration/projection.py:36` | `points.shape[1]` nổ IndexError khi input là mảng 1D shape (N,) | Reshape 1D về (1,-1), kiểm tra ndim trước khi đọc shape |
| 3 | `fusion/distance.py` `_trimmed_mean` | Với n=2 thì `hi=int(0.9*2)=1`, chỉ giữ 1 mẫu (cắt 50% thay vì 10%) | n ≤ 4 thì fallback median |
| 4 | `fusion/distance.py` `estimate_distance` | Input 1D shape (N,) là vector khoảng cách nhưng bị reshape thành (1,N) rồi lấy 2 phần tử đầu — sai hoàn toàn; ngoài ra nhánh 2D dùng biến `distances` có thể chưa định nghĩa | Tách nhánh: 1D xử lý như vector khoảng cách, 2D mới tính norm; `cleaned` luôn được định nghĩa trong từng nhánh |
| 5 | `lidar/clustering.py` `cluster_points` | Crash với NaN/Inf khi gọi trực tiếp; nhánh fallback và nhánh gom cụm dùng `points` gốc thay vì mảng đã lọc | Lọc finite vào `arr` ngay đầu, mọi nhánh dùng `arr` |
| 6 | `fusion/engine.py` `_estimate` | `points[idx]` nổ IndexError nếu points 1D; `len(points)` trên mảng 0-d cũng lỗi | Ép `np.asarray`, kiểm tra ndim==2 và shape hợp lệ trước khi argmin |

Ghi chú: đã thử áp `min_points` vào `resolve_overlapping` nhưng làm 3 test fail (test kỳ vọng assignment giữ nguyên điểm thô, còn ngưỡng do `FusionEngine` xử lý qua `INSUFFICIENT_POINTS`). Đã revert — ngưỡng thuộc về engine, không thuộc association.

## 8. Điểm cần lưu ý (không phải bug, nhưng dễ vấp)

- Khoảng cách đang tính bằng norm **XY** (`points[:, :2]`), bỏ Z — nhất quán toàn pipeline nhưng underestimate khi đường dốc. Nếu đổi sang 3D phải đổi đồng loạt ở `lidar/clustering.py`, `fusion/distance.py`, `fusion/engine.py`.
- `position_3d[1]` được coi là lateral (hệ LiDAR). Nếu đổi sang hệ camera thì lateral là index 0 — quy ước này chưa được document trong code.
- `resolve_overlapping` có tham số `min_points` nhưng hiện không dùng (chờ quyết định thiết kế: lọc ở association hay ở engine như hiện tại).
- `MockDetector` sinh bbox âm khi ảnh hẹp hơn 280 px — chỉ ảnh hưởng demo với ảnh nhỏ.
- `TemporalFilter` giữ `_last_t` cũ khi hold nên hold dài + `stale_seconds` ngắn sẽ mất track sớm — cấu hình hai giá trị này nhất quán với nhau.
- `preflight.py` không bao giờ pass giả: thiếu model, engine TensorRT, calibration thật, driver LiDAR, placeholder `CHANGE_ME` đều là blocker ở live.
