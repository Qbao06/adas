# ADAS LiDAR–Camera (No ROS)

> Production launcher (`run.py`, `run.sh`, `start_jetson.sh` và shortcut)
> chỉ cho phép `live`. Mock/demo chỉ còn là fixture nội bộ cho test tự động và
> không thể khởi động qua CLI production.

Một pipeline ADAS kết hợp **camera + LiDAR** chạy hoàn toàn bằng Python thuần, không dùng ROS, ROS2, rviz hay bất kỳ middleware nào khác.

Mục tiêu: người dùng **giải nén thư mục** → **nhấp đúp** (Windows) hoặc **chạy `./run.sh`** (Linux / Raspberry Pi) → hệ thống chạy ngay, kể cả khi không có camera, không có LiDAR, không có video.

## 1. Pipeline

```
Camera / Video / Mock ──►  YOLO Detection ──►  Tracking (track_id ổn định)
                                                            │
LiDAR thật / Playback / Mock ──►  Filter (NaN/Inf/range/ROI) ──► DBSCAN ──► clusters
                                                            │
                                  Calibration (YAML) ──► Projection LiDAR → pixel
                                                            │
                                            Fusion ──► khoảng cách + EMA theo track_id
                                                            │
                                            Distance Warning (không cần vận tốc/TTC)
                                                            │
                                            UNKNOWN / SAFE / CAUTION / DANGER / EMERGENCY + hysteresis
                                                            │
                                            Visualizer / Video recorder / Health monitor
```

## 2. Chế độ hỗ trợ

| Mode         | Mô tả                                                                 |
|--------------|------------------------------------------------------------------------|
| `demo`       | Mặc định. Không cần phần cứng. Sinh camera + LiDAR giả.               |
| `live`       | Camera thật + LiDAR thật (hoặc playback).                              |
| `playback`   | Video file + LiDAR playback (.npy / .npz / .csv).                      |
| `camera-only`| Chỉ chạy camera, LiDAR được tắt (vẫn chạy detection + tracking).      |
| `lidar-only` | Chỉ chạy LiDAR, không detect.                                          |
| `fusion-test`| Tạo dữ liệu tổng hợp để smoke test toàn bộ pipeline fusion.           |
| `benchmark`  | Đo FPS + latency trung bình + p95 sau khi warm-up.                     |

## 3. Cách chạy nhanh

### Windows

1. Giải nén project.
2. Nhấp đúp `run.bat` (nó sẽ tự tạo `.venv`, cài `requirements.txt`, tạo thư mục còn thiếu và chạy pipeline mặc định).

### Linux / Raspberry Pi

```bash
chmod +x run.sh setup.sh   # chỉ cần làm một lần
./run.sh                    # tương đương ./run.sh --mode demo
```

### Từ bất kỳ thư mục nào

```bash
python /đường/dẫn/tới/adas_project/run.py --mode demo
```

## 4. Các lệnh thường dùng

```bash
# Demo không cần phần cứng
python run.py --mode demo

# Headless (không mở cửa sổ) — chạy 20 frame rồi thoát
python run.py --mode demo --headless --max-frames 20

# Lưu frame mẫu đầu tiên
python run.py --mode demo --save-frame

# Live: camera index 0 + LiDAR playback npy
python run.py --mode live --camera-source 0 --lidar data/lidar.npy

# Playback: video mp4 + LiDAR playback npz
python run.py --mode playback --video data/test.mp4 --lidar data/lidar.npz

# Fusion-test: ép tất cả module chạy trên dữ liệu mô phỏng
python run.py --mode fusion-test

# Benchmark: in FPS / latency 50 frame
python run.py --mode benchmark --max-frames 50

# Ghi video kết quả
python run.py --record --record-path output/my_result.mp4
```

## 5. Cấu hình (`config/adas.yaml`)

Mọi tham số đều nằm trong file YAML. Nếu thiếu trường nào, hệ thống tự dùng giá trị mặc định an toàn và ghi cảnh báo — không crash.

```yaml
mode: demo             # demo | live | playback | camera-only | lidar-only | fusion-test | benchmark
camera:
  source: 0            # 0, /dev/video0, data/demo.mp4
  width: 1280
  height: 720
  fps: 30
lidar:
  source: mock         # mock | playback
  driver: mock         # mock | playback | <driver-tuỳ-chỉnh>
  playback_path: data/lidar_frames.npy
  fps: 10
vision:
  model: yolov8n.pt
  confidence: 0.45
  device: auto         # auto | cpu | cuda
  allowed_classes: [person, bicycle, motorcycle, car, bus, truck]
lidar_processing:
  min_range_m: 0.3
  max_range_m: 60.0
  roi: { x_min: 0, x_max: 60, y_min: -12, y_max: 12, z_min: -3, z_max: 4 }
  clustering: { eps: 0.7, min_samples: 4 }
synchronization:
  max_difference_ms: 120
fusion:
  min_points: 3
  distance_mode: adaptive    # adaptive | nearest | mean | median | q1 | trimmed_mean
  ema_alpha: 0.35
warning:
  mode: distance
  caution_distance_m: 12.0
  danger_distance_m: 6.0
  emergency_distance_m: 3.0
visualization:
  enabled: true
  show_lidar_points: true
```

## 6. Calibration

`config/calibration.yaml` chứa `camera_matrix`, `distortion`, `rotation`, `translation`. **Ma trận trong repo chỉ là calibration mô phỏng**, KHÔNG dùng cho xe thật.

Khi lắp thiết bị thật, hãy calibrate lại (OpenCV `calibrateCamera`, Kalibr, v.v.) và thay nội dung file. Hệ thống sẽ tự động phát hiện và báo lỗi nếu ma trận sai shape.

## 7. Driver LiDAR thật

Tạo một class kế thừa `_BaseLidarReader` trong `sensors/lidar_reader.py`, với các method:

```python
class MyLidarDriver(_BaseLidarReader):
    def connect(self): ...
    def read(self) -> Optional[LidarFrame]: ...
    def is_connected(self) -> bool: ...
    def disconnect(self): ...
```

Sau đó đăng ký:

```python
LidarReaderFactory.register("ten_driver", lambda **kw: MyLidarDriver(**kw))
```

Và đặt trong YAML:

```yaml
lidar:
  driver: ten_driver
  source: ten_driver
```

Nếu driver không có sẵn, pipeline tự fallback sang `mock` và in cảnh báo — **không crash**.

## 8. Thay model YOLO

- Đặt file trọng số tại `models/your_model.pt`.
- Sửa `vision.model` trong YAML hoặc dùng CLI `--model models/your_model.pt`.
- Có thể chuyển thiết bị bằng `--device cpu` / `--device cuda`.

Nếu model không load được, hệ thống fallback sang `MockDetector` để pipeline vẫn chạy.

## 9. Cài đặt thủ công

```bash
python -m venv .venv
source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python run.py --mode demo
```

## 10. Test

```bash
# If the host machine has ROS /opt/ros site-packages installed, clear
# PYTHONPATH first so that ROS pytest plugins don't get auto-discovered.
PYTHONPATH="" pytest -q

# Or use the helper script that handles this automatically:
python run.py --mode demo --headless --max-frames 5
```

`tests/conftest.py` also strips `/opt/ros` from `sys.path` automatically so
``pytest -q`` works on hosts with system ROS too.

`tests/` chứa các test cho: input rỗng, NaN/Inf, nhiều vật thể, bbox chồng nhau, outlier LiDAR, timestamp lệch, track_id đổi, khoảng cách nhảy, vật đứng yên / tiến gần / đi xa, camera mất dữ liệu, LiDAR mất dữ liệu, calibration sai, TTC không hợp lệ, smoke test khởi động/dừng và end-to-end.

## 11. Cấu trúc thư mục

```text
adas_project/
├── main.py            # forward sang run.py (tương thích ngược)
├── run.py             # CLI chính (đa nền tảng)
├── run.bat            # launcher Windows
├── run.sh             # launcher Linux / Raspberry Pi
├── setup.bat / setup.shadas_project_full
├── pyproject.toml
├── requirements.txt
├── .gitignore
├── README.md
├── app/               # pipeline tổng
├── core/              # types, enums
├── config/            # loader mặc định + YAML
├── sensors/           # camera + lidar readers
├── synchronization/   # sensor_synchronizer
├── vision/            # detector + tracker + vision_processor
├── lidar/             # filter + clustering + processor
├── calibration/       # projection + load_calibration
├── fusion/            # distance + association + engine + temporal_filter
├── adas/              # motion + warning
├── system/            # health + logging
├── visualization/     # visualizer
├── data/              # video/lidar đầu vào (user thêm)
├── models/            # model weights (user thêm)
├── logs/              # log runtime
├── output/            # video/frame kết quả
└── tests/             # unit + integration tests
```

## 12. Lỗi thường gặp

| Triệu chứng | Xử lý |
|-------------|-------|
| `Không mở được camera/video` | Kiểm tra chỉ số camera (`0`, `1`, ...), quyền truy cập thiết bị trên Linux. Hoặc chạy `--mode demo`. |
| `Không tải được model YOLO` | Đặt `models/yolov8n.pt` hoặc dùng `--device cpu`. Hệ thống sẽ tự fallback sang `MockDetector`. |
| `Calibration sai định dạng` | Kiểm tra `camera_matrix` 3x3, `rotation` 3x3, `translation` độ dài 3. |
| `Không tìm thấy file LiDAR playback` | Đặt file vào `data/` rồi trỏ `lidar.playback_path`. Hoặc dùng `--mode demo`. |
| `Cửa sổ đóng ngay trên Windows` | Cần cài đặt Visual C++ Redistributable. `run.bat` sẽ pause để bạn đọc lỗi. |

## 13. An toàn & trách nhiệm

> **CẢNH BÁO**

1. **`config/calibration.yaml` là mô phỏng** — KHÔNG dùng để điều khiển phương tiện thật.
2. Hệ thống **chỉ phục vụ nghiên cứu / thử nghiệm**, không thay thế hệ thống an toàn chứng nhận trên ô tô thương mại (ISO 26262, ASIL-D...).
3. Tài xế vẫn phải luôn quan sát và chịu trách nhiệm cuối cùng.

## 13. Radar Gateway (ESP32 A + 4× LD2410C)

Hệ thống dùng một ESP32 A Gateway duy nhất đọc **bốn** radar HLK-LD2410C
trái/phải trước/sau, gửi mỗi batch thành **một dòng JSON v2** qua USB
Serial. ESP32 B (Remote) chỉ là bridge thu hai radar phải về ESP32 A
qua ESP-NOW. Live **không bao giờ** fallback sang mock radar; một radar
lỗi chỉ làm radar đó stale. Cấu hình, mapping camera, validation và
lệnh migration nằm trong [`docs/RADAR_GATEWAY_MIGRATION.md`](docs/RADAR_GATEWAY_MIGRATION.md).

## 14. Giấy phép

MIT — dùng tự do cho mục đích nghiên cứu, giáo dục, hackathon, demo.

## Chạy camera thật khi chưa có model AI

```bash
./run.sh --mode camera-only
# Hoặc: .venv/bin/python run.py --mode camera-only --port 8765
```

Mở http://127.0.0.1:8765. Chế độ này dùng dashboard hiện có, không cần AI, LiDAR, radar hoặc calibration. Các vị trí camera được gán tạm theo thứ tự thiết bị; chưa đại diện vị trí lắp thực tế. Có thể dùng `--camera-source /dev/video0` để chọn một camera. Nhấn Ctrl+C trong terminal để dừng.
