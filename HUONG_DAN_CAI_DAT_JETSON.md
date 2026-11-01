# Bộ cài ADAS trên Linux — Jetson Orin Nano

Tài liệu này là hướng dẫn cài trên máy. Làm lần lượt. Không bỏ bước kiểm tra.

## 1. Bo nào chạy được

| Bo | JetPack | Kết luận |
|---|---|---|
| Jetson Orin Nano 4GB / 8GB | 6.x (Ubuntu 22.04, Python 3.10) | Bo mục tiêu. Nên dùng bản 8GB. |
| Jetson Orin NX, AGX Orin | 6.x | Cùng cách cài. |
| Jetson Nano đời cũ (B01, 2GB/4GB) | tối đa 4.6.5, Ubuntu 18.04, Python 3.6 | Không chạy được dự án này. |

Jetson Nano cũ thiếu Python 3.10, thiếu PyTorch/TensorRT đời mới và không đủ RAM cho YOLO + 5 camera + LiDAR. `setup_jetson.sh` sẽ từ chối máy không phải Jetson Linux hoặc Python dưới 3.10.

Tên “Jetson Nano Orin” trong hướng dẫn này nghĩa là **Jetson Orin Nano**.

## 2. Phần cứng mà config live đang đòi

`config/jetson_live.yaml` không chạy nếu thiếu một trong các mục sau:

- 5 camera USB: `front`, `left_1`, `left_2`, `right_1`, `right_2`
- ESP32 A (gateway) nói JSON protocol v2, baud **115200**, cổng `/dev/serial/by-id/...`
- Bốn radar HLK-**LD2410C** (firmware trong `firmware/esp32_radar_gateway`). Đừng nạp firmware LD2420 rồi chờ JSON v2.
- Một LiDAR thật. Repo có khung `drivers/real_lidar.py` — `connect()` cố ý raise cho đến khi viết SDK thật. Preflight vẫn chặn `lidar.driver: CHANGE_ME`.
- File model TensorRT `models/best.engine`, export trên chính con Jetson
- File `config/calibration_live.yaml` calibrate trên cụm camera–LiDAR đã bắt vít

Năm camera USB cùng lúc dễ nghẽn băng thông. Cắm front vào cổng riêng nếu có thể. Orin Nano 4GB dễ thiếu RAM; hạ `camera.width/height` xuống 640x480 trước khi kết luận là lỗi phần mềm.

## 3. Flash hệ điều hành

1. Tải JetPack 6.x cho đúng module (Orin Nano Super / Orin Nano) từ NVIDIA.
2. Flash bằng NVIDIA SDK Manager từ một máy Ubuntu, hoặc dùng thẻ SD / NVMe image chính thức.
3. Boot lần đầu, tạo user, cập nhật:

```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

4. Đặt chế độ nguồn cao nhất (tên mode tùy bản JetPack; xem `nvpmodel -q`):

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

`jetson_clocks` chỉ để đo. Không bắt buộc khi chạy lâu.

## 4. Gói hệ thống

```bash
sudo apt install -y \
  python3-venv python3-dev python3-pip \
  python3-opencv \
  v4l-utils \
  libopenblas-dev libjpeg-dev \
  git
sudo usermod -aG dialout,video,plugdev "$USER"
```

Đăng xuất rồi đăng nhập lại để nhóm `dialout` và `video` có hiệu lực.

Không chạy `pip install opencv-python` và không chạy `pip install torch` từ PyPI. Hai lệnh đó kéo wheel sai kiến trúc hoặc đè NumPy của JetPack.

Kiểm tra:

```bash
python3 -c 'import sys; print(sys.version)'    # phải >= 3.10
python3 -c 'import cv2; print(cv2.__version__)'
ls /etc/nv_tegra_release
v4l2-ctl --list-devices
```

## 5. Cài PyTorch CUDA của NVIDIA

Cài **trước** `./setup_jetson.sh`. Wheel phải đúng phiên bản JetPack đang chạy (`cat /etc/nv_tegra_release`).

Lấy wheel từ tài liệu NVIDIA “Installing PyTorch for Jetson Platform”, không lấy từ `pip install torch`.

Sau khi cài vào user site hoặc vào môi trường sẽ dùng với `--system-site-packages`, kiểm tra:

```bash
python3 -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
```

Dòng in ra phải kết thúc bằng `True`. Nếu `False`, dừng. Export TensorRT và chế độ live sẽ không chạy.

## 6. Đưa source lên Jetson và cài

Copy cả thư mục `adas_project` (không cần `.venv` tạo trên Windows). Cài wheel PyTorch NVIDIA **trước** (mục 5). Trên Jetson:

```bash
cd /đường/dẫn/adas_project
chmod +x setup_jetson.sh start_jetson.sh run.sh
./setup_jetson.sh
```

Không chạy `./setup.sh` hay `pip install -r requirements.txt` trên Jetson.

Script làm các việc sau:

- Từ chối nếu không thấy `/etc/nv_tegra_release` hoặc Python < 3.10
- Cài apt: `python3-opencv`, `v4l-utils`, `python3-libnvinfer` (nếu có)
- Thêm user vào `dialout`, `video`, `plugdev`; cài `deploy/99-adas-hardware.rules`
- Tạo `.venv` với `--system-site-packages`
- Gỡ `opencv-python` / `onnxruntime` pip nếu lần trước cài nhầm
- Cài `requirements-jetson.txt` (không có torch, không có opencv-python)
- Cài `ultralytics --no-deps` sau khi `torch.cuda` đã True
- Chạy `tools/verify_install.py --strict`
- Tạo shortcut `ADAS-Live.desktop`

Nếu script báo thiếu PyTorch CUDA: làm lại mục 5, **không** `pip install torch`. Nếu vừa thêm nhóm `dialout`/`video`: đăng xuất rồi đăng nhập lại.

## 7. Model

Đặt checkpoint `.pt` đã train vào `models/best.pt`. Export **trên Jetson**, không copy file `.engine` từ máy x86:

```bash
.venv/bin/python tools/export_tensorrt.py --model models/best.pt --imgsz 640
```

Kết quả phải là `models/best.engine`. In ra danh sách class. Sửa `vision.allowed_classes` trong `config/jetson_live.yaml` cho trùng tên class của model (chữ thường). Preflight từ chối nếu không có class nào giao nhau.

`vision.model` đang trỏ `models/best.engine`. Giữ nguyên nếu file nằm đúng chỗ đó.

## 8. Camera

Xem tên ổn định, đừng dùng `/dev/video0` lâu dài vì thứ tự đổi sau mỗi lần cắm:

```bash
ls -l /dev/v4l/by-id/
v4l2-ctl --list-devices
```

Sửa khối `camera.sources` trong `config/jetson_live.yaml`:

```yaml
camera:
  source: 0
  sources:
    front: /dev/v4l/by-id/...-index0
    left_1: /dev/v4l/by-id/...
    left_2: /dev/v4l/by-id/...
    right_1: /dev/v4l/by-id/...
    right_2: /dev/v4l/by-id/...
```

Chỉ ghi node kết thúc bằng `index0`. Node `index1` là metadata, mở sẽ lỗi.

## 9. Radar ESP32

1. Nạp `firmware/esp32_radar_gateway` cho ESP32 A. ESP32 B chỉ là cầu ESP-NOW cho hai radar bên phải.
2. Cắm ESP32 A bằng USB.
3. Lấy đường dẫn cố định:

```bash
ls -l /dev/serial/by-id/
```

4. Sửa `config/jetson_live.yaml`:

```yaml
radar_gateway:
  enabled: true
  device: /dev/serial/by-id/usb-... 
  baudrate: 115200
  protocol_version: 2
```

Xóa chữ `CHANGE_ME`. Preflight coi đó là lỗi. Không bật đồng thời `side_radar.enabled` và `radar_gateway.enabled`.

`warning_distance_m` đang được kiểm tra schema (tối đa 5.0 m) nhưng bộ chọn camera **chưa dùng** ngưỡng này: radar hợp lệ nào cũng có thể chuyển camera. Ngưỡng 5.0 m trùng tầm tối đa của LD2410C nên với cấu hình mặc định không đổi hành vi.

Radar không tạo mức cảnh báo SAFE/CAUTION/DANGER. Nó chỉ chọn camera hông. Cảnh báo khoảng cách phía trước đến từ fusion camera trước + LiDAR.

## 10. LiDAR

Repo không có driver phần cứng. Tạo `drivers/real_lidar.py` theo mẫu trong `drivers/README.md`. Hàm factory phải nhận `config` và `fps`. `read()` trả điểm float shape `(N, 3)`, đơn vị mét. Không sinh điểm ngẫu nhiên.

Đăng ký đúng tên sẽ ghi trong YAML, ví dụ `"livox"`, rồi sửa:

```yaml
lidar:
  enabled: true
  source: livox
  driver: livox
  driver_module: drivers.real_lidar
  fps: 10
```

`source: CHANGE_ME` hoặc `driver: mock` làm preflight và pipeline live dừng.

## 11. Calibration

Tạo `config/calibration_live.yaml` từ cụm đã lắp cố định (ma trận K, distortion, R, t). Không chép `config/calibration.yaml` của demo sang.

Kiểm overlay:

```bash
.venv/bin/python tools/check_projection.py \
  --image data/calibration_frame.png \
  --lidar data/calibration_points.npy \
  --calibration config/calibration_live.yaml
```

Chỉ khi điểm LiDAR nằm đúng vật trên ảnh mới sửa:

```yaml
safety:
  allow_mock: false
  is_demo_calibration: false
```

`is_demo_calibration: true` (giá trị đang có trong repo) khiến preflight và pipeline từ chối chạy. Đó là chặn có chủ đích, không phải lỗi cài đặt.

## 12. Chạy

Kiểm cổng trước:

```bash
.venv/bin/python preflight.py --config config/jetson_live.yaml
```

Hết lỗi thì:

```bash
./start_jetson.sh
```

Script luôn chạy pipeline `--mode live --device cuda --headless`. Cửa sổ OpenCV không mở. Dashboard:

- Preflight lỗi: `http://127.0.0.1:8765` (lắng nghe mọi interface)
- Preflight đạt: cùng cổng, chỉ localhost

Trạng thái ghi ở `logs/preflight_status.json`.

Đừng chạy `./run.sh` hoặc `python run.py` không đối số với hy vọng ra demo. Launcher production chỉ nhận `live` và `camera-only`. README gốc còn mô tả mode demo; phần đó không còn đúng với `run.py` hiện tại.

Chỉ xem camera, không AI:

```bash
.venv/bin/python run.py --mode camera-only --port 8765
```

## 13. Tự chạy khi bật nguồn

Chỉ làm sau khi `./start_jetson.sh` chạy tay ổn định:

```bash
./deploy/install_service.sh
sudo systemctl start adas.service
sudo systemctl status adas.service
journalctl -u adas.service -f
```

Service dùng user hiện tại và `start_jetson.sh`. Sửa source xong không cần cài lại service.

## 14. Việc pipeline làm khi đã chạy

```text
5 camera USB ── chọn 1 camera theo radar hông
camera trước + LiDAR ── lọc ── DBSCAN ── chiếu lên ảnh ── fusion khoảng cách
                                              │
                                    DistanceWarning (không TTC)
                                              │
                         UNKNOWN / SAFE / CAUTION / DANGER / EMERGENCY
```

Ngưỡng mặc định: caution 12 m, danger 6 m, emergency 3 m, nửa làn 1.75 m. Vật ngoài làn bị coi là SAFE. Chỉ frame `camera_id=front` được fusion. Camera hông có hình và có radar, không có khoảng cách LiDAR.

Live không được phép mock. Thiếu model, thiếu CUDA, mất camera, mất LiDAR, lệch timestamp quá `sync_timeout_seconds` (2 s) thì pipeline dừng thay vì bịa dữ liệu.

## 15. Lỗi hay gặp

| Triệu chứng | Cách xử lý |
|---|---|
| `Không phát hiện Jetson Linux` | Không chạy `setup_jetson.sh` trên PC. Máy phải có `/etc/nv_tegra_release`. |
| `Thiếu PyTorch CUDA` | Cài wheel NVIDIA đúng JetPack. `torch.cuda.is_available()` phải là True. |
| `Thiếu OpenCV` | `sudo apt install python3-opencv`. Không pip opencv-python. |
| `Calibration vẫn được đánh dấu demo` | Đặt `safety.is_demo_calibration: false` sau khi đã kiểm overlay. |
| `CHANGE_ME` ở radar hoặc LiDAR | Sửa `radar_gateway.device`, `lidar.driver`, `lidar.source`. |
| `Không tìm thấy module driver LiDAR: drivers.real_lidar` | File đó chưa có. Viết driver rồi mới chạy live. |
| `allowed_classes không khớp model` | Sửa YAML theo tên class in ra lúc export. |
| `Không đọc được camera` | Dùng `/dev/v4l/by-id/...index0`, user thuộc nhóm `video`. |
| `Gateway không gửi JSON v2` | Đúng firmware gateway, baud 115200, cổng by-id, user thuộc nhóm `dialout`. |
| `remote_ok=false` | ESP32 B mất ESP-NOW hoặc mất nguồn. Hai radar phải không vào live. |
| Dashboard không mở | Xem `logs/preflight_status.json`. Cổng mặc định 8765. |

## 16. Thứ tự nghiệm thu ngắn

1. `python3 -c 'import torch; print(torch.cuda.is_available())'` in `True`.
2. `v4l2-ctl --list-devices` thấy đủ camera.
3. ESP32 A xuất hiện trong `/dev/serial/by-id/`.
4. `preflight.py` không in `[FAIL]`.
5. `./start_jetson.sh` lên dashboard và có ảnh camera trước.
6. Che hoặc đưa vật vào tầm một radar hông: ảnh dashboard đổi sang camera tương ứng.
7. Vật trong làn, trong tầm LiDAR: mức cảnh báo đổi theo khoảng cách, không theo TTC.
