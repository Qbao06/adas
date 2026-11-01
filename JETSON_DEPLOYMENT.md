# Triển khai Jetson Orin Nano 8GB

Hướng dẫn đầy đủ (apt, udev, thư viện, phần cứng): [`HUONG_DAN_CAI_DAT_JETSON.md`](HUONG_DAN_CAI_DAT_JETSON.md).

## Cài đặt một lần

1. Flash JetPack 6.x và xác nhận CUDA/TensorRT hoạt động.
2. Cài PyTorch **và torchvision** wheel NVIDIA đúng JetPack (không `pip install torch` từ PyPI).
3. Trong thư mục dự án, chạy:

   ```bash
   chmod +x setup_jetson.sh start_jetson.sh
   ./setup_jetson.sh
   ```

`setup_jetson.sh` cài apt (`python3-opencv`, v4l-utils), udev UART/camera, `.venv --system-site-packages`, `requirements-jetson.txt`, `ultralytics --no-deps`, rồi `tools/verify_install.py --strict`. Không dùng `setup.sh` / `requirements.txt` trên Jetson (chúng cài `opencv-python` pip).

Nếu chỉ muốn tạo lại shortcut:

```bash
./deploy/create_shortcut.sh
```

## Ba đầu vào bắt buộc trước khi chạy live

### 1. Model TensorRT

Đặt `best.pt` trong `models/`, kiểm tra class names rồi export trên chính Jetson:

```bash
.venv/bin/python tools/export_tensorrt.py --model models/best.pt --imgsz 640
```

Kết quả phải là `models/best.engine`. Cập nhật `vision.allowed_classes` trong `config/jetson_live.yaml` để khớp chính xác tên class của model.

### 2. Driver LiDAR

Tạo module Python trong `drivers/`, đăng ký implementation bằng:

```python
LidarReaderFactory.register("ten_lidar", factory)
```

Factory nhận `config` và `fps`; reader phải triển khai `connect`, `read`, `is_connected`, `disconnect`. Sửa `lidar.driver`, `lidar.source` và `lidar.driver_module` trong `config/jetson_live.yaml`.

### 3. Calibration thật

Tạo `config/calibration_live.yaml` từ bộ camera–LiDAR đã lắp cố định. Sau khi kiểm chứng overlay và sai số, đổi:

```yaml
safety:
  is_demo_calibration: false
```

Không sao chép calibration mô phỏng sang file live.

Kiểm tra overlay trước khi đổi cờ:

```bash
.venv/bin/python tools/check_projection.py \
  --image data/calibration_frame.png \
  --lidar data/calibration_points.npy \
  --calibration config/calibration_live.yaml
```

## Khởi động

Nhấp biểu tượng `ADAS` trên Desktop hoặc chạy:

```bash
./start_jetson.sh
```

`preflight.py` sẽ chặn startup nếu thiếu model, CUDA, camera, driver hoặc calibration thật. Chế độ live không cho phép mock.

Nếu preflight thất bại, `start_jetson.sh` tự chạy dashboard chẩn đoán tại
`http://127.0.0.1:8765` và tự mở trình duyệt khi có Desktop. Dashboard hiển thị
trạng thái Camera, LiDAR, AI Engine, Calibration và cấu hình, đồng thời có nút
**Kiểm tra lại**. Nó không bao giờ khởi động demo.

Pipeline live mặc định cảnh báo theo khoảng cách, không tính vận tốc tương đối
hoặc TTC. Hãy hiệu chỉnh các ngưỡng `caution_distance_m`, `danger_distance_m`
và `emergency_distance_m` bằng thử nghiệm trong khu vực đóng trước khi sử dụng.

## Tự chạy khi bật nguồn (tùy chọn)

Thay `@PROJECT_DIR@` trong `deploy/adas.service.in` bằng đường dẫn tuyệt đối, sao chép thành service systemd, sau đó enable service. Chỉ thực hiện sau khi `./start_jetson.sh --headless` chạy ổn định bằng tay.

Hoặc dùng helper sau khi đã kiểm thử thủ công:

```bash
./deploy/install_service.sh
```
