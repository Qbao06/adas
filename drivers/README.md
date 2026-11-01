# LiDAR hardware driver

Tạo một module, ví dụ `drivers/my_lidar.py`, và đăng ký tên trùng với
`lidar.driver` trong `config/jetson_live.yaml`.

```python
from sensors.lidar_reader import LidarReaderFactory, _BaseLidarReader
from core.types import LidarFrame


class MyLidarReader(_BaseLidarReader):
    def __init__(self, config: dict, fps: int = 10):
        self.config = config
        self.connected = False
        self.frame_id = 0

    def connect(self) -> None:
        # Khởi tạo SDK/socket của đúng model LiDAR.
        self.connected = True

    def read(self) -> LidarFrame | None:
        # Decode một scan thật thành float ndarray shape (N, 3), đơn vị mét.
        # Timestamp phải dùng clock đồng bộ với camera.
        raise NotImplementedError

    def is_connected(self) -> bool:
        return self.connected

    def disconnect(self) -> None:
        self.connected = False


LidarReaderFactory.register(
    "my_lidar", lambda **kwargs: MyLidarReader(**kwargs)
)
```

Không tạo dữ liệu ngẫu nhiên trong driver live. SDK timeout, packet loss và lỗi
kết nối phải được báo qua exception hoặc trạng thái disconnected để watchdog
chuyển hệ thống sang lỗi.
