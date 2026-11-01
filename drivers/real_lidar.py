"""Khung driver LiDAR cho live. Không sinh điểm giả.

Đổi ``lidar.driver`` trong ``config/jetson_live.yaml`` thành ``real_lidar``
chỉ sau khi đã viết ``connect`` / ``read`` thật. ``CHANGE_ME`` vẫn bị preflight chặn.
"""
from __future__ import annotations

from typing import Optional

from core.types import LidarFrame
from sensors.lidar_reader import LidarReaderFactory, _BaseLidarReader


class RealLidarReader(_BaseLidarReader):
    name = "real_lidar"

    def __init__(self, config: dict | None = None, fps: int = 10, **_: object) -> None:
        self.config = dict(config or {})
        self.fps = int(fps)
        self._connected = False
        self.frame_id = 0

    def connect(self) -> None:
        raise RuntimeError(
            "drivers.real_lidar là khung trống. Hãy triển khai connect/read "
            "cho LiDAR thật theo drivers/README.md rồi mới chạy live."
        )

    def read(self) -> Optional[LidarFrame]:
        return None

    def is_connected(self) -> bool:
        return self._connected

    def disconnect(self) -> None:
        self._connected = False


LidarReaderFactory.register(
    "real_lidar",
    lambda **kwargs: RealLidarReader(**kwargs),
)
