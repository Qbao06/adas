import time

import numpy as np
import pytest

from core.types import CameraFrame, LidarFrame
from synchronization import SensorSynchronizer


def _cam(t: float, frame_id: int = 0) -> CameraFrame:
    return CameraFrame(np.zeros((10, 10, 3), dtype=np.uint8), t, frame_id)


def _lid(t: float, frame_id: int = 0, points: np.ndarray | None = None) -> LidarFrame:
    pts = points if points is not None else np.zeros((1, 3), dtype=float)
    return LidarFrame(pts, t, frame_id)


def test_empty_buffers_return_none():
    sync = SensorSynchronizer(50, 5, 500)
    assert sync.get_pair() is None


def test_no_lidar_data_returns_none():
    sync = SensorSynchronizer(50, 5, 500)
    sync.add_camera(_cam(1.0))
    assert sync.get_pair() is None


def test_picks_closest_pair():
    sync = SensorSynchronizer(100, 10, 1000)
    sync.add_camera(_cam(1.0))
    sync.add_camera(_cam(1.5))
    sync.add_lidar(_lid(1.05))
    sync.add_lidar(_lid(1.45))
    pair = sync.get_pair()
    assert pair is not None
    assert abs(pair.time_difference) < 0.1


def test_drops_pairs_outside_max_diff():
    sync = SensorSynchronizer(20, 10, 1000)
    sync.add_camera(_cam(0.0))
    sync.add_lidar(_lid(0.5))
    assert sync.get_pair() is None


def test_drops_stale_frames():
    sync = SensorSynchronizer(10, 10, stale_after_ms=300)
    sync.add_camera(_cam(0.0))
    sync.add_lidar(_lid(0.0))
    time.sleep(0.4)
    # Both buffers contain only stale data now → still returns None
    # and prunes them.
    sync._drop_stale(time.monotonic())
    assert len(sync.cameras) == 0


def test_pairing_after_pause():
    sync = SensorSynchronizer(50, 10, 5000)
    sync.add_camera(_cam(1.0, frame_id=0))
    sync.add_lidar(_lid(1.05, frame_id=0))
    first = sync.get_pair()
    assert first is not None
    sync.add_camera(_cam(1.20, frame_id=1))
    sync.add_lidar(_lid(1.22, frame_id=1))
    second = sync.get_pair()
    assert second is not None
    assert first.camera.frame_id != second.camera.frame_id
