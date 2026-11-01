import math

import numpy as np
import pytest

from core.types import CameraFrame, Detection, FusedObject, LidarCluster, LidarFrame, ProjectedPoints
from core.enums import FusionStatus, SystemStatus, WarningLevel


def test_camera_frame_structure():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    frame = CameraFrame(image=img, timestamp=1.0, frame_id=5)
    assert frame.image.shape == (480, 640, 3)
    assert frame.timestamp == 1.0
    assert frame.frame_id == 5


def test_lidar_frame_empty():
    frame = LidarFrame(points=np.empty((0, 3)), timestamp=2.0, frame_id=3)
    assert frame.points.shape == (0, 3)


def test_detection_with_track_id():
    det = Detection(2, "car", 0.95, (0, 0, 100, 100), 0.0, track_id=7)
    assert det.track_id == 7


def test_fused_object_has_all_fields():
    pos = np.array([10.0, 0.5, 0.0])
    obj = FusedObject(
        track_id=1,
        class_name="car",
        confidence=0.9,
        bbox=(0, 0, 100, 100),
        timestamp=0.0,
        position_3d=pos,
        raw_distance=10.5,
        filtered_distance=10.3,
        relative_speed=1.2,
        ttc=8.5,
        lidar_point_count=25,
        fusion_status=FusionStatus.FUSED,
        warning_level=WarningLevel.SAFE,
    )
    assert obj.position_3d is not None
    assert obj.ttc is not None
    assert obj.warning_level == WarningLevel.SAFE


def test_fused_object_nan_distance():
    obj = FusedObject(
        track_id=1,
        class_name="car",
        confidence=0.9,
        bbox=(0, 0, 10, 10),
        timestamp=0.0,
        position_3d=None,
        raw_distance=float("nan"),
        filtered_distance=None,
        fusion_status=FusionStatus.INVALID_DISTANCE,
        warning_level=WarningLevel.UNKNOWN,
    )
    assert not math.isfinite(obj.raw_distance)
    assert obj.filtered_distance is None


def test_system_status_enum():
    for s in SystemStatus:
        assert isinstance(s.value, str)


def test_warning_level_enum():
    for w in WarningLevel:
        assert w.value in {"unknown", "safe", "caution", "danger", "emergency"}


def test_projected_points_empty():
    empty = ProjectedPoints(
        pixels=np.empty((0, 2)),
        camera_points=np.empty((0, 3)),
        lidar_points=np.empty((0, 3)),
        source_indices=np.empty(0, dtype=int),
    )
    assert len(empty.pixels) == 0
    assert len(empty.camera_points) == 0
