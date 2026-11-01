"""Extra edge-case tests covering all scenarios listed in the spec."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from adas.motion import MotionEstimator
from adas.warning import CollisionWarning
from config import load_config, coerce_config
from core.enums import FusionStatus, WarningLevel
from core.types import CameraFrame, Detection, FusedObject, LidarFrame, ProjectedPoints
from fusion.engine import FusionEngine
from fusion.temporal_filter import TemporalFilter
from lidar.filters import filter_points, reject_outliers


ROOT = Path(__file__).resolve().parents[1]


# ----------------- input rỗng -----------------

def test_empty_detection_list():
    # Empty detection list should produce empty fused objects.
    from calibration import LidarCameraProjector
    cfg = load_config(str(ROOT / "config" / "adas.yaml"))
    coerce_config(cfg)
    projector = LidarCameraProjector(str(ROOT / "config" / "calibration.yaml"))
    engine = FusionEngine(projector, cfg["fusion"])
    objects, projected = engine.fuse(
        [], np.empty((0, 3)), (10, 10, 3), 1.0
    )
    assert objects == []
    assert projected is not None


def test_empty_lidar_points():
    from calibration import LidarCameraProjector
    cfg = load_config(str(ROOT / "config" / "adas.yaml"))
    coerce_config(cfg)
    projector = LidarCameraProjector(str(ROOT / "config" / "calibration.yaml"))
    engine = FusionEngine(projector, cfg["fusion"])
    dets = [Detection(2, "car", 0.9, (10, 10, 50, 50), 1.0)]
    objects, projected = engine.fuse(dets, np.empty((0, 3)), (10, 10, 3), 1.0)
    assert len(objects) == 1
    assert objects[0].lidar_point_count == 0


# ----------------- NaN/Inf -----------------

def test_nan_inf_in_distance():
    arr = np.array([10.0, np.nan, 11.0, np.inf, 10.5])
    out = reject_outliers(arr)
    assert np.isfinite(out).all()


def test_nan_lidar_filtered():
    pts = np.array([[1, 0, 0], [np.nan, 1, 0], [2, 0, 0]])
    out = filter_points(pts, 0.5, 10.0, dict(x_min=-10, x_max=10, y_min=-10, y_max=10, z_min=-2, z_max=2))
    assert np.isfinite(out).all()


# ----------------- Một vật thể -----------------

def test_single_object_distance_is_reasonable():
    pts = np.array([[10, 0, 0], [10.1, 0, 0], [10.05, 0, 0]])
    from fusion.distance import estimate_distance
    d = estimate_distance(pts, "median")
    assert 9.5 < d < 10.5


# ----------------- Nhiều vật thể -----------------

def test_multiple_clusters_in_distance():
    # Two clear clusters: 10m and 30m.
    a = np.array([[10, 0, 0]] * 5)
    b = np.array([[30, 0, 0]] * 5)
    rng = np.random.default_rng(0)
    pts = np.vstack([
        a + rng.normal(0, 0.1, a.shape),
        b + rng.normal(0, 0.1, b.shape),
    ])
    # The median would be 20. q1 would be closer to 10.
    from fusion.distance import estimate_distance
    q1 = estimate_distance(pts, "q1")
    median = estimate_distance(pts, "median")
    assert q1 < median


# ----------------- Outlier -----------------

def test_outlier_excluded_iqr():
    pts = np.array([[10, 0, 0]] * 5 + [[10.05, 0, 0]] * 5 + [[200, 0, 0]])
    from fusion.distance import estimate_distance
    d = estimate_distance(pts, "median")
    assert d < 11


# ----------------- Bounding box chồng nhau -----------------

def test_overlapping_bboxes_distribute_points():
    from fusion.association import resolve_overlapping
    detections = [
        Detection(2, "car", 0.9, (10, 10, 50, 50), 0.0),
        Detection(2, "car", 0.7, (30, 10, 70, 50), 0.0),
    ]
    pixels = np.array([[20, 20], [40, 30], [60, 30]], dtype=float)
    lidar = np.array([[5.0, 0, 0], [12.0, 0, 0], [25.0, 0, 0]])
    proj = ProjectedPoints(
        pixels=pixels,
        camera_points=np.zeros((3, 3)),
        lidar_points=lidar,
        source_indices=np.arange(3),
    )
    assignment = resolve_overlapping(detections, proj, bbox_scale=1.0)
    # Each detection should get exactly one LiDAR point (the other one was
    # claimed by the higher-confidence detection).
    assert len(assignment[0]) >= 1
    assert len(assignment[1]) >= 1
    assert len(assignment[0]) + len(assignment[1]) == 3


# ----------------- Track ID thay đổi -----------------

def test_track_id_is_stable_across_frames():
    from vision.tracker import IoUTracker
    t = IoUTracker(min_hits=1, max_age=30, iou_threshold=0.3)
    id1 = t.update([Detection(2, "car", 0.9, (10, 10, 50, 50), 0.0)], 0.0)[0].track_id
    id2 = t.update([Detection(2, "car", 0.9, (12, 12, 52, 52), 0.1)], 0.1)[0].track_id
    assert id1 == id2


# ----------------- Khoảng cách nhảy bất thường -----------------

def test_ema_smoothing_jumps():
    f = TemporalFilter(alpha=0.35, stale_seconds=10.0)
    # Initial distance 10.
    obj = FusedObject(track_id=1, class_name="car", confidence=0.9,
                     bbox=(0, 0, 10, 10), timestamp=0.0,
                     raw_distance=10.0, filtered_distance=None,
                     fusion_status=FusionStatus.FUSED)
    f.update([obj], 0.0)
    assert obj.filtered_distance == 10.0
    # Next frame jumps to 5. EMA: 0.35*5 + 0.65*10 = 8.25
    obj2 = FusedObject(track_id=1, class_name="car", confidence=0.9,
                      bbox=(0, 0, 10, 10), timestamp=0.1,
                      raw_distance=5.0, filtered_distance=None,
                      fusion_status=FusionStatus.FUSED)
    f.update([obj2], 0.1)
    assert 8.0 < obj2.filtered_distance < 8.5


def test_ema_holds_value_when_lidar_lost():
    f = TemporalFilter(alpha=0.35, stale_seconds=10.0, max_hold=5)
    obj = FusedObject(track_id=1, class_name="car", confidence=0.9,
                     bbox=(0, 0, 10, 10), timestamp=0.0,
                     raw_distance=10.0, filtered_distance=None,
                     fusion_status=FusionStatus.FUSED)
    f.update([obj], 0.0)
    # LiDAR returns nothing — EMA keeps last value.
    obj2 = FusedObject(track_id=1, class_name="car", confidence=0.9,
                      bbox=(0, 0, 10, 10), timestamp=0.1,
                      raw_distance=None, filtered_distance=None,
                      fusion_status=FusionStatus.NO_POINTS)
    f.update([obj2], 0.1)
    assert obj2.filtered_distance == 10.0


def test_ema_releases_when_max_hold_reached():
    # After max_hold consecutive "no data" frames, the value is dropped.
    f = TemporalFilter(alpha=0.35, stale_seconds=100.0, max_hold=2)
    obj = FusedObject(track_id=1, class_name="car", confidence=0.9,
                     bbox=(0, 0, 10, 10), timestamp=0.0,
                     raw_distance=10.0, filtered_distance=None,
                     fusion_status=FusionStatus.FUSED)
    f.update([obj], 0.0)
    obj2 = FusedObject(track_id=1, class_name="car", confidence=0.9,
                      bbox=(0, 0, 10, 10), timestamp=0.1,
                      raw_distance=None, filtered_distance=None,
                      fusion_status=FusionStatus.NO_POINTS)
    f.update([obj2], 0.1)
    obj3 = FusedObject(track_id=1, class_name="car", confidence=0.9,
                      bbox=(0, 0, 10, 10), timestamp=0.2,
                      raw_distance=None, filtered_distance=None,
                      fusion_status=FusionStatus.NO_POINTS)
    f.update([obj3], 0.2)
    obj4 = FusedObject(track_id=1, class_name="car", confidence=0.9,
                      bbox=(0, 0, 10, 10), timestamp=0.3,
                      raw_distance=None, filtered_distance=None,
                      fusion_status=FusionStatus.NO_POINTS)
    f.update([obj4], 0.3)
    assert obj4.filtered_distance is None


# ----------------- Vật thể đứng yên / tiến gần / đi xa -----------------

def test_stationary_object_speed_zero():
    m = MotionEstimator(alpha=1.0, min_closing_mps=0.05)
    m.update([
        _obj(1, 10.0, None)
    ], 0.0)
    obj = _obj(1, 10.0, None)
    m.update([obj], 1.0)
    assert obj.relative_speed == 0.0


def test_approaching_object_speed_positive():
    m = MotionEstimator(alpha=1.0, min_closing_mps=0.05)
    m.update([_obj(1, 20.0, None)], 0.0)
    obj = _obj(1, 10.0, None)
    m.update([obj], 1.0)
    assert obj.relative_speed > 0


def test_receding_object_speed_negative():
    m = MotionEstimator(alpha=1.0, min_closing_mps=0.05)
    m.update([_obj(1, 10.0, None)], 0.0)
    obj = _obj(1, 15.0, None)
    m.update([obj], 1.0)
    assert obj.relative_speed < 0


# ----------------- Camera / LiDAR mất dữ liệu -----------------

def test_pipeline_handles_missing_camera():
    from app import ADASPipeline
    cfg = load_config(str(ROOT / "config" / "adas.yaml"))
    coerce_config(cfg)
    cfg["mode"] = "lidar-only"
    pipeline = ADASPipeline(cfg, root=ROOT)
    # camera=None means missing → pipeline must fall back to placeholder.
    assert pipeline.camera is None
    pipeline.run(max_frames=2, display=False)


def test_pipeline_handles_missing_lidar():
    from app import ADASPipeline
    cfg = load_config(str(ROOT / "config" / "adas.yaml"))
    coerce_config(cfg)
    cfg["mode"] = "camera-only"
    pipeline = ADASPipeline(cfg, root=ROOT)
    assert pipeline.lidar is None
    pipeline.run(max_frames=2, display=False)


# ----------------- Calibration sai định dạng -----------------

def test_bad_calibration_raises(tmp_path):
    from calibration.loader import load_calibration, CalibrationError
    bad = tmp_path / "bad.yaml"
    bad.write_text("camera_matrix: [[1, 0, 0], [0, 1, 0]]\n")
    with pytest.raises((CalibrationError, ValueError, KeyError)):
        load_calibration(bad)


# ----------------- TTC không hợp lệ -----------------

def test_ttc_is_inf_when_distance_constant():
    m = MotionEstimator(alpha=1.0, min_closing_mps=0.05)
    m.update([_obj(1, 10.0, None)], 0.0)
    obj = _obj(1, 10.0, None)
    m.update([obj], 1.0)
    assert np.isinf(obj.ttc)


def _obj(tid, distance, ttc=None):
    obj = FusedObject(track_id=tid, class_name="car", confidence=0.9,
                     bbox=(0, 0, 10, 10), timestamp=0.0,
                     raw_distance=distance, filtered_distance=distance,
                     fusion_status=FusionStatus.FUSED)
    obj.ttc = ttc
    return obj