import math

import numpy as np
import pytest

from core.enums import FusionStatus, WarningLevel
from core.types import Detection, FusedObject, ProjectedPoints
from fusion.association import points_for_bbox, resolve_overlapping
from fusion.distance import estimate_distance


def test_estimate_distance_modes():
    pts = np.array([[10.0, 0, 0], [10.1, 0, 0], [9.9, 0, 0]])
    assert estimate_distance(pts, "nearest") == pytest.approx(9.9, abs=0.1)
    assert estimate_distance(pts, "mean") == pytest.approx(10.0, abs=0.1)
    assert estimate_distance(pts, "median") == pytest.approx(10.0, abs=0.1)
    assert estimate_distance(pts, "q1") < 10.0
    assert estimate_distance(pts, "trimmed_mean") == pytest.approx(10.0, abs=0.1)


def test_estimate_distance_adaptive_picks_q1_when_enough_points():
    pts = np.random.RandomState(0).normal(0.0, 0.2, (40, 3))
    pts[:, 0] += 15.0   # all x = 15±0.2, y=z=0±0.2
    pts[:, 1] = 0
    pts[:, 2] = 0
    d = estimate_distance(pts, "adaptive")
    assert d is not None and 14.5 < d < 15.5


def test_estimate_distance_empty():
    assert estimate_distance(np.empty((0, 3))) is None


def test_estimate_distance_rejects_outlier():
    pts = np.array([[10, 0, 0], [10.1, 0, 0], [9.9, 0, 0], [200, 0, 0]])
    d = estimate_distance(pts, "median")
    assert d is not None and d < 12


def test_invalid_distance_mode_falls_back_to_median():
    pts = np.array([[10, 0, 0], [10.1, 0, 0]])
    d = estimate_distance(pts, "garbage")
    assert d == pytest.approx(10.05, abs=0.05)


def _projected(pixels, lidar_points=None):
    n = len(pixels)
    if lidar_points is None:
        lidar_points = np.zeros((n, 3))
    return ProjectedPoints(
        pixels=np.asarray(pixels, dtype=float),
        camera_points=np.zeros((n, 3)),
        lidar_points=np.asarray(lidar_points, dtype=float),
        source_indices=np.arange(n, dtype=int),
    )


def test_points_for_bbox_basic():
    proj = _projected(
        [[5, 5], [50, 50]],
        lidar_points=[[10, 0, 0], [20, 0, 0]],
    )
    pts = points_for_bbox(proj, (0, 0, 20, 20), scale=1.0)
    assert len(pts) == 1
    assert pts[0][0] == 10


def test_points_for_bbox_scale_shrinks():
    proj = _projected(
        [[16, 5]],     # outside scaled half-size box (5, 5, 15, 15)
        lidar_points=[[10, 0, 0]],
    )
    pts = points_for_bbox(proj, (0, 0, 20, 20), scale=0.5)
    assert len(pts) == 0


def test_points_for_bbox_empty_projection():
    proj = _projected([])
    pts = points_for_bbox(proj, (0, 0, 100, 100))
    assert len(pts) == 0


def test_resolve_overlapping_disjoint_assignment():
    detections = [
        Detection(2, "car", 0.9, (10, 10, 30, 30), 0.0),
        Detection(2, "car", 0.8, (60, 10, 80, 30), 0.0),
    ]
    proj = _projected(
        [[15, 15], [70, 20]],
        lidar_points=[[8.0, 0, 0], [22.0, 0, 0]],
    )
    assignment = resolve_overlapping(detections, proj, bbox_scale=1.0)
    assert len(assignment[0]) == 1
    assert len(assignment[1]) == 1


def test_resolve_overlapping_steals_pixels_from_lower_confidence():
    detections = [
        Detection(2, "car", 0.9, (0, 0, 50, 50), 0.0),
        Detection(2, "car", 0.5, (10, 10, 60, 60), 0.0),
    ]
    proj = _projected(
        [[20, 20], [30, 30]],
        lidar_points=[[5.0, 0, 0], [30.0, 0, 0]],
    )
    assignment = resolve_overlapping(detections, proj, bbox_scale=1.0)
    assert len(assignment[0]) == 2
    assert len(assignment[1]) == 0
