import numpy as np
import pytest

from lidar.clustering import cluster_points
from lidar.filters import filter_points, normalize_points, reject_outliers


ROI = dict(x_min=-50, x_max=50, y_min=-5, y_max=5, z_min=-2, z_max=2)


def test_normalize_points_accepts_2d():
    pts = np.array([[1.0, 2.0], [3.0, 4.0]])
    arr = normalize_points(pts)
    assert arr.shape == (2, 3)
    assert np.all(arr[:, 2] == 0)


def test_normalize_points_rejects_bad_shape():
    with pytest.raises(ValueError):
        normalize_points(np.array([1.0, 2.0, 3.0]))


def test_filter_points_nan_inf():
    pts = np.array(
        [
            [10.0, 0.0, 0.0],
            [np.nan, 0.0, 0.0],
            [np.inf, 0.0, 0.0],
            [-np.inf, 0.0, 0.0],
            [200.0, 0.0, 0.0],
        ]
    )
    out = filter_points(pts, 1.0, 50.0, ROI)
    assert len(out) == 1


def test_filter_points_distance_window():
    pts = np.array(
        [
            [0.1, 0.0, 0.0],   # too close (0.1 < 0.3)
            [10.0, 0.0, 0.0],  # kept
            [70.0, 0.0, 0.0],  # too far
        ]
    )
    out = filter_points(pts, 0.3, 60.0, ROI)
    assert len(out) == 1


def test_filter_points_roi():
    pts = np.array(
        [
            [10.0, 0.0, 0.0],     # ok
            [10.0, 12.0, 0.0],    # y outside
            [10.0, 0.0, 10.0],    # z outside
        ]
    )
    out = filter_points(pts, 0.3, 60.0, ROI)
    assert len(out) == 1


def test_filter_points_empty():
    out = filter_points(np.empty((0, 3)), 0.3, 60.0, ROI)
    assert out.shape == (0, 3)


def test_reject_outliers_iqr_robust():
    values = np.array([10.0, 10.2, 9.9, 10.1, 200.0])
    out = reject_outliers(values, method="iqr", k=1.5)
    assert 200.0 not in out


def test_reject_outliers_mad():
    values = np.array([1.0, 1.05, 1.1, 1.02, 50.0])
    out = reject_outliers(values, method="mad", k=3.0)
    assert 50.0 not in out


def test_reject_outliers_handles_small_samples():
    values = np.array([1.0, 2.0])
    assert len(reject_outliers(values)) == 2


def test_cluster_points_groups_known_clusters():
    rng = np.random.default_rng(0)
    cluster_a = rng.normal([5, 0, 0], 0.1, (20, 3))
    cluster_b = rng.normal([25, -2, 0], 0.1, (20, 3))
    pts = np.vstack([cluster_a, cluster_b])
    clusters = cluster_points(pts, timestamp=0.0, eps=0.5, min_samples=4)
    assert len(clusters) == 2


def test_cluster_points_empty():
    clusters = cluster_points(np.empty((0, 3)), timestamp=0.0)
    assert clusters == []
