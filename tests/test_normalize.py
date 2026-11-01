import numpy as np

from sensors.normalize import (
    normalize_camera_image,
    normalize_lidar_cloud,
    undistort_camera_image,
)


def test_normalize_camera_grayscale_to_bgr():
    gray = np.full((4, 6), 128, dtype=np.uint8)
    out = normalize_camera_image(gray)
    assert out.shape == (4, 6, 3)
    assert out.dtype == np.uint8


def test_normalize_camera_rgba_drops_alpha():
    img = np.zeros((4, 6, 4), dtype=np.uint8)
    out = normalize_camera_image(img)
    assert out.shape == (4, 6, 3)


def test_normalize_camera_float_scales():
    img = np.full((4, 6, 3), 0.5, dtype=float)
    out = normalize_camera_image(img)
    assert out.dtype == np.uint8
    assert out.max() > 100


def test_normalize_camera_rgb_swaps_to_bgr():
    img = np.zeros((2, 2, 3), dtype=np.uint8)
    img[:, :, 0] = 10
    img[:, :, 2] = 200
    out = normalize_camera_image(img, input_order="rgb")
    assert out[0, 0, 0] == 200
    assert out[0, 0, 2] == 10


def test_normalize_lidar_pads_xy_and_strips_intensity():
    out = normalize_lidar_cloud([[1.0, 2.0]])
    assert out.shape == (1, 3)
    assert out[0, 2] == 0.0
    out4 = normalize_lidar_cloud([[1.0, 2.0, 3.0, 0.9]])
    assert out4.shape == (1, 3)


def test_normalize_lidar_drops_nonfinite():
    pts = np.array([[1.0, 0.0, 0.0], [np.nan, 0.0, 0.0], [np.inf, 0.0, 0.0]])
    out = normalize_lidar_cloud(pts)
    assert len(out) == 1


def test_undistort_returns_original_when_no_distortion():
    img = np.zeros((4, 6, 3), dtype=np.uint8)
    k = np.eye(3)
    out = undistort_camera_image(img, k, np.zeros(5))
    assert out.shape == img.shape
