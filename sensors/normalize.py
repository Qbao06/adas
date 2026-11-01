"""Runtime normalization for camera frames and LiDAR clouds.

Contract chuẩn đầu vào của pipeline:

Camera (BGR uint8, HxWx3, contiguous):
  - grayscale (H,W)      -> stack thành BGR
  - RGBA (H,W,4)         -> bỏ alpha
  - float [0,1]          -> scale lên uint8
  - RGB thay vì BGR      -> caller truyền ``input_order="rgb"`` để đổi kênh
  - không resize ở đây: resize làm sai calibration/projection nên YOLO
    tự letterbox nội bộ qua ``imgsz``.

LiDAR (float (N,3), hệ LiDAR: x-tiến, y-ngang, z-đứng):
  - (N,2)                -> pad z=0
  - (N,4+) có intensity  -> cắt lấy xyz
  - list/transposed      -> ép về (N,3)
  - loại NaN/Inf ngay tại chuẩn hoá để downstream không phải guard lại
"""
from __future__ import annotations

import logging

import numpy as np

LOGGER = logging.getLogger(__name__)


def normalize_camera_image(image, *, input_order: str = "bgr") -> np.ndarray:
    """Ép ảnh về BGR uint8 HxWx3 contiguous."""
    arr = np.asarray(image)
    if arr.ndim == 2:
        arr = np.stack((arr,) * 3, axis=-1)
    elif arr.ndim == 3 and arr.shape[2] == 4:
        arr = arr[:, :, :3]
    elif arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError(f"Ảnh phải có shape (H,W,3), got {arr.shape}")
    if arr.dtype != np.uint8:
        arr = np.asarray(arr, dtype=float)
        scale = 255.0 if np.nanmax(arr) <= 1.0 else 1.0
        arr = np.clip(arr * scale, 0, 255).astype(np.uint8)
    if input_order.lower() == "rgb":
        arr = arr[:, :, ::-1].copy()
    return np.ascontiguousarray(arr)


def undistort_camera_image(image, k, distortion):
    """Khử méo kính; trả nguyên ảnh khi không có cv2 hoặc D ~ 0."""
    try:
        import cv2
    except Exception:
        return image
    d = np.asarray(distortion, dtype=float).reshape(-1)
    if d.size == 0 or np.all(np.abs(d) < 1e-12):
        return image
    try:
        return cv2.undistort(np.ascontiguousarray(image), np.asarray(k, dtype=float), d)
    except Exception as exc:
        LOGGER.warning("Undistort thất bại, dùng ảnh gốc: %s", exc)
        return image


def normalize_lidar_cloud(points) -> np.ndarray:
    """Ép point cloud về float (N,3), đã loại NaN/Inf.

    Chấp nhận (N,2)/(N,3)/(N,4+), list rỗng, hoặc mảng 1D rỗng.
    """
    if points is None:
        return np.empty((0, 3), dtype=float)
    arr = np.asarray(points, dtype=float)
    if arr.size == 0:
        return np.empty((0, 3), dtype=float)
    if arr.ndim == 1:
        if arr.size % 3 != 0:
            raise ValueError(f"LiDAR 1D phải có số phần tử chia hết cho 3, got {arr.size}")
        arr = arr.reshape(-1, 3)
    if arr.ndim != 2 or arr.shape[1] < 2:
        raise ValueError(f"LiDAR points phải có shape (N,2+), got {arr.shape}")
    if arr.shape[1] == 2:
        arr = np.column_stack((arr, np.zeros(len(arr))))
    else:
        arr = arr[:, :3]
    return arr[np.isfinite(arr).all(axis=1)]


__all__ = [
    "normalize_camera_image",
    "normalize_lidar_cloud",
    "undistort_camera_image",
]
