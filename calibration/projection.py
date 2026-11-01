"""Project LiDAR points onto the image plane using extrinsics + intrinsics."""
from __future__ import annotations

import logging
from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore

from core.types import ProjectedPoints
from .loader import load_calibration

LOGGER = logging.getLogger(__name__)


class LidarCameraProjector:
    """Project LiDAR points onto the camera image plane.

    The transformation follows the standard LiDAR→Camera convention:
    ``p_cam = R @ p_lidar + t`` followed by ``p_img = K @ p_cam``.
    """

    def __init__(self, calibration_path) -> None:
        self.k, self.distortion, self.r, self.t = load_calibration(calibration_path)

    def project(self, points, image_shape) -> ProjectedPoints:
        points = np.asarray(points, dtype=float)
        if points.size == 0:
            empty = np.empty((0, 2))
            empty3 = np.empty((0, 3))
            return ProjectedPoints(empty, empty3, empty3.copy(), np.empty(0, dtype=int))
        if points.ndim == 1:
            points = points.reshape(1, -1)
        if points.ndim != 2 or points.shape[1] < 2:
            empty = np.empty((0, 2))
            empty3 = np.empty((0, 3))
            return ProjectedPoints(empty, empty3, empty3.copy(), np.empty(0, dtype=int))
        if points.shape[1] < 3:
            points = np.column_stack(
                (points[:, :2], np.zeros(len(points)))
            )
        source = np.arange(len(points))
        camera = (self.r @ points[:, :3].T).T + self.t.reshape(1, 3)
        valid = np.isfinite(camera).all(axis=1) & (camera[:, 2] > 1e-6)
        camera = camera[valid]
        lidar = points[valid, :3]
        source = source[valid]
        if not len(camera):
            return ProjectedPoints(
                np.empty((0, 2)),
                np.empty((0, 3)),
                np.empty((0, 3)),
                np.empty(0, dtype=int),
            )
        if cv2 is not None:
            pixels, _ = cv2.projectPoints(
                camera.reshape(-1, 1, 3),
                np.zeros(3), np.zeros(3), self.k, self.distortion,
            )
            pixels = pixels.reshape(-1, 2)
        else:
            if np.any(np.abs(self.distortion) > 1e-12):
                LOGGER.warning("OpenCV không có: projection bỏ qua distortion")
            homogeneous = (self.k @ camera.T).T
            with np.errstate(divide="ignore", invalid="ignore"):
                z = np.where(np.abs(homogeneous[:, 2]) < 1e-9, 1e-9, homogeneous[:, 2])
                pixels = homogeneous[:, :2] / z[:, None]
        h, w = image_shape[:2]
        inside = (
            (pixels[:, 0] >= 0)
            & (pixels[:, 0] < w)
            & (pixels[:, 1] >= 0)
            & (pixels[:, 1] < h)
        )
        return ProjectedPoints(
            pixels=pixels[inside],
            camera_points=camera[inside],
            lidar_points=lidar[inside],
            source_indices=source[inside],
        )


__all__ = ["LidarCameraProjector"]
