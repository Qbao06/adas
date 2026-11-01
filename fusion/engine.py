"""Compose detections + LiDAR points into ``FusedObject`` records."""
from __future__ import annotations

import logging
from typing import Any, Optional, Tuple

import numpy as np

from core.enums import FusionStatus
from core.types import Detection, FusedObject, ProjectedPoints
from .association import points_for_bbox, resolve_overlapping
from .distance import estimate_distance

LOGGER = logging.getLogger(__name__)


class FusionEngine:
    """Pair each detection with the LiDAR points that fall inside its bbox."""

    def __init__(
        self,
        projector,
        config: dict,
    ) -> None:
        self.projector = projector
        self.config = config or {}

    def _estimate(self, points: np.ndarray) -> Tuple[Optional[float], Optional[np.ndarray]]:
        cfg = self.config
        distance = estimate_distance(
            points,
            mode=cfg.get("distance_mode", "adaptive"),
            outlier_method=cfg.get("outlier_method", "iqr"),
            outlier_k=float(cfg.get("outlier_k", 1.5)),
        )
        position: Optional[np.ndarray] = None
        if distance is not None:
            arr = np.asarray(points, dtype=float)
            if arr.ndim == 2 and arr.shape[0] > 0 and arr.shape[1] >= 2:
                norms = np.linalg.norm(arr[:, :2], axis=1)
                idx = int(np.argmin(np.abs(norms - distance)))
                position = arr[idx]
        return distance, position

    def fuse(
        self,
        detections,
        lidar_points,
        image_shape,
        timestamp: float,
    ) -> Tuple[list[FusedObject], ProjectedPoints]:
        cfg = self.config
        min_points = int(cfg.get("min_points", 3))
        bbox_scale = float(cfg.get("bbox_scale", 0.85))
        enabled = bool(cfg.get("enabled", True))

        if not enabled:
            empty = ProjectedPoints(
                pixels=np.empty((0, 2)),
                camera_points=np.empty((0, 3)),
                lidar_points=np.empty((0, 3)),
                source_indices=np.empty(0, dtype=int),
            )
            return [
                self._to_fused(det, None, None, FusionStatus.NO_POINTS, timestamp, 0)
                for det in detections
            ], empty

        if lidar_points is None:
            lidar_points = np.empty((0, 3), dtype=float)
        projected = self.projector.project(lidar_points, image_shape)
        assignment = resolve_overlapping(
            detections,
            projected,
            bbox_scale=bbox_scale,
            min_points=min_points,
        )
        objects: list[FusedObject] = []
        for i, det in enumerate(detections):
            pts = assignment.get(i)
            if pts is None:
                pts = points_for_bbox(projected, det.bbox, scale=bbox_scale)
            status, distance, position = (
                FusionStatus.NO_POINTS,
                None,
                None,
            )
            count = int(len(pts))
            if 0 < count < min_points:
                status = FusionStatus.INSUFFICIENT_POINTS
            elif count:
                distance, position = self._estimate(pts)
                if distance is None:
                    status = FusionStatus.INVALID_DISTANCE
                else:
                    status = FusionStatus.FUSED
            objects.append(
                self._to_fused(
                    det,
                    distance,
                    position,
                    status,
                    timestamp,
                    count,
                )
            )
        return objects, projected

    def _to_fused(
        self,
        det: Detection,
        distance: Optional[float],
        position: Optional[np.ndarray],
        status: FusionStatus,
        timestamp: float,
        count: int,
    ) -> FusedObject:
        return FusedObject(
            track_id=det.track_id if det.track_id is not None else -1,
            class_name=det.class_name,
            confidence=det.confidence,
            bbox=det.bbox,
            timestamp=timestamp,
            position_3d=position,
            raw_distance=distance,
            filtered_distance=distance,
            relative_speed=None,
            ttc=None,
            lidar_point_count=count,
            fusion_status=status,
        )


__all__ = ["FusionEngine"]
