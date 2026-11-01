"""High-level LiDAR processing pipeline."""
from __future__ import annotations

import logging
from typing import Any, Tuple

import numpy as np

from core.types import LidarCluster, LidarFrame

from .clustering import cluster_points
from .filters import filter_points, reject_outliers

LOGGER = logging.getLogger(__name__)


class LidarProcessor:
    """Filter + cluster. Always returns numpy arrays / lists even when empty."""

    def __init__(self, config: dict) -> None:
        self.config = config or {}

    def process(
        self, frame: LidarFrame
    ) -> Tuple[np.ndarray, list[LidarCluster]]:
        cfg = self.config
        try:
            from sensors.normalize import normalize_lidar_cloud

            frame_points = normalize_lidar_cloud(frame.points)
        except Exception as exc:
            LOGGER.warning("Chuẩn hoá LiDAR lỗi: %s", exc)
            frame_points = np.empty((0, 3), dtype=float)
        try:
            points = filter_points(
                frame_points,
                min_range=cfg["min_range_m"],
                max_range=cfg["max_range_m"],
                roi=cfg["roi"],
            )
        except Exception as exc:  # pragma: no cover - defensive
            LOGGER.warning("filter_points lỗi: %s", exc)
            points = np.empty((0, 3), dtype=float)
        try:
            cluster_cfg = dict(cfg.get("clustering", {}))
            # Optional pre-DBSCAN reduction; both default to off (0 = disabled)
            # so demo/test behaviour is unchanged unless configured.
            cluster_cfg.setdefault("voxel_size", cfg.get("voxel_size", 0.0))
            cluster_cfg.setdefault("max_points", cfg.get("max_points", 0))
            clusters = cluster_points(
                points,
                frame.timestamp,
                **cluster_cfg,
            )
        except Exception as exc:  # pragma: no cover - defensive
            LOGGER.warning("cluster_points lỗi: %s", exc)
            clusters = []
        return points, clusters


__all__ = ["LidarProcessor"]
