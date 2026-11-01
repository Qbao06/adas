"""DBSCAN clustering with safe fallback when sklearn is missing."""
from __future__ import annotations

import logging
from typing import Optional

import numpy as np

from core.types import LidarCluster

LOGGER = logging.getLogger(__name__)

try:
    from sklearn.cluster import DBSCAN  # type: ignore

    _HAVE_SKLEARN = True
except Exception:  # pragma: no cover - optional dependency
    DBSCAN = None  # type: ignore
    _HAVE_SKLEARN = False


def _dbscan_fallback(points: np.ndarray, eps: float, min_samples: int) -> np.ndarray:
    """Small NumPy DBSCAN implementation used when sklearn is unavailable.

    This is intentionally O(n^2): it keeps demo/test installations functional
    without silently merging unrelated objects. Real LiDAR workloads should
    still install scikit-learn for its optimized neighbour search.
    """
    n = len(points)
    labels = np.full(n, -1, dtype=int)
    visited = np.zeros(n, dtype=bool)
    distances = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
    neighbours = [np.flatnonzero(distances[i] <= eps) for i in range(n)]
    cluster_id = 0
    for seed in range(n):
        if visited[seed]:
            continue
        visited[seed] = True
        if len(neighbours[seed]) < min_samples:
            continue
        labels[seed] = cluster_id
        queue = list(neighbours[seed])
        queued = set(queue)
        cursor = 0
        while cursor < len(queue):
            point_id = queue[cursor]
            cursor += 1
            if not visited[point_id]:
                visited[point_id] = True
                local = neighbours[point_id]
                if len(local) >= min_samples:
                    for candidate in local:
                        candidate = int(candidate)
                        if candidate not in queued:
                            queued.add(candidate)
                            queue.append(candidate)
            if labels[point_id] < 0:
                labels[point_id] = cluster_id
        cluster_id += 1
    return labels


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Keep one point per voxel cell. No-op when ``voxel_size`` <= 0."""
    arr = np.asarray(points, dtype=float)
    if arr.size == 0 or voxel_size is None or float(voxel_size) <= 0:
        return arr
    keys = np.floor(arr / float(voxel_size)).astype(np.int64)
    _, keep = np.unique(keys, axis=0, return_index=True)
    return arr[np.sort(keep)]


def cluster_points(
    points: np.ndarray,
    timestamp: float,
    eps: float = 0.7,
    min_samples: int = 4,
    voxel_size: float = 0.0,
    max_points: int = 0,
) -> list[LidarCluster]:
    """Cluster the point cloud into ``LidarCluster`` objects.

    Falls back to a single "everything is one cluster" mode if sklearn is
    not installed (so the rest of the pipeline can still exercise).
    """
    if len(points) < max(int(min_samples), 1):
        return []
    arr = np.asarray(points, dtype=float)
    arr = arr[np.isfinite(arr).all(axis=1)]
    if voxel_size and float(voxel_size) > 0:
        arr = voxel_downsample(arr, float(voxel_size))
    if max_points and int(max_points) > 0 and len(arr) > int(max_points):
        idx = np.linspace(0, len(arr) - 1, int(max_points)).astype(int)
        arr = arr[idx]
    if len(arr) < max(int(min_samples), 1):
        return []
    if _HAVE_SKLEARN:
        labels = DBSCAN(eps=float(eps), min_samples=int(min_samples)).fit_predict(arr)
    else:
        LOGGER.debug("sklearn không có — dùng NumPy DBSCAN fallback")
        labels = _dbscan_fallback(arr, float(eps), int(min_samples))
    clusters: list[LidarCluster] = []
    for label in sorted(set(int(v) for v in labels) - {-1}):
        cloud = arr[labels == label]
        if len(cloud) == 0:
            continue
        center = np.median(cloud, axis=0)
        dimensions = np.ptp(cloud, axis=0)
        distance = float(np.linalg.norm(center[:2]))
        clusters.append(
            LidarCluster(
                cluster_id=int(label),
                points=cloud,
                center=center,
                dimensions=dimensions,
                distance=distance,
                timestamp=timestamp,
            )
        )
    return clusters


__all__ = ["cluster_points", "voxel_downsample", "_HAVE_SKLEARN"]
