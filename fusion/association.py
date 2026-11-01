"""Associate projected LiDAR points with bounding boxes."""
from __future__ import annotations

import logging

import numpy as np

from core.types import ProjectedPoints

LOGGER = logging.getLogger(__name__)


def points_for_bbox(
    projected: ProjectedPoints,
    bbox,
    scale: float = 0.85,
) -> np.ndarray:
    """Return the LiDAR points whose pixel projection falls inside bbox.

    ``scale`` shrinks the bbox slightly to avoid picking up neighbouring
    objects' LiDAR points when two bboxes overlap.
    """
    if projected is None or len(projected.pixels) == 0:
        return np.empty((0, 3), dtype=float)
    x1, y1, x2, y2 = bbox
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    hw = (x2 - x1) * float(scale) / 2.0
    hh = (y2 - y1) * float(scale) / 2.0
    sx1, sx2 = cx - hw, cx + hw
    sy1, sy2 = cy - hh, cy + hh
    p = np.asarray(projected.pixels, dtype=float)
    mask = (p[:, 0] >= sx1) & (p[:, 0] <= sx2) & (p[:, 1] >= sy1) & (p[:, 1] <= sy2)
    lidar = np.asarray(projected.lidar_points, dtype=float)
    return lidar[mask]


def resolve_overlapping(
    detections,
    projected: ProjectedPoints,
    *,
    bbox_scale: float = 0.85,
    min_points: int = 3,
) -> dict[int, np.ndarray]:
    """Greedy confidence-based assignment for overlapping boxes.

    For each cluster of overlapping boxes we keep the highest-confidence
    detection and blank out its pixels so neighbours cannot claim them.
    Returns a mapping of detection index -> LiDAR points.
    """
    n = len(detections)
    if n == 0 or projected is None or len(projected.pixels) == 0:
        return {}
    order = sorted(
        range(n),
        key=lambda i: (detections[i].confidence, i),
        reverse=True,
    )
    pixels_arr = np.asarray(projected.pixels, dtype=float)
    lidar_arr = np.asarray(projected.lidar_points, dtype=float)
    m = len(pixels_arr)
    # Precompute the scaled bbox window for every detection once: O(n).
    bounds = np.empty((n, 4), dtype=float)
    for i, det in enumerate(detections):
        x1, y1, x2, y2 = det.bbox
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        hw = (x2 - x1) * float(bbox_scale) / 2.0
        hh = (y2 - y1) * float(bbox_scale) / 2.0
        bounds[i] = (cx - hw, cy - hh, cx + hw, cy + hh)
    # Shared mask over the single point arrays: no per-detection copy of
    # ProjectedPoints, total work is O(n*m) with one vectorized pass each.
    claimed_by = np.full(m, -1, dtype=np.int64)
    px = pixels_arr[:, 0]
    py = pixels_arr[:, 1]
    for i in order:
        sx1, sy1, sx2, sy2 = bounds[i]
        inside = (px >= sx1) & (px <= sx2) & (py >= sy1) & (py <= sy2)
        inside &= claimed_by < 0
        claimed_by[inside] = i
    assignment: dict[int, np.ndarray] = {}
    for i in range(n):
        assignment[i] = lidar_arr[claimed_by == i]
    return assignment


__all__ = ["points_for_bbox", "resolve_overlapping"]