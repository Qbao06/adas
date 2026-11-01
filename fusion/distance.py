"""Distance estimation helpers.

Implemented modes:

- nearest           : min over outlier-filtered set
- mean              : mean
- median            : median
- q1                : 25th percentile (closer to the closest wall)
- trimmed_mean      : 10%-trimmed mean
- adaptive          : median when few points, q1 otherwise
"""
from __future__ import annotations

import logging
from typing import Iterable, Optional

import numpy as np

from lidar.filters import reject_outliers

LOGGER = logging.getLogger(__name__)

_MODE_FNS = {
    "nearest": lambda x: float(np.min(x)),
    "mean": lambda x: float(np.mean(x)),
    "median": lambda x: float(np.median(x)),
    "q1": lambda x: float(np.quantile(x, 0.25)),
}


def _trimmed_mean(x: np.ndarray) -> float:
    n = len(x)
    if n <= 4:
        return float(np.median(x))
    lo = max(0, int(0.10 * n))
    hi = max(lo + 1, int(0.90 * n))
    return float(np.mean(np.sort(x)[lo:hi]))


def estimate_distance(
    points,
    mode: str = "adaptive",
    *,
    outlier_method: str = "iqr",
    outlier_k: float = 1.5,
) -> Optional[float]:
    """Return the estimated LiDAR-object distance in metres, or None."""
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return None
    if points.ndim == 1:
        # 1D input is a vector of precomputed distances, not a point cloud.
        cleaned = reject_outliers(points, method=outlier_method, k=outlier_k)
        if cleaned.size == 0:
            cleaned = points[np.isfinite(points)]
        if cleaned.size == 0:
            return None
    else:
        if points.shape[1] < 2:
            return None
        distances = np.linalg.norm(points[:, :2], axis=1)
        cleaned = reject_outliers(distances, method=outlier_method, k=outlier_k)
        if cleaned.size == 0:
            cleaned = distances[np.isfinite(distances)]
        if cleaned.size == 0:
            return None
    chosen = (mode or "adaptive").lower()
    if chosen == "adaptive":
        chosen = "median" if cleaned.size < 8 else "q1"
    if chosen == "trimmed_mean":
        return _trimmed_mean(cleaned)
    if chosen not in _MODE_FNS:
        LOGGER.warning("distance_mode '%s' không hợp lệ, fallback median", chosen)
        chosen = "median"
    fn = _MODE_FNS[chosen]
    return float(fn(cleaned))


__all__ = ["estimate_distance"]
