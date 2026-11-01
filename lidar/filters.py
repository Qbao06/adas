"""LiDAR filtering: NaN/Inf removal, distance/ROI clipping, outlier rejection."""
from __future__ import annotations

import logging
from typing import Iterable

import numpy as np

LOGGER = logging.getLogger(__name__)


def normalize_points(points) -> np.ndarray:
    """Return ``points`` as an ``(N, 3)`` float array; pad with z=0 if needed."""
    arr = np.asarray(points, dtype=float)
    if arr.size == 0:
        return np.empty((0, 3), dtype=float)
    if arr.ndim != 2 or arr.shape[1] < 2:
        raise ValueError("LiDAR points phải có shape (N, 2+)")
    if arr.shape[1] == 2:
        arr = np.column_stack((arr, np.zeros(len(arr))))
    return arr[:, :3]


def filter_points(points, min_range: float, max_range: float, roi) -> np.ndarray:
    """Strip NaN/Inf, drop points outside range and ROI.

    ``roi`` is a dict with x_min/x_max/y_min/y_max/z_min/z_max keys.
    """
    arr = normalize_points(points)
    if len(arr) == 0:
        return arr
    arr = arr[np.isfinite(arr).all(axis=1)]
    if len(arr) == 0:
        return arr
    distance = np.linalg.norm(arr[:, :2], axis=1)
    mask = (distance >= float(min_range)) & (distance <= float(max_range))
    mask &= (arr[:, 0] >= roi["x_min"]) & (arr[:, 0] <= roi["x_max"])
    mask &= (arr[:, 1] >= roi["y_min"]) & (arr[:, 1] <= roi["y_max"])
    mask &= (arr[:, 2] >= roi["z_min"]) & (arr[:, 2] <= roi["z_max"])
    return arr[mask]


def reject_outliers(
    values: np.ndarray, method: str = "iqr", k: float = 1.5
) -> np.ndarray:
    """Remove outliers using IQR (Tukey fences) or MAD."""
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if len(values) < 4:
        return values
    method = (method or "iqr").lower()
    if method == "mad":
        median = np.median(values)
        mad = np.median(np.abs(values - median))
        if mad < 1e-9:
            return values
        threshold = k * 1.4826 * mad
        return values[np.abs(values - median) <= threshold]
    if method == "none":
        return values
    q1, q3 = np.quantile(values, [0.25, 0.75])
    iqr = q3 - q1
    if iqr < 1e-9:
        return values
    return values[(values >= q1 - k * iqr) & (values <= q3 + k * iqr)]


__all__ = [
    "normalize_points",
    "filter_points",
    "reject_outliers",
]
