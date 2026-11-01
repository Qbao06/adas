"""Calibration loader and validator.

The YAML is expected to contain camera_matrix (3x3), distortion (Nx1+),
rotation (3x3), and translation (3,). The loader verifies shapes and types
so a wrong file cannot crash the pipeline at runtime.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Tuple

import numpy as np
import yaml

LOGGER = logging.getLogger(__name__)


class CalibrationError(ValueError):
    """Raised when calibration file is malformed or shape-incorrect."""


def load_calibration(
    path: str | Path,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Load calibration from YAML, returning (K, D, R, t)."""
    cfg_path = Path(path)
    if not cfg_path.exists():
        raise FileNotFoundError(f"Không tìm thấy calibration: {cfg_path}")
    with cfg_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    try:
        k = np.asarray(data["camera_matrix"], dtype=float)
        r = np.asarray(data["rotation"], dtype=float)
        t = np.asarray(data["translation"], dtype=float).reshape(-1)
        d_raw = data.get("distortion", [0.0] * 5)
        d = np.asarray(d_raw, dtype=float).reshape(-1)
    except Exception as exc:
        raise CalibrationError(f"Calibration sai định dạng: {exc}") from exc
    if k.shape != (3, 3):
        raise CalibrationError(f"camera_matrix phải 3x3, got {k.shape}")
    if r.shape != (3, 3):
        raise CalibrationError(f"rotation phải 3x3, got {r.shape}")
    if t.size != 3:
        raise CalibrationError(f"translation phải có 3 phần tử, got {t.size}")
    if d.size < 4:
        # Pad to a length of 5 so OpenCV undistort works.
        d = np.concatenate([d, np.zeros(5 - d.size)])
    if d.size not in (4, 5, 8, 12, 14):
        LOGGER.warning("distortion có %d phần tử — sẽ vẫn dùng với OpenCV nếu được", d.size)
    LOGGER.info("Đã nạp calibration từ %s", cfg_path)
    return k, d, r, t


__all__ = ["load_calibration", "CalibrationError"]
