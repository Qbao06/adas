"""Render LiDAR points over a reference image for calibration inspection."""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from calibration import LidarCameraProjector


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--lidar", required=True, help="Một frame .npy shape (N,3)")
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--output", default="output/calibration_overlay.png")
    args = parser.parse_args()
    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit(f"Không đọc được ảnh: {args.image}")
    points = np.asarray(np.load(args.lidar), dtype=float)
    if points.ndim != 2 or points.shape[1] < 3:
        raise SystemExit(f"Point cloud phải có shape (N,3+), got {points.shape}")
    projected = LidarCameraProjector(args.calibration).project(points, image.shape)
    distances = np.linalg.norm(projected.lidar_points[:, :2], axis=1)
    for (u, v), distance in zip(projected.pixels.astype(int), distances):
        normalized = min(max(distance / 60.0, 0.0), 1.0)
        color = (int(255 * normalized), int(255 * (1.0 - normalized)), 50)
        cv2.circle(image, (int(u), int(v)), 2, color, -1)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), image):
        raise SystemExit(f"Không ghi được: {output}")
    print(f"Projected {len(projected.pixels)} points -> {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
