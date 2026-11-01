"""Render pipeline output (boxes, distances, TTC, LiDAR overlays)."""
from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

from core.enums import SystemStatus, WarningLevel

LOGGER = logging.getLogger(__name__)

try:
    import cv2  # type: ignore

    _HAVE_CV2 = True
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore
    _HAVE_CV2 = False


_LEVEL_COLORS = {
    WarningLevel.SAFE: (60, 220, 60),
    WarningLevel.CAUTION: (0, 220, 255),
    WarningLevel.DANGER: (40, 90, 255),
    WarningLevel.EMERGENCY: (0, 0, 255),
    WarningLevel.UNKNOWN: (160, 160, 160),
}


_STATUS_COLORS = {
    SystemStatus.STARTING: (200, 200, 200),
    SystemStatus.OK: (60, 220, 60),
    SystemStatus.DEGRADED: (0, 200, 255),
    SystemStatus.ERROR: (0, 0, 255),
    SystemStatus.STOPPED: (160, 160, 160),
}


def _format_distance(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value):
        return "--"
    return f"{value:.1f}m"


def _format_speed(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value):
        return "--"
    return f"{value:.1f}m/s"


def _format_ttc(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value):
        return "inf"
    return f"{value:.1f}s"


class Visualizer:
    """Draw detection + tracking + LiDAR overlay onto a frame."""

    def __init__(self, config: dict | None = None) -> None:
        self.config = config or {}
        self.writer: Optional["cv2.VideoWriter"] = None
        self.video_path: Optional[Path] = None

    def _open_writer(self, image: np.ndarray) -> None:
        if self.writer is not None:
            return
        path_str = self.config.get("record_path", "")
        if not path_str:
            return
        path = Path(path_str)
        path.parent.mkdir(parents=True, exist_ok=True)
        h, w = image.shape[:2]
        if w % 2:
            w -= 1
        if h % 2:
            h -= 1
        fourcc = cv2.VideoWriter_fourcc(*"mp4v") if _HAVE_CV2 else None
        fps = float(self.config.get("video_fps", 30))
        try:
            if _HAVE_CV2:
                self.writer = cv2.VideoWriter(str(path), fourcc, fps, (w, h))
            self.video_path = path
            LOGGER.info("Ghi video kết quả vào %s", path)
        except Exception as exc:  # pragma: no cover
            LOGGER.warning("Không mở được VideoWriter: %s", exc)
            self.writer = None

    def _draw_lidar_points(self, canvas, projected) -> None:
        """Vectorized LiDAR overlay with configurable subsample cap."""
        import threading

        max_pts = int(self.config.get("lidar_max_points", 2000))
        pts = np.asarray(projected.pixels)
        if pts.size == 0:
            return
        if max_pts > 0 and len(pts) > max_pts:
            step = (len(pts) + max_pts - 1) // max_pts
            pts = pts[::step]
        h, w = canvas.shape[:2]
        u = pts[:, 0].astype(np.intp)
        v = pts[:, 1].astype(np.intp)
        valid = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        canvas[v[valid], u[valid]] = (255, 100, 50)

    def draw_async(self, image, *args, **kwargs):
        """Non-blocking render: latest frame wins, worker renders in background."""
        if not bool(self.config.get("async_render", False)):
            return self.draw(image, *args, **kwargs)
        import threading
        state = getattr(self, "_async_state", None)
        if state is None:
            state = self._async_state = {"lock": threading.Lock(),
                                         "thread": None, "result": None}
        with state["lock"]:
            state["pending"] = (image.copy(), args, kwargs)
            if state["thread"] is None or not state["thread"].is_alive():
                def _work():
                    while True:
                        with state["lock"]:
                            job = state.pop("pending", None)
                            if job is None:
                                state["thread"] = None
                                return
                        img, a, kw = job
                        try:
                            out = self.draw(img, *a, **kw)
                        except Exception:
                            out = img
                        with state["lock"]:
                            state["result"] = out
                state["thread"] = threading.Thread(target=_work, daemon=True)
                state["thread"].start()
            return state["result"] if state["result"] is not None else image.copy()

    def draw(
        self,
        image: np.ndarray,
        objects,
        projected=None,
        fps: float = 0.0,
        latency_ms: float = 0.0,
        status: str = "ok",
        lidar_status: str = "ok",
        camera_status: str = "ok",
        sensor_summary: Optional[dict] = None,
    ) -> np.ndarray:
        if not self.config.get("enabled", True):
            return image.copy()
        if not _HAVE_CV2:
            return image.copy()
        canvas = image.copy()
        if projected is not None and self.config.get("show_lidar_points", True):
            try:
                self._draw_lidar_points(canvas, projected)
            except Exception:  # pragma: no cover
                pass
        for obj in objects or []:
            color = _LEVEL_COLORS.get(obj.warning_level, (255, 255, 255))
            x1, y1, x2, y2 = obj.bbox
            cv2.rectangle(canvas, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            label_parts = [
                f"#{obj.track_id}",
                obj.class_name,
                f"c={obj.confidence:.2f}",
                _format_distance(obj.filtered_distance),
            ]
            speed = getattr(obj, "relative_speed", None)
            if speed is not None and math.isfinite(speed):
                label_parts.append(f"v={_format_speed(speed)}")
            if getattr(obj, "ttc", None) is not None:
                label_parts.append(f"TTC:{_format_ttc(obj.ttc)}")
            label_parts.append(obj.warning_level.value.upper())
            label = " ".join(label_parts)
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
            )
            tx = max(0, int(x1))
            ty = max(th + 6, int(y1) - 6)
            cv2.rectangle(
                canvas,
                (tx, ty - th - 4),
                (tx + tw + 4, ty + 4),
                color,
                thickness=-1,
            )
            cv2.putText(
                canvas, label, (tx + 2, ty - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1,
                cv2.LINE_AA,
            )
        # HUD: fps, latency, sensor status.
        status_color = (
            _STATUS_COLORS.get(SystemStatus(status), (255, 255, 255))
            if isinstance(status, str) and status in _STATUS_COLORS
            else (255, 255, 255)
        )
        info_lines = [
            f"FPS:{fps:.1f} Latency:{latency_ms:.1f}ms System:{status}",
            f"Camera:{camera_status} LiDAR:{lidar_status}",
        ]
        for i, line in enumerate(info_lines):
            cv2.putText(
                canvas, line,
                (15, 25 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3,
                cv2.LINE_AA,
            )
            cv2.putText(
                canvas, line,
                (15, 25 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1,
                cv2.LINE_AA,
            )
        # Bottom-right warning count.
        warning_count = {
            "EMERGENCY": 0, "DANGER": 0, "CAUTION": 0, "SAFE": 0, "UNKNOWN": 0
        }
        for obj in objects or []:
            warning_count[obj.warning_level.value.upper()] += 1
        summary = " ".join(
            f"{k}:{v}" for k, v in warning_count.items()
        )
        cv2.putText(
            canvas, summary,
            (15, canvas.shape[0] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2,
        )
        self._open_writer(canvas)
        if self.writer is not None:
            try:
                self.writer.write(canvas)
            except Exception as exc:  # pragma: no cover
                LOGGER.warning("Không ghi được frame: %s", exc)
        return canvas

    def show(self, image: np.ndarray, window_name: str = "ADAS LiDAR-Camera") -> int:
        if not _HAVE_CV2:
            return -1
        cv2.imshow(window_name, image)
        key = cv2.waitKey(1) & 0xFF
        return int(key)

    def save(self, image: np.ndarray, path: str | Path) -> None:
        if not _HAVE_CV2:
            return
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(p), image)

    def close(self) -> None:
        if self.writer is not None:
            try:
                self.writer.release()
            except Exception:  # pragma: no cover
                pass
            self.writer = None
        if _HAVE_CV2:
            try:
                cv2.destroyAllWindows()
            except Exception:  # pragma: no cover
                pass


__all__ = ["Visualizer"]
