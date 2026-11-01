"""Object tracking.

We provide a stable IoU-based tracker implemented in pure Python. If the
optional Norfair/ByteTrack dep is available, users can plug it via
``build_tracker`` and adjust ``vision`` configuration to request it.
"""
from __future__ import annotations

import logging
from dataclasses import replace
from typing import Iterable, Optional

from core.types import Detection

LOGGER = logging.getLogger(__name__)


def bbox_iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    """IoU between two (x1, y1, x2, y2) boxes."""
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = area_a + area_b - inter
    if union <= 0:
        return 0.0
    return inter / union


class _Track:
    __slots__ = ("track_id", "detection", "age", "hits", "first_seen", "last_seen")

    def __init__(self, track_id: int, detection: Detection, timestamp: float) -> None:
        self.track_id = track_id
        self.detection = detection
        self.age = 0
        self.hits = 1
        self.first_seen = timestamp
        self.last_seen = timestamp


class IoUTracker:
    """Lightweight IoU-based tracker.

    Each new detection becomes a tentative track; only after ``min_hits``
    consecutive matches does it emit a stable ``track_id``.
    """

    name = "iou"

    def __init__(
        self,
        iou_threshold: float = 0.3,
        max_age: int = 15,
        min_hits: int = 3,
    ) -> None:
        self.threshold = float(iou_threshold)
        self.max_age = int(max_age)
        self.min_hits = max(1, int(min_hits))
        self.tracks: dict[int, _Track] = {}
        self.next_id = 1

    def _new_id(self) -> int:
        tid = self.next_id
        self.next_id += 1
        return tid

    def update(
        self, detections: list[Detection], timestamp: float
    ) -> list[Detection]:
        candidates: list[tuple[float, int, int]] = []
        for ti, track in self.tracks.items():
            for di, det in enumerate(detections):
                if det.class_name != track.detection.class_name:
                    continue
                score = bbox_iou(track.detection.bbox, det.bbox)
                if score <= 0.0:
                    continue
                candidates.append((score, ti, di))
        candidates.sort(key=lambda x: x[0], reverse=True)
        used_tracks: set[int] = set()
        used_dets: set[int] = set()
        matches: dict[int, int] = {}
        for score, ti, di in candidates:
            if score < self.threshold or ti in used_tracks or di in used_dets:
                continue
            used_tracks.add(ti)
            used_dets.add(di)
            matches[di] = ti
        for di, det in enumerate(detections):
            ti = matches.get(di)
            if ti is None:
                continue
            track = self.tracks[ti]
            track.detection = det
            track.hits += 1
            track.age = 0
            track.last_seen = timestamp
        for di, det in enumerate(detections):
            if di in matches:
                continue
            new_id = self._new_id()
            self.tracks[new_id] = _Track(new_id, det, timestamp)
            matches[di] = new_id
        matched_set = set(matches.values())
        for ti in list(self.tracks):
            if ti in matched_set:
                continue
            self.tracks[ti].age += 1
            if self.tracks[ti].age > self.max_age:
                del self.tracks[ti]
        result: list[Detection] = []
        for di, det in enumerate(detections):
            ti = matches.get(di)
            if ti is None:
                continue
            track = self.tracks.get(ti)
            if track is None:
                continue
            if track.hits < self.min_hits:
                continue
            result.append(replace(det, track_id=ti))
        return result

    def reset(self) -> None:
        self.tracks.clear()
        self.next_id = 1


class NullTracker:
    """Tracker that assigns track_id by detection index — for tests only."""

    name = "null"

    def update(
        self, detections: list[Detection], timestamp: float
    ) -> list[Detection]:
        out: list[Detection] = []
        for i, det in enumerate(detections, start=1):
            out.append(replace(det, track_id=i))
        return out

    def reset(self) -> None:
        return None


def build_tracker(vision_cfg: dict) -> object:
    """Select tracker based on config; fall back gracefully.

    ``vision.tracker`` may be ``"iou"`` (default) or any vendor name. The
    vendor tracker interface is expected to expose ``update``.
    """
    name = (vision_cfg.get("tracker") or "iou").lower()
    if name == "null":
        return NullTracker()
    if name in {"iou", "default", ""}:
        return IoUTracker(
            iou_threshold=vision_cfg.get("tracker_iou_threshold", 0.3),
            max_age=vision_cfg.get("tracker_max_age", 15),
            min_hits=vision_cfg.get("tracker_min_hits", 3),
        )
    # Attempt to load external trackers lazily without forcing a hard dep.
    try:
        if name == "norfair":
            import norfair  # type: ignore # noqa: F401

            LOGGER.info("Đã phát hiện Norfair — dùng IoUTracker làm fallback")
        elif name in {"bytetrack", "botsort"}:
            try:
                import ultralytics  # type: ignore # noqa: F401

                LOGGER.info(
                    "Tracker '%s' có thể dùng trực tiếp qua ultralytics; "
                    "fallback IoUTracker cho pipeline này.",
                    name,
                )
            except Exception:
                LOGGER.warning(
                    "Tracker '%s' yêu cầu ultralytics — fallback IoUTracker",
                    name,
                )
    except Exception as exc:  # pragma: no cover
        LOGGER.warning("Không khởi tạo được tracker '%s': %s", name, exc)
    return IoUTracker(
        iou_threshold=vision_cfg.get("tracker_iou_threshold", 0.3),
        max_age=vision_cfg.get("tracker_max_age", 15),
        min_hits=vision_cfg.get("tracker_min_hits", 3),
    )


__all__ = [
    "bbox_iou",
    "IoUTracker",
    "NullTracker",
    "build_tracker",
]
