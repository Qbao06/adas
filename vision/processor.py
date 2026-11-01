"""Run detection + tracking."""
from __future__ import annotations

import logging
from dataclasses import replace
from typing import Any

from core.types import Detection

LOGGER = logging.getLogger(__name__)


class VisionProcessor:
    """Combines a detector and tracker."""

    def __init__(self, detector: Any, tracker: Any, *, input_order: str = "bgr") -> None:
        self.detector = detector
        self.tracker = tracker
        self.input_order = input_order

    def process(self, frame) -> list[Detection]:
        try:
            from sensors.normalize import normalize_camera_image

            image = normalize_camera_image(frame.image, input_order=self.input_order)
            frame = replace(frame, image=image)
        except Exception as exc:
            LOGGER.warning("Ảnh không chuẩn hoá được, dùng ảnh gốc: %s", exc)
        try:
            detections = self.detector.detect(frame)
        except Exception as exc:  # pragma: no cover - defensive
            LOGGER.warning("Detector lỗi, trả về rỗng: %s", exc)
            detections = []
        try:
            return self.tracker.update(detections, frame.timestamp)
        except Exception as exc:  # pragma: no cover - defensive
            LOGGER.warning("Tracker lỗi, trả về rỗng: %s", exc)
            return []

    def warmup(self) -> None:
        warmup = getattr(self.detector, "warmup", None)
        if callable(warmup):
            warmup()

    def close(self) -> None:
        close = getattr(self.detector, "close", None)
        if callable(close):
            close()


__all__ = ["VisionProcessor"]
