"""Stable automatic camera selection from left/right blind-spot radar."""
from __future__ import annotations

import time
from dataclasses import dataclass

from .esp32_radar import SideRadarState


@dataclass(slots=True)
class CameraSelection:
    camera: str
    reason: str
    changed: bool = False


class RadarCameraSelector:
    """Select side camera without flicker when radar detections chatter."""

    def __init__(self, *, hold_seconds: float = 1.2, default_camera: str = "front") -> None:
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.default_camera = default_camera
        self.current = default_camera
        self.reason = "default"
        self._last_detection_at = 0.0

    def update(
        self,
        radar: SideRadarState,
        *,
        reverse: bool = False,
        now: float | None = None,
    ) -> CameraSelection:
        timestamp = time.monotonic() if now is None else float(now)
        target = self.current
        reason = self.reason

        if reverse:
            target, reason = "rear", "reverse"
        elif radar.link_ok and (radar.left_detected or radar.right_detected):
            self._last_detection_at = timestamp
            if radar.left_detected and radar.right_detected:
                left = radar.left_distance_m
                right = radar.right_distance_m
                if left is not None and right is not None and left != right:
                    target = "left" if left < right else "right"
                elif self.current in {"left", "right"}:
                    target = self.current
                else:
                    target = "left"
                reason = "both-radars"
            elif radar.left_detected:
                target, reason = "left", "left-radar"
            else:
                target, reason = "right", "right-radar"
        elif self.current in {"left", "right"} and timestamp - self._last_detection_at < self.hold_seconds:
            target, reason = self.current, "hold"
        else:
            target, reason = self.default_camera, "clear"

        changed = target != self.current
        self.current = target
        self.reason = reason
        return CameraSelection(target, reason, changed)


__all__ = ["CameraSelection", "RadarCameraSelector"]
