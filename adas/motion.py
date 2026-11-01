"""Estimate relative speed + TTC per track using EMA on per-track history."""
from __future__ import annotations

import logging
import math
from typing import Dict, Optional, Tuple

LOGGER = logging.getLogger(__name__)


class MotionEstimator:
    """Estimate closing speed and TTC for each track.

    Conventions:
      - ``relative_speed > 0`` means the gap is shrinking (object approaching).
      - TTC is capped to ``ttc_cap_s`` and is ``+inf`` when not closing.
    """

    def __init__(
        self,
        alpha: float = 0.4,
        stale_seconds: float = 2.0,
        min_closing_mps: float = 0.05,
        ttc_cap_s: float = 30.0,
        max_speed_mps: float = 60.0,
        ego_speed_mps: float = 0.0,
    ) -> None:
        self.alpha = float(alpha)
        self.stale_seconds = float(stale_seconds)
        self.min_closing = float(min_closing_mps)
        self.ttc_cap = float(ttc_cap_s)
        self.max_speed = float(max_speed_mps)
        self.ego_speed = float(ego_speed_mps)
        self.history: Dict[int, Tuple[float, float, float]] = {}

    def _safe_ttc(self, distance: float, closing_speed: float) -> float:
        if closing_speed <= self.min_closing:
            return float("inf")
        if distance <= 0:
            return 0.0
        if not math.isfinite(closing_speed) or not math.isfinite(distance):
            return float("inf")
        ttc = distance / closing_speed
        if math.isnan(ttc) or ttc < 0:
            return float("inf")
        return min(ttc, self.ttc_cap)

    def update(self, objects, timestamp: float):
        for obj in objects:
            if obj.track_id is None or obj.track_id < 0:
                obj.relative_speed = None
                obj.ttc = None
                continue
            distance = obj.filtered_distance
            if distance is None or not math.isfinite(distance):
                continue
            previous = self.history.get(obj.track_id)
            speed = 0.0
            if previous is not None:
                old_d, old_t, old_v = previous
                dt = timestamp - old_t
                if dt <= 0:
                    continue
                raw_v = (old_d - distance) / dt + self.ego_speed
                if abs(raw_v) > self.max_speed:
                    continue
                speed = self.alpha * raw_v + (1.0 - self.alpha) * old_v
            obj.relative_speed = speed
            obj.ttc = self._safe_ttc(distance, speed) if speed > self.min_closing else float("inf")
            self.history[obj.track_id] = (distance, timestamp, speed)
        # Drop stale entries.
        self.history = {
            k: v for k, v in self.history.items()
            if timestamp - v[1] <= self.stale_seconds
        }
        return objects

    def reset(self) -> None:
        self.history.clear()


__all__ = ["MotionEstimator"]
