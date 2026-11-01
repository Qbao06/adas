"""Distance-only warning policy for deployments without speed/TTC input."""
from __future__ import annotations

import math

from core.enums import WarningLevel


class DistanceWarning:
    """Classify fused objects using longitudinal distance and lane position.

    Escalation is immediate. De-escalation requires the distance to clear the
    relevant boundary by ``hysteresis_distance_m`` to prevent flicker.
    """

    def __init__(self, config: dict) -> None:
        self.config = config or {}
        self.emergency = float(self.config.get("emergency_distance_m", 3.0))
        self.danger = float(self.config.get("danger_distance_m", 6.0))
        self.caution = float(self.config.get("caution_distance_m", 12.0))
        if not 0 < self.emergency <= self.danger <= self.caution:
            raise ValueError("Khoảng cách phải thỏa 0 < emergency <= danger <= caution")
        self.margin = max(0.0, float(self.config.get("hysteresis_distance_m", 0.5)))
        self._accepted: dict[int, WarningLevel] = {}

    def _raw(self, obj, margin: float = 0.0) -> WarningLevel:
        distance = obj.filtered_distance
        if distance is None or not math.isfinite(float(distance)):
            return WarningLevel.UNKNOWN
        if obj.position_3d is None:
            return WarningLevel.UNKNOWN
        if abs(float(obj.position_3d[1])) > float(self.config.get("lane_half_width_m", 1.75)):
            return WarningLevel.SAFE
        d = float(distance)
        if d <= self.emergency + margin:
            return WarningLevel.EMERGENCY
        if d <= self.danger + margin:
            return WarningLevel.DANGER
        if d <= self.caution + margin:
            return WarningLevel.CAUTION
        return WarningLevel.SAFE

    def evaluate(self, objects):
        rank = {
            WarningLevel.UNKNOWN: 0,
            WarningLevel.SAFE: 1,
            WarningLevel.CAUTION: 2,
            WarningLevel.DANGER: 3,
            WarningLevel.EMERGENCY: 4,
        }
        active_ids: set[int] = set()
        for obj in objects:
            raw = self._raw(obj)
            tid = obj.track_id
            if tid is None or tid < 0 or raw == WarningLevel.UNKNOWN:
                obj.warning_level = raw
                continue
            active_ids.add(tid)
            previous = self._accepted.get(tid)
            # A higher rank means more danger: accept it immediately.
            if previous is None or rank[raw] >= rank[previous]:
                accepted = raw
            else:
                # For a safer transition, retain the old level until the
                # object clears its distance boundary by the margin.
                with_margin = self._raw(obj, self.margin)
                accepted = previous if rank[with_margin] >= rank[previous] else raw
            obj.warning_level = accepted
            self._accepted[tid] = accepted
        self._accepted = {k: v for k, v in self._accepted.items() if k in active_ids}
        return objects

    def reset(self) -> None:
        self._accepted.clear()


__all__ = ["DistanceWarning"]
