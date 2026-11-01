"""Collision warning classification with hysteresis.

Outputs one of:
    UNKNOWN, SAFE, CAUTION, DANGER, EMERGENCY

Hysteresis prevents the level from bouncing when TTC hovers near a
threshold. Safety-oriented configurations set ``delay_escalation=false``
so increasing danger is accepted immediately. The legacy delayed-escalation
mode remains available for compatibility and UI-only applications.
"""
from __future__ import annotations

import logging
import math
import time
from typing import Optional

from core.enums import WarningLevel

LOGGER = logging.getLogger(__name__)

_LEVEL_RANK = {
    WarningLevel.UNKNOWN: 0,
    WarningLevel.SAFE: 1,
    WarningLevel.CAUTION: 2,
    WarningLevel.DANGER: 3,
    WarningLevel.EMERGENCY: 4,
}


class CollisionWarning:
    """Compute the warning level for every ``FusedObject``."""

    def __init__(self, config: dict) -> None:
        self.config = config or {}
        self._accepted_level: dict[int, tuple[WarningLevel, float]] = {}
        self._pending_level: dict[int, tuple[WarningLevel, float]] = {}

    def _raw_level(self, obj) -> WarningLevel:
        c = self.config
        if obj.filtered_distance is None:
            return WarningLevel.UNKNOWN
        if obj.position_3d is None:
            return WarningLevel.SAFE
        lateral = abs(float(obj.position_3d[1]))
        lateral_max = float(c.get("lane_half_width_m", 1.75))
        if lateral > lateral_max:
            return WarningLevel.SAFE
        ttc = obj.ttc
        if ttc is None or math.isinf(ttc) or math.isnan(ttc):
            closing_speed = float(getattr(obj, "relative_speed", 0.0) or 0.0)
            if closing_speed < float(c.get("min_closing_speed_mps", 0.2)):
                return WarningLevel.SAFE
            return WarningLevel.CAUTION
        emergency = float(c.get("emergency_ttc", 1.0))
        danger = float(c.get("danger_ttc", 2.0))
        caution = float(c.get("caution_ttc", 3.5))
        if ttc <= emergency:
            return WarningLevel.EMERGENCY
        if ttc <= danger:
            return WarningLevel.DANGER
        if ttc <= caution:
            return WarningLevel.CAUTION
        return WarningLevel.SAFE

    def evaluate(self, objects):
        c = self.config
        hysteresis = float(c.get("hysteresis_seconds", 0.6))
        levels_diff = int(c.get("hysteresis_levels", 1))
        delay_escalation = bool(c.get("delay_escalation", True))
        now = time.monotonic()
        for obj in objects:
            tid = obj.track_id
            raw = self._raw_level(obj)
            if tid is None or tid < 0:
                obj.warning_level = raw
                continue
            accepted = self._accepted_level.get(tid)
            accepted_level = accepted[0] if accepted else None
            if accepted_level is None:
                # First time we see this track. If hysteresis is disabled, accept
                # the raw level immediately. Otherwise start at SAFE and require
                # persistence.
                if hysteresis <= 0 or not delay_escalation:
                    obj.warning_level = raw
                    self._accepted_level[tid] = (raw, now)
                    self._pending_level.pop(tid, None)
                else:
                    self._pending_level[tid] = (raw, now)
                    obj.warning_level = WarningLevel.SAFE
                continue
            if _LEVEL_RANK[raw] <= _LEVEL_RANK[accepted_level]:
                # Same level or de-escalation: accept immediately.
                obj.warning_level = raw
                self._accepted_level[tid] = (raw, now)
                self._pending_level.pop(tid, None)
                continue
            # Escalation: require enough headroom + persistence.
            if not delay_escalation:
                obj.warning_level = raw
                self._accepted_level[tid] = (raw, now)
                self._pending_level.pop(tid, None)
                continue
            diff = _LEVEL_RANK[raw] - _LEVEL_RANK[accepted_level]
            if diff < max(1, levels_diff):
                obj.warning_level = accepted_level
                continue
            pending = self._pending_level.get(tid)
            if pending is None or pending[0] != raw:
                self._pending_level[tid] = (raw, now)
                obj.warning_level = accepted_level
                continue
            if now - pending[1] >= hysteresis:
                obj.warning_level = raw
                self._accepted_level[tid] = (raw, now)
                self._pending_level.pop(tid, None)
            else:
                obj.warning_level = accepted_level
        # Cleanup stale entries.
        cutoff = now - 5.0
        self._accepted_level = {
            k: v for k, v in self._accepted_level.items() if v[1] >= cutoff
        }
        return objects

    def reset(self) -> None:
        self._accepted_level.clear()
        self._pending_level.clear()


__all__ = ["CollisionWarning"]
