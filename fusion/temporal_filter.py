"""EMA on per-track distance, with bounded hold on missing measurements."""
from __future__ import annotations

import logging
from typing import Any, Dict, Optional, Tuple

LOGGER = logging.getLogger(__name__)


class TemporalFilter:
    """EMA on ``raw_distance`` plus bounded hold-over when LiDAR goes silent.

    State per track:
      - value: current filtered distance
      - timestamp: wall time of last update
      - hold_count: how many consecutive frames we held without new data
    """

    def __init__(
        self,
        alpha: float = 0.35,
        stale_seconds: float = 1.5,
        max_hold: int = 10,
    ) -> None:
        self.alpha = float(alpha)
        self.stale_seconds = float(stale_seconds)
        self.max_hold = max(1, int(max_hold))
        self.state: Dict[int, Tuple[float, float, int]] = {}

    def update(self, objects, timestamp: float):
        """Update in-place and return the same list for convenience."""
        for obj in objects:
            tid = obj.track_id
            if tid is None or tid < 0:
                obj.filtered_distance = obj.raw_distance
                continue
            old = self.state.get(tid)
            if obj.raw_distance is None:
                if old is None:
                    obj.filtered_distance = None
                    continue
                value, _last_t, hold_count = old
                if hold_count >= self.max_hold:
                    obj.filtered_distance = None
                    self.state.pop(tid, None)
                    continue
                obj.filtered_distance = value
                self.state[tid] = (value, _last_t, hold_count + 1)
                continue
            if old is None:
                value = float(obj.raw_distance)
                hold_count = 0
            else:
                prev_value, _lt, _hc = old
                value = self.alpha * float(obj.raw_distance) + (
                    1.0 - self.alpha
                ) * prev_value
                hold_count = 0
            obj.filtered_distance = value
            self.state[tid] = (value, timestamp, hold_count)
        # Drop stale entries.
        self.state = {
            k: v for k, v in self.state.items()
            if timestamp - v[1] <= self.stale_seconds
        }
        return objects

    def reset(self) -> None:
        self.state.clear()


__all__ = ["TemporalFilter"]
