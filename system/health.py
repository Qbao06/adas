"""Health monitor + lightweight performance counter."""
from __future__ import annotations

import logging
import time
from typing import Optional

from core.enums import SystemStatus

LOGGER = logging.getLogger(__name__)


class HealthMonitor:
    """Track per-component heartbeats and decide system status.

    Components call ``heartbeat(name)`` whenever they have produced new
    data. If any component goes more than ``timeout_seconds`` without
    producing data, the system enters DEGRADED state.
    """

    def __init__(self, timeout_seconds: float = 1.0) -> None:
        self.timeout = float(timeout_seconds)
        self.last: dict[str, float] = {}
        self.status = SystemStatus.STARTING
        self._latched_error = False
        self._latched_degraded = False

    def heartbeat(self, name: str, timestamp: Optional[float] = None) -> None:
        self.last[name] = float(timestamp if timestamp is not None else time.monotonic())

    def sensor_state(self, name: str, *, enabled: bool = True) -> str:
        """Per-sensor state: ok / lost / disabled / stale."""
        if not enabled:
            return "disabled"
        if name not in self.last:
            return "lost"
        if time.monotonic() - self.last[name] > self.timeout:
            return "stale"
        return "ok"

    def sensor_summary(self, sensors: dict[str, bool]) -> dict[str, str]:
        """Map sensor name to its state given an enabled flag per sensor."""
        return {name: self.sensor_state(name, enabled=on) for name, on in sensors.items()}

    def set_status(self, status: SystemStatus) -> None:
        # ERROR/DEGRADED latch until an explicit clear: evaluate() must not
        # silently overwrite a gateway/remote fault with OK.
        if status == SystemStatus.ERROR:
            self._latched_error = True
        elif status == SystemStatus.DEGRADED:
            self._latched_degraded = True
        self.status = status

    def clear_latch(self) -> None:
        self._latched_error = False
        self._latched_degraded = False

    def evaluate(self) -> SystemStatus:
        if not self.last:
            self.status = SystemStatus.STARTING
            return self.status
        now = time.monotonic()
        stale = any(now - t > self.timeout for t in self.last.values())
        if self._latched_error:
            self.status = SystemStatus.ERROR
        elif stale or self._latched_degraded:
            self.status = SystemStatus.DEGRADED
        else:
            self.status = SystemStatus.OK
        return self.status


class PerformanceMonitor:
    """Running frame counter + moving FPS estimate."""

    def __init__(self) -> None:
        self.start = time.perf_counter()
        self.frames = 0
        self.last_latency_ms = 0.0
        self._last_frame_ts = self.start

    def mark(self, started_at: float) -> None:
        self.frames += 1
        self.last_latency_ms = (time.perf_counter() - started_at) * 1000.0
        self._last_frame_ts = time.perf_counter()

    def fps(self) -> float:
        elapsed = max(time.perf_counter() - self.start, 1e-6)
        return self.frames / elapsed

    def reset(self) -> None:
        self.start = time.perf_counter()
        self.frames = 0
        self.last_latency_ms = 0.0


__all__ = ["HealthMonitor", "PerformanceMonitor"]
