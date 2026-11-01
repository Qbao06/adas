"""Runtime orchestration for up to two camera/radar modules on each side."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Optional

from .esp32_radar import ESP32RadarReader, SideRadarState


@dataclass(frozen=True, slots=True)
class SideModuleState:
    module_id: str
    side: str
    position: str
    detected: bool = False
    distance_m: Optional[float] = None
    link_ok: bool = False
    sequence: int = -1


class SideModuleManager:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.readers: dict[str, ESP32RadarReader] = {}
        self.meta: dict[str, dict[str, Any]] = {}
        for module_id, item in config.get("modules", {}).items():
            radar = item.get("radar", {})
            if not item.get("enabled", False) or not radar.get("enabled", True):
                continue
            self.meta[module_id] = item
            self.readers[module_id] = ESP32RadarReader(
                radar["device"],
                baudrate=int(radar.get("baudrate", 115200)),
                stale_after_seconds=float(radar.get("stale_after_seconds", 0.6)),
                max_distance_m=float(radar.get("max_distance_m", 30.0)),
            )

    def connect(self) -> None:
        connected: list[ESP32RadarReader] = []
        try:
            for reader in self.readers.values():
                reader.connect()
                connected.append(reader)
        except Exception:
            for reader in connected:
                reader.disconnect()
            raise

    def disconnect(self) -> None:
        for reader in self.readers.values():
            reader.disconnect()

    def read(self) -> dict[str, SideModuleState]:
        result: dict[str, SideModuleState] = {}
        for module_id, reader in self.readers.items():
            raw: SideRadarState = reader.read()
            item = self.meta[module_id]
            side = item["side"]
            detected = raw.left_detected if side == "left" else raw.right_detected
            distance = raw.left_distance_m if side == "left" else raw.right_distance_m
            result[module_id] = SideModuleState(
                module_id=module_id,
                side=side,
                position=item.get("position", "front"),
                detected=bool(detected and raw.link_ok),
                distance_m=distance if raw.link_ok else None,
                link_ok=raw.link_ok,
                sequence=raw.sequence,
            )
        return result


class ModuleCameraSelector:
    """Select the closest detecting module and hold its camera to prevent flicker."""
    def __init__(self, *, hold_seconds: float = 1.2, default_camera: str = "front") -> None:
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.default_camera = default_camera
        self.current = default_camera
        self.reason = "default"
        self._last_detection_at = 0.0

    def update(self, states: dict[str, SideModuleState], *, now: float | None = None) -> tuple[str, str, bool]:
        timestamp = time.monotonic() if now is None else float(now)
        candidates = [state for state in states.values() if state.link_ok and state.detected]
        if candidates:
            candidates.sort(key=lambda state: (state.distance_m is None, state.distance_m or float("inf"), state.module_id))
            target = candidates[0].module_id
            reason = f"module:{target}"
            self._last_detection_at = timestamp
        elif self.current != self.default_camera and timestamp - self._last_detection_at < self.hold_seconds:
            target, reason = self.current, "hold"
        else:
            target, reason = self.default_camera, "clear"
        changed = target != self.current
        self.current, self.reason = target, reason
        return target, reason, changed


__all__ = ["ModuleCameraSelector", "SideModuleManager", "SideModuleState"]
