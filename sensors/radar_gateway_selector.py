"""Camera selector that consumes a :class:`GatewaySnapshot`.

The selector never invents detections: if the Gateway is offline, every radar
is stale or all distances are null, it falls back to the configured default
camera after the hold window expires. Ties are broken by the priority list
shipped with the radar_gateway config.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterable, Optional

from .radar_gateway import GatewayRadarSample, GatewaySnapshot


@dataclass(slots=True)
class GatewayCameraSelection:
    camera: str
    reason: str
    changed: bool = False
    radar_id: str = ""


class GatewayCameraSelector:
    """Select the camera for the four-radar Gateway with anti-flicker hold."""

    def __init__(
        self,
        *,
        radar_camera_map: dict[str, str],
        hold_seconds: float = 1.2,
        default_camera: str = "front",
        priority: Optional[Iterable[str]] = None,
    ) -> None:
        self._map = dict(radar_camera_map)
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.default_camera = default_camera
        # Default priority is the deterministic order of the four radars.
        self.priority = tuple(priority) if priority else tuple(self._map.keys())
        self.current = default_camera
        self.reason = "default"
        self.current_radar = ""
        self._last_detection_at = 0.0
        self._last_distance: Optional[float] = None

    def _radar_priority(self) -> list[str]:
        ordered: list[str] = []
        seen: set[str] = set()
        for radar_id in self.priority:
            if radar_id in self._map and radar_id not in seen:
                ordered.append(radar_id)
                seen.add(radar_id)
        for radar_id in self._map:
            if radar_id not in seen:
                ordered.append(radar_id)
                seen.add(radar_id)
        return ordered

    def update(
        self,
        snapshot: GatewaySnapshot,
        *,
        now: Optional[float] = None,
        reverse: bool = False,
    ) -> GatewayCameraSelection:
        timestamp = time.monotonic() if now is None else float(now)
        target = self.current
        reason = self.reason
        radar_id = self.current_radar

        if reverse:
            target = "rear"
            reason = "reverse"
            radar_id = ""
        elif not snapshot.link_ok:
            target, reason, radar_id = self._apply_clear(timestamp)
        else:
            candidates: list[tuple[float, str, GatewayRadarSample]] = []
            for rid in self._radar_priority():
                sample = snapshot.radars.get(rid)
                if sample is None:
                    continue
                if not (sample.valid and sample.detected and sample.distance_m is not None):
                    continue
                candidates.append((sample.distance_m, rid, sample))
            if candidates:
                candidates.sort(key=lambda item: (item[0], self._priority_index(item[1]), item[1]))
                distance, rid, sample = candidates[0]
                target = self._map.get(rid, target)
                reason = f"radar:{rid}"
                radar_id = rid
                self._last_detection_at = timestamp
                self._last_distance = distance
            else:
                # detected=true but distance=null, or no detections at all.
                target, reason, radar_id = self._apply_hold_or_distance_null(
                    snapshot, timestamp
                )

        changed = target != self.current
        self.current = target
        self.reason = reason
        self.current_radar = radar_id
        return GatewayCameraSelection(target, reason, changed, radar_id)

    # ------------------------------------------------------------- helpers

    def _priority_index(self, radar_id: str) -> int:
        try:
            return self._radar_priority().index(radar_id)
        except ValueError:
            return len(self.priority)

    def _apply_clear(self, timestamp: float) -> tuple[str, str, str]:
        if self.current == self.default_camera:
            return self.default_camera, "clear", ""
        if timestamp - self._last_detection_at < self.hold_seconds:
            return self.current, "hold", self.current_radar
        return self.default_camera, "clear", ""

    def _apply_hold_or_distance_null(
        self, snapshot: GatewaySnapshot, timestamp: float
    ) -> tuple[str, str, str]:
        # If any radar still flags detected=true (even with null distance) and
        # the current camera still belongs to that radar, keep the camera.
        detected_radars = [
            rid
            for rid, sample in snapshot.radars.items()
            if sample.valid and sample.detected
        ]
        if detected_radars:
            current_camera_radar = self._radar_for_camera(self.current)
            if current_camera_radar and current_camera_radar in detected_radars:
                return self.current, "hold-distance-null", current_camera_radar
            # Pick the highest-priority detected radar with a known camera.
            for rid in self._radar_priority():
                if rid in detected_radars:
                    camera = self._map.get(rid)
                    if camera:
                        self._last_detection_at = timestamp
                        return camera, f"radar:{rid}", rid
        return self._apply_clear(timestamp)

    def _radar_for_camera(self, camera: str) -> str:
        for radar_id, mapped in self._map.items():
            if mapped == camera:
                return radar_id
        return ""


__all__ = ["GatewayCameraSelector", "GatewayCameraSelection"]
