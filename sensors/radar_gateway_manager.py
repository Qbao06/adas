"""Runtime orchestration for the unified ESP32 A Gateway reader."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Optional

from .radar_gateway import GatewaySnapshot, RadarGatewayReader
from .radar_gateway_selector import GatewayCameraSelection, GatewayCameraSelector


@dataclass(frozen=True, slots=True)
class RadarGatewayView:
    """Single immutable view of the gateway state used by the pipeline."""

    snapshot: GatewaySnapshot
    selection: GatewayCameraSelection
    received_monotonic: float

    @property
    def radar_states(self) -> dict[str, Any]:
        """Backwards-compatible projection used by live_snapshot/dashboard."""
        result: dict[str, Any] = {}
        for radar_id, sample in self.snapshot.radars.items():
            result[radar_id] = {
                "radar_id": sample.radar_id,
                "side": "left" if radar_id.startswith("left") else "right",
                "position": "front" if radar_id.endswith("front") else "rear",
                "detected": bool(sample.detected),
                "target_state": sample.target_state,
                "distance_m": sample.distance_m,
                "moving_distance_m": sample.moving_distance_m,
                "moving_energy": sample.moving_energy,
                "stationary_distance_m": sample.stationary_distance_m,
                "stationary_energy": sample.stationary_energy,
                "valid": bool(sample.valid),
                "stale": bool(sample.stale),
                "age_ms": sample.age_ms,
                "error": sample.error,
                "sequence": self.snapshot.sequence,
            }
        return result


class RadarGatewayManager:
    """Own the reader + selector and produce a stable view for the pipeline."""

    def __init__(self, config: dict[str, Any]) -> None:
        if not config.get("enabled", False):
            raise ValueError("RadarGatewayManager yêu cầu radar_gateway.enabled=true")
        self.config = config
        self.reader = RadarGatewayReader(
            str(config["device"]),
            baudrate=int(config.get("baudrate", 115200)),
            stale_after_seconds=float(config.get("stale_after_seconds", 0.6)),
            reconnect_seconds=float(config.get("reconnect_seconds", 2.0)),
            max_line_bytes=int(config.get("max_line_bytes", 4096)),
        )
        radar_camera_map = {
            radar_id: str(item.get("camera", radar_id))
            for radar_id, item in config.get("radars", {}).items()
        }
        priority = list(config.get("priority", list(radar_camera_map.keys())))
        self.selector = GatewayCameraSelector(
            radar_camera_map=radar_camera_map,
            hold_seconds=float(config.get("camera_hold_seconds", 1.2)),
            priority=priority,
        )
        self._last_view: Optional[RadarGatewayView] = None
        self._selection_changed_at = 0.0

    # ------------------------------------------------------------ lifecycle

    def connect(self) -> None:
        self.reader.connect()

    def disconnect(self) -> None:
        self.reader.disconnect()

    def is_connected(self) -> bool:
        return self.reader.is_connected()

    # ------------------------------------------------------------ read API

    def read(self, *, now: Optional[float] = None) -> RadarGatewayView:
        timestamp = time.monotonic() if now is None else float(now)
        snapshot = self.reader.read(now=timestamp)
        selection = self.selector.update(snapshot, now=timestamp)
        view = RadarGatewayView(snapshot=snapshot, selection=selection, received_monotonic=timestamp)
        if selection.changed:
            self._selection_changed_at = timestamp
        self._last_view = view
        return view

    @property
    def last_view(self) -> Optional[RadarGatewayView]:
        return self._last_view

    @property
    def selection_changed_at(self) -> float:
        return self._selection_changed_at

    def stats(self) -> dict[str, Any]:
        return self.reader.stats()


__all__ = ["RadarGatewayManager", "RadarGatewayView"]
