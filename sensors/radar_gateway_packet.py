"""Read-only helpers for the new single-Gateway JSON v2 protocol.

These helpers **never** open a serial port.  They:

* parse a single JSON line received from ESP32 A Gateway,
* enforce the v2 schema (sequence, four radars, gateway_ok / remote_ok,
  protocol_version 2),
* normalise ``null``/``NaN``/``Infinity`` values into ``None`` so the
  dashboard and preflight never see a synthetic 0 m or a fake detection,
* expose ``RadarGatewaySnapshot`` with derived diagnostic counters (packet
  rate, packet loss, packet age).

The actual background reader lives in
``sensors/radar_gateway.py`` (added in a later task).  Here we keep the
pure-parser surface so the dashboard, preflight and tests can share it
without any hardware imports.
"""
from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

from config.radar_gateway import MAX_LINE_BYTES, RADAR_IDS, PROTOCOL_VERSIONS

LINE_PREFIX_RE = re.compile(rb"^[^\n]{1,%d}\n?" % MAX_LINE_BYTES)


def _coerce_float(value: Any) -> Optional[float]:
    """Reject ``None``, NaN, +/-Infinity and non-numeric values."""
    if value is None:
        return None
    if isinstance(value, bool):
        # JSON booleans are technically numeric; never treat True == 1.
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number) or math.isinf(number):
        return None
    return number


def _coerce_energy(value: Any) -> Optional[int]:
    number = _coerce_float(value)
    if number is None:
        return None
    return int(round(number))


def _coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return bool(value)


def _coerce_str(value: Any, *, default: str = "") -> str:
    if value is None:
        return default
    if isinstance(value, str):
        return value
    return str(value)


@dataclass(slots=True, frozen=True)
class RadarFrame:
    """One radar decoded from a Gateway JSON v2 packet."""

    radar_id: str
    detected: bool = False
    target_state: str = "none"
    distance_m: Optional[float] = None
    moving_distance_m: Optional[float] = None
    moving_energy: Optional[int] = None
    stationary_distance_m: Optional[float] = None
    stationary_energy: Optional[int] = None
    valid: bool = False
    stale: bool = True
    age_ms: Optional[int] = None
    error: str = ""

    @classmethod
    def empty(cls, radar_id: str) -> "RadarFrame":
        return cls(radar_id=radar_id)


@dataclass(slots=True, frozen=True)
class RadarGatewayPacket:
    """Decoded JSON v2 packet from ESP32 A Gateway."""

    seq: int
    gateway_ok: bool
    remote_ok: bool
    gateway_boot_id: str
    remote_boot_id: str
    packet_loss: float
    radars: dict[str, RadarFrame]
    age_ms: Optional[int]
    json_error: str = ""

    @property
    def valid(self) -> bool:
        return not self.json_error


def parse_gateway_packet(
    line: str | bytes,
    *,
    received_monotonic: Optional[float] = None,
    max_distance_m: float = 5.0,
) -> RadarGatewayPacket:
    """Parse one JSON line from the Gateway.

    Raises :class:`ValueError` for any malformed/incomplete packet so the
    reader can update its JSON-error counter and keep going.
    """
    if isinstance(line, bytes):
        try:
            text = line.decode("utf-8", errors="strict")
        except UnicodeDecodeError as exc:
            raise ValueError(f"Gateway frame not UTF-8: {exc}") from exc
    else:
        text = line
    if len(text) > MAX_LINE_BYTES:
        raise ValueError(f"Gateway frame too long ({len(text)} > {MAX_LINE_BYTES})")
    text = text.strip()
    if not text:
        raise ValueError("empty gateway frame")
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("Gateway frame must be a JSON object")

    version = payload.get("v")
    try:
        version_int = int(version)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Gateway frame missing numeric 'v': {exc}") from exc
    if version_int not in PROTOCOL_VERSIONS:
        raise ValueError(f"Unsupported protocol version: {version_int}")

    try:
        seq = int(payload.get("seq"))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"bad seq: {exc}") from exc
    if seq < 0:
        raise ValueError("seq must be >= 0")

    gateway_ok = _coerce_bool(payload.get("gateway_ok"))
    remote_ok = _coerce_bool(payload.get("remote_ok"))
    gateway_boot_id = _coerce_str(payload.get("gateway_boot_id"))
    remote_boot_id = _coerce_str(payload.get("remote_boot_id"))
    try:
        packet_loss = float(_coerce_float(payload.get("packet_loss")) or 0.0)
    except (TypeError, ValueError):
        packet_loss = 0.0

    raw_radars = payload.get("radars")
    if not isinstance(raw_radars, dict):
        raise ValueError("radars must be an object")

    age_ms_value = _coerce_float(payload.get("age_ms"))
    age_ms = int(round(age_ms_value)) if age_ms_value is not None else None

    radars: dict[str, RadarFrame] = {}
    for radar_id in RADAR_IDS:
        item = raw_radars.get(radar_id)
        if not isinstance(item, dict):
            radars[radar_id] = RadarFrame.empty(radar_id)
            continue
        distance = _coerce_float(item.get("distance_m"))
        if distance is not None and not 0.0 <= distance <= max_distance_m:
            distance = None
        moving = _coerce_float(item.get("moving_distance_m"))
        if moving is not None and not 0.0 <= moving <= max_distance_m:
            moving = None
        stationary = _coerce_float(item.get("stationary_distance_m"))
        if stationary is not None and not 0.0 <= stationary <= max_distance_m:
            stationary = None
        frame = RadarFrame(
            radar_id=radar_id,
            detected=_coerce_bool(item.get("detected")),
            target_state=_coerce_str(item.get("target_state", "none"), default="none"),
            distance_m=distance,
            moving_distance_m=moving,
            moving_energy=_coerce_energy(item.get("moving_energy")),
            stationary_distance_m=stationary,
            stationary_energy=_coerce_energy(item.get("stationary_energy")),
            valid=bool(item.get("valid", False)),
            stale=bool(item.get("stale", True)),
            age_ms=(
                int(round(_coerce_float(item.get("age_ms"))))
                if _coerce_float(item.get("age_ms")) is not None
                else None
            ),
            error=_coerce_str(item.get("error", "")),
        )
        radars[radar_id] = frame

    return RadarGatewayPacket(
        seq=seq,
        gateway_ok=gateway_ok,
        remote_ok=remote_ok,
        gateway_boot_id=gateway_boot_id,
        remote_boot_id=remote_boot_id,
        packet_loss=packet_loss,
        radars=radars,
        age_ms=age_ms,
    )


@dataclass(slots=True)
class RadarGatewaySnapshot:
    """Diagnostic snapshot of the Gateway link for the dashboard."""

    packets_total: int = 0
    packets_invalid: int = 0
    crc_errors: int = 0
    json_errors: int = 0
    last_packet: Optional[RadarGatewayPacket] = None
    last_valid_monotonic: float = 0.0
    packets_dropped: int = 0  # seq gaps on received numbers
    sample_lines: list[str] = field(default_factory=list)

    @property
    def packet_rate_hz(self) -> float:
        if not self.last_packet:
            return 0.0
        elapsed = time.monotonic() - self.last_valid_monotonic
        if elapsed <= 0 or self.packets_total < 2:
            return 0.0
        return round(self.packets_total / max(0.001, elapsed), 2)

    def update_from_packet(self, packet: RadarGatewayPacket, *, monotonic: Optional[float] = None) -> None:
        now = time.monotonic() if monotonic is None else float(monotonic)
        if self.last_packet and packet.seq <= self.last_packet.seq:
            # Monotonic seq (32-bit wrap).  Older/duplicate is not a "drop".
            pass
        elif self.last_packet and packet.seq > self.last_packet.seq + 1:
            self.packets_dropped += packet.seq - self.last_packet.seq - 1
        self.packets_total += 1
        self.last_packet = packet
        self.last_valid_monotonic = now

    def record_invalid(self, *, json_error: bool = True) -> None:
        self.packets_invalid += 1
        if json_error:
            self.json_errors += 1
        else:
            self.crc_errors += 1

    def age_ms(self, *, monotonic: Optional[float] = None) -> Optional[int]:
        if self.last_packet is None:
            return None
        now = time.monotonic() if monotonic is None else float(monotonic)
        return int(round((now - self.last_valid_monotonic) * 1000.0))


__all__ = [
    "RadarFrame",
    "RadarGatewayPacket",
    "RadarGatewaySnapshot",
    "parse_gateway_packet",
]
