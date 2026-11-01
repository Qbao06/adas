"""Radar Gateway UART reader and JSON v2 codec for the Jetson side.

The Gateway ESP32 (ESP32 A) streams one JSON v2 object per line over USB
Serial.  This module owns:

* :class:`GatewayRadarSample` -- one parsed radar sample (matches the
  protocol v2 contract from Prompt 1).
* :class:`GatewaySnapshot` -- a coherent batch of four samples plus the
  diagnostic counters needed by the dashboard.
* :class:`RadarGatewayReader` -- background reader thread that opens the
  UART, parses the stream, drops malformed JSON, and exposes
  :func:`read` for the pipeline.  The reader never invents detections:
  stale or malformed frames result in ``valid=False`` and ``stale=True``
  with all numeric fields at the no-target sentinel.
* :func:`parse_gateway_line` -- the pure-parser function reused by the
  CLI monitor and by tests.

This file is the Python counterpart of the firmware/ESP32 Radar Gateway
C++ implementation.  The protocol is byte-stable so the same parser can
also work on a host harness replaying a captured log.
"""
from __future__ import annotations

import json
import math
import os
import select
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Optional

from config.radar_gateway import (
    MAX_LINE_BYTES,
    PROTOCOL_VERSIONS,
    RADAR_IDS,
)

try:  # POSIX only; parser/selector tests still work on Windows.
    import termios
except ImportError:  # pragma: no cover - Jetson/Linux always has termios.
    termios = None  # type: ignore[assignment]


# --------------------------------------------------------------------------- #
# Sentinel values (must match the firmware protocol).
# --------------------------------------------------------------------------- #
TARGET_STATE_NONE = 0x00
TARGET_STATE_MOVING = 0x01
TARGET_STATE_STATIONARY = 0x02
TARGET_STATE_BOTH = 0x03

TARGET_STATE_STRINGS = {
    TARGET_STATE_MOVING: "moving",
    TARGET_STATE_STATIONARY: "stationary",
    TARGET_STATE_BOTH: "both",
}


# --------------------------------------------------------------------------- #
# Pure helpers
# --------------------------------------------------------------------------- #
def _coerce_float(value: Any) -> Optional[float]:
    """Reject None, NaN, +/-Infinity and non-numeric values."""
    if value is None:
        return None
    if isinstance(value, bool):
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


# --------------------------------------------------------------------------- #
# Data containers
# --------------------------------------------------------------------------- #
@dataclass(slots=True, frozen=True)
class GatewayRadarSample:
    """One radar decoded from a Gateway JSON v2 packet."""

    radar_id: str
    detected: bool = False
    target_state: int = TARGET_STATE_NONE
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
    def empty(cls, radar_id: str) -> "GatewayRadarSample":
        return cls(radar_id=radar_id)


@dataclass(slots=True)
class GatewaySnapshot:
    """Diagnostic snapshot of the Gateway link for the pipeline + dashboard."""

    sequence: int = -1
    gateway_ok: bool = False
    remote_ok: bool = False
    gateway_boot_id: str = ""
    remote_boot_id: str = ""
    packet_loss: float = 0.0
    radars: Dict[str, GatewayRadarSample] = field(default_factory=dict)
    age_ms: Optional[int] = None
    received_monotonic: float = 0.0

    @property
    def link_ok(self) -> bool:
        """True when the Gateway is online (its own USB link)."""
        return bool(self.gateway_ok)

    def as_dict(self) -> Dict[str, Any]:
        return {
            "v": 2,
            "seq": self.sequence,
            "gateway_ok": self.gateway_ok,
            "remote_ok": self.remote_ok,
            "gateway_boot_id": self.gateway_boot_id,
            "remote_boot_id": self.remote_boot_id,
            "packet_loss": self.packet_loss,
            "age_ms": self.age_ms,
            "radars": {
                rid: {
                    "detected": sample.detected,
                    "target_state": TARGET_STATE_STRINGS.get(
                        sample.target_state, "none"
                    ),
                    "distance_m": sample.distance_m,
                    "moving_distance_m": sample.moving_distance_m,
                    "moving_energy": sample.moving_energy,
                    "stationary_distance_m": sample.stationary_distance_m,
                    "stationary_energy": sample.stationary_energy,
                    "valid": sample.valid,
                    "stale": sample.stale,
                    "age_ms": sample.age_ms,
                    "error": sample.error,
                }
                for rid, sample in self.radars.items()
            },
        }


# --------------------------------------------------------------------------- #
# Pure parser  (delegates to radar_gateway_packet for DRY)
# --------------------------------------------------------------------------- #
def parse_gateway_line(
    raw: bytes | str,
    *,
    received_monotonic: Optional[float] = None,
    max_distance_m: float = 5.0,
) -> GatewaySnapshot:
    """Validate a single v2 Gateway JSON line.

    M4 refactor: delegates to :func:`radar_gateway_packet.parse_gateway_packet`
    so the schema enforcement lives in one place.  The result is converted to
    the :class:`GatewaySnapshot` / :class:`GatewayRadarSample` types expected
    by the rest of the pipeline.
    """
    # Import lazily to avoid a circular dependency at module load time.
    from sensors.radar_gateway_packet import (  # noqa: E402
        RadarGatewayPacket,
        parse_gateway_packet,
        RadarFrame,
    )

    pkt = parse_gateway_packet(
        raw,
        received_monotonic=received_monotonic,
        max_distance_m=max_distance_m,
    )
    now_monotonic = (
        time.monotonic() if received_monotonic is None else float(received_monotonic)
    )
    gateways = {
        rid: GatewayRadarSample(
            radar_id=rid,
            detected=rf.detected,
            target_state=_state_to_int(rf.target_state),
            distance_m=rf.distance_m,
            moving_distance_m=rf.moving_distance_m,
            moving_energy=rf.moving_energy,
            stationary_distance_m=rf.stationary_distance_m,
            stationary_energy=rf.stationary_energy,
            valid=rf.valid,
            stale=rf.stale,
            age_ms=rf.age_ms,
            error=rf.error,
        )
        for rid, rf in pkt.radars.items()
    }
    for rid in RADAR_IDS:
        if rid not in gateways:
            gateways[rid] = GatewayRadarSample.empty(rid)

    return GatewaySnapshot(
        sequence=pkt.seq,
        gateway_ok=pkt.gateway_ok,
        remote_ok=pkt.remote_ok,
        gateway_boot_id=pkt.gateway_boot_id,
        remote_boot_id=pkt.remote_boot_id,
        packet_loss=pkt.packet_loss,
        radars=gateways,
        age_ms=pkt.age_ms,
        received_monotonic=now_monotonic,
    )


def _state_to_int(value: Any) -> int:
    if isinstance(value, int) and not isinstance(value, bool):
        if value in TARGET_STATE_STRINGS:
            return value
        return TARGET_STATE_NONE
    if isinstance(value, str):
        text = value.strip().lower()
        for code, name in TARGET_STATE_STRINGS.items():
            if name == text:
                return code
    return TARGET_STATE_NONE


# --------------------------------------------------------------------------- #
# Background UART reader
# --------------------------------------------------------------------------- #
class RadarGatewayReader:
    """Stdlib UART reader for the JSON-v2 Gateway stream.

    The reader is intentionally simple: a background thread pulls bytes,
    splits them on ``\\n`` and feeds each line to :func:`parse_gateway_line`.
    A bounded buffer protects against runaway producers, and any exception
    in the read loop is surfaced through ``self.error`` so the health
    monitor can flag it.  No pyserial dependency is required.
    """

    _BAUD_MAP: Dict[int, int] = {}

    def __init__(
        self,
        device: str,
        *,
        baudrate: int = 115200,
        stale_after_seconds: float = 0.6,
        reconnect_seconds: float = 2.0,
        max_line_bytes: int = MAX_LINE_BYTES,
    ) -> None:
        self.device = str(device)
        self.baudrate = int(baudrate)
        self.stale_after_seconds = max(0.1, float(stale_after_seconds))
        self.reconnect_seconds = max(0.1, float(reconnect_seconds))
        self.max_line_bytes = int(max_line_bytes) if max_line_bytes else MAX_LINE_BYTES
        if termios is not None and not self._BAUD_MAP:
            self._BAUD_MAP.update(
                {
                    9600: termios.B9600,
                    19200: termios.B19200,
                    38400: termios.B38400,
                    57600: termios.B57600,
                    115200: termios.B115200,
                    230400: termios.B230400,
                }
            )
            baud_256k = getattr(termios, "B256000", None)
            if baud_256k is not None:
                self._BAUD_MAP[256000] = baud_256k
        self._fd: Optional[int] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._snapshot = self._empty_snapshot()
        self._last_sequence = -1
        self._last_gateway_boot_id = ""
        self._last_remote_seen = 0.0
        self._last_gateway_seen = 0.0
        self._received_packets = 0
        self._duplicate_packets = 0
        self._open_attempts = 0
        self._reconnect_count = 0
        self._json_errors = 0
        self._crc_errors = 0  # v2 has no CRC; reserved for future use.
        self.error: Optional[BaseException] = None
        self._last_valid_monotonic = 0.0

    # ------------------------------------------------------------------ public
    @staticmethod
    def _empty_snapshot() -> GatewaySnapshot:
        radars = {rid: GatewayRadarSample.empty(rid) for rid in RADAR_IDS}
        return GatewaySnapshot(
            sequence=-1,
            gateway_ok=False,
            remote_ok=False,
            gateway_boot_id="",
            remote_boot_id="",
            packet_loss=0.0,
            radars=radars,
            age_ms=None,
            received_monotonic=0.0,
        )

    def connect(self) -> None:
        if self._running:
            return
        self._open_device()
        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop, name="radar-gateway-uart", daemon=True
        )
        self._thread.start()

    def _open_device(self) -> None:
        """Open/configure UART without creating another reader thread."""
        if termios is None:
            raise RuntimeError("RadarGatewayReader requires Linux/POSIX termios")
        speed = self._BAUD_MAP.get(self.baudrate)
        if speed is None:
            raise ValueError(f"Unsupported UART baudrate: {self.baudrate}")
        self._open_attempts += 1
        fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            attrs[4] = speed
            attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 1
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIFLUSH)
        except Exception:
            os.close(fd)
            raise
        self._fd = fd
        self.error = None

    def disconnect(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

    def is_connected(self) -> bool:
        return self._running and self._fd is not None and self._snapshot.gateway_ok

    def stats(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "received_packets": self._received_packets,
                "duplicate_packets": self._duplicate_packets,
                "json_errors": self._json_errors,
                "crc_errors": self._crc_errors,
                "reconnect_count": self._reconnect_count,
                "open_attempts": self._open_attempts,
                "last_sequence": self._last_sequence,
                "last_valid_monotonic": self._last_valid_monotonic,
            }

    def read(self, *, now: Optional[float] = None) -> GatewaySnapshot:
        current_time = time.monotonic() if now is None else float(now)
        with self._lock:
            snapshot = self._snapshot
        # Apply freshness rules.  The Gateway sample is considered fresh
        # when we saw a packet within stale_after_seconds.  The Remote
        # side is judged separately because its packets travel over ESP-NOW
        # which can drop independently of the USB link.
        stale_link = (
            snapshot.received_monotonic == 0.0
            or current_time - snapshot.received_monotonic > self.stale_after_seconds
        )
        radars: Dict[str, GatewayRadarSample] = {}
        for rid, sample in snapshot.radars.items():
            if stale_link:
                radars[rid] = GatewayRadarSample(
                    radar_id=rid,
                    detected=False,
                    target_state=TARGET_STATE_NONE,
                    distance_m=None,
                    moving_distance_m=None,
                    moving_energy=None,
                    stationary_distance_m=None,
                    stationary_energy=None,
                    valid=False,
                    stale=True,
                    age_ms=None,
                    error="no-packet",
                )
            else:
                radars[rid] = sample
        return GatewaySnapshot(
            sequence=snapshot.sequence,
            gateway_ok=(not stale_link) and snapshot.gateway_ok,
            remote_ok=(not stale_link) and snapshot.remote_ok,
            gateway_boot_id=snapshot.gateway_boot_id,
            remote_boot_id=snapshot.remote_boot_id,
            packet_loss=snapshot.packet_loss,
            radars=radars,
            age_ms=snapshot.age_ms,
            received_monotonic=snapshot.received_monotonic,
        )

    # ----------------------------------------------------------------- internals
    def _read_loop(self) -> None:
        import logging as _logging

        _log = _logging.getLogger("RadarGatewayReader")
        buffer = bytearray()
        try:
            while self._running:
                # If fd is None, we are in a reconnect phase; sleep and
                # retry connect until success or the caller sets _running=False.
                if self._fd is None:
                    time.sleep(min(0.5, self.reconnect_seconds))
                    try:
                        self._open_device()
                    except Exception:
                        pass
                    if not self._running:
                        break
                    continue
                ready, _, _ = select.select([self._fd], [], [], 0.2)
                if not ready:
                    continue
                try:
                    chunk = os.read(self._fd, 512)
                except OSError as exc:
                    # Cable pulled / USB disconnect → reconnect automatically.
                    _log.warning(
                        "Gateway UART read OSError (%s), reconnecting in %.1fs",
                        exc,
                        self.reconnect_seconds,
                    )
                    self.error = exc
                    try:
                        os.close(self._fd)
                    except OSError:
                        pass
                    self._fd = None
                    self._reconnect_count += 1
                    time.sleep(self.reconnect_seconds)
                    try:
                        self._open_device()
                    except Exception:
                        pass
                    continue
                if not chunk:
                    continue
                buffer.extend(chunk)
                if len(buffer) > 8192:
                    # M3: log warning (not silent clear) on buffer overflow.
                    _log.warning(
                        "Gateway buffer overflow (>%d bytes), dropping. "
                        "Increase MAX_LINE_BYTES if this recurs.",
                        8192,
                    )
                    buffer.clear()
                    self._json_errors += 1
                    continue
                while b"\n" in buffer:
                    line, _, buffer = buffer.partition(b"\n")
                    if line.endswith(b"\r"):
                        line = line[:-1]
                    if not line.strip():
                        continue
                    self._handle_line(line)
        except BaseException as exc:  # surfaced to health monitor.
            self.error = exc
        finally:
            self._running = False

    def _handle_line(self, line: bytes) -> None:
        try:
            snapshot = parse_gateway_line(
                line,
                received_monotonic=time.monotonic(),
                max_distance_m=5.0,
            )
        except ValueError:
            self._json_errors += 1
            return

        with self._lock:
            self._received_packets += 1
            if (
                snapshot.gateway_boot_id
                and snapshot.gateway_boot_id != self._last_gateway_boot_id
            ):
                self._last_sequence = -1
                self._last_gateway_boot_id = snapshot.gateway_boot_id
            if (
                snapshot.sequence >= 0
                and snapshot.sequence <= self._last_sequence
                and snapshot.sequence != 0
            ):
                self._duplicate_packets += 1
                return
            self._last_sequence = snapshot.sequence
            self._last_valid_monotonic = time.monotonic()
            self._snapshot = snapshot

    # Reconnect helper used by tests/diagnostics.
    def reconnect(self) -> None:
        self.disconnect()
        time.sleep(min(0.5, self.reconnect_seconds))
        self._reconnect_count += 1
        self.connect()


__all__ = [
    "GatewayRadarSample",
    "GatewaySnapshot",
    "RadarGatewayReader",
    "parse_gateway_line",
    "TARGET_STATE_NONE",
    "TARGET_STATE_MOVING",
    "TARGET_STATE_STATIONARY",
    "TARGET_STATE_BOTH",
    "TARGET_STATE_STRINGS",
]
