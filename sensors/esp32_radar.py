"""Two-side radar receiver over an ESP32 UART bridge.

Protocol (one UTF-8 JSON object per line)::

    {"v":1,"seq":42,"left":true,"right":false,
     "left_m":2.4,"right_m":null}

Only local receive time is trusted for freshness.  A stale or malformed link
never creates a positive detection.
"""
from __future__ import annotations

import json
import logging
import os
import select
import threading
import time
from dataclasses import dataclass, replace
from typing import Optional

try:  # POSIX only; parsing/selector tests still work on Windows.
    import termios
except ImportError:  # pragma: no cover - Jetson/Linux always has termios
    termios = None  # type: ignore[assignment]

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class SideRadarState:
    sequence: int = -1
    left_detected: bool = False
    right_detected: bool = False
    left_distance_m: Optional[float] = None
    right_distance_m: Optional[float] = None
    received_monotonic: float = 0.0
    link_ok: bool = False


def _distance(value: object, name: str, max_distance_m: float) -> Optional[float]:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric or null") from exc
    if not 0.0 <= result <= max_distance_m:
        raise ValueError(f"{name} outside 0..{max_distance_m}m")
    return result


def parse_radar_line(
    raw: bytes | str,
    *,
    received_monotonic: Optional[float] = None,
    max_distance_m: float = 30.0,
) -> SideRadarState:
    """Validate one newline-delimited ESP32 message."""
    text = raw.decode("utf-8", errors="strict") if isinstance(raw, bytes) else raw
    if len(text) > 512:
        raise ValueError("radar frame too long")
    payload = json.loads(text)
    if not isinstance(payload, dict) or payload.get("v") != 1:
        raise ValueError("unsupported radar protocol version")
    sequence = int(payload["seq"])
    if sequence < 0:
        raise ValueError("seq must be >= 0")
    left = payload.get("left")
    right = payload.get("right")
    if not isinstance(left, bool) or not isinstance(right, bool):
        raise ValueError("left/right must be JSON booleans")
    left_m = _distance(payload.get("left_m"), "left_m", max_distance_m)
    right_m = _distance(payload.get("right_m"), "right_m", max_distance_m)
    return SideRadarState(
        sequence=sequence,
        left_detected=left,
        right_detected=right,
        left_distance_m=left_m,
        right_distance_m=right_m,
        received_monotonic=(
            time.monotonic() if received_monotonic is None else float(received_monotonic)
        ),
        link_ok=True,
    )


class ESP32RadarReader:
    """Background stdlib UART reader; no pyserial dependency required."""

    _BAUD = {} if termios is None else {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }

    def __init__(
        self,
        device: str,
        *,
        baudrate: int = 115200,
        stale_after_seconds: float = 0.6,
        max_distance_m: float = 30.0,
    ) -> None:
        self.device = str(device)
        self.baudrate = int(baudrate)
        self.stale_after_seconds = max(0.1, float(stale_after_seconds))
        self.max_distance_m = float(max_distance_m)
        self._fd: Optional[int] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._state = SideRadarState()
        self._last_sequence = -1
        self.error: Optional[BaseException] = None

    def connect(self) -> None:
        if self._running:
            return
        if termios is None:
            raise RuntimeError("ESP32 UART reader requires Linux/POSIX termios")
        speed = self._BAUD.get(self.baudrate)
        if speed is None:
            raise ValueError(f"Unsupported UART baudrate: {self.baudrate}")
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
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, name="esp32-radar-uart", daemon=True)
        self._thread.start()
        LOGGER.info("ESP32 radar UART connected: %s @ %d", self.device, self.baudrate)

    def _read_loop(self) -> None:
        buffer = bytearray()
        try:
            while self._running and self._fd is not None:
                ready, _, _ = select.select([self._fd], [], [], 0.2)
                if not ready:
                    continue
                chunk = os.read(self._fd, 256)
                if not chunk:
                    continue
                buffer.extend(chunk)
                if len(buffer) > 2048:
                    buffer.clear()
                    LOGGER.warning("ESP32 radar UART buffer overflow; frame dropped")
                    continue
                while b"\n" in buffer:
                    line, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    if not line.strip():
                        continue
                    try:
                        state = parse_radar_line(line, max_distance_m=self.max_distance_m)
                    except (UnicodeError, ValueError, KeyError, json.JSONDecodeError) as exc:
                        LOGGER.warning("Invalid ESP32 radar frame ignored: %s", exc)
                        continue
                    # ESP32 sequence is monotonic until reboot. Sequence 0 is
                    # accepted as a reboot marker.
                    if state.sequence <= self._last_sequence and state.sequence != 0:
                        continue
                    self._last_sequence = state.sequence
                    with self._lock:
                        self._state = state
        except BaseException as exc:  # surfaced to health/diagnostics
            self.error = exc
            LOGGER.error("ESP32 radar UART stopped: %s", exc)
        finally:
            self._running = False

    def read(self, now: Optional[float] = None) -> SideRadarState:
        current_time = time.monotonic() if now is None else float(now)
        with self._lock:
            state = self._state
        if not state.link_ok or current_time - state.received_monotonic > self.stale_after_seconds:
            return replace(
                state,
                left_detected=False,
                right_detected=False,
                left_distance_m=None,
                right_distance_m=None,
                link_ok=False,
            )
        return state

    def is_connected(self) -> bool:
        return self._running and self._fd is not None and self.read().link_ok

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


__all__ = ["ESP32RadarReader", "SideRadarState", "parse_radar_line"]
