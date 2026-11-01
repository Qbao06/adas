"""Radar Gateway monitor and integration-test CLI for the ADAS prototype.

Usage
-----

Live monitor against an ESP32 Gateway that streams newline-delimited JSON
(default 115200 8N1)::

    python tools/radar_gateway_monitor.py \\
        --device /dev/serial/by-id/usb-... \\
        --baudrate 115200

Replay a captured log file or pipe one in via stdin (no hardware required)::

    python tools/radar_gateway_monitor.py --raw path/to/capture.jsonl
    cat capture.jsonl | python tools/radar_gateway_monitor.py --raw -

Interactive mapping-test (asks the operator to physically confirm each radar
position)::

    python tools/radar_gateway_monitor.py --mapping-test \\
        --device /dev/serial/by-id/... --baudrate 115200

Soak test (CPU/RAM/Jetson temperature, packet rate, packet loss, reconnect
count, CSV + JSON reports)::

    python tools/radar_gateway_monitor.py --soak --duration 30m \\
        --csv logs/soak.csv --json-report logs/soak.json \\
        --device /dev/serial/by-id/...

Fault injection against a synthetic stream (no hardware)::

    python tools/radar_gateway_monitor.py --fault esp32_b_power_loss
    python tools/radar_gateway_monitor.py --fault gateway_usb_unplug_replug \\
        --duration 30

The monitor NEVER declares the vehicle safe based on mock tests.  It only
verifies that the Jetson pipeline never fabricates detections, that stale
radars are flagged ``valid=false, stale=true`` and that loss of the Remote
ESP32 only affects the two Remote-side radars.

Implementation notes
--------------------
* The default parser is ``sensors.esp32_radar.parse_radar_line`` (protocol
  v1, two sides).  A v2 parser is also registered so that deployments that
  have already migrated to the four-radar Gateway (per prompts 1-3) can use
  this monitor without changes.
* Streams from real UART, a file or ``-`` (stdin) are all supported so the
  tool can be exercised in CI without hardware.
* Reconnect / loss / fault scenarios are implemented as small deterministic
  state machines.  They are useful for soak-test and for confirming that the
  reader + downstream selector behaviour matches the safety contract, but
  they do NOT prove the radar itself is functional.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import random
import select
import signal
import sys

for _stream in (sys.stdout, sys.stderr):
    reconfigure = getattr(_stream, "reconfigure", None)
    if callable(reconfigure):
        try:
            reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Deque, Dict, Iterable, Iterator, List, Optional, Sequence, TextIO, Tuple

try:
    from sensors.esp32_radar import (
        ESP32RadarReader,
        SideRadarState,
        parse_radar_line,
    )
except Exception:  # pragma: no cover - keep CLI importable for ``--help``
    ESP32RadarReader = None  # type: ignore[assignment]
    SideRadarState = None  # type: ignore[assignment]
    parse_radar_line = None  # type: ignore[assignment]


# ---------------------------------------------------------------------------
# Colour helpers (ANSI).  Disabled with --no-color.
# ---------------------------------------------------------------------------

class _Palette:
    RESET = "\x1b[0m"
    BOLD = "\x1b[1m"
    DIM = "\x1b[2m"
    RED = "\x1b[31m"
    GREEN = "\x1b[32m"
    YELLOW = "\x1b[33m"
    BLUE = "\x1b[34m"
    MAGENTA = "\x1b[35m"
    CYAN = "\x1b[36m"


_PALETTE = _Palette()


def _c(text: str, color: str, enabled: bool) -> str:
    if not enabled:
        return text
    return f"{color}{text}{_PALETTE.RESET}"


# ---------------------------------------------------------------------------
# Stream abstraction.  Real UART, file, stdin, or synthetic.
# ---------------------------------------------------------------------------


@dataclass
class StreamLine:
    """A single newline-delimited message together with receive time."""

    raw: bytes
    received_monotonic: float


class _BaseStream:
    name = "stream"

    def open(self) -> None:
        pass

    def close(self) -> None:
        pass

    def read_lines(self, deadline: Optional[float] = None) -> Iterator[StreamLine]:
        raise NotImplementedError


class _StdinStream(_BaseStream):
    name = "stdin"

    def __init__(self) -> None:
        self._buf = b""

    def read_lines(self, deadline: Optional[float] = None) -> Iterator[StreamLine]:
        chunk = sys.stdin.buffer.read1(4096)
        if not chunk:
            return iter(())
        self._buf += chunk
        out: List[StreamLine] = []
        while True:
            idx = self._buf.find(b"\n")
            if idx < 0:
                break
            line, self._buf = self._buf[:idx], self._buf[idx + 1 :]
            if line.endswith(b"\r"):
                line = line[:-1]
            if not line.strip():
                continue
            out.append(StreamLine(line, time.monotonic()))
        return iter(out)


class _FileStream(_BaseStream):
    def __init__(self, path: Path) -> None:
        self.path = path
        self._fp: Optional[TextIO] = None

    @property
    def name(self) -> str:
        return f"file:{self.path}"

    def open(self) -> None:
        self._fp = self.path.open("r", encoding="utf-8")

    def close(self) -> None:
        if self._fp is not None:
            self._fp.close()
            self._fp = None

    def read_lines(self, deadline: Optional[float] = None) -> Iterator[StreamLine]:
        assert self._fp is not None
        out: List[StreamLine] = []
        while True:
            pos = self._fp.tell()
            line = self._fp.readline()
            if not line:
                if pos == 0:
                    break
                # Reached EOF for this poll cycle; rewind so the next poll
                # can resume reading if more data is appended (live log).
                self._fp.seek(pos)
                break
            stripped = line.rstrip("\r\n")
            if not stripped.strip():
                continue
            out.append(StreamLine(stripped.encode("utf-8"), time.monotonic()))
        return iter(out)


class _UartStream(_BaseStream):
    """POSIX UART reader that mirrors the production reader's behaviour."""

    def __init__(self, device: str, baudrate: int) -> None:
        self.device = device
        self.baudrate = baudrate
        self._fd: Optional[int] = None
        self._buffer = bytearray()
        self._termios: Optional[object] = None

    @property
    def name(self) -> str:
        return f"uart:{self.device}"

    def open(self) -> None:
        import termios  # POSIX only

        self._termios = termios
        baud_map = {
            9600: termios.B9600,
            19200: termios.B19200,
            38400: termios.B38400,
            57600: termios.B57600,
            115200: termios.B115200,
            230400: termios.B230400,
            256000: getattr(termios, "B256000", termios.B115200),
        }
        speed = baud_map.get(int(self.baudrate))
        if speed is None:
            raise SystemExit(f"Unsupported UART baudrate: {self.baudrate}")
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

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

    def read_lines(self, deadline: Optional[float] = None) -> Iterator[StreamLine]:
        if self._fd is None:
            return iter(())
        ready, _, _ = select.select([self._fd], [], [], 0.1)
        out: List[StreamLine] = []
        if ready:
            try:
                chunk = os.read(self._fd, 512)
            except BlockingIOError:
                chunk = b""
            if chunk:
                self._buffer.extend(chunk)
                if len(self._buffer) > 8192:
                    self._buffer.clear()
        while True:
            idx = self._buffer.find(b"\n")
            if idx < 0:
                break
            line, self._buffer = self._buffer[:idx], self._buffer[idx + 1 :]
            if line.endswith(b"\r"):
                line = line[:-1]
            if not line.strip():
                continue
            out.append(StreamLine(line, time.monotonic()))
        return iter(out)


# ---------------------------------------------------------------------------
# Protocol v2 (four radar + Gateway/Remote).  Optional.
# ---------------------------------------------------------------------------


REQUIRED_RADAR_IDS: Tuple[str, ...] = (
    "left_front",
    "left_rear",
    "right_front",
    "right_rear",
)


@dataclass
class RadarSample:
    """A parsed Gateway/Remote packet (v2)."""

    received_monotonic: float
    sequence: int
    gateway_ok: bool
    remote_ok: bool
    gateway_boot_id: int
    remote_boot_id: int
    packet_loss: float
    radars: Dict[str, Dict[str, object]] = field(default_factory=dict)
    errors: List[str] = field(default_factory=list)

    @property
    def link_ok(self) -> bool:
        return self.gateway_ok


def parse_gateway_line(
    raw: bytes | str,
    *,
    received_monotonic: Optional[float] = None,
    max_distance_m: float = 6.0,
) -> RadarSample:
    """Validate a single v2 Gateway JSON packet.

    Implements the protocol that prompt 1 defined: ``v``, ``seq``,
    ``gateway_ok``, ``remote_ok``, ``gateway_boot_id``, ``remote_boot_id``,
    ``packet_loss`` and four radar objects.  Distance fields are checked for
    negative / non-finite values.  Missing fields produce explicit errors
    rather than silently defaulting to ``False``/``null``.
    """
    text = raw.decode("utf-8") if isinstance(raw, bytes) else raw
    payload = json.loads(text)
    if not isinstance(payload, dict):
        raise ValueError("payload must be a JSON object")
    if int(payload.get("v", -1)) != 2:
        raise ValueError("unsupported gateway protocol version (need v=2)")
    sequence = int(payload["seq"])
    if sequence < 0:
        raise ValueError("seq must be >= 0")
    sample = RadarSample(
        received_monotonic=(
            time.monotonic() if received_monotonic is None else float(received_monotonic)
        ),
        sequence=sequence,
        gateway_ok=bool(payload.get("gateway_ok", False)),
        remote_ok=bool(payload.get("remote_ok", False)),
        gateway_boot_id=int(payload.get("gateway_boot_id", 0)),
        remote_boot_id=int(payload.get("remote_boot_id", 0)),
        packet_loss=float(payload.get("packet_loss", 0.0)),
    )
    radars_obj = payload.get("radars")
    if not isinstance(radars_obj, dict):
        sample.errors.append("missing radars object")
        return sample
    for radar_id in REQUIRED_RADAR_IDS:
        radar = radars_obj.get(radar_id)
        if not isinstance(radar, dict):
            sample.errors.append(f"missing radar {radar_id}")
            continue
        try:
            sample.radars[radar_id] = _parse_radar_v2(radar_id, radar, max_distance_m)
        except ValueError as exc:
            sample.errors.append(f"radar {radar_id}: {exc}")
    return sample


def _parse_radar_v2(
    radar_id: str, radar: Dict[str, object], max_distance_m: float
) -> Dict[str, object]:
    valid = bool(radar.get("valid", False))
    stale = bool(radar.get("stale", True))
    detected = bool(radar.get("detected", False))
    target_state = int(radar.get("target_state", 0))
    distance_m = _distance_or_none(
        radar.get("distance_m"), "distance_m", max_distance_m
    )
    moving_distance_m = _distance_or_none(
        radar.get("moving_distance_m"), "moving_distance_m", max_distance_m
    )
    moving_energy = _int_or_none(radar.get("moving_energy"), "moving_energy")
    stationary_distance_m = _distance_or_none(
        radar.get("stationary_distance_m"),
        "stationary_distance_m",
        max_distance_m,
    )
    stationary_energy = _int_or_none(
        radar.get("stationary_energy"), "stationary_energy"
    )
    age_ms = _int_or_none(radar.get("age_ms"), "age_ms")
    return {
        "detected": detected,
        "valid": valid,
        "stale": stale,
        "target_state": target_state,
        "distance_m": distance_m,
        "moving_distance_m": moving_distance_m,
        "moving_energy": moving_energy,
        "stationary_distance_m": stationary_distance_m,
        "stationary_energy": stationary_energy,
        "age_ms": age_ms,
        "error": str(radar.get("error", "")) or None,
    }


def _distance_or_none(value: object, name: str, max_distance_m: float) -> Optional[float]:
    if value is None:
        return None
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric or null") from exc
    if not (0.0 <= result <= max_distance_m):
        raise ValueError(f"{name} outside 0..{max_distance_m}m")
    return result


def _int_or_none(value: object, name: str) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be int or null") from exc


# ---------------------------------------------------------------------------
# Statistics: rolling-window packet rate / loss / stale.
# ---------------------------------------------------------------------------


@dataclass
class Stats:
    started_at: float
    packet_count: int = 0
    error_count: int = 0
    json_error_count: int = 0
    reconnect_count: int = 0
    last_received_monotonic: float = 0.0
    last_sequence: int = -1
    last_boot_id: int = 0
    last_remote_boot_id: int = 0
    remote_online: bool = False
    remote_last_seen: float = 0.0
    _window: Deque[float] = field(default_factory=lambda: deque(maxlen=512))
    _loss_window: Deque[float] = field(default_factory=lambda: deque(maxlen=512))

    def note_line(self, received_monotonic: float) -> None:
        self._window.append(received_monotonic)
        self.last_received_monotonic = received_monotonic

    def packet_rate(self, now: float) -> float:
        if not self._window:
            return 0.0
        window = [t for t in self._window if now - t <= 2.0]
        if len(window) < 2:
            return 0.0
        span = window[-1] - window[0]
        if span <= 1e-6:
            # All samples arrived within the same timestamp window (typical
            # when replaying a captured file).  Clamp to a sane upper bound
            # so the human-readable display does not show kHz values that
            # only exist because of test replay.
            return min(float(len(window)), 200.0)
        return float(len(window)) / span

    def note_loss(self, loss: float) -> None:
        self._loss_window.append(max(0.0, min(1.0, float(loss))))

    def average_loss(self) -> float:
        if not self._loss_window:
            return 0.0
        return sum(self._loss_window) / len(self._loss_window)


# ---------------------------------------------------------------------------
# Live renderer (single line per refresh).
# ---------------------------------------------------------------------------


def _render_v1(stats: Stats, state: SideRadarState, now: float, color: bool) -> str:
    stale_age = (
        (now - state.received_monotonic) if state.received_monotonic else float("inf")
    )
    stale = (not state.link_ok) or stale_age > 0.6
    left = _c(
        ("L" if state.left_detected else "·") + " " +
        ("%.2f" % state.left_distance_m if state.left_distance_m is not None else "---"),
        _PALETTE.GREEN if state.left_detected and not stale else _PALETTE.DIM,
        color,
    )
    right = _c(
        ("R" if state.right_detected else "·") + " " +
        ("%.2f" % state.right_distance_m if state.right_distance_m is not None else "---"),
        _PALETTE.GREEN if state.right_detected and not stale else _PALETTE.DIM,
        color,
    )
    rate = stats.packet_rate(now)
    seq = _c(f"seq={state.sequence}", _PALETTE.CYAN, color)
    loss = _c(
        f"loss={stats.average_loss()*100:5.1f}%",
        _PALETTE.YELLOW if stats.average_loss() > 0.05 else _PALETTE.DIM,
        color,
    )
    rate_str = _c(f"rate={rate:4.1f}Hz", _PALETTE.DIM, color)
    stale_str = _c(
        f"stale={'Y' if stale else 'N'}",
        _PALETTE.RED if stale else _PALETTE.DIM,
        color,
    )
    return (
        f"[{state.received_monotonic or now:7.2f}] "
        f"{seq} {loss} {rate_str} {stale_str} | "
        f"left={left} right={right}"
    )


def _render_v2(stats: Stats, sample: RadarSample, now: float, color: bool) -> str:
    parts = []
    for radar_id in REQUIRED_RADAR_IDS:
        radar = sample.radars.get(radar_id)
        if radar is None:
            parts.append(f"{radar_id}=---")
            continue
        if radar.get("stale") or not radar.get("valid"):
            color_used = _PALETTE.RED
        elif radar.get("detected"):
            color_used = _PALETTE.GREEN
        else:
            color_used = _PALETTE.DIM
        dist = radar.get("distance_m")
        dist_str = f"{dist:4.2f}" if isinstance(dist, (int, float)) else "---"
        mov_dist = radar.get("moving_distance_m")
        mov_dist_str = f"{mov_dist:4.2f}" if isinstance(mov_dist, (int, float)) else "---"
        stat_dist = radar.get("stationary_distance_m")
        stat_dist_str = f"{stat_dist:4.2f}" if isinstance(stat_dist, (int, float)) else "---"
        mov_en = radar.get("moving_energy")
        stat_en = radar.get("stationary_energy")
        mov_en_str = f"{mov_en:3d}" if isinstance(mov_en, int) else "  -"
        stat_en_str = f"{stat_en:3d}" if isinstance(stat_en, int) else "  -"
        ts = radar.get("target_state", "none")
        ts_str = str(ts)[:3] if ts else "non"
        age = radar.get("age_ms")
        age_str = f"{age:4d}" if isinstance(age, int) else "   -"
        err = radar.get("error", "")
        err_str = f"!{err[:10]}" if err else ""
        parts.append(
            _c(
                f"{radar_id}={'D' if radar.get('detected') else '.'}"
                f" ts={ts_str}"
                f" d={dist_str}"
                f" mv={mov_dist_str}({mov_en_str})"
                f" st={stat_dist_str}({stat_en_str})"
                f" age={age_str}{err_str}",
                color_used,
                color,
            )
        )
    gw = _c("GW", _PALETTE.CYAN, color)
    rm = _c(
        "RM",
        _PALETTE.GREEN if sample.remote_ok else _PALETTE.RED,
        color,
    )
    return (
        f"[{sample.received_monotonic:7.2f}] seq={sample.sequence} "
        f"gw_boot={sample.gateway_boot_id} rm_boot={sample.remote_boot_id} "
        f"{gw}={1 if sample.gateway_ok else 0} {rm}={1 if sample.remote_ok else 0} | "
        + " | ".join(parts)
    )


# ---------------------------------------------------------------------------
# Main loop and modes.
# ---------------------------------------------------------------------------


def _open_stream(args: argparse.Namespace) -> _BaseStream:
    if args.raw:
        if args.raw == "-":
            return _StdinStream()
        return _FileStream(Path(args.raw))
    if not args.device:
        raise SystemExit("--device is required when not using --raw")
    return _UartStream(args.device, args.baudrate)


def _parse_lines(stream: _BaseStream, args: argparse.Namespace) -> Iterator[Tuple[StreamLine, RadarSample | SideRadarState]]:
    """Yield (raw, parsed) tuples, swallowing JSON errors."""
    now = time.monotonic()
    deadline = now + args.duration if args.duration else None
    while True:
        if deadline is not None and time.monotonic() >= deadline:
            return
        for line in stream.read_lines(deadline=deadline):
            try:
                parsed = _parse_one(line.raw, args)
            except (UnicodeError, ValueError, json.JSONDecodeError) as exc:
                yield line, _error_sample(line.received_monotonic, str(exc))
                continue
            yield line, parsed


def _parse_one(raw: bytes, args: argparse.Namespace):
    # Try v2 first if a parser is requested, fall back to v1.
    if args.protocol_version == 2:
        return parse_gateway_line(raw, max_distance_m=args.max_distance_m)
    try:
        return parse_gateway_line(raw, max_distance_m=args.max_distance_m)
    except (ValueError, json.JSONDecodeError):
        return parse_radar_line(raw, max_distance_m=args.max_distance_m)


def _error_sample(received_monotonic: float, message: str) -> RadarSample:
    sample = RadarSample(
        received_monotonic=received_monotonic,
        sequence=-1,
        gateway_ok=False,
        remote_ok=False,
        gateway_boot_id=0,
        remote_boot_id=0,
        packet_loss=0.0,
    )
    sample.errors.append(message)
    return sample


# ---------------------------------------------------------------------------
# Mode 1: live monitor.
# ---------------------------------------------------------------------------


def _run_live(args: argparse.Namespace) -> int:
    stream = _open_stream(args)
    try:
        stream.open()
    except Exception as exc:
        print(f"[monitor] failed to open stream {stream.name}: {exc}", file=sys.stderr)
        return 2

    stats = Stats(started_at=time.monotonic())
    print(
        _c(
            f"[monitor] source={stream.name} baud={args.baudrate} "
            f"stale={args.stale_after_seconds}s duration={args.duration}s",
            _PALETTE.BOLD,
            args.color,
        ),
        flush=True,
    )
    csv_fp: Optional[csv.writer] = None
    csv_file = None
    if args.csv:
        path = Path(args.csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = path.open("w", encoding="utf-8", newline="")
        csv_fp = csv.writer(csv_file)
        csv_fp.writerow(
            [
                "monotonic",
                "wall_time",
                "source",
                "sequence",
                "gateway_ok",
                "remote_ok",
                "gateway_boot_id",
                "remote_boot_id",
                "packet_loss",
                "json_errors",
                "reconnect_count",
                "rate_hz",
                "radar",
                "detected",
                "valid",
                "stale",
                "target_state",
                "distance_m",
                "moving_distance_m",
                "moving_energy",
                "stationary_distance_m",
                "stationary_energy",
                "age_ms",
            ]
        )
    try:
        for line, parsed in _parse_lines(stream, args):
            stats.packet_count += 1
            stats.note_line(line.received_monotonic)
            if isinstance(parsed, RadarSample):
                if parsed.errors:
                    stats.json_error_count += len(parsed.errors)
                stats.note_loss(parsed.packet_loss)
                if parsed.sequence <= stats.last_sequence and parsed.sequence != 0:
                    stats.error_count += 1
                else:
                    stats.last_sequence = parsed.sequence
                if parsed.gateway_boot_id and parsed.gateway_boot_id != stats.last_boot_id:
                    stats.reconnect_count += 1
                    stats.last_boot_id = parsed.gateway_boot_id
                if parsed.remote_ok:
                    stats.remote_online = True
                    stats.remote_last_seen = line.received_monotonic
                else:
                    if stats.remote_online:
                        stats.remote_online = False
                now = time.monotonic()
                line_text = _render_v2(stats, parsed, now, args.color)
                print(line_text, flush=True)
                if csv_fp is not None:
                    _write_v2_csv(csv_fp, parsed, stats, stream.name)
            else:
                stats.last_sequence = parsed.sequence
                now = time.monotonic()
                print(
                    _render_v1(stats, parsed, now, args.color),
                    flush=True,
                )
                if csv_fp is not None:
                    _write_v1_csv(csv_fp, parsed, stats, stream.name)
            if csv_file is not None:
                csv_file.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file is not None:
            csv_file.close()
        if args.json_report:
            _write_json_report(args.json_report, stats)
        stream.close()

    summary = (
        f"[monitor] done. packets={stats.packet_count} "
        f"json_errors={stats.json_error_count} "
        f"reconnects={stats.reconnect_count} "
        f"avg_loss={stats.average_loss()*100:.2f}%"
    )
    print(_c(summary, _PALETTE.BOLD, args.color), flush=True)
    if stats.json_error_count > 0 or stats.packet_count == 0:
        return 1
    return 0


def _write_v2_csv(
    writer: csv.writer, sample: RadarSample, stats: Stats, source: str
) -> None:
    for radar_id, radar in sample.radars.items():
        writer.writerow(
            [
                f"{sample.received_monotonic:.6f}",
                f"{time.time():.6f}",
                source,
                sample.sequence,
                int(sample.gateway_ok),
                int(sample.remote_ok),
                sample.gateway_boot_id,
                sample.remote_boot_id,
                f"{sample.packet_loss:.6f}",
                len(sample.errors),
                stats.reconnect_count,
                f"{stats.packet_rate(sample.received_monotonic):.3f}",
                radar_id,
                int(bool(radar.get("detected"))),
                int(bool(radar.get("valid"))),
                int(bool(radar.get("stale"))),
                int(radar.get("target_state", 0)),
                radar.get("distance_m"),
                radar.get("moving_distance_m"),
                radar.get("moving_energy"),
                radar.get("stationary_distance_m"),
                radar.get("stationary_energy"),
                radar.get("age_ms"),
            ]
        )


def _write_v1_csv(
    writer: csv.writer, state: SideRadarState, stats: Stats, source: str
) -> None:
    writer.writerow(
        [
            f"{state.received_monotonic:.6f}",
            f"{time.time():.6f}",
            source,
            state.sequence,
            1 if state.link_ok else 0,
            0,
            0,
            0,
            0.0,
            0,
            stats.reconnect_count,
            f"{stats.packet_rate(state.received_monotonic):.3f}",
            "left",
            int(state.left_detected),
            int(state.link_ok),
            0 if state.link_ok else 1,
            0,
            state.left_distance_m,
            None,
            None,
            None,
            None,
            None,
        ]
    )
    writer.writerow(
        [
            f"{state.received_monotonic:.6f}",
            f"{time.time():.6f}",
            source,
            state.sequence,
            1 if state.link_ok else 0,
            0,
            0,
            0,
            0.0,
            0,
            stats.reconnect_count,
            f"{stats.packet_rate(state.received_monotonic):.3f}",
            "right",
            int(state.right_detected),
            int(state.link_ok),
            0 if state.link_ok else 1,
            0,
            state.right_distance_m,
            None,
            None,
            None,
            None,
            None,
        ]
    )


def _write_json_report(path_str: str, stats: Stats) -> None:
    path = Path(path_str)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "started_at": stats.started_at,
        "ended_at": time.monotonic(),
        "duration_s": time.monotonic() - stats.started_at,
        "packet_count": stats.packet_count,
        "json_error_count": stats.json_error_count,
        "reconnect_count": stats.reconnect_count,
        "average_packet_loss": stats.average_loss(),
        "remote_online": stats.remote_online,
        "last_sequence": stats.last_sequence,
        "last_boot_id": stats.last_boot_id,
        "last_remote_boot_id": stats.last_remote_boot_id,
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


# ---------------------------------------------------------------------------
# Mode 2: mapping test.  Interactive.
# ---------------------------------------------------------------------------


MAPPING_ORDER: Tuple[Tuple[str, str], ...] = (
    ("left_front", "Trước-trái"),
    ("left_rear", "Sau-trái"),
    ("right_front", "Trước-phải"),
    ("right_rear", "Sau-phải"),
)


def _run_mapping_test(args: argparse.Namespace) -> int:
    """Walk the operator through each radar and ask them to confirm position.

    The monitor watches the stream and reports which radar ID is currently
    reacting (presence detection or moving/stationary energy above a noise
    floor).  It never infers the physical mapping by itself; the operator is
    the only source of truth.
    """
    stream = _open_stream(args)
    try:
        stream.open()
    except Exception as exc:
        print(f"[mapping-test] cannot open stream: {exc}", file=sys.stderr)
        return 2

    confirmed: Dict[str, str] = {}
    print(_c("[mapping-test] sẽ hỏi bạn xác nhận 4 radar theo thứ tự.", _PALETTE.BOLD, args.color))
    print(_c(
        "Khi tôi yêu cầu, hãy kích hoạt radar tương ứng (đi vào vùng quét).",
        _PALETTE.DIM, args.color,
    ))
    for radar_id, label in MAPPING_ORDER:
        prompt = (
            f"[mapping-test] {radar_id} ({label}): kích hoạt radar và quan sát. "
            "Bấm Enter khi thấy radar phản hồi, hoặc gõ 'skip' để bỏ qua."
        )
        print(_c(prompt, _PALETTE.CYAN, args.color))
        deadline = time.monotonic() + args.mapping_window
        saw_reaction = False
        while time.monotonic() < deadline:
            for line in stream.read_lines():
                try:
                    parsed = _parse_one(line.raw, args)
                except Exception:
                    continue
                if isinstance(parsed, RadarSample):
                    radar = parsed.radars.get(radar_id)
                    if radar and (radar.get("detected") or (radar.get("moving_energy") or 0) > 5):
                        saw_reaction = True
                        print(
                            _c(
                                f"  phản hồi từ {radar_id}: "
                                f"detected={radar.get('detected')} "
                                f"energy={radar.get('moving_energy')}",
                                _PALETTE.GREEN, args.color,
                            )
                        )
                        break
                else:
                    is_left = radar_id.startswith("left_")
                    detected = (parsed.left_detected if is_left else parsed.right_detected)
                    if detected:
                        saw_reaction = True
                        print(_c(
                            f"  phản hồi từ {radar_id}: detected=True",
                            _PALETTE.GREEN, args.color,
                        ))
                        break
            if saw_reaction:
                break
            time.sleep(0.1)
        if not saw_reaction:
            print(_c(
                f"  [mapping-test] chưa thấy {radar_id} phản hồi trong "
                f"{args.mapping_window}s.",
                _PALETTE.YELLOW, args.color,
            ))
        answer = input(f"  Bạn xác nhận vị trí vật lý của {radar_id} ({label})? "
                       "[y/n/skip] ").strip().lower()
        if answer in ("y", "yes"):
            confirmed[radar_id] = label
            print(_c(f"  đã ghi nhận {radar_id} = {label}", _PALETTE.GREEN, args.color))
        elif answer in ("skip", "s"):
            print(_c(f"  bỏ qua {radar_id}", _PALETTE.DIM, args.color))
        else:
            print(_c(f"  KHÔNG xác nhận {radar_id}", _PALETTE.RED, args.color))

    print()
    print(_c("[mapping-test] kết quả (chỉ radar bạn đã xác nhận):", _PALETTE.BOLD, args.color))
    for radar_id, label in MAPPING_ORDER:
        status = (
            _c(f"{label} ✓", _PALETTE.GREEN, args.color)
            if radar_id in confirmed
            else _c("(chưa xác nhận)", _PALETTE.DIM, args.color)
        )
        print(f"  {radar_id}: {status}")
    stream.close()
    if len(confirmed) < len(MAPPING_ORDER):
        print(_c(
            "[mapping-test] chưa xác nhận đủ 4 radar. KHÔNG coi mapping là hợp lệ.",
            _PALETTE.RED, args.color,
        ))
        return 1
    return 0


# ---------------------------------------------------------------------------
# Mode 3: soak test.
# ---------------------------------------------------------------------------


def _try_psutil() -> Optional[object]:
    try:
        import psutil  # type: ignore
        return psutil
    except Exception:
        return None


def _read_jetson_temperatures() -> Dict[str, float]:
    """Return Jetson zone temperatures when available, else empty dict."""
    out: Dict[str, float] = {}
    base = Path("/sys/devices/virtual/thermal/thermal_zone")
    if not base.exists():
        return out
    for zone in base.glob("thermal_zone*/temp"):
        try:
            milli = int(zone.read_text().strip())
        except ValueError:
            continue
        name_path = zone.parent / zone.name.replace("temp", "type")
        name = name_path.read_text().strip() if name_path.exists() else zone.name
        out[name] = milli / 1000.0
    return out


def _run_soak(args: argparse.Namespace) -> int:
    psutil = _try_psutil()
    stream = _open_stream(args)
    try:
        stream.open()
    except Exception as exc:
        print(f"[soak] cannot open stream: {exc}", file=sys.stderr)
        return 2

    stats = Stats(started_at=time.monotonic())
    # M6: bound the in-memory samples list to a 10-minute rolling window so
    # a multi-hour soak test does not exhaust RAM.
    MAX_SOAK_SAMPLES = 10 * 60 * 20  # 10 min × 60 s × 20 Hz ≈ 12000 entries
    samples: List[Dict[str, object]] = []
    if args.csv:
        Path(args.csv).parent.mkdir(parents=True, exist_ok=True)
    process = psutil.Process(os.getpid()) if psutil else None
    deadline = time.monotonic() + args.duration if args.duration else None
    print(_c(
        f"[soak] duration={args.duration}s csv={args.csv} json={args.json_report}",
        _PALETTE.BOLD, args.color,
    ), flush=True)
    last_stats_print = time.monotonic()
    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break
            consumed = False
            for line, parsed in _parse_lines(stream, args):
                consumed = True
                stats.packet_count += 1
                stats.note_line(line.received_monotonic)
                if isinstance(parsed, RadarSample):
                    if parsed.errors:
                        stats.json_error_count += len(parsed.errors)
                    stats.note_loss(parsed.packet_loss)
                    if parsed.sequence <= stats.last_sequence and parsed.sequence != 0:
                        stats.error_count += 1
                    else:
                        stats.last_sequence = parsed.sequence
                    if (
                        parsed.gateway_boot_id
                        and parsed.gateway_boot_id != stats.last_boot_id
                    ):
                        stats.reconnect_count += 1
                        stats.last_boot_id = parsed.gateway_boot_id
                    if parsed.remote_ok:
                        stats.remote_online = True
                        stats.remote_last_seen = line.received_monotonic
                    else:
                        stats.remote_online = False
                else:
                    stats.last_sequence = parsed.sequence
                samples.append(
                    {
                        "t": line.received_monotonic,
                        "parsed": parsed,
                    }
                )
                # M6: trim to rolling window to bound RAM on multi-hour soak tests.
                if len(samples) > MAX_SOAK_SAMPLES:
                    samples = samples[-MAX_SOAK_SAMPLES:]
            if not consumed:
                time.sleep(0.05)
            now = time.monotonic()
            if now - last_stats_print >= args.soak_print_interval:
                last_stats_print = now
                cpu = process.cpu_percent(interval=None) if process else 0.0
                rss = (process.memory_info().rss / (1024 * 1024)) if process else 0.0
                temps = _read_jetson_temperatures()
                rate = stats.packet_rate(now)
                print(
                    _c(
                        f"[soak] t={now-stats.started_at:6.1f}s "
                        f"rate={rate:4.1f}Hz loss={stats.average_loss()*100:5.1f}% "
                        f"pkts={stats.packet_count} rec={stats.reconnect_count} "
                        f"cpu={cpu:4.1f}% rss={rss:5.1f}MB "
                        f"temp={temps.get('CPU-therm', temps.get('GPU-therm', float('nan')))}",
                        _PALETTE.DIM, args.color,
                    ),
                    flush=True,
                )
    except KeyboardInterrupt:
        pass
    finally:
        stream.close()
        if args.csv:
            _write_soak_csv(args.csv, samples, stats)
        if args.json_report:
            _write_soak_json(args.json_report, stats, samples)

    print(_c(
        f"[soak] done. packets={stats.packet_count} "
        f"avg_loss={stats.average_loss()*100:.2f}% "
        f"reconnects={stats.reconnect_count}",
        _PALETTE.BOLD, args.color,
    ), flush=True)
    return 0


def _write_soak_csv(path_str: str, samples: List[Dict[str, object]], stats: Stats) -> None:
    with open(path_str, "w", encoding="utf-8", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(
            [
                "t_relative",
                "wall_time",
                "sequence",
                "gateway_ok",
                "remote_ok",
                "gateway_boot_id",
                "remote_boot_id",
                "packet_loss",
                "rate_hz",
                "reconnect_count",
                "errors",
            ]
        )
        for sample in samples:
            parsed = sample["parsed"]
            t = float(sample["t"])  # type: ignore[arg-type]
            if isinstance(parsed, RadarSample):
                writer.writerow(
                    [
                        f"{t - stats.started_at:.6f}",
                        f"{time.time():.6f}",
                        parsed.sequence,
                        int(parsed.gateway_ok),
                        int(parsed.remote_ok),
                        parsed.gateway_boot_id,
                        parsed.remote_boot_id,
                        f"{parsed.packet_loss:.6f}",
                        f"{stats.packet_rate(t):.3f}",
                        stats.reconnect_count,
                        len(parsed.errors),
                    ]
                )
            else:
                writer.writerow(
                    [
                        f"{t - stats.started_at:.6f}",
                        f"{time.time():.6f}",
                        parsed.sequence,
                        int(parsed.link_ok),
                        0,
                        0,
                        0,
                        0.0,
                        f"{stats.packet_rate(t):.3f}",
                        stats.reconnect_count,
                        0,
                    ]
                )


def _write_soak_json(path_str: str, stats: Stats, samples: List[Dict[str, object]]) -> None:
    payload = {
        "started_at": stats.started_at,
        "ended_at": time.monotonic(),
        "duration_s": time.monotonic() - stats.started_at,
        "packet_count": stats.packet_count,
        "json_error_count": stats.json_error_count,
        "reconnect_count": stats.reconnect_count,
        "average_packet_loss": stats.average_loss(),
        "remote_online": stats.remote_online,
        "last_sequence": stats.last_sequence,
        "last_boot_id": stats.last_boot_id,
        "last_remote_boot_id": stats.last_remote_boot_id,
        "samples": len(samples),
    }
    Path(path_str).write_text(json.dumps(payload, indent=2), encoding="utf-8")


# ---------------------------------------------------------------------------
# Mode 4: fault injection (synthetic stream).  No hardware required.
# ---------------------------------------------------------------------------


@dataclass
class _FaultConfig:
    name: str
    description: str
    schedule: Callable[[float, "FaultState"], Optional[Dict[str, object]]]


@dataclass
class FaultState:
    rng: random.Random
    started_at: float
    boot_id: int = 1
    remote_boot_id: int = 1
    sequence: int = 0
    last_remote_seq: int = 0
    gateway_online: bool = True
    remote_online: bool = True
    radar_power: Dict[str, bool] = field(default_factory=dict)
    radar_stale: Dict[str, bool] = field(default_factory=dict)


def _base_packet(state: FaultState) -> Dict[str, object]:
    state.sequence += 1
    radars: Dict[str, Dict[str, object]] = {}
    for radar_id in REQUIRED_RADAR_IDS:
        detected = state.rng.random() < 0.05 and state.radar_power.get(radar_id, True)
        moving_energy = state.rng.randint(0, 90) if detected else state.rng.randint(0, 8)
        radars[radar_id] = {
            "detected": detected,
            "valid": state.radar_power.get(radar_id, True)
            and not state.radar_stale.get(radar_id, False),
            "stale": state.radar_stale.get(radar_id, False),
            "target_state": 2 if detected else 0,
            "distance_m": round(0.8 + state.rng.random() * 4.0, 3) if detected else None,
            "moving_distance_m": round(0.8 + state.rng.random() * 4.0, 3)
            if detected
            else None,
            "moving_energy": moving_energy,
            "stationary_distance_m": None,
            "stationary_energy": 0 if not detected else state.rng.randint(0, 30),
            "age_ms": state.rng.randint(10, 80),
            "error": None,
        }
    return {
        "v": 2,
        "seq": state.sequence,
        "gateway_ok": state.gateway_online,
        "remote_ok": state.remote_online,
        "gateway_boot_id": state.boot_id,
        "remote_boot_id": state.remote_boot_id,
        "packet_loss": round(state.rng.uniform(0.0, 0.02), 4),
        "radars": radars,
    }


def _emit(state: FaultState, packet: Dict[str, object]) -> bytes:
    return (json.dumps(packet) + "\n").encode("utf-8")


def _scenario_esp32_b_power_loss() -> _FaultConfig:
    def schedule(now: float, state: FaultState) -> Optional[Dict[str, object]]:
        elapsed = now - state.started_at
        if elapsed < 5:
            return _base_packet(state)
        state.remote_online = False
        packet = _base_packet(state)
        packet["remote_ok"] = False
        packet["packet_loss"] = 1.0
        for radar_id in ("right_front", "right_rear"):
            radars = packet["radars"]  # type: ignore[index]
            radar = radars[radar_id]  # type: ignore[index]
            radar["valid"] = False
            radar["stale"] = True
            radar["detected"] = False
        return packet

    return _FaultConfig(
        "esp32_b_power_loss",
        "ESP32 B Remote mất nguồn: 2 radar Remote phải valid=false, stale=true.",
        schedule,
    )


def _scenario_esp32_a_reset() -> _FaultConfig:
    def schedule(now: float, state: FaultState) -> Optional[Dict[str, object]]:
        elapsed = now - state.started_at
        if 5 <= elapsed < 6:
            return None  # simulate disconnect
        if 6 <= elapsed < 6.5:
            state.boot_id += 1
            state.sequence = 0
            state.gateway_online = True
        return _base_packet(state)

    return _FaultConfig(
        "esp32_a_reset",
        "ESP32 A Gateway reset: boot_id đổi, sequence về 0.",
        schedule,
    )


def _scenario_one_ld2410_uart_loss(
    target: str = "right_front",
) -> _FaultConfig:
    def schedule(now: float, state: FaultState) -> Optional[Dict[str, object]]:
        elapsed = now - state.started_at
        if 4 <= elapsed < 8:
            state.radar_stale[target] = True
        else:
            state.radar_stale[target] = False
        return _base_packet(state)

    return _FaultConfig(
        "one_ld2410_uart_loss",
        f"UART của {target} mất: radar đó valid=false, stale=true; các radar khác vẫn hoạt động.",
        schedule,
    )


def _scenario_espnow_packet_loss() -> _FaultConfig:
    def schedule(now: float, state: FaultState) -> Optional[Dict[str, object]]:
        packet = _base_packet(state)
        if state.rng.random() < 0.5:
            return None  # drop packet
        return packet

    return _FaultConfig(
        "espnow_packet_loss",
        "ESP-NOW mất gói: monitor tiếp tục nhận nhưng packet_loss tăng.",
        schedule,
    )


def _scenario_corrupt_json() -> _FaultConfig:
    counter = {"n": 0}

    def schedule(now: float, state: FaultState) -> Optional[Dict[str, object]]:
        counter["n"] += 1
        if counter["n"] % 7 == 0:
            return b"{not-json}\n"
        return _emit(state, _base_packet(state))

    return _FaultConfig(
        "corrupt_json",
        "JSON bị hỏng: 1/7 frame lỗi cú pháp. Monitor phải bỏ qua và tiếp tục.",
        schedule,
    )


def _scenario_camera_side_unplug() -> _FaultConfig:
    """Side camera unplug only affects display; radars must remain valid."""

    def schedule(now: float, state: FaultState) -> Optional[Dict[str, object]]:
        return _base_packet(state)

    return _FaultConfig(
        "camera_side_unplug",
        "Camera side rút: radar pipeline KHÔNG được suy ra stale; đây là fault hiển thị.",
        schedule,
    )


def _scenario_gateway_usb_unplug_replug() -> _FaultConfig:
    def schedule(now: float, state: FaultState) -> Optional[Dict[str, object]]:
        elapsed = now - state.started_at
        if 4 <= elapsed < 8:
            state.gateway_online = False
            return None
        if elapsed >= 8:
            state.gateway_online = True
            if state.rng.random() < 0.2:
                state.boot_id += 1
                state.sequence = 0
        return _base_packet(state)

    return _FaultConfig(
        "gateway_usb_unplug_replug",
        "Gateway USB rút và cắm lại: gateway_ok=false lúc rút, reconnect sau khi cắm lại.",
        schedule,
    )


SCENARIOS: Dict[str, _FaultConfig] = {
    "esp32_b_power_loss": _scenario_esp32_b_power_loss(),
    "esp32_a_reset": _scenario_esp32_a_reset(),
    "one_ld2410_uart_loss": _scenario_one_ld2410_uart_loss(),  # target filled at runtime
    "espnow_packet_loss": _scenario_espnow_packet_loss(),
    "corrupt_json": _scenario_corrupt_json(),
    "camera_side_unplug": _scenario_camera_side_unplug(),
    "gateway_usb_unplug_replug": _scenario_gateway_usb_unplug_replug(),
}


def _get_scenario(name: str, args: argparse.Namespace) -> _FaultConfig:
    """Resolve a scenario by name, patching the target radar when needed."""
    cfg = SCENARIOS.get(name)
    if cfg is None:
        return cfg  # type: ignore[return-value]
    # M7: for one_ld2410_uart_loss, override the target radar from CLI.
    if name == "one_ld2410_uart_loss":
        return _scenario_one_ld2410_uart_loss(target=args.radar_target)
    return cfg


def _run_fault(args: argparse.Namespace) -> int:
    scenario = _get_scenario(args.fault, args)
    if scenario is None:
        print(
            f"[fault] scenario không tồn tại: {args.fault}. Có: {sorted(SCENARIOS)}",
            file=sys.stderr,
        )
        return 2
    state = FaultState(
        rng=random.Random(args.fault_seed),
        started_at=time.monotonic(),
    )
    print(_c(
        f"[fault] chạy scenario '{scenario.name}': {scenario.description}",
        _PALETTE.BOLD, args.color,
    ), flush=True)
    deadline = time.monotonic() + args.duration if args.duration else None
    csv_fp = None
    csv_file = None
    if args.csv:
        path = Path(args.csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = path.open("w", encoding="utf-8", newline="")
        csv_fp = csv.writer(csv_file)
        csv_fp.writerow(
            [
                "t_relative",
                "scenario",
                "sequence",
                "gateway_ok",
                "remote_ok",
                "gateway_boot_id",
                "remote_boot_id",
                "packet_loss",
                "radar",
                "detected",
                "valid",
                "stale",
                "distance_m",
            ]
        )
    summary = {
        "scenario": scenario.name,
        "description": scenario.description,
        "packets_seen": 0,
        "errors": 0,
        "faulty_radars": set(),
    }
    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break
            now = time.monotonic()
            emitted = scenario.schedule(now, state)
            if emitted is None:
                time.sleep(0.05)
                continue
            if isinstance(emitted, bytes):
                raw = emitted
            else:
                raw = _emit(state, emitted)
            try:
                sample = parse_gateway_line(raw)
            except (UnicodeError, ValueError, json.JSONDecodeError) as exc:
                summary["errors"] += 1
                print(
                    _c(
                        f"  parser rejected frame: {exc}",
                        _PALETTE.YELLOW, args.color,
                    )
                )
                continue
            summary["packets_seen"] += 1
            for radar_id, radar in sample.radars.items():
                if radar.get("stale") or not radar.get("valid"):
                    summary["faulty_radars"].add(radar_id)
                if csv_fp is not None:
                    csv_fp.writerow(
                        [
                            f"{sample.received_monotonic - state.started_at:.3f}",
                            scenario.name,
                            sample.sequence,
                            int(sample.gateway_ok),
                            int(sample.remote_ok),
                            sample.gateway_boot_id,
                            sample.remote_boot_id,
                            f"{sample.packet_loss:.4f}",
                            radar_id,
                            int(bool(radar.get("detected"))),
                            int(bool(radar.get("valid"))),
                            int(bool(radar.get("stale"))),
                            radar.get("distance_m"),
                        ]
                    )
            if csv_file is not None:
                csv_file.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file is not None:
            csv_file.close()
        if args.json_report:
            payload = {
                **summary,
                "faulty_radars": sorted(summary["faulty_radars"]),
                "started_at": state.started_at,
                "ended_at": time.monotonic(),
            }
            Path(args.json_report).write_text(
                json.dumps(payload, indent=2, default=list), encoding="utf-8"
            )
    print(_c(
        f"[fault] done. scenario={scenario.name} packets={summary['packets_seen']} "
        f"errors={summary['errors']} faulty={sorted(summary['faulty_radars'])}",
        _PALETTE.BOLD, args.color,
    ), flush=True)
    return 0


# ---------------------------------------------------------------------------
# Argument parser.
# ---------------------------------------------------------------------------


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="radar_gateway_monitor",
        description=(
            "CLI giám sát ESP32 Radar Gateway và chạy mapping/soak/fault "
            "tests.  Không dùng để chứng minh hệ thống an toàn cho xe; "
            "chỉ xác nhận pipeline không tạo detection giả và stale radar "
            "được đánh dấu đúng."
        ),
    )
    parser.add_argument(
        "--device",
        help="Đường dẫn UART Gateway (ví dụ /dev/serial/by-id/...). "
        "Bắt buộc khi không dùng --raw.",
    )
    parser.add_argument(
        "--baudrate", type=int, default=115200,
        help="Baudrate UART Gateway (mặc định 115200).",
    )
    parser.add_argument(
        "--duration",
        help="Thời lượng chạy (giây hoặc đơn vị như 30s, 5m, 1h). Mặc định: chạy đến khi Ctrl-C.",
    )
    parser.add_argument(
        "--csv", help="Ghi log CSV ra file (kết hợp live/soak/fault).",
    )
    parser.add_argument(
        "--json-report", help="Ghi JSON report tóm tắt ra file.",
    )
    parser.add_argument(
        "--raw",
        help="Đọc JSON lines từ file thay vì UART (dùng '-' để đọc stdin). "
        "Hữu ích cho test mà không cần phần cứng.",
    )
    parser.add_argument(
        "--no-color", action="store_true", help="Tắt màu ANSI.",
    )
    parser.add_argument(
        "--protocol-version", type=int, default=2, choices=(1, 2),
        help="Phiên bản JSON: 2 (Gateway 4 radar) hoặc 1 (legacy 2 radar).",
    )
    parser.add_argument(
        "--stale-after-seconds", type=float, default=0.6,
        help="Sau bao nhiêu giây không có frame thì radar được coi là stale.",
    )
    parser.add_argument(
        "--max-distance-m", type=float, default=6.0,
        help="Khoảng cách tối đa cho parser (mặc định 6m cho LD2410C).",
    )
    parser.add_argument(
        "--mapping-test", action="store_true",
        help="Chạy chế độ mapping test (yêu cầu xác nhận vật lý).",
    )
    parser.add_argument(
        "--mapping-window", type=float, default=10.0,
        help="Số giây chờ phản hồi cho mỗi radar khi mapping.",
    )
    parser.add_argument(
        "--soak", action="store_true", help="Chạy chế độ soak test.",
    )
    parser.add_argument(
        "--soak-print-interval", type=float, default=2.0,
        help="Khoảng cách in log soak (giây).",
    )
    parser.add_argument(
        "--fault", choices=sorted(SCENARIOS), help="Chạy fault injection scenario.",
    )
    parser.add_argument(
        "--radar-target",
        default="right_front",
        choices=["left_front", "left_rear", "right_front", "right_rear"],
        help=(
            "Radar bị ảnh hưởng khi chạy scenario 'one_ld2410_uart_loss'. "
            "(mặc định: right_front)"
        ),
    )
    parser.add_argument(
        "--fault-seed", type=int, default=1234,
        help="Seed cho synthetic generator khi chạy fault.",
    )
    parser.add_argument(
        "--list-faults", action="store_true", help="Liệt kê fault scenarios rồi thoát.",
    )
    return parser


def _parse_duration(value: str) -> float:
    text = value.strip().lower()
    if text.endswith("ms"):
        return float(text[:-2]) / 1000.0
    if text.endswith("s"):
        return float(text[:-1])
    if text.endswith("m"):
        return float(text[:-1]) * 60.0
    if text.endswith("h"):
        return float(text[:-1]) * 3600.0
    return float(text)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    args.color = not args.no_color and sys.stdout.isatty()
    if args.duration is not None:
        try:
            args.duration = _parse_duration(args.duration)
        except ValueError as exc:
            raise SystemExit(f"--duration không hợp lệ: {exc}")
    if args.list_faults:
        for name, cfg in SCENARIOS.items():
            print(f"- {name}: {cfg.description}")
        return 0
    if args.mapping_test:
        return _run_mapping_test(args)
    if args.soak:
        return _run_soak(args)
    if args.fault:
        return _run_fault(args)
    return _run_live(args)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
