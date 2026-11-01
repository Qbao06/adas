"""POSIX UART reader for the JSON v2 stream emitted by the Gateway.

This module is intentionally tiny.  It mirrors the wire-format contract
documented in :mod:`radar_gateway.jetson_protocol` and exposes a single
synchronous generator :func:`iter_json_lines` that yields one validated
JSON object per line.  The generator raises :class:`ValueError` (or
:class:`json.JSONDecodeError`) on bad frames so the caller can decide
whether to record the error and continue.

The full background reader used by the production pipeline lives in
``sensors/radar_gateway.py``; this module exists so that the CLI
monitor, soak and fault-injection tools (and the host test harness) can
all consume the same JSON-v2 stream with a single helper.
"""
from __future__ import annotations

import json
import os
import select
import time
from typing import Any, Callable, Iterable, Iterator, Optional

from .jetson_protocol import JSON_PROTOCOL_VERSION

# Cap a single line at 4096 bytes (matches ``MAX_LINE_BYTES`` in the
# pipeline).  Frames longer than this are dropped.
DEFAULT_MAX_LINE_BYTES: int = 4096


def _open_uart(device: str, baudrate: int) -> int:
    """Open a POSIX UART.  Caller is responsible for closing the fd."""
    import termios  # POSIX only.

    speed_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
        256000: getattr(termios, "B256000", termios.B115200),
    }
    speed = speed_map.get(int(baudrate))
    if speed is None:
        raise ValueError(f"Unsupported UART baudrate: {baudrate}")
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
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
    return fd


def iter_json_lines(
    source: Iterable[bytes] | str,
    *,
    timeout_s: float = 0.0,
    max_line_bytes: int = DEFAULT_MAX_LINE_BYTES,
    on_error: Optional[Callable[[bytes, BaseException], None]] = None,
) -> Iterator[tuple[float, dict[str, Any]]]:
    """Yield ``(received_monotonic, payload)`` for each valid JSON v2 line.

    ``source`` may be:

    * an iterable of bytes objects (used by tests with a captured stream),
    * the string ``"-"`` to read from ``sys.stdin.buffer``,
    * a ``/dev/serial/by-id/...`` path, in which case the UART is
      opened with the standard termios settings.

    A line is considered valid when it:

    1. fits within ``max_line_bytes``,
    2. decodes as UTF-8,
    3. parses as JSON,
    4. has ``v == JSON_PROTOCOL_VERSION``,
    5. has an integer ``seq >= 0``.

    Any failure is reported through ``on_error`` (if provided) and the
    next line is read; the reader never silently fabricates data.
    """
    import sys

    iterator: Iterator[bytes]
    if isinstance(source, str):
        if source == "-":
            iterator = iter(lambda: sys.stdin.buffer.read1(4096), b"")
        else:
            fd = _open_uart(source, 115200)
            try:
                buffer = bytearray()
                deadline = time.monotonic() + timeout_s if timeout_s else None
                while True:
                    if deadline is not None and time.monotonic() >= deadline:
                        return
                    ready, _, _ = select.select([fd], [], [], 0.2)
                    if not ready:
                        continue
                    chunk = os.read(fd, 512)
                    if not chunk:
                        continue
                    buffer.extend(chunk)
                    if len(buffer) > 8192:
                        buffer.clear()
                        continue
                    while b"\n" in buffer:
                        line, _, buffer = buffer.partition(b"\n")
                        if line.endswith(b"\r"):
                            line = line[:-1]
                        if not line.strip():
                            continue
                        yield from _decode_line(line, max_line_bytes, on_error)
            finally:
                try:
                    os.close(fd)
                except OSError:
                    pass
            return
    else:
        iterator = iter(source)

    buffer = bytearray()
    for chunk in iterator:
        if not chunk:
            continue
        buffer.extend(chunk)
        if len(buffer) > 8192:
            buffer.clear()
            continue
        while b"\n" in buffer:
            line, _, buffer = buffer.partition(b"\n")
            if line.endswith(b"\r"):
                line = line[:-1]
            if not line.strip():
                continue
            yield from _decode_line(line, max_line_bytes, on_error)


def _decode_line(
    line: bytes,
    max_line_bytes: int,
    on_error: Optional[Callable[[bytes, BaseException], None]],
) -> Iterator[tuple[float, dict[str, Any]]]:
    try:
        if len(line) > max_line_bytes:
            raise ValueError(f"frame too long ({len(line)} > {max_line_bytes})")
        text = line.decode("utf-8", errors="strict")
        payload = json.loads(text)
        if not isinstance(payload, dict):
            raise ValueError("payload is not a JSON object")
        version = payload.get("v")
        if version != JSON_PROTOCOL_VERSION:
            raise ValueError(f"unsupported protocol version {version}")
        seq = payload.get("seq")
        if not isinstance(seq, int) or seq < 0:
            raise ValueError("seq must be a non-negative integer")
    except (ValueError, json.JSONDecodeError, UnicodeDecodeError) as exc:
        if on_error is not None:
            try:
                on_error(line, exc)
            except Exception:
                pass
        return
    yield time.monotonic(), payload


__all__ = [
    "DEFAULT_MAX_LINE_BYTES",
    "iter_json_lines",
]
