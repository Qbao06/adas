"""Hardware probe helpers used by the dashboard endpoints.

These helpers open the real device in **live** mode and only do so when the
caller has passed the allowlist guard.  In **demo** mode the dashboard never
calls them (it returns 403 before reaching this code).

Two entry points are exposed:

* :func:`probe_gateway` — opens a UART, waits up to ``duration`` seconds for a
  valid JSON-v2 packet, then closes the device in a ``finally`` block.  When
  the caller supplies a :class:`FakeGatewaySource` (only used by tests), the
  UART is never touched and the supplied lines are parsed instead.
* :func:`probe_camera_managed` — opens a ``/dev/video*`` node with OpenCV and
  pulls one frame to prove the device is alive.

Both functions are defensive: they never execute shell commands, they always
clean up after themselves, and any failure is reported back as a structured
``{ok: bool, error: str, ...}`` payload.
"""
from __future__ import annotations

import json
import logging
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional

from config.hardware import is_allowed_camera_source, normalize_camera_source
from config.radar_gateway import (
    RADAR_IDS,
    is_allowed_device,
    sanitize_device_path,
)
from config.validation import ConfigError

LOGGER = logging.getLogger(__name__)

try:  # cv2 is optional at runtime; missing OpenCV is surfaced as an error.
    import cv2  # type: ignore
    _HAVE_CV2 = True
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore
    _HAVE_CV2 = False


# --------------------------------------------------------------------------- #
# Fake source for tests
# --------------------------------------------------------------------------- #


class FakeGatewaySource:
    """Pre-baked line source used by tests to drive :func:`probe_gateway`."""

    def __init__(self, lines: Iterable[str]) -> None:
        self._lines = [str(line) for line in lines]

    def readline(self) -> str:  # noqa: D401 - duck-typed file API
        if not self._lines:
            return ""
        return self._lines.pop(0)


# --------------------------------------------------------------------------- #
# Real Gateway probe
# --------------------------------------------------------------------------- #


def _open_uart(device: str, baudrate: int) -> Any:
    """Open the UART with POSIX termios.  Caller must close the fd."""
    import termios  # POSIX only.

    speed_attr = _termios_speed(baudrate)
    if speed_attr is None:
        raise ValueError(f"Baudrate {baudrate} chưa được hỗ trợ")
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = speed_attr
        attrs[5] = speed_attr
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)
    except Exception:
        os.close(fd)
        raise
    return fd


def _termios_speed(baudrate: int) -> Optional[int]:
    import termios

    table = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
        256000: getattr(termios, "B256000", termios.B230400),
    }
    return table.get(int(baudrate))


def _categorise_uart_exception(exc: BaseException, *, device: str) -> dict[str, Any]:
    """Translate a raw UART open exception into a Prompt-4 friendly report.

    The dashboard and preflight both rely on the ``category`` field to render
    a precise Vietnamese error message; the categories are intentionally
    stable identifiers so callers can map them to user-facing strings.
    """
    if isinstance(exc, FileNotFoundError):
        return {"ok": False, "category": "device_missing", "device": device,
                "error": f"Không tìm thấy cổng Gateway: {device}"}
    if isinstance(exc, PermissionError):
        return {"ok": False, "category": "permission_denied", "device": device,
                "error": f"Không có quyền đọc/ghi cổng Gateway: {device}"}
    # OSError 16 / 26 = EBUSY on Linux; on macOS the same op raises EBUSY too.
    errno = getattr(exc, "errno", None)
    if errno in {16, 26} or "busy" in str(exc).lower():
        return {"ok": False, "category": "port_busy", "device": device,
                "error": f"Cổng Gateway đang bận: {device}"}
    return {"ok": False, "category": "device_missing", "device": device,
            "error": f"Không mở được cổng Gateway: {exc}"}


def probe_gateway(
    device: str,
    *,
    baudrate: int = 115200,
    duration: float = 2.0,
    stale_after_seconds: float = 0.6,
    source: Optional[FakeGatewaySource] = None,
) -> dict[str, Any]:
    """Open the Gateway UART and try to receive one valid JSON-v2 packet.

    The function never raises: any error is reflected in the returned dict so
    the dashboard endpoint can return 200 with ``ok=false``.  Every error
    payload carries a stable ``category`` identifier so the dashboard and
    preflight can show a precise Vietnamese message.

    ``stale_after_seconds`` is threaded into the report so callers (the
    preflight + dashboard) can warn when the negotiated freshness window
    exceeds the actual packet cadence.  It does not change the probe loop
    itself; the loop is bounded by ``duration``.
    """
    if source is not None:
        # A FakeGatewaySource bypasses the allowlist guard by design — it is
        # only used by tests.  Skip straight to the deterministic fake path.
        if not isinstance(baudrate, int) or baudrate not in {
            9600, 19200, 38400, 57600, 115200, 230400, 256000
        }:
            return {"ok": False, "category": "bad_baudrate",
                    "error": f"Baudrate Gateway không được hỗ trợ: {baudrate}",
                    "device": str(device), "baudrate": baudrate}
        return _probe_with_fake_source(
            source,
            baudrate=int(baudrate),
            duration=float(duration),
            stale_after_seconds=float(stale_after_seconds),
        )

    if not is_allowed_device(device):
        return {"ok": False, "category": "disallowed_device",
                "error": "Đường dẫn Gateway nằm ngoài allowlist /dev.",
                "device": device}
    try:
        cleaned = sanitize_device_path(device)
    except ConfigError as exc:
        return {"ok": False, "category": "disallowed_device",
                "error": str(exc), "device": device}

    if not isinstance(baudrate, int) or baudrate not in {
        9600, 19200, 38400, 57600, 115200, 230400, 256000
    }:
        return {"ok": False, "category": "bad_baudrate",
                "error": f"Baudrate Gateway không được hỗ trợ: {baudrate}",
                "device": cleaned, "baudrate": baudrate}

    fd: Optional[int] = None
    try:
        fd = _open_uart(cleaned, int(baudrate))
    except ValueError as exc:
        return {"ok": False, "category": "bad_baudrate", "device": cleaned,
                "baudrate": int(baudrate),
                "error": f"Baudrate Gateway không được hỗ trợ: {baudrate} ({exc})"}
    except Exception as exc:
        return _categorise_uart_exception(exc, device=cleaned)

    json_errors = 0
    version_errors = 0
    try:
        buffer = b""
        deadline = time.monotonic() + float(duration)
        valid_packets: list[dict[str, Any]] = []
        seq_seen: list[int] = []
        # H1 fix: don't break on the first valid packet.  We collect
        # packets until we have >=3 OR the deadline elapses, then pick the
        # most recent one.  This way ``seq_increasing`` reflects real
        # monotonic motion instead of always reporting False because we
        # only ever saw one frame.
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                chunk = os.read(fd, 256)
            except OSError as exc:
                return _categorise_uart_exception(exc, device=cleaned)
            if chunk:
                buffer += chunk
                while b"\n" in buffer:
                    line, _, buffer = buffer.partition(b"\n")
                    text = line.decode("utf-8", errors="replace").strip()
                    if not text:
                        continue
                    try:
                        payload = json.loads(text)
                    except json.JSONDecodeError as exc:
                        json_errors += 1
                        LOGGER.debug("probe_gateway JSON lỗi: %s", exc)
                        continue
                    if not isinstance(payload, dict):
                        json_errors += 1
                        continue
                    if payload.get("v") != 2:
                        version_errors += 1
                        continue
                    seq = int(payload.get("seq", -1))
                    radars = payload.get("radars") if isinstance(payload.get("radars"), dict) else None
                    if radars is None or not all(rid in radars for rid in RADAR_IDS):
                        continue
                    seq_seen.append(seq)
                    valid_packets.append(payload)
                    if len(valid_packets) >= 3:
                        break
                if len(valid_packets) >= 3:
                    break
            else:
                time.sleep(0.02)
        if not valid_packets:
            # Decide the most useful category for the caller.
            if version_errors and json_errors == 0:
                category = "version_mismatch"
                message = "JSON Gateway có protocol_version sai (yêu cầu 2)."
            elif json_errors:
                category = "json_invalid"
                message = f"JSON Gateway lỗi ({json_errors} gói hỏng)."
            else:
                category = "timeout"
                message = (
                    f"Gateway không gửi JSON v2 hợp lệ trong {duration:.1f}s; "
                    "hãy kiểm tra ESP32 A, baudrate và cổng USB."
                )
            return {
                "ok": False,
                "category": category,
                "error": message,
                "duration_seconds": float(duration),
                "device": cleaned,
                "baudrate": int(baudrate),
                "json_errors": json_errors,
                "version_errors": version_errors,
                "stale_after_seconds": float(stale_after_seconds),
                "packets_received": len(valid_packets),
            }
        last_valid = valid_packets[-1]
        seq = int(last_valid.get("seq", -1))
        return {
            "ok": True,
            "device": cleaned,
            "baudrate": int(baudrate),
            "duration_seconds": float(duration),
            "stale_after_seconds": float(stale_after_seconds),
            "packets_received": len(valid_packets),
            "seq": seq,
            "gateway_ok": bool(last_valid.get("gateway_ok", False)),
            "remote_ok": bool(last_valid.get("remote_ok", False)),
            "gateway_boot_id": str(last_valid.get("gateway_boot_id", "")),
            "remote_boot_id": str(last_valid.get("remote_boot_id", "")),
            "radars": last_valid.get("radars", {}),
            "seq_increasing": len(seq_seen) >= 2 and seq_seen[-1] > seq_seen[0],
            "json_errors": json_errors,
            "version_errors": version_errors,
        }
    finally:
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass


def _probe_with_fake_source(
    source: FakeGatewaySource,
    *,
    baudrate: int,
    duration: float,
    stale_after_seconds: float = 0.6,
) -> dict[str, Any]:
    """Drive :func:`probe_gateway` with a fake line source (tests only).

    The fake path mirrors the categorisation used by the real probe so the
    dashboard and preflight see the same ``category`` identifiers regardless
    of whether the bytes came from a UART or a test fixture.
    """
    seq_seen: list[int] = []
    valid_packets: list[dict[str, Any]] = []
    json_errors = 0
    version_errors = 0
    deadline = time.monotonic() + float(duration)
    while time.monotonic() < deadline:
        if len(valid_packets) >= 3:
            break
        line = source.readline()
        if not line:
            time.sleep(0.005)
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            json_errors += 1
            continue
        if not isinstance(payload, dict):
            json_errors += 1
            continue
        if payload.get("v") != 2:
            version_errors += 1
            continue
        radars = payload.get("radars") if isinstance(payload.get("radars"), dict) else None
        if radars is None or not all(rid in radars for rid in RADAR_IDS):
            continue
        seq_seen.append(int(payload.get("seq", -1)))
        valid_packets.append(payload)
    if not valid_packets:
        if version_errors and json_errors == 0:
            category = "version_mismatch"
            message = "JSON Gateway có protocol_version sai (yêu cầu 2)."
        elif json_errors:
            category = "json_invalid"
            message = f"JSON Gateway lỗi ({json_errors} gói hỏng)."
        else:
            category = "timeout"
            message = (
                f"Gateway không gửi JSON v2 hợp lệ trong {duration:.1f}s; "
                "hãy kiểm tra ESP32 A, baudrate và cổng USB."
            )
        return {
            "ok": False,
            "category": category,
            "error": message,
            "duration_seconds": float(duration),
            "device": "<fake>",
            "baudrate": int(baudrate),
            "json_errors": json_errors,
            "version_errors": version_errors,
            "stale_after_seconds": float(stale_after_seconds),
            "packets_received": len(valid_packets),
        }
    last_valid = valid_packets[-1]
    seq = int(last_valid.get("seq", -1))
    return {
        "ok": True,
        "device": "<fake>",
        "baudrate": int(baudrate),
        "duration_seconds": float(duration),
        "stale_after_seconds": float(stale_after_seconds),
        "packets_received": len(valid_packets),
        "seq": seq,
        "gateway_ok": bool(last_valid.get("gateway_ok", False)),
        "remote_ok": bool(last_valid.get("remote_ok", False)),
        "gateway_boot_id": str(last_valid.get("gateway_boot_id", "")),
        "remote_boot_id": str(last_valid.get("remote_boot_id", "")),
        "radars": last_valid.get("radars", {}),
        "seq_increasing": len(seq_seen) >= 2 and seq_seen[-1] > seq_seen[0],
        "json_errors": json_errors,
        "version_errors": version_errors,
    }


# --------------------------------------------------------------------------- #
# Camera probe
# --------------------------------------------------------------------------- #


def probe_camera_managed(device: str) -> dict[str, Any]:
    """Try to read one frame from a camera index or ``/dev/video*`` node."""
    if device in (None, ""):
        return {"ok": False, "error": "device trống", "device": device}
    if not is_allowed_camera_source(device):
        return {
            "ok": False,
            "error": "Nguồn camera nằm ngoài allowlist (index 0-31, /dev/video*, /dev/v4l/).",
            "device": device,
        }
    try:
        source = normalize_camera_source(device)
    except ConfigError as exc:
        return {"ok": False, "error": str(exc), "device": device}
    if not _HAVE_CV2:
        return {"ok": False, "error": "OpenCV (cv2) chưa được cài.", "device": source}
    cap = None
    try:
        if sys.platform == "win32" and isinstance(source, int):
            cap_dshow = getattr(cv2, "CAP_DSHOW", 0)
            cap = cv2.VideoCapture(source, cap_dshow) if cap_dshow else cv2.VideoCapture(source)
        else:
            cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            return {"ok": False, "error": "Không mở được camera.", "device": source}
        ok, frame = cap.read()
        if not ok or frame is None:
            return {
                "ok": False,
                "error": "Mở được camera nhưng không đọc được frame.",
                "device": source,
            }
        return {
            "ok": True,
            "device": source,
            "frame_shape": list(frame.shape),
        }
    except Exception as exc:
        return {"ok": False, "error": f"Lỗi camera: {exc}", "device": source}
    finally:
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass


__all__ = [
    "FakeGatewaySource",
    "probe_camera_managed",
    "probe_gateway",
]
