"""Local hardware-readiness & cockpit dashboard server for ADAS.

The dashboard server exposes a single HTTP entry-point used by both the
in-memory ``CockpitSimulator`` (``--demo``) and the live pipeline.  All
endpoints are read-only by default; the only mutators are:

* ``PUT /api/radar/config``          — atomic YAML save of ``radar_gateway`` block
* ``PUT /api/config/modules``        — atomic YAML save of ``side_sensor_modules`` (legacy)
* ``PUT /api/hardware/config``       — atomic YAML save of camera slots, COM, LiDAR
* ``POST /api/recheck``              — run preflight.py as a subprocess
* ``POST /api/radar/test``           — open the Gateway UART and probe one JSON-v2 packet
* ``POST /api/camera/test``          — open a camera node and read at least one frame
* ``POST /api/hardware/test``        — presence/probe check for camera, serial, or LiDAR

Every mutator follows three rules:

1. Reject payloads outside the allowlist (``/dev/...`` only, max body 64 KB).
2. Validate every field with the same helper used by ``preflight.py``.
3. In **demo mode** return 403 instead of touching the hardware.
"""
from __future__ import annotations

import argparse
import json
import mimetypes
import platform
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional

from dashboard.simulator import CockpitSimulator
from config import ConfigError, inspect_module_devices, modules_from_config, save_side_modules
from config.hardware import (
    hardware_from_config,
    inspect_camera_device,
    inspect_lidar_source,
    inspect_serial_device,
    list_hardware_devices,
    save_hardware,
)
from config.loader import load_config
from config.radar_gateway import (
    RADAR_IDS,
    default_radar_gateway,
    is_allowed_device,
    list_serial_devices,
    radar_gateway_from_config as gateway_from_config,
    save_radar_gateway,
    validate_radar_gateway,
)
from sensors import (
    FakeGatewaySource,
    probe_camera_managed as probe_camera,
    probe_gateway,
)
from sensors.radar_gateway_packet import RadarFrame, RadarGatewayPacket, RadarGatewaySnapshot

ROOT = Path(__file__).resolve().parents[1]
STATIC_DIR = Path(__file__).resolve().parent / "static"
RUNTIME_LOG_DIR = ROOT / "logs"
RUNTIME_GATEWAY_LOG = RUNTIME_LOG_DIR / "radar_gateway.json"

MIME_MAP = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".png": "image/png",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
    ".woff2": "font/woff2",
}

ALLOWED_DEMO_ACTIONS = {
    "normal",
    "pedestrian",
    "danger",
    "lost_cam",
    "lost_lidar",
    "lost_ai",
    "turn_signal",
    "reverse",
    "pause_resume",
    "reset",
}

MAX_PAYLOAD = 65536
MAX_TEST_PAYLOAD = 8192


def _empty_radar_frame(rid: str) -> Dict[str, Any]:
    return {
        "detected": False,
        "target_state": "none",
        "distance_m": None,
        "moving_distance_m": None,
        "moving_energy": None,
        "stationary_distance_m": None,
        "stationary_energy": None,
        "valid": False,
        "stale": True,
        "age_ms": None,
        "error": "no-packet",
    }


def _radar_status_from_disk(path: Path, *, monotonic: float) -> Dict[str, Any]:
    """Return the latest gateway snapshot recorded by the live pipeline."""
    if not path.exists():
        return _empty_radar_status()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {
            **_empty_radar_status(),
            "error": f"Không đọc được radar_gateway.json: {exc}",
        }
    return _normalise_radar_status(payload, monotonic=monotonic)


def _empty_radar_status() -> Dict[str, Any]:
    return {
        "ready": False,
        "packet_rate_hz": 0.0,
        "packet_loss": 0.0,
        "packets_total": 0,
        "json_errors": 0,
        "crc_errors": 0,
        "packets_dropped": 0,
        "last_seq": -1,
        "packet_age_ms": None,
        "gateway_online": False,
        "remote_online": False,
        "gateway_boot_id": "",
        "remote_boot_id": "",
        "seq_increasing": False,
        "error": "Chưa có dữ liệu radar từ Gateway.",
        "radars": {rid: _empty_radar_frame(rid) for rid in RADAR_IDS},
        "checked_at": datetime.now(timezone.utc).isoformat(),
    }


def _normalise_radar_status(
    raw: Dict[str, Any], *, monotonic: float
) -> Dict[str, Any]:
    radars_out: Dict[str, Dict[str, Any]] = {}
    radars_in = raw.get("radars") or {}
    if not isinstance(radars_in, dict):
        radars_in = {}
    for rid in RADAR_IDS:
        frame = radars_in.get(rid) if isinstance(radars_in.get(rid), dict) else {}
        radars_out[rid] = {
            "detected": bool(frame.get("detected")),
            "target_state": str(frame.get("target_state", "none")),
            "distance_m": frame.get("distance_m"),
            "moving_distance_m": frame.get("moving_distance_m"),
            "moving_energy": frame.get("moving_energy"),
            "stationary_distance_m": frame.get("stationary_distance_m"),
            "stationary_energy": frame.get("stationary_energy"),
            "valid": bool(frame.get("valid")),
            "stale": bool(frame.get("stale", True)),
            "age_ms": frame.get("age_ms"),
            "error": str(frame.get("error", "")),
        }
    timestamp = raw.get("timestamp")
    age_ms = None
    if isinstance(timestamp, (int, float)):
        age_ms = max(0, int(round((monotonic - float(timestamp)) * 1000.0)))
    ready = bool(raw.get("gateway_online")) and not any(
        frame.get("error") for frame in radars_out.values()
    )
    return {
        "ready": ready,
        "packet_rate_hz": float(raw.get("packet_rate_hz", 0.0)),
        "packet_loss": float(raw.get("packet_loss", 0.0)),
        "packets_total": int(raw.get("packets_total", 0)),
        "json_errors": int(raw.get("json_errors", 0)),
        "crc_errors": int(raw.get("crc_errors", 0)),
        "packets_dropped": int(raw.get("packets_dropped", 0)),
        "last_seq": int(raw.get("last_seq", -1)),
        "packet_age_ms": age_ms,
        "gateway_online": bool(raw.get("gateway_online")),
        "remote_online": bool(raw.get("remote_online")),
        "gateway_boot_id": str(raw.get("gateway_boot_id", "")),
        "remote_boot_id": str(raw.get("remote_boot_id", "")),
        "seq_increasing": bool(raw.get("seq_increasing")),
        "error": str(raw.get("error", "")) if not ready else "",
        "radars": radars_out,
        "checked_at": datetime.now(timezone.utc).isoformat(),
    }


class DashboardServer(ThreadingHTTPServer):
    """Threading HTTP server with state, simulator, and concurrency controls."""

    allow_reuse_address = True

    def __init__(
        self,
        server_address: tuple[str, int],
        RequestHandlerClass: type[BaseHTTPRequestHandler],
        status_path: Path,
        config_path: Path,
        is_demo: bool = False,
    ) -> None:
        super().__init__(server_address, RequestHandlerClass)
        self.status_path = status_path
        self.config_path = config_path
        self.is_demo = is_demo
        self.recheck_lock = threading.Lock()
        self.test_lock = threading.Lock()
        self.simulator: Optional[CockpitSimulator] = CockpitSimulator() if is_demo else None


class DashboardHandler(BaseHTTPRequestHandler):
    """Request handler for cockpit endpoints and static assets."""

    server: DashboardServer

    # ------------------------------------------------------------------ utils

    def _send(
        self,
        body: bytes,
        content_type: str,
        status: int = HTTPStatus.OK,
        extra_headers: Optional[dict[str, str]] = None,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        if extra_headers:
            for k, v in extra_headers.items():
                self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, payload: Dict[str, Any], status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self._send(body, "application/json; charset=utf-8", status=status)

    def _read_json_body(self) -> Dict[str, Any]:
        try:
            content_length = int(self.headers.get("Content-Length", 0))
        except ValueError:
            content_length = 0
        if content_length < 0 or content_length > MAX_TEST_PAYLOAD:
            raise ValueError("payload size out of range")
        raw = self.rfile.read(content_length) if content_length > 0 else b"{}"
        if not raw:
            return {}
        try:
            data = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"invalid json: {exc}") from exc
        if not isinstance(data, dict):
            raise ValueError("payload must be a JSON object")
        return data

    # ------------------------------------------------------------------- GET

    def do_GET(self) -> None:
        path = self.path.split("?")[0]
        if path == "/":
            index_path = STATIC_DIR / "index.html"
            if index_path.exists():
                self._send(index_path.read_bytes(), "text/html; charset=utf-8")
            else:
                self._send(b"Dashboard HTML missing", "text/plain; charset=utf-8", HTTPStatus.INTERNAL_SERVER_ERROR)
            return

        if path == "/healthz":
            payload = {
                "status": "ok",
                "time": datetime.now(timezone.utc).isoformat(),
                "demo": bool(self.server.is_demo),
            }
            self._json(payload)
            return

        if path == "/api/status":
            if self.server.is_demo and self.server.simulator is not None:
                snap = self.server.simulator.get_snapshot()
                data = {
                    "ready": True,
                    "demo": True,
                    "checked_at": datetime.now(timezone.utc).isoformat(),
                    "platform": "In-Memory Cockpit Simulator",
                    "python": sys.version.split()[0],
                    "config": "demo-preview-cockpit",
                    "errors": [],
                    "snapshot": snap,
                }
            else:
                try:
                    if self.server.status_path.exists():
                        raw = self.server.status_path.read_text(encoding="utf-8")
                        data = json.loads(raw)
                    else:
                        data = {
                            "ready": False,
                            "checked_at": datetime.now(timezone.utc).isoformat(),
                            "platform": platform.platform(),
                            "python": sys.version.split()[0],
                            "config": str(self.server.config_path.name),
                            "errors": ["Chưa có kết quả kiểm tra preflight."],
                        }
                except Exception as exc:
                    data = {
                        "ready": False,
                        "checked_at": datetime.now(timezone.utc).isoformat(),
                        "config": str(self.server.config_path.name),
                        "errors": [f"Lỗi đọc trạng thái: {exc}"],
                    }

                # Strictly force demo to False for live mode
                data["demo"] = False

                live_snap_file = ROOT / "logs" / "pipeline_snapshot.json"
                if live_snap_file.exists():
                    try:
                        snapshot = json.loads(live_snap_file.read_text(encoding="utf-8"))
                        if time.time() - float(snapshot.get("timestamp", 0.0)) <= 2.0:
                            data["snapshot"] = snapshot
                    except Exception:
                        pass

                # Attach radar gateway status if available.
                radar_path = RUNTIME_GATEWAY_LOG
                data["radar_status"] = _radar_status_from_disk(
                    radar_path, monotonic=time.monotonic()
                )

            self._json(data)
            return

        if path == "/api/config/modules":
            try:
                config = load_config(self.server.config_path)
                modules = modules_from_config(config)
                self._json({
                    "ok": True,
                    "demo": bool(self.server.is_demo),
                    "restart_required": False,
                    "config": modules,
                })
            except Exception as exc:
                self._json(
                    {"ok": False, "error": f"Không đọc được cấu hình: {exc}"},
                    status=HTTPStatus.INTERNAL_SERVER_ERROR,
                )
            return

        if path == "/api/radar/config":
            try:
                config = load_config(self.server.config_path)
                block = gateway_from_config(config)
                self._json({
                    "ok": True,
                    "demo": bool(self.server.is_demo),
                    "restart_required": False,
                    "config": block,
                })
            except Exception as exc:
                self._json(
                    {"ok": False, "error": f"Không đọc được radar_gateway: {exc}"},
                    status=HTTPStatus.INTERNAL_SERVER_ERROR,
                )
            return

        if path == "/api/devices":
            try:
                devices = list_serial_devices(Path("/"))
            except Exception as exc:
                self._json(
                    {"ok": False, "error": f"Lỗi liệt kê thiết bị: {exc}"},
                    status=HTTPStatus.INTERNAL_SERVER_ERROR,
                )
                return
            self._json({
                "ok": True,
                "demo": bool(self.server.is_demo),
                "devices": devices,
            })
            return

        if path == "/api/hardware/config":
            try:
                config = load_config(self.server.config_path)
                self._json({
                    "ok": True,
                    "demo": bool(self.server.is_demo),
                    "restart_required": False,
                    "config": hardware_from_config(config),
                })
            except Exception as exc:
                self._json(
                    {"ok": False, "error": f"Không đọc được cấu hình phần cứng: {exc}"},
                    status=HTTPStatus.INTERNAL_SERVER_ERROR,
                )
            return

        if path == "/api/hardware/devices":
            try:
                devices = list_hardware_devices()
            except Exception as exc:
                self._json(
                    {"ok": False, "error": f"Lỗi liệt kê phần cứng: {exc}"},
                    status=HTTPStatus.INTERNAL_SERVER_ERROR,
                )
                return
            self._json({
                "ok": True,
                "demo": bool(self.server.is_demo),
                "devices": devices,
            })
            return

        if path == "/api/radar/status":
            try:
                payload = _radar_status_from_disk(
                    RUNTIME_GATEWAY_LOG, monotonic=time.monotonic()
                )
                self._json({"ok": True, **payload})
            except Exception as exc:
                self._json(
                    {"ok": False, "error": f"Không đọc được radar status: {exc}"},
                    status=HTTPStatus.INTERNAL_SERVER_ERROR,
                )
            return

        if path == "/api/camera/active.jpg":
            if self.server.is_demo:
                self._send(b"Not found", "text/plain; charset=utf-8", HTTPStatus.NOT_FOUND)
                return
            frame_path = ROOT / "logs" / "camera_active.jpg"
            snapshot_path = ROOT / "logs" / "pipeline_snapshot.json"
            fresh = False
            try:
                snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
                fresh = time.time() - float(snapshot.get("timestamp", 0.0)) <= 2.0
            except Exception:
                pass
            if not frame_path.is_file() or not fresh:
                self._send(b"Camera frame unavailable", "text/plain; charset=utf-8", HTTPStatus.SERVICE_UNAVAILABLE)
                return
            self._send(frame_path.read_bytes(), "image/jpeg")
            return

        camera_prefix = "/api/camera/"
        if path.startswith(camera_prefix) and path.endswith(".jpg"):
            camera_id = path[len(camera_prefix):-4].lower()
            allowed = {
                "front", "rear", "left", "right",
                "left_1", "left_2", "right_1", "right_2",
            }
            if self.server.is_demo or camera_id not in allowed:
                self._send(b"Not found", "text/plain; charset=utf-8", HTTPStatus.NOT_FOUND)
                return
            frame_path = ROOT / "logs" / f"camera_{camera_id}.jpg"
            snapshot_path = ROOT / "logs" / "pipeline_snapshot.json"
            try:
                snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
                fresh = time.time() - float(snapshot.get("timestamp", 0.0)) <= 2.0
                available = camera_id in snapshot.get("cameras_available", [])
            except Exception:
                fresh = False
                available = False
            if not frame_path.is_file() or not fresh or not available:
                self._send(b"Camera frame unavailable", "text/plain; charset=utf-8", HTTPStatus.SERVICE_UNAVAILABLE)
                return
            self._send(frame_path.read_bytes(), "image/jpeg")
            return

        # Static assets
        if path.startswith("/static/"):
            rel_name = path[8:]
            target = (STATIC_DIR / rel_name).resolve()
            if target.is_file() and target.is_relative_to(STATIC_DIR):
                mime = MIME_MAP.get(target.suffix.lower()) or mimetypes.guess_type(target.name)[0] or "application/octet-stream"
                self._send(target.read_bytes(), mime)
                return
            self._send(b"Asset not found", "text/plain; charset=utf-8", HTTPStatus.NOT_FOUND)
            return

        # Direct asset fallback (styles.css, app.js if requested without /static/)
        direct_target = (STATIC_DIR / path.lstrip("/")).resolve()
        if direct_target.is_file() and direct_target.is_relative_to(STATIC_DIR):
            mime = MIME_MAP.get(direct_target.suffix.lower()) or "text/plain; charset=utf-8"
            self._send(direct_target.read_bytes(), mime)
            return

        self._send(b"Not found", "text/plain; charset=utf-8", HTTPStatus.NOT_FOUND)

    # ------------------------------------------------------------------ POST

    def do_POST(self) -> None:
        path = self.path.split("?")[0]

        # Recheck endpoint
        if path == "/api/recheck":
            if self.server.is_demo:
                self._json(
                    {"ok": False, "error": "Chế độ Demo không được phép kiểm tra phần cứng."},
                    status=HTTPStatus.FORBIDDEN,
                )
                return

            acquired = self.server.recheck_lock.acquire(blocking=False)
            if not acquired:
                self._json(
                    {
                        "ok": False,
                        "busy": True,
                        "error": "Tiến trình kiểm tra preflight đang chạy, vui lòng đợi.",
                    },
                    status=HTTPStatus.CONFLICT,
                )
                return

            try:
                preflight_script = ROOT / "preflight.py"
                result = subprocess.run(
                    [sys.executable, str(preflight_script), "--config", str(self.server.config_path)],
                    cwd=ROOT,
                    capture_output=True,
                    text=True,
                    timeout=60,
                )
                self._json({
                    "ok": result.returncode == 0,
                    "code": result.returncode,
                })
            except subprocess.TimeoutExpired:
                self._json(
                    {"ok": False, "error": "Quá trình kiểm tra preflight bị timeout (vượt quá 60 giây)."},
                    status=HTTPStatus.GATEWAY_TIMEOUT,
                )
            except Exception as exc:
                self._json(
                    {"ok": False, "error": f"Lỗi khi thực thi preflight: {exc}"},
                    status=HTTPStatus.INTERNAL_SERVER_ERROR,
                )
            finally:
                self.server.recheck_lock.release()
            return

        if path == "/api/radar/test":
            if self.server.is_demo:
                self._json(
                    {"ok": False, "error": "Chế độ Demo không được phép kiểm tra thiết bị thật."},
                    status=HTTPStatus.FORBIDDEN,
                )
                return

            if not self.server.test_lock.acquire(blocking=False):
                self._json(
                    {"ok": False, "busy": True, "error": "Đang có một tiến trình kiểm tra khác; vui lòng đợi."},
                    status=HTTPStatus.CONFLICT,
                )
                return
            try:
                self._run_radar_test()
            except (ValueError, ConfigError) as exc:
                self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            except Exception as exc:
                self._json(
                    {"ok": False, "error": f"Lỗi kiểm tra Gateway: {exc}"},
                    status=HTTPStatus.INTERNAL_SERVER_ERROR,
                )
            finally:
                self.server.test_lock.release()
            return

        if path == "/api/camera/test":
            if self.server.is_demo:
                self._json(
                    {"ok": False, "error": "Chế độ Demo không được phép kiểm tra camera thật."},
                    status=HTTPStatus.FORBIDDEN,
                )
                return
            try:
                payload = self._read_json_body()
            except ValueError as exc:
                self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return
            device = str(payload.get("device", "")).strip()
            if "/" in device and not device.startswith("/dev/"):
                self._json(
                    {"ok": False, "error": "Đường dẫn camera phải nằm trong /dev."},
                    status=HTTPStatus.BAD_REQUEST,
                )
                return
            report = probe_camera(device)
            status = HTTPStatus.OK if report.get("ok") else HTTPStatus.BAD_REQUEST
            self._json(report, status=status)
            return

        if path == "/api/hardware/test":
            if self.server.is_demo:
                self._json(
                    {"ok": False, "error": "Chế độ Demo không được phép kiểm tra thiết bị thật."},
                    status=HTTPStatus.FORBIDDEN,
                )
                return
            try:
                payload = self._read_json_body()
            except ValueError as exc:
                self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return
            kind = str(payload.get("kind", "")).strip().lower()
            device = str(payload.get("device", "")).strip()
            if kind == "camera":
                report = probe_camera(device) if payload.get("probe", True) else inspect_camera_device(device)
            elif kind == "serial":
                report = inspect_serial_device(device)
            elif kind == "lidar":
                report = inspect_lidar_source(device)
            else:
                self._json(
                    {"ok": False, "error": "kind phải là camera, serial hoặc lidar."},
                    status=HTTPStatus.BAD_REQUEST,
                )
                return
            status = HTTPStatus.OK if report.get("ok") else HTTPStatus.BAD_REQUEST
            self._json(report, status=status)
            return

        if path == "/api/demo/action":
            if not self.server.is_demo or self.server.simulator is None:
                self._send(b"Not found", "text/plain; charset=utf-8", HTTPStatus.NOT_FOUND)
                return

            content_length = int(self.headers.get("Content-Length", 0))
            if content_length > MAX_TEST_PAYLOAD:
                self._json({"ok": False, "error": "Payload too large"},
                           status=HTTPStatus.REQUEST_ENTITY_TOO_LARGE)
                return

            try:
                data = self._read_json_body() if content_length > 0 else {}
            except ValueError as exc:
                self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return

            action = str(data.get("action", "")).strip().lower()
            if action not in ALLOWED_DEMO_ACTIONS:
                self._json(
                    {"ok": False, "error": f"Hành động không hợp lệ. Cho phép: {sorted(ALLOWED_DEMO_ACTIONS)}"},
                    status=HTTPStatus.BAD_REQUEST,
                )
                return

            self.server.simulator.apply_action(action)
            snap = self.server.simulator.get_snapshot()
            self._json({
                "ok": True,
                "action": action,
                "snapshot": snap,
            })
            return

        if path == "/api/demo/camera":
            if not self.server.is_demo or self.server.simulator is None:
                self._send(b"Not found", "text/plain; charset=utf-8", HTTPStatus.NOT_FOUND)
                return

            content_length = int(self.headers.get("Content-Length", 0))
            try:
                data = self._read_json_body() if content_length > 0 else {}
            except ValueError as exc:
                self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return

            cam_id = str(data.get("camera", "front")).strip().lower()
            self.server.simulator.set_camera(cam_id)
            self._json({
                "ok": True,
                "active_camera": self.server.simulator.active_camera,
            })
            return

        if path == "/api/config/modules/test":
            if self.server.is_demo:
                self._json({"ok": False, "error": "Demo không truy cập thiết bị thật."},
                           status=HTTPStatus.FORBIDDEN)
                return
            try:
                payload = self._read_json_body()
            except ValueError as exc:
                self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return
            try:
                module_id = str(payload.get("module_id", ""))
                config = modules_from_config(load_config(self.server.config_path))
                if module_id not in config["modules"]:
                    raise ConfigError("Module ID không hợp lệ")
                devices = inspect_module_devices(config["modules"][module_id])
                ok = devices["camera"]["present"] and devices["radar"]["present"] and devices["radar"]["readable"]
                self._json({"ok": ok, "module_id": module_id, "devices": devices})
            except (ValueError, ConfigError) as exc:
                self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            return

        self._send(b"Not found", "text/plain; charset=utf-8", HTTPStatus.NOT_FOUND)

    # ------------------------------------------------------------------- PUT

    def do_PUT(self) -> None:
        path = self.path.split("?")[0]
        if path == "/api/radar/config":
            if self.server.is_demo:
                self._json(
                    {"ok": False, "error": "Demo chỉ cho phép xem cấu hình Gateway."},
                    status=HTTPStatus.FORBIDDEN,
                )
                return
            try:
                payload = self._read_json_body()
            except ValueError as exc:
                self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return
            if not isinstance(payload, dict):
                self._json({"ok": False, "error": "Payload phải là JSON object."},
                           status=HTTPStatus.BAD_REQUEST)
                return
            block = payload.get("config", payload)
            if not isinstance(block, dict):
                self._json({"ok": False, "error": "config phải là JSON object."},
                           status=HTTPStatus.BAD_REQUEST)
                return
            try:
                normalised = save_radar_gateway(self.server.config_path, block)
            except (ConfigError, ValueError) as exc:
                self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return
            except Exception as exc:
                self._json(
                    {"ok": False, "error": f"Không lưu được radar_gateway: {exc}"},
                    status=HTTPStatus.INTERNAL_SERVER_ERROR,
                )
                return
            self._json({
                "ok": True,
                "restart_required": True,
                "message": "Đã lưu radar_gateway. Hãy khởi động lại ADAS Live để áp dụng.",
                "config": normalised,
            })
            return
        if path == "/api/hardware/config":
            if self.server.is_demo:
                self._json(
                    {"ok": False, "error": "Demo chỉ cho phép xem cấu hình phần cứng."},
                    status=HTTPStatus.FORBIDDEN,
                )
                return
            try:
                payload = self._read_json_body()
            except ValueError as exc:
                self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return
            if not isinstance(payload, dict):
                self._json({"ok": False, "error": "Payload phải là JSON object."},
                           status=HTTPStatus.BAD_REQUEST)
                return
            block = payload.get("config", payload)
            if not isinstance(block, dict):
                self._json({"ok": False, "error": "config phải là JSON object."},
                           status=HTTPStatus.BAD_REQUEST)
                return
            try:
                saved = save_hardware(self.server.config_path, block)
            except (ConfigError, ValueError) as exc:
                self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return
            except Exception as exc:
                self._json(
                    {"ok": False, "error": f"Không lưu được cấu hình phần cứng: {exc}"},
                    status=HTTPStatus.INTERNAL_SERVER_ERROR,
                )
                return
            self._json({
                "ok": True,
                "restart_required": True,
                "message": "Đã lưu cấu hình phần cứng. Hãy khởi động lại ADAS Live để áp dụng.",
                "config": saved,
            })
            return
        if path != "/api/config/modules":
            self._send(b"Not found", "text/plain; charset=utf-8", HTTPStatus.NOT_FOUND)
            return
        if self.server.is_demo:
            self._json(
                {"ok": False, "error": "Demo chỉ cho phép xem cấu hình."},
                status=HTTPStatus.FORBIDDEN,
            )
            return
        try:
            payload = self._read_json_body()
        except ValueError as exc:
            self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            return
        modules = payload.get("config", payload)
        try:
            normalized = save_side_modules(self.server.config_path, modules)
            self._json({
                "ok": True,
                "restart_required": True,
                "message": "Đã lưu cấu hình. Hãy khởi động lại ADAS Live để áp dụng.",
                "config": normalized,
            })
        except (ValueError, ConfigError) as exc:
            self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
        except Exception as exc:
            self._json(
                {"ok": False, "error": f"Không lưu được cấu hình: {exc}"},
                status=HTTPStatus.INTERNAL_SERVER_ERROR,
            )

    # ------------------------------------------------------------------ misc

    def log_message(self, fmt: str, *args) -> None:
        # Suppress routine HTTP request logging to keep console clean
        return

    # ---------------------------------------------------------- radar probe

    def _run_radar_test(self) -> None:
        """Run a JSON-v2 Gateway probe.

        Security guarantees:

        * ``device`` MUST be in the allowlist (see
          :func:`config.radar_gateway.sanitize_device_path`).
        * No shell command is built from user-supplied input.
        * Probe is bounded in time (default 2.0s, max 5.0s).
        * Reader is always closed in a ``finally`` block.
        """
        payload = self._read_json_body()
        device = str(payload.get("device", "")).strip()
        # allowlist enforced *before* opening the device
        if "/" in device and not device.startswith("/dev/"):
            raise ConfigError("Đường dẫn Gateway phải nằm trong /dev.")
        baudrate = int(payload.get("baudrate", 115200) or 115200)
        duration = float(payload.get("duration", 2.0) or 2.0)
        duration = max(0.5, min(duration, 5.0))
        fake_lines = payload.get("fake_lines")
        source: Optional[FakeGatewaySource] = None
        if isinstance(fake_lines, list):
            source = FakeGatewaySource([str(line) for line in fake_lines])
        report = probe_gateway(
            device,
            baudrate=baudrate,
            duration=duration,
            source=source,
        )
        if not report.get("ok"):
            status = HTTPStatus.BAD_REQUEST
        else:
            status = HTTPStatus.OK
        self._json(report, status=status)


def create_dashboard_server(
    host: str = "127.0.0.1",
    port: int = 8765,
    status_path: Optional[Path] = None,
    config_path: Optional[Path] = None,
    demo: bool = False,
) -> DashboardServer:
    st_path = (status_path or (ROOT / "logs" / "preflight_status.json")).resolve()
    cfg_path = (config_path or (ROOT / "config" / "jetson_live.yaml")).resolve()

    if demo:
        st_path.parent.mkdir(parents=True, exist_ok=True)
        st_path.write_text(
            json.dumps({
                "ready": True,
                "demo": True,
                "checked_at": datetime.now(timezone.utc).isoformat(),
                "platform": "In-Memory Cockpit Simulator",
                "python": sys.version.split()[0],
                "config": "demo-preview-cockpit",
                "errors": [],
            }, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    return DashboardServer((host, port), DashboardHandler, st_path, cfg_path, is_demo=demo)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="ADAS Cockpit & Hardware Readiness Dashboard")
    parser.add_argument("--host", default="127.0.0.1", help="Host address to bind (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8765, help="Port to listen on (default: 8765)")
    parser.add_argument("--status", default="logs/preflight_status.json", help="Path to preflight status json")
    parser.add_argument("--config", default="config/jetson_live.yaml", help="Path to ADAS config yaml")
    parser.add_argument("--demo", action="store_true", help="Run UI preview only with in-memory simulator; never access hardware")
    args = parser.parse_args(argv)

    server = create_dashboard_server(
        host=args.host,
        port=args.port,
        status_path=ROOT / args.status,
        config_path=ROOT / args.config,
        demo=args.demo,
    )
    mode_str = "DEMO COCKPIT" if args.demo else "LIVE DIAGNOSTICS"
    print(f"[dashboard] [{mode_str}] http://{args.host}:{args.port}", flush=True)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
