"""Fail-fast deployment checks for Jetson live mode.

The preflight script is the final gate before ADAS Live can run.  It runs in
two modes:

* ``live``     – every hardware prerequisite is mandatory; failing any of them
                 returns exit code 1 and writes the structured error list into
                 ``logs/preflight_status.json`` so the dashboard can show a
                 Vietnamese error description per failure.
* ``demo/playback/camera-only/...`` – used by ``run.py`` and tooling during
                 development; warnings are tolerated.

Important architectural notes (Prompt 4 contract):

* The new single-Gateway radar architecture (``radar_gateway`` block) is the
  canonical live contract; the legacy ``side_sensor_modules`` is **disabled**
  by default and not exercised by ``main()``.
* All Vietnamese error messages are kept here so the dashboard and CLI agree
  on what to display.
* Preflight **never** invents hardware: missing models, missing TensorRT
  engines, missing calibration, missing real LiDAR driver, missing
  ``CHANGE_ME`` placeholders → blocker, never a synthetic pass.
"""
from __future__ import annotations

import argparse
import io
import importlib
import importlib.util
import json
import os
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml

from config import (
    RADAR_IDS,
    ConfigError,
    is_allowed_device,
    radar_gateway_from_config as gateway_from_config,
    validate_radar_gateway,
)
from config.radar_gateway import LD2410_MAX_DETECTION_M
from sensors import FakeGatewaySource, probe_camera_managed, probe_gateway
from sensors.radar_gateway_packet import parse_gateway_packet

ROOT = Path(__file__).resolve().parent
STATUS_PATH = ROOT / "logs" / "preflight_status.json"

# Windows consoles (cp1252) crash on Vietnamese messages: force UTF-8.
if sys.platform == "win32":  # pragma: no cover - host specific
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")

# Vietnamese error messages – single source of truth between CLI and dashboard.
ERR_NO_CONFIG = "Không có config: {path}"
ERR_MODE_NOT_LIVE = "Config triển khai phải có mode: live"
ERR_ALLOW_MOCK = "Live phải đặt safety.allow_mock: false"
ERR_CALIBRATION_DEMO = "Calibration vẫn được đánh dấu demo"
ERR_NO_MODEL = "Không có model: {model}"
ERR_MODEL_LOAD = "Không load được model inference: {exc}"
ERR_BAD_VISION_CLASSES = (
    "allowed_classes không khớp model; model_classes={classes}"
)
ERR_NO_CALIBRATION = "Không có calibration live: {path}"
ERR_NO_LIDAR_DRIVER = "Chưa cấu hình driver LiDAR thật"
ERR_NO_LIDAR_MODULE = "Không tìm thấy module driver LiDAR: {module}"
ERR_LIDAR_MODULE_LOAD = "Không load được driver LiDAR: {exc}"
ERR_NO_LIDAR_DRIVER_REGISTERED = "Module chưa đăng ký driver '{name}'"
ERR_BAD_WARNING_MODE = "Bản triển khai này yêu cầu warning.mode: distance"
ERR_BAD_DISTANCE_ORDER = (
    "Ngưỡng khoảng cách phải thỏa 0 < emergency <= danger <= caution"
)
ERR_NO_CUDA = "PyTorch không nhìn thấy CUDA"
ERR_NO_TORCH = "Không import được PyTorch Jetson: {exc}"
ERR_CAMERA_READ = "Không đọc được camera {camera} source={source}"
ERR_CAMERA_EXCEPTION = "Kiểm tra camera lỗi: {exc}"
ERR_GATEWAY_BLOCK = "radar_gateway block không hợp lệ: {exc}"
ERR_GATEWAY_DISABLED = (
    "radar_gateway.enabled=false nhưng radar giữa xe đã được bật qua "
    "side_sensor_modules hoặc radar block khác."
)
ERR_GATEWAY_CHANGE_ME = (
    "radar_gateway.device vẫn là placeholder CHANGE_ME; "
    "hãy đặt /dev/serial/by-id/... của ESP32 A Gateway."
)
ERR_GATEWAY_DEVICE_MISSING = "Không tìm thấy cổng Gateway: {device}"
ERR_GATEWAY_DEVICE_DISALLOWED = (
    "Đường dẫn Gateway không thuộc allowlist /dev: {device}"
)
ERR_GATEWAY_DEVICE_PERMISSION = (
    "Không có quyền đọc/ghi cổng Gateway: {device}"
)
ERR_GATEWAY_DEVICE_PORT_BUSY = "Cổng Gateway đang bận: {device}"
ERR_GATEWAY_BAD_BAUDRATE = (
    "Baudrate Gateway không được hỗ trợ: {baudrate}"
)
ERR_GATEWAY_VERSION = (
    "JSON Gateway có protocol_version sai (yêu cầu 2)."
)
ERR_GATEWAY_JSON = "JSON Gateway lỗi: {exc}"
ERR_GATEWAY_TIMEOUT = (
    "Gateway không gửi JSON v2 hợp lệ trong {seconds:.1f}s; "
    "hãy kiểm tra ESP32 A, baudrate và cổng USB."
)
ERR_GATEWAY_SEQ = "Gateway sequence không tăng sau {count} gói."
ERR_GATEWAY_NO_PACKET = "Gateway không nhận được gói nào trong preflight."
ERR_GATEWAY_RADAR_MISSING = (
    "Gateway thiếu radar '{radar}' trong JSON v2; "
    "hãy kiểm tra mapping bốn radar trên firmware ESP32 A."
)
ERR_REMOTE_REQUIRED = (
    "Radar bên ESP32 B (Remote) đang enabled nhưng remote_ok=false; "
    "hãy kiểm tra ESP-NOW và nguồn ESP32 B."
)
ERR_GATEWAY_OFFLINE = "Gateway không online trong Live preflight."
ERR_MOCK_RADAR_FORBIDDEN = (
    "Live không được phép dùng dữ liệu radar mock; "
    "hãy chạy với radar_gateway.device thật của ESP32 A Gateway."
)
ERR_PYTHON_VERSION = "Cần Python >= 3.10"


def fail(message: str) -> None:
    print(f"[FAIL] {message}", file=sys.stderr)


def write_status(config_path: Path, errors: list[str], extras: dict | None = None) -> None:
    STATUS_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "ready": not errors,
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "config": str(config_path),
        "errors": errors,
    }
    if extras:
        payload.update(extras)
    STATUS_PATH.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _check_python(errors: list[str]) -> None:
    if sys.version_info < (3, 10):
        errors.append(ERR_PYTHON_VERSION)


def _check_safety(cfg: dict, errors: list[str], *, mode: str = "live") -> None:
    """Enforce the production safety contract.

    The contract only applies to ``live`` mode; in ``demo`` mode the safety
    flags intentionally default to permissive so the developer can iterate
    without TensorRT / real LiDAR.  Calling this without ``mode`` defaults
    to ``live`` to keep backwards compatibility with any external callers.
    """
    if mode != "live":
        return
    safety = cfg.get("safety", {})
    if safety.get("allow_mock", True):
        errors.append(ERR_ALLOW_MOCK)
    if safety.get("is_demo_calibration", True):
        errors.append(ERR_CALIBRATION_DEMO)


def _check_model(cfg: dict, root: Path, errors: list[str]) -> None:
    model = Path(cfg.get("vision", {}).get("model", ""))
    if model and not model.is_absolute():
        model = root / model
    if not model.exists():
        errors.append(ERR_NO_MODEL.format(model=model))
        return
    try:
        from ultralytics import YOLO
        loaded_model = YOLO(str(model))
        names = {str(value).lower() for value in loaded_model.names.values()}
        allowed = {str(value).lower() for value in cfg.get("vision", {}).get("allowed_classes", [])}
        if allowed and not (allowed & names):
            errors.append(
                ERR_BAD_VISION_CLASSES.format(classes=sorted(names))
            )
        else:
            print(f"[OK] Model classes={sorted(names)}")
    except Exception as exc:
        errors.append(ERR_MODEL_LOAD.format(exc=exc))


def _check_calibration(cfg: dict, root: Path, errors: list[str]) -> None:
    calibration = Path(cfg.get("calibration", {}).get("file", ""))
    if calibration and not calibration.is_absolute():
        calibration = root / calibration
    if not calibration.exists():
        errors.append(ERR_NO_CALIBRATION.format(path=calibration))


def _check_lidar(cfg: dict, errors: list[str]) -> None:
    lidar = cfg.get("lidar", {})
    if str(lidar.get("driver", "")).lower() in {"", "mock", "change_me"}:
        errors.append(ERR_NO_LIDAR_DRIVER)
        return
    driver_module = lidar.get("driver_module")
    if not driver_module or importlib.util.find_spec(str(driver_module)) is None:
        errors.append(ERR_NO_LIDAR_MODULE.format(module=driver_module))
        return
    try:
        importlib.import_module(str(driver_module))
        from sensors import LidarReaderFactory
        driver_name = str(lidar.get("driver", "")).lower()
        if driver_name not in LidarReaderFactory.available():
            errors.append(ERR_NO_LIDAR_DRIVER_REGISTERED.format(name=driver_name))
    except Exception as exc:
        errors.append(ERR_LIDAR_MODULE_LOAD.format(exc=exc))


def _check_warning(cfg: dict, errors: list[str]) -> None:
    warning = cfg.get("warning", {})
    if warning.get("mode") != "distance":
        errors.append(ERR_BAD_WARNING_MODE)
    try:
        emergency = float(warning["emergency_distance_m"])
        danger = float(warning["danger_distance_m"])
        caution = float(warning["caution_distance_m"])
        if not 0 < emergency <= danger <= caution:
            raise ValueError
    except (KeyError, TypeError, ValueError):
        errors.append(ERR_BAD_DISTANCE_ORDER)


def _check_jetson(errors: list[str], cfg: dict, root: Path) -> None:
    """Jetson Orin Nano live gates: TensorRT, engine file, device nodes, RAM."""
    try:
        import tensorrt  # noqa: F401
        print(f"[OK] TensorRT: {tensorrt.__version__}")
    except Exception as exc:
        errors.append(f"Không import được TensorRT: {exc}")
    model = str(cfg.get("vision", {}).get("model", ""))
    if model.endswith(".engine"):
        mp = Path(model) if Path(model).is_absolute() else root / model
        if not mp.exists():
            errors.append(ERR_NO_MODEL.format(model=mp))
        elif mp.stat().st_size < 1024 * 1024:
            errors.append(f"Engine quá nhỏ, có thể export lỗi: {mp}")
    else:
        errors.append(f"Live Jetson yêu cầu model .engine TensorRT FP16, got: {model}")
    for node in ("/dev/video0", "/dev/ttyACM0"):
        p = Path(node)
        if not p.exists():
            errors.append(f"Không thấy thiết bị {node} — kiểm tra camera/LiDAR đã cắm")
        elif not os.access(node, os.R_OK | os.W_OK):
            errors.append(ERR_GATEWAY_DEVICE_PERMISSION.format(device=node))
    try:
        mem_kb = 0
        with open("/proc/meminfo") as fh:
            for line in fh:
                if line.startswith("MemTotal:"):
                    mem_kb = int(line.split()[1])
                    break
        if mem_kb and mem_kb < 6 * 1024 * 1024:
            errors.append(f"RAM thấp ({mem_kb // 1024}MB), Jetson Orin Nano cần 8GB")
    except FileNotFoundError:
        pass  # not Linux/Jetson (dev on Windows): skip
    except Exception as exc:
        errors.append(f"Không đọc được RAM: {exc}")


def _check_cuda(errors: list[str]) -> None:
    try:
        import torch
        if not torch.cuda.is_available():
            errors.append(ERR_NO_CUDA)
        else:
            print(f"[OK] CUDA: {torch.cuda.get_device_name(0)}")
    except Exception as exc:
        errors.append(ERR_NO_TORCH.format(exc=exc))


def _check_cameras(cfg: dict, errors: list[str], *, require_front: bool = True) -> bool:
    """Check cameras.

    Returns ``True`` iff the **front** camera is satisfied (when
    ``require_front`` is set).  Side cameras are checked only when they
    appear in the ``camera.sources`` mapping AND the radar gateway maps a
    side radar to that camera (side-front and side-rear).
    """
    import cv2  # local import – matches legacy behaviour

    camera_cfg = cfg.get("camera", {})
    sources = camera_cfg.get("sources") or {"front": camera_cfg.get("source", 0)}
    gateway = gateway_from_config(cfg)
    side_cameras = {
        str(radar_block.get("camera", "")).strip()
        for radar_block in gateway.get("radars", {}).values()
        if radar_block.get("enabled")
    }
    ok = True
    for camera_id, source_config in sources.items():
        source = (
            source_config.get("device", source_config.get("source"))
            if isinstance(source_config, dict)
            else source_config
        )
        if not source:
            continue
        if camera_id != "front" and (
            camera_id not in side_cameras or not side_cameras
        ):
            continue
        resolved = int(source) if str(source).isdigit() else str(source)
        cap = None
        try:
            cap = cv2.VideoCapture(resolved)
            if not cap.isOpened():
                if require_front and camera_id == "front":
                    errors.append(ERR_CAMERA_READ.format(camera=camera_id, source=source))
                    ok = False
                continue
            ok_read, _ = cap.read()
            if not ok_read:
                if require_front and camera_id == "front":
                    errors.append(
                        ERR_CAMERA_READ.format(camera=camera_id, source=source)
                    )
                    ok = False
            else:
                print(f"[OK] Camera {camera_id} source={source}")
        except Exception as exc:
            if require_front and camera_id == "front":
                errors.append(ERR_CAMERA_EXCEPTION.format(exc=exc))
                ok = False
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
    if require_front and not any(
        (cid == "front" and src) for cid, src in sources.items()
    ):
        errors.append("Live yêu cầu camera 'front' trong camera.sources.")
        ok = False
    return ok


def _check_radar_gateway(
    cfg: dict,
    errors: list[str],
    *,
    require_online: bool,
) -> dict:
    """Validate the new ``radar_gateway`` block and probe the link.

    ``require_online=True`` opens a real UART probe for ~2 seconds.  If the
    block is disabled we only sanity-check its structure.
    """
    block = cfg.get("radar_gateway")
    if not isinstance(block, dict):
        block = gateway_from_config(cfg)
    try:
        normalised = validate_radar_gateway(block)
    except ConfigError as exc:
        errors.append(ERR_GATEWAY_BLOCK.format(exc=exc))
        return {"valid": False, "block": block}

    enabled = bool(normalised.get("enabled", True))
    if not enabled:
        # If radar is disabled, fail loud if legacy modules imply otherwise.
        legacy = cfg.get("side_sensor_modules", {})
        if legacy.get("enabled", False):
            errors.append(ERR_GATEWAY_DISABLED)
        return {"valid": True, "block": normalised}

    device = normalised.get("device", "")
    if "CHANGE_ME" in device.upper():
        errors.append(ERR_GATEWAY_CHANGE_ME)
        return {"valid": True, "block": normalised}
    # H2: refuse mock device strings.  In live mode the Gateway MUST point
    # at a real UART; ``mock://`` paths or any path containing ``mock``
    # as a segment are reserved for tests and the demo mode.  This stops
    # an accidental copy/paste from a config.example into a production
    # YAML silently passing preflight.
    device_lower = device.lower()
    if device_lower.endswith("mock") or "/mock" in device_lower or device_lower.startswith("mock://"):
        errors.append(
            f"radar_gateway.device='{device}' trỏ vào mock — live mode "
            "(require_online=true) phải dùng cổng UART thật (/dev/ttyUSB*). "
            "Hãy đổi sang /dev/ttyUSB0 hoặc đường dẫn phần cứng thực tế."
        )
        return {"valid": True, "block": normalised}
    if not is_allowed_device(device):
        errors.append(ERR_GATEWAY_DEVICE_DISALLOWED.format(device=device))
        return {"valid": True, "block": normalised}
    if not Path(device).exists():
        errors.append(ERR_GATEWAY_DEVICE_MISSING.format(device=device))
        return {"valid": True, "block": normalised}
    # Permission check
    if not os.access(device, os.R_OK | os.W_OK):
        errors.append(ERR_GATEWAY_DEVICE_PERMISSION.format(device=device))
        return {"valid": True, "block": normalised}

    if not require_online:
        # In non-live preflight (e.g. integration tests) we skip the real
        # device probe but still report success.
        return {"valid": True, "block": normalised}

    # Real probe -----------------------------------------------------
    report = probe_gateway(
        device,
        baudrate=int(normalised.get("baudrate", 115200)),
        duration=2.0,
        stale_after_seconds=float(normalised.get("stale_after_seconds", 0.6)),
    )
    if report.get("error"):
        # Categorise the failure
        category = report.get("category")
        msg = report.get("error", "")
        if category == "device_missing":
            errors.append(ERR_GATEWAY_DEVICE_MISSING.format(device=device))
        elif category == "permission_denied":
            errors.append(ERR_GATEWAY_DEVICE_PERMISSION.format(device=device))
        elif category == "port_busy":
            errors.append(ERR_GATEWAY_DEVICE_PORT_BUSY.format(device=device))
        elif category == "bad_baudrate":
            errors.append(ERR_GATEWAY_BAD_BAUDRATE.format(baudrate=normalised.get("baudrate")))
        elif category == "version_mismatch":
            errors.append(ERR_GATEWAY_VERSION)
        elif category == "json_invalid":
            errors.append(ERR_GATEWAY_JSON.format(exc=msg))
        elif category == "timeout":
            errors.append(ERR_GATEWAY_TIMEOUT.format(seconds=2.0))
        else:
            errors.append(ERR_GATEWAY_TIMEOUT.format(seconds=2.0))
        return {"valid": True, "block": normalised, "report": report}

    if not report.get("seq_increasing"):
        errors.append(ERR_GATEWAY_SEQ.format(count=report.get("packets_total", 0)))
    if not report.get("gateway_ok"):
        errors.append(ERR_GATEWAY_OFFLINE)
    radars = report.get("radars", {})
    for radar_id in RADAR_IDS:
        # Each configured enabled radar must show at least once.
        radar_cfg = normalised["radars"].get(radar_id, {})
        if radar_cfg.get("enabled", True):
            if radar_id not in radars:
                errors.append(ERR_GATEWAY_RADAR_MISSING.format(radar=radar_id))
            else:
                frame = radars[radar_id]
                if not frame.get("valid") and frame.get("error"):
                    # Surface error only if it's an actual fault (not "no-packet")
                    pass
    remote_required = any(
        radar_cfg.get("enabled", True)
        for radar_id, radar_cfg in normalised["radars"].items()
        if radar_id.startswith("right_")
    )
    if remote_required and not report.get("remote_ok"):
        errors.append(ERR_REMOTE_REQUIRED)
    return {"valid": True, "block": normalised, "report": report}


def _check_mock_radar(cfg: dict, errors: list[str]) -> None:
    safety = cfg.get("safety", {})
    if safety.get("allow_mock", True):
        # already covered by safety check; nothing extra
        return
    # Make sure no legacy side module is providing mock data.
    legacy = cfg.get("side_sensor_modules", {})
    if legacy.get("enabled", False):
        for module_id, item in legacy.get("modules", {}).items():
            radar = item.get("radar", {})
            device = str(radar.get("device", "")).lower()
            if not device or "change_me" in device or "mock" in device:
                errors.append(ERR_MOCK_RADAR_FORBIDDEN)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/jetson_live.yaml")
    parser.add_argument(
        "--skip-hardware",
        action="store_true",
        help="Skip probes that open real hardware (used by integration tests only)",
    )
    args = parser.parse_args()
    path = Path(args.config)
    if not path.is_absolute():
        path = ROOT / path
    errors: list[str] = []
    print(f"[INFO] platform={platform.platform()} python={platform.python_version()}")
    _check_python(errors)
    if not path.exists():
        errors.append(ERR_NO_CONFIG.format(path=path))
        cfg = {}
    else:
        cfg = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    mode = cfg.get("mode", "demo")
    if mode == "live":
        _check_safety(cfg, errors, mode="live")
        _check_model(cfg, ROOT, errors)
        _check_calibration(cfg, ROOT, errors)
        _check_lidar(cfg, errors)
        _check_warning(cfg, errors)
        _check_cuda(errors)
        _check_jetson(errors, cfg, ROOT)
        _check_cameras(cfg, errors, require_front=True)
        _check_radar_gateway(cfg, errors, require_online=not args.skip_hardware)
        _check_mock_radar(cfg, errors)
    elif mode == "demo":
        # Demo mode is allowed to run without CUDA / TensorRT / hardware.
        # We still verify the safety defaults so a deployment that flips
        # ``allow_mock=true`` on a Jetson is flagged.
        _check_safety(cfg, errors, mode="demo")
    elif mode == "playback":
        from config import validate_config as _validate
        try:
            _validate(cfg)
        except ConfigError as exc:
            errors.append(str(exc))
        video = cfg.get("camera", {}).get("video_path") or cfg.get("camera", {}).get("source")
        lidar_path = cfg.get("lidar", {}).get("playback_path", "")
        if video and str(video).lower() != "mock" and not Path(str(video)).exists() and str(video).isdigit() is False:
            try:
                if not Path(str(ROOT / str(video))).exists():
                    errors.append(f"Không có video playback: {video}")
            except Exception:
                errors.append(f"Không có video playback: {video}")
        if lidar_path and not Path(str(lidar_path)).exists():
            try:
                if not Path(str(ROOT / str(lidar_path))).exists():
                    errors.append(f"Không có file LiDAR playback: {lidar_path}")
            except Exception:
                errors.append(f"Không có file LiDAR playback: {lidar_path}")
    elif mode in {"camera-only", "lidar-only"}:
        from config import validate_config as _validate
        try:
            _validate(cfg)
        except ConfigError as exc:
            errors.append(str(exc))
        if mode == "camera-only" and not cfg.get("camera", {}).get("enabled", True):
            errors.append("camera-only yêu cầu camera.enabled=true")
        if mode == "lidar-only" and not cfg.get("lidar", {}).get("enabled", True):
            errors.append("lidar-only yêu cầu lidar.enabled=true")
    elif mode in {"fusion-test", "benchmark"}:
        from config import validate_config as _validate
        try:
            _validate(cfg)
        except ConfigError as exc:
            errors.append(str(exc))
    else:
        errors.append(ERR_MODE_NOT_LIVE)
    extras: dict = {}
    if mode == "live":
        normalised_block = gateway_from_config(cfg)
        extras["radar_gateway"] = normalised_block
    for error in errors:
        fail(error)
    write_status(path, errors, extras=extras)
    if errors:
        print(f"[RESULT] Preflight thất bại: {len(errors)} lỗi", file=sys.stderr)
        return 1
    print("[RESULT] Preflight đạt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
