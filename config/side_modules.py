"""Validation and persistence helpers for configurable side sensor modules."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml

from .validation import ConfigError

MODULE_IDS = ("left_1", "left_2", "right_1", "right_2")
POSITIONS = {"front", "rear"}
BAUDRATES = {9600, 19200, 38400, 57600, 115200}


def default_side_modules() -> dict[str, Any]:
    return {
        "enabled": True,
        "max_modules_per_side": 2,
        "camera_hold_seconds": 1.2,
        "modules": {
            module_id: {
                "enabled": module_id in {"left_1", "right_1"},
                "side": module_id.split("_", 1)[0],
                "position": "front" if module_id.endswith("_1") else "rear",
                "camera": {"device": "", "width": 1280, "height": 720, "fps": 30},
                "radar": {
                    "enabled": True,
                    "device": "",
                    "baudrate": 115200,
                    "stale_after_seconds": 0.6,
                    "max_distance_m": 30.0,
                    "warning_distance_m": 8.0,
                },
            }
            for module_id in MODULE_IDS
        },
    }


def modules_from_config(config: dict[str, Any]) -> dict[str, Any]:
    """Return the new schema, migrating legacy left/right settings in memory."""
    raw = config.get("side_sensor_modules")
    defaults = default_side_modules()
    if isinstance(raw, dict) and raw:
        result = {**defaults, **{k: v for k, v in raw.items() if k != "modules"}}
        supplied = raw.get("modules", {})
        result["modules"] = {}
        for module_id in MODULE_IDS:
            base = defaults["modules"][module_id]
            item = supplied.get(module_id, {}) if isinstance(supplied, dict) else {}
            result["modules"][module_id] = {
                **base,
                **item,
                "camera": {**base["camera"], **item.get("camera", {})},
                "radar": {**base["radar"], **item.get("radar", {})},
            }
        return result

    # Legacy: one dual-side ESP32 plus left/right cameras.
    sources = config.get("camera", {}).get("sources", {})
    radar = config.get("side_radar", {})
    for module_id, camera_id in (("left_1", "left"), ("right_1", "right")):
        item = defaults["modules"][module_id]
        item["enabled"] = bool(radar.get("enabled", False) and camera_id in sources)
        item["camera"]["device"] = str(sources.get(camera_id, ""))
        item["radar"].update({
            "enabled": bool(radar.get("enabled", False)),
            "device": str(radar.get("device", "")),
            "baudrate": int(radar.get("baudrate", 115200)),
            "stale_after_seconds": float(radar.get("stale_after_seconds", 0.6)),
            "max_distance_m": float(radar.get("max_distance_m", 30.0)),
        })
    defaults["camera_hold_seconds"] = float(radar.get("camera_hold_seconds", 1.2))
    return defaults


def validate_side_modules(value: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigError("side_sensor_modules phải là object")
    normalized = modules_from_config({"side_sensor_modules": value})
    if int(normalized.get("max_modules_per_side", 2)) != 2:
        raise ConfigError("Mỗi bên chỉ hỗ trợ tối đa 2 module")
    hold = float(normalized.get("camera_hold_seconds", 1.2))
    if not 0 <= hold <= 10:
        raise ConfigError("camera_hold_seconds phải nằm trong [0,10]")

    used_cameras: dict[str, str] = {}
    used_uarts: dict[str, str] = {}
    enabled_per_side = {"left": 0, "right": 0}
    for module_id, item in normalized["modules"].items():
        expected_side = module_id.split("_", 1)[0]
        if item.get("side") != expected_side:
            raise ConfigError(f"{module_id}.side phải là {expected_side}")
        if item.get("position") not in POSITIONS:
            raise ConfigError(f"{module_id}.position chỉ nhận front hoặc rear")
        if not item.get("enabled", False):
            continue
        enabled_per_side[expected_side] += 1
        camera = item["camera"]
        radar = item["radar"]
        camera_device = str(camera.get("device", "")).strip()
        if not camera_device:
            raise ConfigError(f"{module_id}: camera.device không được trống")
        if camera_device in used_cameras:
            raise ConfigError(f"Camera {camera_device} bị trùng giữa {used_cameras[camera_device]} và {module_id}")
        used_cameras[camera_device] = module_id
        for field in ("width", "height", "fps"):
            if int(camera.get(field, 0)) <= 0:
                raise ConfigError(f"{module_id}.camera.{field} phải > 0")
        if radar.get("enabled", True):
            uart = str(radar.get("device", "")).strip()
            if not uart:
                raise ConfigError(f"{module_id}: radar.device không được trống")
            if uart in used_uarts:
                raise ConfigError(f"UART {uart} bị trùng giữa {used_uarts[uart]} và {module_id}")
            used_uarts[uart] = module_id
            baudrate = int(radar.get("baudrate", 0))
            if baudrate not in BAUDRATES:
                raise ConfigError(f"{module_id}: baudrate không được hỗ trợ")
            maximum = float(radar.get("max_distance_m", 0))
            warning = float(radar.get("warning_distance_m", 0))
            if maximum <= 0 or not 0 < warning <= maximum:
                raise ConfigError(f"{module_id}: khoảng cảnh báo phải > 0 và <= khoảng tối đa")
            if float(radar.get("stale_after_seconds", 0)) <= 0:
                raise ConfigError(f"{module_id}: stale_after_seconds phải > 0")
    if any(count > 2 for count in enabled_per_side.values()):
        raise ConfigError("Mỗi bên chỉ được bật tối đa 2 module")
    return normalized


def apply_side_modules(config: dict[str, Any], modules: dict[str, Any]) -> dict[str, Any]:
    """Apply module schema and derive camera sources consumed by the runtime."""
    normalized = validate_side_modules(modules)
    config["side_sensor_modules"] = normalized
    sources = dict(config.setdefault("camera", {}).get("sources", {}))
    for name in MODULE_IDS:
        sources.pop(name, None)
    for module_id, item in normalized["modules"].items():
        if item["enabled"]:
            sources[module_id] = dict(item["camera"])
    config["camera"]["sources"] = sources
    return config


def save_side_modules(config_path: Path, modules: dict[str, Any]) -> dict[str, Any]:
    """Atomically update only the module section of a YAML configuration."""
    path = Path(config_path).resolve()
    current = yaml.safe_load(path.read_text(encoding="utf-8")) if path.exists() else {}
    if not isinstance(current, dict):
        raise ConfigError("File cấu hình hiện tại không hợp lệ")
    normalized = validate_side_modules(modules)
    apply_side_modules(current, normalized)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(yaml.safe_dump(current, allow_unicode=True, sort_keys=False), encoding="utf-8")
    os.replace(tmp, path)
    return normalized


def inspect_module_devices(item: dict[str, Any]) -> dict[str, Any]:
    """Read-only device presence check; it never opens or locks hardware."""
    camera = str(item.get("camera", {}).get("device", "")).strip()
    uart = str(item.get("radar", {}).get("device", "")).strip()
    return {
        "camera": {"device": camera, "present": bool(camera and Path(camera).exists())},
        "radar": {
            "device": uart,
            "present": bool(uart and Path(uart).exists()),
            "readable": bool(uart and os.access(uart, os.R_OK | os.W_OK)),
        },
    }


__all__ = [
    "MODULE_IDS", "apply_side_modules", "default_side_modules", "inspect_module_devices",
    "modules_from_config", "save_side_modules", "validate_side_modules",
]
