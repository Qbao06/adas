"""Hardware assignment: cameras by vehicle slot, COM/UART, and LiDAR.

The cockpit tab ``Cấu hình phần cứng`` is the operator-facing editor for:

* which physical camera sits at which vehicle position
* which COM / USB-serial port is the Radar Gateway
* LiDAR driver + source

It writes ``camera.sources``, patches ``radar_gateway.{enabled,device,baudrate}``,
and patches ``lidar.{enabled,driver,source,fps,driver_module}`` without wiping
unrelated YAML keys.  Live-mode preflight remains the go-live gate; this
module allows ``CHANGE_ME`` placeholders so the operator can save a partial
assignment.
"""
from __future__ import annotations

import os
import posixpath
import re
import sys
from pathlib import Path
from typing import Any, Mapping

import yaml

from .radar_gateway import (
    DEFAULT_USB_BAUDRATES,
    is_allowed_device,
    is_com_port,
    list_serial_devices,
    radar_gateway_from_config,
    sanitize_device_path,
)
from .validation import ConfigError

CAMERA_SLOTS: tuple[dict[str, Any], ...] = (
    {"id": "front", "label": "Camera trước", "short": "TRƯỚC", "required": True},
    {"id": "left_1", "label": "Camera trái trước", "short": "TRÁI TRƯỚC", "required": False},
    {"id": "left_2", "label": "Camera trái sau", "short": "TRÁI SAU", "required": False},
    {"id": "right_1", "label": "Camera phải trước", "short": "PHẢI TRƯỚC", "required": False},
    {"id": "right_2", "label": "Camera phải sau", "short": "PHẢI SAU", "required": False},
    {"id": "rear", "label": "Camera sau", "short": "SAU", "required": False},
)
CAMERA_SLOT_IDS = tuple(item["id"] for item in CAMERA_SLOTS)
_DRIVER_MODULE_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)*$")
_LIDAR_PLACEHOLDERS = {"mock", "change_me"}
MAX_CAMERA_INDEX = 31


def camera_source_to_device(value: Any) -> str:
    """Flatten a ``camera.sources`` entry (string, int, or mapping) to a device string."""
    if isinstance(value, Mapping):
        raw = value.get("device", value.get("source", ""))
        return "" if raw is None else str(raw).strip()
    if value is None:
        return ""
    return str(value).strip()


def coerce_camera_device(value: Any) -> Any:
    """Store numeric camera indices as int, everything else as a stripped string."""
    if isinstance(value, bool):
        raise ConfigError("Nguồn camera không hợp lệ")
    if isinstance(value, int):
        return value
    text = str(value).strip()
    if text.isdigit():
        return int(text)
    return text


def is_allowed_camera_source(value: Any) -> bool:
    """True for camera index 0-31 or an allowlisted ``/dev/video*`` / ``/dev/v4l/`` path."""
    if isinstance(value, bool):
        return False
    if isinstance(value, int):
        return 0 <= value <= MAX_CAMERA_INDEX
    if not isinstance(value, str):
        return False
    text = value.strip()
    if not text:
        return False
    if text.isdigit():
        return 0 <= int(text) <= MAX_CAMERA_INDEX
    if not is_allowed_device(text):
        return False
    normalized = posixpath.normpath(text.replace("\\", "/"))
    return normalized.startswith("/dev/video") or normalized.startswith("/dev/v4l/")


def normalize_camera_source(value: Any) -> int | str:
    """Return an int index or a sanitised ``/dev`` camera path."""
    if isinstance(value, bool):
        raise ConfigError("Nguồn camera không hợp lệ")
    if isinstance(value, int):
        if not 0 <= value <= MAX_CAMERA_INDEX:
            raise ConfigError("Chỉ số camera phải trong [0, 31]")
        return value
    text = str(value).strip()
    if not text:
        raise ConfigError("Nguồn camera trống")
    if text.isdigit():
        index = int(text)
        if not 0 <= index <= MAX_CAMERA_INDEX:
            raise ConfigError("Chỉ số camera phải trong [0, 31]")
        return index
    if not is_allowed_camera_source(text):
        raise ConfigError(
            "Nguồn camera nằm ngoài allowlist (/dev/video*, /dev/v4l/by-id/*, index 0-31)"
        )
    return sanitize_device_path(text)


def _slot_meta(slot_id: str) -> dict[str, Any]:
    for item in CAMERA_SLOTS:
        if item["id"] == slot_id:
            return item
    raise ConfigError(f"Vị trí camera không hợp lệ: {slot_id}")


def hardware_from_config(config: dict[str, Any]) -> dict[str, Any]:
    """Hydrate the hardware-assignment editor payload from a loaded config dict."""
    camera = config.get("camera") if isinstance(config.get("camera"), dict) else {}
    sources = camera.get("sources") if isinstance(camera.get("sources"), dict) else {}
    slots: dict[str, Any] = {}
    for meta in CAMERA_SLOTS:
        slot_id = meta["id"]
        device = camera_source_to_device(sources.get(slot_id))
        slots[slot_id] = {
            "id": slot_id,
            "label": meta["label"],
            "short": meta["short"],
            "required": bool(meta["required"]),
            "enabled": bool(device) if not meta["required"] else True,
            "device": device,
        }
    if not slots["front"]["device"]:
        fallback = camera.get("source")
        if fallback not in (None, "", "mock"):
            slots["front"]["device"] = camera_source_to_device(fallback)

    lidar = config.get("lidar") if isinstance(config.get("lidar"), dict) else {}
    gateway = radar_gateway_from_config(config)
    return {
        "camera": {
            "enabled": bool(camera.get("enabled", True)),
            "width": int(camera.get("width", 1280) or 1280),
            "height": int(camera.get("height", 720) or 720),
            "fps": int(camera.get("fps", 30) or 30),
            "source": camera.get("source", 0),
            "slots": slots,
        },
        "lidar": {
            "enabled": bool(lidar.get("enabled", True)),
            "driver": str(lidar.get("driver", "mock") or "mock"),
            "source": str(lidar.get("source", "mock") or "mock"),
            "fps": int(lidar.get("fps", 10) or 10),
            "driver_module": str(lidar.get("driver_module", "") or ""),
        },
        "radar_gateway": {
            "enabled": bool(gateway.get("enabled", False)),
            "device": str(gateway.get("device", "") or ""),
            "baudrate": int(gateway.get("baudrate", 115200) or 115200),
        },
        "slots": [dict(item) for item in CAMERA_SLOTS],
    }


def _positive_int(value: Any, name: str) -> int:
    try:
        if isinstance(value, bool):
            raise TypeError
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{name} phải là số nguyên") from exc
    if result <= 0:
        raise ConfigError(f"{name} phải > 0")
    return result


def _validate_lidar_source(source: Any) -> str:
    text = str(source or "").strip()
    if not text:
        raise ConfigError("lidar.source không được trống")
    lowered = text.lower()
    if lowered in _LIDAR_PLACEHOLDERS or "CHANGE_ME" in text.upper():
        return text
    if is_com_port(text) or text.startswith("/dev/"):
        return sanitize_device_path(text)
    posix = text.replace("\\", "/")
    if any(segment == ".." for segment in posix.split("/")):
        raise ConfigError("lidar.source không được chứa ..")
    if posix.startswith("/") or (len(text) > 1 and text[1] == ":"):
        raise ConfigError("lidar.source đường dẫn tuyệt đối không thuộc allowlist")
    forbidden = set(";&|`$<>(){}[]~*?\\\"'\n\r\t")
    if any(ch in forbidden for ch in text):
        raise ConfigError("lidar.source chứa ký tự không hợp lệ")
    return text


def _validate_driver_module(value: Any) -> str:
    text = str(value or "").strip()
    if not text:
        return ""
    if not _DRIVER_MODULE_RE.fullmatch(text):
        raise ConfigError("lidar.driver_module phải là dotted identifier (ví dụ drivers.real_lidar)")
    return text


def _validate_slots(raw_slots: Any, *, camera_enabled: bool) -> dict[str, Any]:
    if raw_slots is None:
        raw_slots = {}
    if not isinstance(raw_slots, dict):
        raise ConfigError("camera.slots phải là object")
    unknown = set(raw_slots) - set(CAMERA_SLOT_IDS)
    if unknown:
        raise ConfigError("Vị trí camera không hợp lệ: " + ", ".join(sorted(unknown)))

    slots: dict[str, Any] = {}
    used: dict[str, str] = {}
    for meta in CAMERA_SLOTS:
        slot_id = meta["id"]
        item = raw_slots.get(slot_id, {})
        if item is None:
            item = {}
        if not isinstance(item, dict):
            raise ConfigError(f"camera.slots.{slot_id} phải là object")
        required = bool(meta["required"])
        enabled = True if required else bool(item.get("enabled", False))
        device_raw = camera_source_to_device(item.get("device", ""))
        if enabled and camera_enabled:
            if not device_raw:
                raise ConfigError(f"{meta['label']}: đã bật nhưng chưa chọn thiết bị")
            normalized = normalize_camera_source(device_raw)
            key = str(normalized)
            if key in used:
                raise ConfigError(
                    f"Thiết bị camera {key} bị trùng giữa {used[key]} và {slot_id}"
                )
            used[key] = slot_id
            device = str(normalized)
        else:
            device = device_raw
            if device and not is_allowed_camera_source(device) and "CHANGE_ME" not in device.upper():
                raise ConfigError(f"{meta['label']}: nguồn camera không hợp lệ")
        slots[slot_id] = {
            "id": slot_id,
            "label": meta["label"],
            "short": meta["short"],
            "required": required,
            "enabled": enabled,
            "device": device,
        }
    return slots


def _validate_gateway_patch(patch: Any) -> dict[str, Any]:
    if patch is None:
        patch = {}
    if not isinstance(patch, dict):
        raise ConfigError("radar_gateway phải là object")
    enabled = bool(patch.get("enabled", False))
    device_raw = str(patch.get("device", "")).strip()
    if enabled and not device_raw:
        raise ConfigError("Cổng Radar Gateway không được trống khi đang bật")
    if device_raw and "CHANGE_ME" not in device_raw.upper():
        device = sanitize_device_path(device_raw)
    else:
        device = device_raw
    baud_raw = patch.get("baudrate", 115200)
    try:
        if isinstance(baud_raw, bool):
            raise TypeError
        baudrate = int(baud_raw)
    except (TypeError, ValueError) as exc:
        raise ConfigError("radar_gateway.baudrate phải là số nguyên") from exc
    if baudrate not in DEFAULT_USB_BAUDRATES:
        raise ConfigError(
            f"radar_gateway.baudrate={baudrate} không thuộc danh sách hỗ trợ: "
            f"{list(DEFAULT_USB_BAUDRATES)}"
        )
    return {"enabled": enabled, "device": device, "baudrate": baudrate}


def validate_hardware(payload: dict[str, Any]) -> dict[str, Any]:
    """Validate the editor payload.  Does not require a fully live-ready config."""
    if not isinstance(payload, dict):
        raise ConfigError("Payload cấu hình phần cứng phải là object")
    camera_in = payload.get("camera", {})
    if not isinstance(camera_in, dict):
        raise ConfigError("camera phải là object")
    camera_enabled = bool(camera_in.get("enabled", True))
    camera = {
        "enabled": camera_enabled,
        "width": _positive_int(camera_in.get("width", 1280), "camera.width"),
        "height": _positive_int(camera_in.get("height", 720), "camera.height"),
        "fps": _positive_int(camera_in.get("fps", 30), "camera.fps"),
        "slots": _validate_slots(camera_in.get("slots"), camera_enabled=camera_enabled),
    }

    lidar_in = payload.get("lidar", {})
    if not isinstance(lidar_in, dict):
        raise ConfigError("lidar phải là object")
    driver = str(lidar_in.get("driver", "mock") or "mock").strip()
    if not driver:
        raise ConfigError("lidar.driver không được trống")
    forbidden = set(";&|`$<>(){}[]~*?\\\"'\n\r\t ")
    if any(ch in forbidden for ch in driver):
        raise ConfigError("lidar.driver chứa ký tự không hợp lệ")
    lidar = {
        "enabled": bool(lidar_in.get("enabled", True)),
        "driver": driver,
        "source": _validate_lidar_source(lidar_in.get("source", "mock")),
        "fps": _positive_int(lidar_in.get("fps", 10), "lidar.fps"),
        "driver_module": _validate_driver_module(lidar_in.get("driver_module", "")),
    }
    return {
        "camera": camera,
        "lidar": lidar,
        "radar_gateway": _validate_gateway_patch(payload.get("radar_gateway")),
        "slots": [dict(item) for item in CAMERA_SLOTS],
    }


def apply_hardware(config: dict[str, Any], payload: dict[str, Any]) -> dict[str, Any]:
    """Write a validated hardware payload into ``config`` in place."""
    normalized = validate_hardware(payload)
    camera = config.setdefault("camera", {})
    if not isinstance(camera, dict):
        camera = {}
        config["camera"] = camera
    camera["enabled"] = normalized["camera"]["enabled"]
    camera["width"] = normalized["camera"]["width"]
    camera["height"] = normalized["camera"]["height"]
    camera["fps"] = normalized["camera"]["fps"]
    sources = dict(camera.get("sources") or {}) if isinstance(camera.get("sources"), dict) else {}
    front_device: Any = None
    for slot_id, slot in normalized["camera"]["slots"].items():
        if slot["enabled"] and slot["device"]:
            stored = coerce_camera_device(slot["device"])
            sources[slot_id] = stored
            if slot_id == "front":
                front_device = stored
        else:
            sources.pop(slot_id, None)
    camera["sources"] = sources
    if front_device is not None:
        camera["source"] = front_device

    lidar = config.setdefault("lidar", {})
    if not isinstance(lidar, dict):
        lidar = {}
        config["lidar"] = lidar
    lidar["enabled"] = normalized["lidar"]["enabled"]
    lidar["driver"] = normalized["lidar"]["driver"]
    lidar["source"] = normalized["lidar"]["source"]
    lidar["fps"] = normalized["lidar"]["fps"]
    lidar["driver_module"] = normalized["lidar"]["driver_module"]

    gateway = config.get("radar_gateway")
    if not isinstance(gateway, dict):
        gateway = {}
        config["radar_gateway"] = gateway
    patch = normalized["radar_gateway"]
    gateway["enabled"] = patch["enabled"]
    gateway["device"] = patch["device"]
    gateway["baudrate"] = patch["baudrate"]

    modules = config.get("side_sensor_modules")
    if isinstance(modules, dict) and isinstance(modules.get("modules"), dict):
        for slot_id, slot in normalized["camera"]["slots"].items():
            item = modules["modules"].get(slot_id)
            if isinstance(item, dict) and isinstance(item.get("camera"), dict) and slot["device"]:
                item["camera"]["device"] = slot["device"]
    return config


def save_hardware(config_path: Path, payload: dict[str, Any]) -> dict[str, Any]:
    """Atomically persist the hardware assignment.  Returns the hydrated editor payload."""
    path = Path(config_path).resolve()
    current = yaml.safe_load(path.read_text(encoding="utf-8")) if path.exists() else {}
    if not isinstance(current, dict):
        raise ConfigError("File cấu hình hiện tại không hợp lệ")
    apply_hardware(current, payload)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(
        yaml.safe_dump(current, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    os.replace(tmp, path)
    return hardware_from_config(current)


def _list_windows_com_ports() -> list[str]:
    """Return existing COMn device names on Windows; empty on other platforms."""
    if sys.platform != "win32":
        return []
    found: list[str] = []
    try:
        import ctypes

        query = ctypes.windll.kernel32.QueryDosDeviceW
        buf = ctypes.create_unicode_buffer(512)
        for index in range(1, 257):
            name = f"COM{index}"
            if query(name, buf, 512):
                found.append(name)
    except Exception:
        return []
    return found


def list_hardware_devices() -> dict[str, Any]:
    """Enumerate cameras and serial ports without opening them."""
    serial = list_serial_devices()
    cameras = sorted(set(list(serial.get("video") or []) + list(serial.get("v4l_by_id") or [])))
    return {
        "cameras": cameras,
        "camera_indices": [str(i) for i in range(8)],
        "serial": {
            "by_id": list(serial.get("by_id") or []),
            "ttyacm": list(serial.get("ttyacm") or []),
            "ttyusb": list(serial.get("ttyusb") or []),
            "com": _list_windows_com_ports(),
        },
        "platform": sys.platform,
    }


def inspect_serial_device(device: str) -> dict[str, Any]:
    """Presence check for a COM / USB-serial path.  Never opens the port."""
    text = (device or "").strip()
    if not text:
        return {"ok": False, "error": "device trống", "device": text}
    if "CHANGE_ME" in text.upper():
        return {"ok": False, "error": "Cổng còn placeholder CHANGE_ME", "device": text, "placeholder": True}
    if not is_allowed_device(text):
        return {"ok": False, "error": "Cổng nằm ngoài allowlist (/dev/... hoặc COMx)", "device": text}
    cleaned = sanitize_device_path(text)
    if is_com_port(cleaned):
        present = cleaned in _list_windows_com_ports()
        return {"ok": present, "device": cleaned, "present": present, "kind": "com"}
    path = Path(cleaned)
    present = path.exists()
    readable = bool(present and os.access(cleaned, os.R_OK))
    return {
        "ok": present,
        "device": cleaned,
        "present": present,
        "readable": readable,
        "kind": "unix",
    }


def inspect_camera_device(device: str) -> dict[str, Any]:
    """Presence check for a camera source.  Does not grab a frame."""
    text = "" if device is None else str(device).strip()
    if not text:
        return {"ok": False, "error": "device trống", "device": text}
    if not is_allowed_camera_source(text):
        return {"ok": False, "error": "Nguồn camera không hợp lệ", "device": text}
    source = normalize_camera_source(text)
    if isinstance(source, int):
        return {
            "ok": True,
            "device": str(source),
            "present": True,
            "kind": "index",
            "note": "Index camera — bấm Kiểm tra để đọc một frame",
        }
    path = Path(source)
    present = path.exists()
    return {"ok": present, "device": source, "present": present, "kind": "v4l"}


def inspect_lidar_source(source: str) -> dict[str, Any]:
    """Syntactic + presence check for a LiDAR source.  Never loads a driver."""
    text = (source or "").strip()
    if not text:
        return {"ok": False, "error": "lidar.source trống", "device": text}
    if text.lower() in _LIDAR_PLACEHOLDERS or "CHANGE_ME" in text.upper():
        return {
            "ok": False,
            "error": "LiDAR còn mock/CHANGE_ME",
            "device": text,
            "placeholder": True,
        }
    if is_com_port(text) or text.startswith("/dev/"):
        report = inspect_serial_device(text)
        report["kind"] = "serial"
        return report
    try:
        cleaned = _validate_lidar_source(text)
    except ConfigError as exc:
        return {"ok": False, "error": str(exc), "device": text}
    return {
        "ok": True,
        "device": cleaned,
        "kind": "path",
        "note": "Đường dẫn tương đối — preflight sẽ kiểm tra file khi chạy Live",
    }


__all__ = [
    "CAMERA_SLOT_IDS",
    "CAMERA_SLOTS",
    "apply_hardware",
    "camera_source_to_device",
    "coerce_camera_device",
    "hardware_from_config",
    "inspect_camera_device",
    "inspect_lidar_source",
    "inspect_serial_device",
    "is_allowed_camera_source",
    "list_hardware_devices",
    "normalize_camera_source",
    "save_hardware",
    "validate_hardware",
]
