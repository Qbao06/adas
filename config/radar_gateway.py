"""Schema, validation, persistence, and atomic save helpers for the new
single-Gateway / four-LD2410C radar architecture.

This module is the Prompt-4 contract for the Jetson-side radar integration:

* One ESP32 Gateway presents *all* four HLK-LD2410C radars over a single USB
  Serial UART running JSON protocol v2.
* ESP32 B is a Remote bridge feeding two radars back to ESP32 A via ESP-NOW.
* ``radar_gateway`` is the canonical block; the older ``side_sensor_modules``
  block is supported for read-only migration but is **disabled by default**.
* Hard limits enforced by the schema:
    - ``warning_distance_m`` cannot exceed 5.0 m (LD2410C detection range).
    - Exactly four radars (``left_front``, ``left_rear``, ``right_front``,
      ``right_rear``).
    - Four camera mappings.
    - Atomic save: only the ``radar_gateway`` block is mutated.
"""
from __future__ import annotations

import os
import posixpath
import re
from pathlib import Path
from typing import Any, Iterable

import yaml

from .validation import ConfigError

RADAR_IDS = ("left_front", "left_rear", "right_front", "right_rear")
LD2410_MAX_DETECTION_M = 5.0  # hardware limit; do not allow exceeding it.
DEFAULT_USB_BAUDRATES = (9600, 19200, 38400, 57600, 115200, 230400, 256000)
PROTOCOL_VERSIONS = (2,)
MAX_LINE_BYTES = 4096
# Dead constant kept for API backwards-compatibility.  The actual UART
# line cap on the Jetson side is MAX_LINE_BYTES (4096).  The firmware
# mirrors this via JETSON_JSON_LINE_MAX in jetson_json_transport.h.
MAX_GATEWAY_SERIAL_BYTES = 2048

LEGACY_CAMERA_IDS = ("left_1", "left_2", "right_1", "right_2")


def default_radar_gateway() -> dict[str, Any]:
    """Return a safe default ``radar_gateway`` block (radars enabled)."""
    return {
        "enabled": True,
        "device": "/dev/serial/by-id/CHANGE_ME",
        "baudrate": 115200,
        "protocol_version": 2,
        "stale_after_seconds": 0.6,
        "camera_hold_seconds": 1.2,
        "max_distance_m": LD2410_MAX_DETECTION_M,
        "radars": {
            radar_id: {
                "enabled": True,
                "camera": LEGACY_CAMERA_IDS[index],
                "warning_distance_m": LD2410_MAX_DETECTION_M,
            }
            for index, radar_id in enumerate(RADAR_IDS)
        },
    }


# ---------------------------------------------------------------------------
# Path-traversal & allowlist safety
# ---------------------------------------------------------------------------

_DEVICE_ALLOWLIST = (
    "/dev/serial/by-id/",
    "/dev/ttyACM",
    "/dev/ttyUSB",
    "/dev/v4l/by-id/",
    "/dev/video",
)
_COM_PORT_RE = re.compile(r"^COM(\d+)$", re.IGNORECASE)


def is_com_port(path: str) -> bool:
    """True for Windows serial names COM1 .. COM256 (case-insensitive)."""
    if not isinstance(path, str):
        return False
    match = _COM_PORT_RE.fullmatch(path.strip())
    if not match:
        return False
    digits = match.group(1)
    if digits.startswith("0"):
        return False
    return 1 <= int(digits) <= 256


def is_allowed_device(path: str) -> bool:
    """Return ``True`` when ``path`` is inside the read-only device allowlist.

    Beyond the prefix allowlist the guard also rejects:

    * any control character (``< 0x20``), NUL (``\\x00``) or DEL (``\\x7f``);
    * shell metacharacters (``;``, ``&``, ``|``, ``$``, ``` ``, ``\\n``, ``\\r``,
      tabs, spaces, ``<``, ``>``, ``(``, ``)``, ``{``, ``}``, ``*``, ``?``,
      ``[``, ``]``, ``~``);
    * empty paths or paths that are not absolute;
    * any segment that resolves to ``..`` after normalisation;
    * any prefix outside the explicit device-prefix list.
    """
    if not isinstance(path, str):
        return False
    cleaned = path.strip()
    if not cleaned:
        return False
    if any(ord(c) < 0x20 or ord(c) == 0x7F for c in cleaned):
        return False
    forbidden = set(";&|`$<>(){}[]~*?\\\"'\n\r\t ")
    if any(ch in forbidden for ch in cleaned):
        return False
    if any(segment == ".." for segment in cleaned.replace("\\", "/").split("/")):
        return False
    if is_com_port(cleaned):
        return True
    # Device paths are Linux /dev nodes. os.path.normpath on Windows would
    # turn "/dev/ttyACM0" into "\\dev\\ttyACM0" and reject a valid Jetson path.
    normalized = posixpath.normpath(cleaned.replace("\\", "/"))
    if not normalized.startswith("/dev/"):
        return False
    return any(normalized.startswith(prefix) for prefix in _DEVICE_ALLOWLIST)


def sanitize_device_path(path: str) -> str:
    """Coerce a client-supplied string into a safe canonical device path."""
    cleaned = (path or "").strip()
    if not is_allowed_device(cleaned):
        raise ConfigError(
            "Đường dẫn thiết bị không hợp lệ hoặc nằm ngoài allowlist (/dev/... hoặc COMx)."
        )
    if is_com_port(cleaned):
        return cleaned.upper()
    return posixpath.normpath(cleaned.replace("\\", "/"))


# ---------------------------------------------------------------------------
# Block extraction & validation
# ---------------------------------------------------------------------------


def radar_gateway_from_config(config: dict[str, Any]) -> dict[str, Any]:
    """Return the ``radar_gateway`` block, hydrating defaults for any missing
    field.  Never mutates the input mapping."""
    defaults = default_radar_gateway()
    raw = config.get("radar_gateway")
    if not isinstance(raw, dict) or not raw:
        return defaults
    merged = {**defaults, **raw}
    radars = {**defaults["radars"]}
    supplied_radars = raw.get("radars", {})
    if isinstance(supplied_radars, dict):
        for radar_id in RADAR_IDS:
            base = radars[radar_id]
            item = supplied_radars.get(radar_id)
            if isinstance(item, dict):
                radars[radar_id] = {
                    **base,
                    **{k: v for k, v in item.items() if k != "camera"},
                    "camera": item.get("camera", base["camera"]),
                }
            else:
                radars[radar_id] = dict(base)
    merged["radars"] = radars
    if merged.get("protocol_version") not in PROTOCOL_VERSIONS:
        merged["protocol_version"] = 2
    return merged


def validate_radar_gateway(value: Any, *, mode: str = "live") -> dict[str, Any]:
    """Validate the ``radar_gateway`` block.  ``mode`` is used for stricter
    rules in live preflight."""
    if not isinstance(value, dict):
        raise ConfigError("radar_gateway phải là object")

    enabled = bool(value.get("enabled", True))
    defaults = default_radar_gateway()
    normalized: dict[str, Any] = {**defaults, **{k: v for k, v in value.items() if k != "radars"}}

    device_raw = str(value.get("device", defaults["device"])).strip()
    if enabled and not device_raw:
        raise ConfigError(
            "radar_gateway.device không được trống khi enabled"
        )
    if "CHANGE_ME" in device_raw.upper():
        if enabled and mode == "live":
            raise ConfigError(
                "radar_gateway.device còn placeholder CHANGE_ME; live không được phép"
            )
        device = device_raw or defaults["device"]
    else:
        device = sanitize_device_path(device_raw) if device_raw else device_raw
    normalized["device"] = device

    baud_raw = value.get("baudrate", defaults["baudrate"])
    if isinstance(baud_raw, bool) or not isinstance(baud_raw, int):
        raise ConfigError("radar_gateway.baudrate phải là số nguyên")
    baudrate = baud_raw
    if baudrate not in DEFAULT_USB_BAUDRATES:
        raise ConfigError(
            f"radar_gateway.baudrate={baudrate} không thuộc danh sách hỗ trợ: "
            f"{list(DEFAULT_USB_BAUDRATES)}"
        )
    normalized["baudrate"] = baudrate

    pv_raw = value.get("protocol_version", defaults["protocol_version"])
    if isinstance(pv_raw, bool) or not isinstance(pv_raw, int):
        raise ConfigError("radar_gateway.protocol_version phải là số nguyên")
    protocol_version = pv_raw
    if protocol_version not in PROTOCOL_VERSIONS:
        raise ConfigError(
            f"radar_gateway.protocol_version={protocol_version} không được hỗ trợ; "
            f"chỉ chấp nhận {list(PROTOCOL_VERSIONS)}."
        )
    normalized["protocol_version"] = protocol_version

    try:
        stale = float(value.get("stale_after_seconds", defaults["stale_after_seconds"]))
    except (TypeError, ValueError) as exc:
        raise ConfigError("radar_gateway.stale_after_seconds phải là số") from exc
    if stale <= 0:
        raise ConfigError("radar_gateway.stale_after_seconds phải > 0")
    normalized["stale_after_seconds"] = stale

    try:
        hold = float(value.get("camera_hold_seconds", defaults["camera_hold_seconds"]))
    except (TypeError, ValueError) as exc:
        raise ConfigError("radar_gateway.camera_hold_seconds phải là số") from exc
    if not 0 <= hold <= 10:
        raise ConfigError("radar_gateway.camera_hold_seconds phải nằm trong [0,10]")
    normalized["camera_hold_seconds"] = hold

    try:
        max_distance = float(value.get("max_distance_m", defaults["max_distance_m"]))
    except (TypeError, ValueError) as exc:
        raise ConfigError("radar_gateway.max_distance_m phải là số") from exc
    if max_distance <= 0 or max_distance > LD2410_MAX_DETECTION_M:
        raise ConfigError(
            f"radar_gateway.max_distance_m={max_distance} phải trong (0, "
            f"{LD2410_MAX_DETECTION_M}] (LD2410C)."
        )
    normalized["max_distance_m"] = max_distance

    supplied = value.get("radars", {})
    if not isinstance(supplied, dict):
        raise ConfigError("radar_gateway.radars phải là object")
    radars: dict[str, Any] = {}
    seen_cameras: dict[str, str] = {}
    for radar_id in RADAR_IDS:
        item = supplied.get(radar_id, {}) if isinstance(supplied.get(radar_id), dict) else {}
        base = defaults["radars"][radar_id]
        merged = {**base, **item}
        camera = str(merged.get("camera", base["camera"])).strip()
        if enabled and not camera:
            raise ConfigError(
                f"radar_gateway.radars.{radar_id}.camera không được trống"
            )
        if merged.get("enabled", False):
            if camera in seen_cameras:
                raise ConfigError(
                    f"radar_gateway.radars.{radar_id}.camera='{camera}' đã được dùng bởi "
                    f"{seen_cameras[camera]}"
                )
            seen_cameras[camera] = radar_id
        try:
            warning = float(merged.get("warning_distance_m", base["warning_distance_m"]))
        except (TypeError, ValueError) as exc:
            raise ConfigError(
                f"radar_gateway.radars.{radar_id}.warning_distance_m phải là số"
            ) from exc
        if warning <= 0:
            raise ConfigError(
                f"radar_gateway.radars.{radar_id}.warning_distance_m phải > 0"
            )
        if warning > LD2410_MAX_DETECTION_M:
            raise ConfigError(
                f"radar_gateway.radars.{radar_id}.warning_distance_m={warning} vượt quá "
                f"{LD2410_MAX_DETECTION_M} m (giới hạn phần cứng LD2410C)."
            )
        merged["warning_distance_m"] = warning
        merged["camera"] = camera
        merged["enabled"] = bool(merged.get("enabled", True))
        radars[radar_id] = merged
    normalized["radars"] = radars

    if enabled and not any(item.get("enabled", False) for item in radars.values()):
        raise ConfigError("radar_gateway.enabled nhưng không có radar nào enabled")

    return normalized


def apply_radar_gateway(config: dict[str, Any], block: dict[str, Any]) -> dict[str, Any]:
    """Validate and write the gateway block back into ``config``."""
    normalised = validate_radar_gateway(block)
    config["radar_gateway"] = normalised
    return config


def save_radar_gateway(config_path: Path, block: dict[str, Any]) -> dict[str, Any]:
    """Persist ``radar_gateway`` block back to YAML via os.replace()."""
    path = Path(config_path).resolve()
    current = yaml.safe_load(path.read_text(encoding="utf-8")) if path.exists() else {}
    if not isinstance(current, dict):
        raise ConfigError("File cấu hình hiện tại không hợp lệ")
    normalised = validate_radar_gateway(block)
    current["radar_gateway"] = normalised
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(
        yaml.safe_dump(current, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    os.replace(tmp, path)
    return normalised


# ---------------------------------------------------------------------------
# Device enumeration
# ---------------------------------------------------------------------------


def gateway_from_config(config: dict[str, Any]) -> dict[str, Any]:
    """Alias for :func:`radar_gateway_from_config` used by the dashboard."""
    return radar_gateway_from_config(config)


# ---------------------------------------------------------------------------
# Migration from the legacy per-side ``side_sensor_modules`` schema.
# ---------------------------------------------------------------------------

_LEGACY_RADAR_TO_CAMERA = {
    "left_1": ("left_front", "left"),
    "left_2": ("left_rear", "left"),
    "right_1": ("right_front", "right"),
    "right_2": ("right_rear", "right"),
}


def migrate_legacy_config(config: dict[str, Any]) -> dict[str, Any]:
    """Derive a default ``radar_gateway`` block from a legacy schema.

    The Prompt-4 contract states that operators with an existing
    ``side_sensor_modules`` block can let the runtime build a sensible
    four-radar Gateway block on the fly.  This helper returns a fresh
    dict derived from the legacy modules' cameras and radar devices so
    the operator does not have to duplicate the configuration.
    """
    modules = (config or {}).get("side_sensor_modules") or {}
    if not isinstance(modules, dict):
        return default_radar_gateway()
    block = default_radar_gateway()
    enabled_radars = 0
    for module_id, item in modules.get("modules", {}).items():
        if not isinstance(item, dict):
            continue
        if not item.get("enabled", False):
            continue
        mapping = _LEGACY_RADAR_TO_CAMERA.get(module_id)
        if mapping is None:
            continue
        radar_id, side = mapping
        radar_cfg = item.get("radar", {}) or {}
        radar_block = {
            "enabled": bool(radar_cfg.get("enabled", True)),
            "camera": module_id,
            "warning_distance_m": float(
                radar_cfg.get("warning_distance_m", LD2410_MAX_DETECTION_M)
            ),
        }
        block["radars"][radar_id] = radar_block
        if radar_block["enabled"]:
            enabled_radars += 1
    if enabled_radars == 0:
        return block
    return block


def list_serial_devices(root: Path = Path("/")) -> dict[str, list[str]]:
    """Enumerate /dev device paths under well-known prefixes."""
    out = {label: [] for label in ("by_id", "ttyacm", "ttyusb", "v4l_by_id", "video")}
    try:
        entries = list(os.scandir("/dev"))
    except OSError:
        entries = []
    for entry in entries:
        name = entry.name
        path = f"/dev/{name}"
        if not is_allowed_device(path):
            continue
        if name.startswith("ttyACM"):
            out["ttyacm"].append(path)
        elif name.startswith("ttyUSB"):
            out["ttyusb"].append(path)
        elif name.startswith("video"):
            out["video"].append(path)
        elif "/by-id/" in path:
            if "v4l" in path:
                out["v4l_by_id"].append(path)
            else:
                out["by_id"].append(path)
    for directory, key in (
        ("/dev/serial/by-id", "by_id"),
        ("/dev/v4l/by-id", "v4l_by_id"),
    ):
        try:
            for entry in os.scandir(directory):
                path = f"{directory}/{entry.name}"
                if is_allowed_device(path):
                    out[key].append(path)
        except OSError:
            continue
    for key in out:
        out[key] = sorted(set(out[key]))
    return out


__all__ = [
    "RADAR_IDS",
    "LEGACY_CAMERA_IDS",
    "LD2410_MAX_DETECTION_M",
    "PROTOCOL_VERSIONS",
    "DEFAULT_USB_BAUDRATES",
    "MAX_LINE_BYTES",
    "MAX_GATEWAY_SERIAL_BYTES",
    "default_radar_gateway",
    "gateway_from_config",
    "migrate_legacy_config",
    "radar_gateway_from_config",
    "validate_radar_gateway",
    "apply_radar_gateway",
    "save_radar_gateway",
    "is_allowed_device",
    "is_com_port",
    "sanitize_device_path",
    "list_serial_devices",
]
