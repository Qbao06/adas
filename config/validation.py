"""Central runtime configuration validation."""
from __future__ import annotations

from typing import Any


class ConfigError(ValueError):
    pass


def _positive(value, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{name} phải là số") from exc
    if result <= 0:
        raise ConfigError(f"{name} phải > 0")
    return result


def validate_config(config: dict) -> dict:
    modes = {"demo", "live", "playback", "camera-only", "lidar-only", "fusion-test", "benchmark"}
    mode = str(config.get("mode", "demo")).lower()
    if mode not in modes:
        raise ConfigError(f"mode không hợp lệ: {mode}")
    if config.get("safety", {}).get("production_only", False) and mode != "live":
        raise ConfigError("Bản production chỉ cho phép mode: live")
    camera = config.get("camera", {})
    _positive(camera.get("width", 0), "camera.width")
    _positive(camera.get("height", 0), "camera.height")
    _positive(camera.get("fps", 0), "camera.fps")
    lidar = config.get("lidar", {})
    _positive(lidar.get("fps", 0), "lidar.fps")
    vision = config.get("vision", {})
    confidence = float(vision.get("confidence", 0.45))
    if not 0 <= confidence <= 1:
        raise ConfigError("vision.confidence phải nằm trong [0,1]")
    fusion = config.get("fusion", {})
    alpha = float(fusion.get("ema_alpha", 0.35))
    if not 0 < alpha <= 1:
        raise ConfigError("fusion.ema_alpha phải nằm trong (0,1]")
    bbox_scale = float(fusion.get("bbox_scale", 0.85))
    if not 0 < bbox_scale <= 1:
        raise ConfigError("fusion.bbox_scale phải nằm trong (0,1]")
    sync = config.get("synchronization", {})
    _positive(sync.get("max_difference_ms", 0), "synchronization.max_difference_ms")
    warning = config.get("warning", {})
    warning_mode = str(warning.get("mode", "distance")).lower()
    if warning_mode not in {"distance", "ttc"}:
        raise ConfigError("warning.mode chỉ nhận distance hoặc ttc")
    if warning_mode == "distance":
        emergency = _positive(warning.get("emergency_distance_m"), "warning.emergency_distance_m")
        danger = _positive(warning.get("danger_distance_m"), "warning.danger_distance_m")
        caution = _positive(warning.get("caution_distance_m"), "warning.caution_distance_m")
        if not emergency <= danger <= caution:
            raise ConfigError("Ngưỡng phải thỏa emergency <= danger <= caution")
    if mode == "live":
        if not camera.get("enabled", True) or not lidar.get("enabled", True):
            raise ConfigError("live yêu cầu cả camera và LiDAR enabled")
        if config.get("safety", {}).get("allow_mock", True):
            raise ConfigError("live yêu cầu safety.allow_mock: false")
        if str(camera.get("source", "")).lower() == "mock":
            raise ConfigError("live từ chối camera.source=mock")
        if str(lidar.get("driver", lidar.get("source", ""))).lower() in {"", "mock", "change_me"}:
            raise ConfigError("live từ chối LiDAR mock/CHANGE_ME: cấu hình driver thật")
        if config.get("safety", {}).get("is_demo_calibration", True):
            raise ConfigError("live từ chối calibration demo: is_demo_calibration phải false")
        radar = config.get("side_radar", {})
        if radar.get("enabled", False):
            if not str(radar.get("device", "")).strip():
                raise ConfigError("side_radar.device không được để trống")
            _positive(radar.get("baudrate", 0), "side_radar.baudrate")
            _positive(radar.get("stale_after_seconds", 0), "side_radar.stale_after_seconds")
            sources = camera.get("sources", {})
            missing = [name for name in ("front", "left", "right") if name not in sources]
            if missing:
                raise ConfigError(
                    "Radar tự chuyển camera yêu cầu camera.sources: " + ", ".join(missing)
                )
        modules = config.get("side_sensor_modules")
        if modules:
            # Local import avoids a circular import with side_modules.ConfigError.
            from .side_modules import validate_side_modules
            validate_side_modules(modules)
        # radar_gateway validation. Imported lazily for the same reason.
        from .radar_gateway import (
            RADAR_IDS,
            migrate_legacy_config,
            radar_gateway_from_config,
            validate_radar_gateway,
        )
        gateway_value = config.get("radar_gateway")
        if isinstance(gateway_value, dict):
            normalised = validate_radar_gateway(gateway_value, mode=mode)
            config["radar_gateway"] = normalised
            _verify_gateway_priority(normalised.get("priority", list(RADAR_IDS)))
        # Migration: if side_sensor_modules is enabled but no radar_gateway is
        # supplied, derive a default gateway block from the legacy module
        # config.  This keeps the runtime contract uniform without forcing
        # operators to duplicate their configuration.
        if (
            gateway_value is None
            and isinstance(modules, dict)
            and modules.get("enabled", False)
        ):
            config["radar_gateway"] = migrate_legacy_config({"side_sensor_modules": modules})
    return config


def _verify_gateway_priority(priority: Any) -> None:
    if isinstance(priority, (list, tuple)) and priority:
        return
    raise ConfigError("radar_gateway.priority phải là danh sách không rỗng")


__all__ = ["ConfigError", "validate_config"]
