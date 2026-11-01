"""Default configuration merged with user configuration.

The loader is intentionally defensive: missing keys are filled with safe
defaults and the user gets a warning instead of a crash. This way the demo
mode can run even when the YAML file is incomplete.
"""
from __future__ import annotations

import copy
import logging
from pathlib import Path
from typing import Any

import yaml

LOGGER = logging.getLogger(__name__)


def _default_config() -> dict[str, Any]:
    """Configuration values used when the user file does not provide them."""
    return {
        "mode": "demo",
        "project_root": ".",
        "outputs": {
            "directory": "output",
            "video_path": "output/result.mp4",
            "log_directory": "logs",
            "log_level": "INFO",
        },
        "camera": {
            "source": 0,
            "sources": {},
            "video_path": "",
            "width": 1280,
            "height": 720,
            "fps": 30,
            "reconnect_seconds": 2.0,
            "loop_video": False,
            "enabled": True,
            "shared_usb_hub": False,
        },
        "side_radar": {
            "enabled": False,
            "device": "/dev/ttyACM0",
            "baudrate": 115200,
            "stale_after_seconds": 0.6,
            "camera_hold_seconds": 1.2,
            "max_distance_m": 30.0,
        },
        "side_sensor_modules": {},
        "radar_gateway": {
            "enabled": False,
            "device": "/dev/serial/by-id/CHANGE_ME",
            "baudrate": 115200,
            "protocol_version": 2,
            "stale_after_seconds": 0.6,
            "reconnect_seconds": 2.0,
            "camera_hold_seconds": 1.2,
            "max_line_bytes": 4096,
            "priority": ["left_front", "left_rear", "right_front", "right_rear"],
            "radars": {
                "left_front": {"enabled": True, "camera": "left_1", "warning_distance_m": 5.0},
                "left_rear": {"enabled": True, "camera": "left_2", "warning_distance_m": 5.0},
                "right_front": {"enabled": True, "camera": "right_1", "warning_distance_m": 5.0},
                "right_rear": {"enabled": True, "camera": "right_2", "warning_distance_m": 5.0},
            },
        },
        "lidar": {
            "source": "mock",
            "driver": "mock",
            "driver_module": "",
            "playback_path": "",
            "playback_format": "auto",
            "fps": 10,
            "frame_timeout_seconds": 5.0,
            "mock_objects": [
                [12.0, 0.0, 0.0],
                [18.0, -3.5, 0.0],
                [22.0, 3.0, 0.0],
            ],
            "mock_seed": 7,
            "mock_jitter": [0.25, 0.25, 0.35],
            "mock_points_per_object": 40,
            "enabled": True,
        },
        "vision": {
            "model": "yolov8n.pt",
            "confidence": 0.45,
            "device": "auto",
            "validate_model_on_startup": False,
            "allowed_classes": [
                "person", "bicycle", "motorcycle", "car", "bus", "truck",
            ],
            "tracker_iou_threshold": 0.3,
            "tracker_max_age": 15,
            "tracker_min_hits": 3,
            "image_size": 640,
            "async_detect": False,
            "engine_path": "",
        },
        "lidar_processing": {
            "min_range_m": 0.3,
            "max_range_m": 60.0,
            "roi": {
                "x_min": 0.0, "x_max": 60.0,
                "y_min": -12.0, "y_max": 12.0,
                "z_min": -3.0, "z_max": 4.0,
            },
            "clustering": {"eps": 0.7, "min_samples": 4, "voxel_size": 0.0, "max_points": 0},
            "outlier_method": "iqr",
            "outlier_k": 1.5,
        },
        "synchronization": {
            "max_difference_ms": 120,
            "buffer_size": 20,
            "stale_after_ms": 1000,
            "sorted_search": True,
        },
        "calibration": {
            "file": "config/calibration.yaml",
            "image_width": 1280,
            "image_height": 720,
        },
        "fusion": {
            "min_points": 3,
            "bbox_scale": 0.85,
            "distance_mode": "adaptive",
            "ema_alpha": 0.35,
            "track_stale_seconds": 1.5,
            "track_max_hold": 10,
            "enabled": True,
            "use_clusters": False,
            "fallback_to_filtered_points": True,
        },
        "adas": {
            "alpha": 0.4,
            "stale_seconds": 2.0,
            "min_closing_mps": 0.05,
            "ttc_cap_s": 30.0,
            "max_speed_mps": 60.0,
            "ego_speed_mps": 0.0,
        },
        "warning": {
            "mode": "distance",
            "lane_half_width_m": 1.75,
            "caution_distance_m": 12.0,
            "danger_distance_m": 6.0,
            "emergency_distance_m": 3.0,
            "hysteresis_distance_m": 0.5,
            "min_closing_speed_mps": 0.2,
            "caution_ttc": 4.0,
            "danger_ttc": 2.5,
            "emergency_ttc": 1.5,
            "hysteresis_seconds": 0.6,
            "hysteresis_levels": 1,
            "delay_escalation": False,
        },
        "visualization": {
            "enabled": True,
            "show_lidar_points": True,
            "window_name": "ADAS LiDAR-Camera",
            "max_window_width": 1280,
            "record_path": "",
            "record_video": False,
            "video_fps": 30,
            "lidar_max_points": 2000,
            "async_render": False,
        },
        "benchmark": {
            "warmup_frames": 5,
            "measure_frames": 50,
        },
        "runtime": {
            "async_sensors": False,
            "parallel_perception": False,
            "no_data_timeout_seconds": 3.0,
            "sensor_timeout_seconds": 2.0,
            "sync_timeout_seconds": 2.0,
        },
        "web_dashboard": {
            "enabled": True,
            "snapshot_hz": 10.0,
            "jpeg_quality": 75,
            "secondary_hz": 2.0,
            "secondary_quality": 60,
            "async_encode": False,
        },
        "safety": {
            "production_only": False,
            "is_demo_calibration": True,
            "allow_mock": True,
            "heartbeat_timeout": 1.5,
        },
    }


def _merge_defaults(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Recursive dict merge that prefers values from ``override``."""
    result = copy.deepcopy(base)
    for key, value in override.items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(value, dict)
        ):
            result[key] = _merge_defaults(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _warn_unknown(base: dict[str, Any], override: dict[str, Any], prefix: str = "") -> None:
    """Warn about user keys that match nothing in defaults (likely typos)."""
    for key, value in override.items():
        path = f"{prefix}{key}"
        if key not in base:
            LOGGER.warning("Config co muc la '%s', bi bo qua", path)
        elif isinstance(value, dict) and isinstance(base[key], dict):
            if base[key]:
                _warn_unknown(base[key], value, prefix=f"{path}.")
            # base dict rong = diem mo rong (vd: camera.sources,
            # side_sensor_modules): khong canh bao unknown.


def _warn_missing(base: dict[str, Any], override: dict[str, Any]) -> None:
    required_top_level = (
        "mode", "camera", "lidar", "vision",
        "lidar_processing", "synchronization",
        "calibration", "fusion", "warning",
    )
    for key in required_top_level:
        if key not in override:
            LOGGER.warning(
                "Config thiếu mục '%s', sử dụng giá trị mặc định an toàn", key
            )


def load_config(
    path: str | Path,
    *,
    overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Load YAML configuration, fall back to defaults, apply CLI overrides."""
    cfg_path = Path(path)
    base = _default_config()

    if cfg_path.exists():
        with cfg_path.open("r", encoding="utf-8") as handle:
            user_data = yaml.safe_load(handle) or {}
        if not isinstance(user_data, dict):
            LOGGER.warning(
                "Nội dung config không phải dict (got %s) — sử dụng defaults",
                type(user_data).__name__,
            )
            user_data = {}
        _warn_missing(base, user_data)
        _warn_unknown(base, user_data)
        merged = _merge_defaults(base, user_data)
    else:
        LOGGER.warning("Không tìm thấy config %s — sử dụng defaults", cfg_path)
        merged = base

    if overrides:
        merged = _merge_defaults(merged, overrides)

    return merged


def resolve_path(root: Path, value: str | Path) -> Path:
    """Resolve ``value`` against ``root`` if not absolute."""
    p = Path(value)
    if p.is_absolute():
        return p
    return (root / p).resolve()


def coerce_config(config: dict[str, Any]) -> dict[str, Any]:
    """Coerce common types so yaml-only users are not surprised."""
    camera = config.setdefault("camera", {})
    if isinstance(camera.get("source"), str) and camera["source"].isdigit():
        camera["source"] = int(camera["source"])
    # Backward-compatible aliases used by older YAML files.
    fusion = config.setdefault("fusion", {})
    if "track_max_hold_frames" in fusion:
        fusion["track_max_hold"] = fusion["track_max_hold_frames"]
    health = config.get("health", {})
    safety = config.setdefault("safety", {})
    if "timeout_seconds" in health:
        safety["heartbeat_timeout"] = health["timeout_seconds"]
    return config
