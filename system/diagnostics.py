"""Write a small, non-secret runtime manifest for deployment support."""
from __future__ import annotations

import hashlib
import importlib.metadata
import json
import platform
from datetime import datetime, timezone
from pathlib import Path


def write_runtime_manifest(root: Path, config: dict) -> Path:
    encoded = json.dumps(config, sort_keys=True, default=str).encode("utf-8")
    packages = {}
    for name in ("numpy", "opencv-python", "torch", "ultralytics", "scikit-learn"):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
    gateway = config.get("radar_gateway") if isinstance(config, dict) else None
    if isinstance(gateway, dict):
        gateway_summary = {
            "enabled": bool(gateway.get("enabled", False)),
            "device": str(gateway.get("device", "")),
            "baudrate": int(gateway.get("baudrate", 115200) or 115200),
            "protocol_version": int(gateway.get("protocol_version", 2) or 2),
        }
        radars = gateway.get("radars", {}) if isinstance(gateway.get("radars"), dict) else {}
        gateway_summary["radars"] = {
            str(radar_id): {
                "enabled": bool(item.get("enabled", False)),
                "camera": str(item.get("camera", "")),
                "warning_distance_m": float(item.get("warning_distance_m", 0.0) or 0.0),
            }
            for radar_id, item in radars.items()
            if isinstance(item, dict)
        }
    else:
        gateway_summary = None
    manifest = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "mode": config.get("mode"),
        "model": config.get("vision", {}).get("model"),
        "calibration": config.get("calibration", {}).get("file"),
        "lidar_driver": config.get("lidar", {}).get("driver"),
        "radar_gateway": gateway_summary,
        "config_sha256": hashlib.sha256(encoded).hexdigest(),
        "packages": packages,
    }
    path = root / "logs" / "runtime_manifest.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return path


__all__ = ["write_runtime_manifest"]
