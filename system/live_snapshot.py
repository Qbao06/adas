"""Atomic live telemetry and active-camera frame publisher for the web HMI."""
from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import Any, Iterable, Optional

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]


def _finite(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


class LiveSnapshotPublisher:
    """Publish only display-safe fields; never serialise the whole config."""

    def __init__(self, root: Path, *, max_hz: float = 10.0, jpeg_quality: int = 75,
                 secondary_hz: float = 2.0, secondary_quality: int = 60,
                 async_encode: bool = False) -> None:
        self.log_dir = Path(root) / "logs"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.snapshot_path = self.log_dir / "pipeline_snapshot.json"
        self.frame_path = self.log_dir / "camera_active.jpg"
        self.period = 1.0 / max(1.0, float(max_hz))
        self.jpeg_quality = max(40, min(95, int(jpeg_quality)))
        # Secondary views publish slower and smaller; 0 disables them.
        self.secondary_period = 1.0 / max(1e-9, float(secondary_hz)) if secondary_hz and float(secondary_hz) > 0 else float("inf")
        self.secondary_quality = max(40, min(95, int(secondary_quality)))
        self.async_encode = bool(async_encode)
        self._last_publish = 0.0
        self._last_secondary = 0.0
        self._encode_thread = None

    @staticmethod
    def _detection(item: Any) -> dict[str, Any]:
        bbox = tuple(getattr(item, "bbox", (0, 0, 0, 0)))
        x1, y1, x2, y2 = (list(bbox) + [0, 0, 0, 0])[:4]
        is_fused = hasattr(item, "filtered_distance")
        return {
            "id": int(getattr(item, "track_id", -1) or -1),
            "label": str(getattr(item, "class_name", "object")),
            "confidence": round(float(getattr(item, "confidence", 0.0)), 3),
            "distance_m": _finite(getattr(item, "filtered_distance", None)),
            "bbox": [int(x1), int(y1), int(x2 - x1), int(y2 - y1)] if is_fused else [int(x1), int(y1), int(x2 - x1), int(y2 - y1)],
            "zone": "front",
        }

    @staticmethod
    def _build_gateway_payload(gateway_view: Any, gateway_stats: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
        """Project a :class:`RadarGatewayView` into a JSON-safe payload.

        The returned structure always contains the canonical four radars so the
        dashboard can render a stable table.
        """
        if gateway_view is None:
            return None
        snapshot = getattr(gateway_view, "snapshot", None)
        selection = getattr(gateway_view, "selection", None)
        if snapshot is None:
            return None
        radars: dict[str, dict[str, Any]] = {}
        detected_count = 0
        valid_count = 0
        for radar_id in ("left_front", "left_rear", "right_front", "right_rear"):
            sample = snapshot.radars.get(radar_id)
            if sample is None:
                radars[radar_id] = {
                    "detected": False,
                    "target_state": 0,
                    "distance_m": None,
                    "moving_distance_m": None,
                    "moving_energy": None,
                    "stationary_distance_m": None,
                    "stationary_energy": None,
                    "valid": False,
                    "stale": True,
                    "age_ms": None,
                    "error": "missing",
                }
                continue
            detected = bool(sample.detected and sample.valid)
            if detected:
                detected_count += 1
            if sample.valid:
                valid_count += 1
            radars[radar_id] = {
                "detected": detected,
                "target_state": int(sample.target_state),
                "distance_m": _finite(sample.distance_m),
                "moving_distance_m": _finite(sample.moving_distance_m),
                "moving_energy": sample.moving_energy,
                "stationary_distance_m": _finite(sample.stationary_distance_m),
                "stationary_energy": sample.stationary_energy,
                "valid": bool(sample.valid),
                "stale": bool(sample.stale),
                "age_ms": sample.age_ms,
                "error": sample.error,
            }
        stats = gateway_stats or {}
        return {
            "link_ok": bool(snapshot.link_ok),
            "gateway_ok": bool(snapshot.gateway_ok),
            "remote_ok": bool(snapshot.remote_ok),
            "gateway_boot_id": snapshot.gateway_boot_id,
            "remote_boot_id": snapshot.remote_boot_id,
            "packet_loss": round(float(snapshot.packet_loss or 0.0), 4),
            "sequence": int(snapshot.sequence),
            "json_errors": int(stats.get("json_errors", 0)),
            "received_packets": int(stats.get("received_packets", 0)),
            "duplicate_packets": int(stats.get("duplicate_packets", 0)),
            "reconnect_count": int(stats.get("reconnect_count", 0)),
            "open_attempts": int(stats.get("open_attempts", 0)),
            "detected_count": detected_count,
            "valid_count": valid_count,
            "radars": radars,
            "selection": {
                "camera": str(getattr(selection, "camera", "front")),
                "reason": str(getattr(selection, "reason", "default")),
                "radar_id": str(getattr(selection, "radar_id", "")),
            } if selection is not None else None,
        }

    def publish(
        self,
        *,
        image: np.ndarray,
        camera_id: str,
        detections: Iterable[Any],
        objects: Iterable[Any],
        lidar_points: np.ndarray,
        radar_state: Any,
        module_states: Optional[dict[str, Any]] = None,
        camera_status: dict[str, bool],
        lidar_ok: bool,
        fps: float,
        latency_ms: float,
        frame_id: int,
        gateway_view: Optional[Any] = None,
        gateway_stats: Optional[dict[str, Any]] = None,
        camera_images: Optional[dict[str, np.ndarray]] = None,
    ) -> None:
        now = time.monotonic()
        if now - self._last_publish < self.period:
            return
        self._last_publish = now
        object_list = list(objects or [])
        detection_source = (
            object_list if object_list else list(detections or [])
        ) if camera_id == "front" else []
        display_detections = [self._detection(item) for item in detection_source]

        severity = {"unknown": 0, "safe": 1, "caution": 2, "danger": 3, "emergency": 4}
        worst = "safe"
        distance: Optional[float] = None
        for obj in object_list:
            level_obj = getattr(obj, "warning_level", "unknown")
            level = str(getattr(level_obj, "value", level_obj)).lower()
            if severity.get(level, 0) > severity.get(worst, 0):
                worst = level
            current = _finite(getattr(obj, "filtered_distance", None))
            if current is not None and (distance is None or current < distance):
                distance = current
        ui_level = "danger" if worst == "emergency" else worst
        if ui_level not in {"safe", "caution", "danger"}:
            ui_level = "safe"
        message = "KHOẢNG CÁCH AN TOÀN"
        if ui_level == "caution":
            message = "CẢNH BÁO VẬT CẢN PHÍA TRƯỚC"
        elif ui_level == "danger":
            message = "NGUY HIỂM — VẬT CẢN QUÁ GẦN"

        pts = np.asarray(lidar_points) if lidar_points is not None else np.empty((0, 3))
        if pts.ndim == 2 and pts.shape[1] >= 2:
            step = max(1, len(pts) // 500)
            bev = [[round(float(p[1]), 2), round(float(p[0]), 2)] for p in pts[::step] if np.isfinite(p[:2]).all()]
        else:
            bev = []

        module_states = module_states or {}
        module_payload = {
            module_id: {
                "side": str(getattr(state, "side", "")),
                "position": str(getattr(state, "position", "")),
                "detected": bool(getattr(state, "detected", False)),
                "distance_m": _finite(getattr(state, "distance_m", None)),
                "link_ok": bool(getattr(state, "link_ok", False)),
                "sequence": int(getattr(state, "sequence", -1)),
            }
            for module_id, state in module_states.items()
        }
        radar = radar_state
        radar_payload = {
            "link_ok": bool(getattr(radar, "link_ok", False)),
            "left": bool(getattr(radar, "left_detected", False)),
            "right": bool(getattr(radar, "right_detected", False)),
            "left_m": _finite(getattr(radar, "left_distance_m", None)),
            "right_m": _finite(getattr(radar, "right_distance_m", None)),
            "sequence": int(getattr(radar, "sequence", -1)),
        }
        if module_payload:
            for side in ("left", "right"):
                candidates = [item for item in module_payload.values() if item["side"] == side and item["link_ok"]]
                detected = [item for item in candidates if item["detected"]]
                radar_payload[side] = bool(detected)
                distances = [item["distance_m"] for item in detected if item["distance_m"] is not None]
                radar_payload[f"{side}_m"] = min(distances) if distances else None
            radar_payload["link_ok"] = any(item["link_ok"] for item in module_payload.values())

        gateway_payload = self._build_gateway_payload(gateway_view, gateway_stats)
        if gateway_payload is not None:
            # When the gateway is present, the legacy "side_radar" fields are
            # shadowed by the four-radar summary. This keeps the dashboard
            # rendering path backward compatible.
            detected = gateway_payload["detected_count"] > 0
            radar_payload["link_ok"] = gateway_payload["link_ok"]
            radar_payload["left"] = detected and any(
                gateway_payload["radars"][rid]["detected"] for rid in ("left_front", "left_rear")
            )
            radar_payload["right"] = detected and any(
                gateway_payload["radars"][rid]["detected"] for rid in ("right_front", "right_rear")
            )
            distances_left = [
                gateway_payload["radars"][rid]["distance_m"]
                for rid in ("left_front", "left_rear")
                if gateway_payload["radars"][rid]["distance_m"] is not None
            ]
            distances_right = [
                gateway_payload["radars"][rid]["distance_m"]
                for rid in ("right_front", "right_rear")
                if gateway_payload["radars"][rid]["distance_m"] is not None
            ]
            radar_payload["left_m"] = min(distances_left) if distances_left else None
            radar_payload["right_m"] = min(distances_right) if distances_right else None
            radar_payload["sequence"] = gateway_payload["sequence"]

        radar_link_ok = bool(radar_payload.get("link_ok")) or (gateway_payload is not None and gateway_payload["link_ok"])
        system_ok = (
            bool(camera_status.get(camera_id, False))
            and lidar_ok
            and radar_link_ok
        )
        payload = {
            "timestamp": time.time(),
            "demo": False,
            "active_camera": camera_id,
            "camera_reason": "side-radar" if camera_id in {"left", "right", "left_1", "left_2", "right_1", "right_2"} else "default",
            "warning": {"level": ui_level, "distance_m": distance if distance is not None else 25.0, "message": message},
            "vehicle": {"gear": "D", "turn_signal": "none"},
            "detections": display_detections,
            "lidar_points": bev,
            "side_radar": radar_payload,
            "side_modules": module_payload,
            "radar_gateway": gateway_payload,
            "health": {
                "camera": "ok" if camera_status.get(camera_id, False) else "error",
                "lidar": "ok" if lidar_ok else "error",
                "radar": "ok" if radar_link_ok else "error",
                "ai": "ok",
                "fusion": "ok" if lidar_ok else "warning",
                "calibration": "ok",
                "system": "ok" if system_ok else "warning",
            },
            "cameras_available": [name for name, ok in camera_status.items() if ok],
            "fps": round(float(fps), 1),
            "latency_ms": round(float(latency_ms), 1),
            "frame_id": int(frame_id),
        }
        tmp_json = self.snapshot_path.with_suffix(".json.tmp")
        tmp_json.write_text(json.dumps(payload, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
        os.replace(tmp_json, self.snapshot_path)

        if cv2 is not None and image is not None:
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                tmp_frame = self.frame_path.with_suffix(".jpg.tmp")
                tmp_frame.write_bytes(encoded.tobytes())
                os.replace(tmp_frame, self.frame_path)

        secondary_due = (now - self._last_secondary) >= self.secondary_period
        if cv2 is not None and secondary_due:
            self._last_secondary = now
            views = {
                str(k).lower(): v for k, v in (camera_images or {}).items()
                if str(k).lower() not in {"", str(camera_id).lower()} and v is not None
            }
            if views:
                if self.async_encode:
                    self._publish_secondary_async(views)
                else:
                    self._publish_secondary(views)

    _ALLOWED_VIEWS = frozenset({
        "front", "rear", "left", "right",
        "left_1", "left_2", "right_1", "right_2",
    })

    def _encode_and_store(self, target: Path, view_image, quality: int) -> None:
        ok, encoded = cv2.imencode(
            ".jpg", view_image,
            [cv2.IMWRITE_JPEG_QUALITY, quality],
        )
        if ok:
            temporary = target.with_suffix(".jpg.tmp")
            temporary.write_bytes(encoded.tobytes())
            os.replace(temporary, target)

    def _publish_secondary(self, views: dict) -> None:
        for view_id, view_image in views.items():
            if view_id not in self._ALLOWED_VIEWS or view_image is None:
                continue
            try:
                self._encode_and_store(
                    self.log_dir / f"camera_{view_id}.jpg",
                    view_image, self.secondary_quality,
                )
            except Exception:
                pass

    def _publish_secondary_async(self, views: dict) -> None:
        import threading
        worker = getattr(self, "_encode_thread", None)
        if worker is not None and worker.is_alive():
            return  # drop: previous batch still encoding, keep main path fast.
        def _work(items):
            for view_id, view_image in items:
                if view_id not in self._ALLOWED_VIEWS or view_image is None:
                    continue
                try:
                    self._encode_and_store(
                        self.log_dir / f"camera_{view_id}.jpg",
                        view_image, self.secondary_quality,
                    )
                except Exception:
                    pass
        thread = threading.Thread(target=_work, args=(list(views.items),),
                                  daemon=True)
        self._encode_thread = thread
        thread.start()


__all__ = ["LiveSnapshotPublisher"]
