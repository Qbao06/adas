"""Deterministic in-memory ADAS Cockpit simulator for Demo mode.

Operates purely in RAM: zero hardware calls, no CUDA, no video device access.
Generates realistic scenes, detections, LiDAR points, and vehicle telemetry.
"""
from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional


class CockpitSimulator:
    """In-memory simulator generating realistic ADAS cockpit telemetry."""

    def __init__(self) -> None:
        self.scenario: str = "normal"
        self.active_camera: str = "front"
        self.gear: str = "D"
        self.turn_signal: str = "none"  # "none", "left", "right"
        self.is_paused: bool = False

        # Failure toggles
        self.cam_lost: bool = False
        self.lidar_lost: bool = False
        self.ai_lost: bool = False

        self.start_time = time.monotonic()
        self.frame_count: int = 0

    def apply_action(self, action: str) -> bool:
        """Apply a simulated scenario or control event from allowlist."""
        action = (action or "").strip().lower()

        if action == "normal":
            self.scenario = "normal"
            self.cam_lost = False
            self.lidar_lost = False
            self.ai_lost = False
            self.gear = "D"
            self.turn_signal = "none"
            self.active_camera = "front"

        elif action == "pedestrian":
            self.scenario = "pedestrian"
            self.gear = "D"

        elif action == "danger":
            self.scenario = "danger"
            self.gear = "D"

        elif action == "lost_cam":
            self.cam_lost = not self.cam_lost

        elif action == "lost_lidar":
            self.lidar_lost = not self.lidar_lost

        elif action == "lost_ai":
            self.ai_lost = not self.ai_lost

        elif action == "turn_signal":
            cycle = {"none": "left", "left": "right", "right": "none"}
            self.turn_signal = cycle.get(self.turn_signal, "none")

        elif action == "reverse":
            if self.gear == "R":
                self.gear = "D"
                self.active_camera = "front"
            else:
                self.gear = "R"
                self.active_camera = "rear"

        elif action == "pause_resume":
            self.is_paused = not self.is_paused

        elif action == "reset":
            self.__init__()

        else:
            return False

        return True

    def set_camera(self, camera_id: str) -> bool:
        """Switch active main camera viewport."""
        if camera_id in ("front", "left", "right", "rear"):
            self.active_camera = camera_id
            return True
        return False

    def get_snapshot(self) -> Dict[str, Any]:
        """Generate current point-in-time snapshot."""
        if not self.is_paused:
            self.frame_count += 1
        elapsed = time.monotonic() - self.start_time

        # Calculate dynamics based on scenario
        t = elapsed if not self.is_paused else 1.0

        detections: List[Dict[str, Any]] = []
        warning_level = "safe"
        min_distance = 25.0
        warning_msg = "LÀN ĐƯỜNG AN TOÀN"

        if not self.ai_lost and not self.cam_lost:
            if self.scenario == "normal":
                # Car in front at steady distance
                lead_dist = 16.0 + 2.0 * math.sin(t * 0.8)
                min_distance = round(lead_dist, 1)
                warning_level = "safe" if min_distance > 15.0 else "caution"
                warning_msg = f"XE PHÍA TRƯỚC · {min_distance}m"
                detections.append({
                    "id": 101,
                    "label": "car",
                    "distance_m": min_distance,
                    "confidence": 0.94,
                    # bbox: [x, y, w, h] normalized in 1280x720 space
                    "bbox": [540, 310, 200, 160],
                    "zone": "front",
                })
                # Distant vehicle in left lane
                detections.append({
                    "id": 102,
                    "label": "car",
                    "distance_m": 28.4,
                    "confidence": 0.88,
                    "bbox": [280, 330, 120, 100],
                    "zone": "left",
                })

            elif self.scenario == "pedestrian":
                # Person crossing road
                p_dist = 6.8 + 1.2 * math.cos(t * 1.2)
                min_distance = round(p_dist, 1)
                warning_level = "caution" if min_distance > 5.0 else "danger"
                warning_msg = f"CẢNH BÁO: NGƯỜI ĐI BỘ · {min_distance}m"
                # Moving pedestrian bbox across road
                px = int(580 + 90 * math.sin(t * 1.5))
                detections.append({
                    "id": 201,
                    "label": "person",
                    "distance_m": min_distance,
                    "confidence": 0.91,
                    "bbox": [px, 320, 75, 175],
                    "zone": "front",
                })
                detections.append({
                    "id": 101,
                    "label": "car",
                    "distance_m": 22.0,
                    "confidence": 0.89,
                    "bbox": [420, 300, 150, 120],
                    "zone": "front",
                })

            elif self.scenario == "danger":
                # Car braking abruptly in front
                danger_dist = 3.2 + 0.8 * math.sin(t * 2.0)
                min_distance = round(danger_dist, 1)
                warning_level = "danger"
                warning_msg = f"NGUY HIỂM: NGUY CƠ VA CHẠM · {min_distance}m"
                detections.append({
                    "id": 301,
                    "label": "car",
                    "distance_m": min_distance,
                    "confidence": 0.97,
                    "bbox": [460, 240, 360, 280],
                    "zone": "front",
                })

        # Generate LiDAR points for Top-down Bird's Eye View (BEV)
        lidar_points = []
        if not self.lidar_lost:
            # Concentric road borders / obstacles
            base_count = 60
            for i in range(base_count):
                ang = (i / base_count) * 2 * math.pi
                r = 14.0 + 3.0 * math.sin(ang * 4 + t)
                lidar_points.append([round(r * math.cos(ang), 2), round(r * math.sin(ang), 2)])
            # Object cluster points
            if detections:
                main_obj = detections[0]
                od = main_obj["distance_m"]
                for dx in [-0.8, -0.4, 0.0, 0.4, 0.8]:
                    for dy in [-0.4, 0.0, 0.4]:
                        lidar_points.append([round(dx, 2), round(od + dy, 2)])

        # Health status of subsystems
        health = {
            "camera": "error" if self.cam_lost else "ok",
            "lidar": "error" if self.lidar_lost else "ok",
            "ai": "error" if self.ai_lost else "ok",
            "fusion": "error" if (self.cam_lost and self.lidar_lost) else "ok",
            "calibration": "ok",
            "system": "ok" if not (self.cam_lost or self.lidar_lost or self.ai_lost) else "warning",
        }

        return {
            "timestamp": time.time(),
            "demo": True,
            "scenario": self.scenario,
            "active_camera": self.active_camera,
            "warning": {
                "level": warning_level,
                "distance_m": min_distance,
                "message": warning_msg,
            },
            "vehicle": {
                "gear": self.gear,
                "turn_signal": self.turn_signal,
                "speed_kph": 0 if self.gear == "P" else (18 if self.scenario == "danger" else 42),
            },
            "detections": detections,
            "lidar_points": lidar_points,
            "health": health,
            "cameras_available": ["front", "left", "right", "rear"],
            "paused": self.is_paused,
            "frame_id": self.frame_count,
        }
