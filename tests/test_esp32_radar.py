from __future__ import annotations

import json

import pytest
import numpy as np

from core.types import CameraFrame
from sensors import (
    ModuleCameraSelector, MultiCameraReader, RadarCameraSelector,
    SideModuleState, SideRadarState, parse_radar_line,
)
from system.live_snapshot import LiveSnapshotPublisher


def _state(*, left=False, right=False, left_m=None, right_m=None, link=True):
    return SideRadarState(
        sequence=1,
        left_detected=left,
        right_detected=right,
        left_distance_m=left_m,
        right_distance_m=right_m,
        received_monotonic=10.0,
        link_ok=link,
    )


def test_parse_valid_esp32_json_line():
    payload = {"v": 1, "seq": 8, "left": True, "right": False, "left_m": 2.4, "right_m": None}
    state = parse_radar_line(json.dumps(payload), received_monotonic=12.0)
    assert state.sequence == 8
    assert state.left_detected is True
    assert state.right_detected is False
    assert state.left_distance_m == 2.4
    assert state.link_ok is True


@pytest.mark.parametrize("payload", [
    {},
    {"v": 2, "seq": 1, "left": True, "right": False},
    {"v": 1, "seq": -1, "left": True, "right": False},
    {"v": 1, "seq": 1, "left": 1, "right": False},
    {"v": 1, "seq": 1, "left": True, "right": False, "left_m": 100},
])
def test_parse_rejects_invalid_frames(payload):
    with pytest.raises((ValueError, KeyError)):
        parse_radar_line(json.dumps(payload))


def test_selector_left_right_and_clear_hold():
    selector = RadarCameraSelector(hold_seconds=1.0)
    assert selector.update(_state(left=True), now=1.0).camera == "left"
    held = selector.update(_state(link=True), now=1.5)
    assert held.camera == "left"
    assert held.reason == "hold"
    assert selector.update(_state(link=True), now=2.1).camera == "front"
    assert selector.update(_state(right=True), now=3.0).camera == "right"


def test_selector_uses_nearest_side_when_both_detect():
    selector = RadarCameraSelector()
    result = selector.update(_state(left=True, right=True, left_m=3.0, right_m=1.5), now=1.0)
    assert result.camera == "right"
    assert result.reason == "both-radars"


def test_selector_stale_link_never_creates_detection():
    selector = RadarCameraSelector(hold_seconds=0)
    assert selector.update(_state(left=True, link=False), now=1.0).camera == "front"


def test_reverse_has_highest_priority():
    selector = RadarCameraSelector()
    result = selector.update(_state(left=True), reverse=True, now=1.0)
    assert result.camera == "rear"


class _FakeWorker:
    def __init__(self, camera_id):
        self.camera_id = camera_id

    def get_latest(self, sequence, timeout=0):
        return sequence + 1, CameraFrame(np.zeros((4, 4, 3), dtype=np.uint8), 1.0, sequence + 1, self.camera_id)


def test_multi_camera_keeps_front_for_fusion_and_side_for_display():
    reader = MultiCameraReader({"front": 0, "left": 1, "right": 2}, width=4, height=4, fps=1)
    reader._workers = {name: _FakeWorker(name) for name in ("front", "left", "right")}
    assert reader.select("left") is True
    assert reader.read().camera_id == "front"
    assert reader.read_display().camera_id == "left"


def test_multi_camera_accepts_two_modules_per_side():
    reader = MultiCameraReader(
        {"front": 0, "left_1": 1, "left_2": 2, "right_1": 3, "right_2": 4},
        width=4, height=4, fps=1,
    )
    assert reader.select("left_2") is True
    assert reader.active_camera == "left_2"


def test_multi_camera_exposes_independent_real_view_frames():
    reader = MultiCameraReader(
        {"front": 0, "left_1": 1, "right_1": 2}, width=4, height=4, fps=1
    )
    reader._workers = {
        name: _FakeWorker(name) for name in ("front", "left_1", "right_1")
    }
    views = reader.read_views()
    assert set(views) == {"front", "left_1", "right_1"}
    assert all(frame.camera_id == name for name, frame in views.items())


def test_module_selector_uses_closest_and_holds_camera():
    selector = ModuleCameraSelector(hold_seconds=1.0)
    states = {
        "left_1": SideModuleState("left_1", "left", "front", True, 5.0, True, 1),
        "left_2": SideModuleState("left_2", "left", "rear", True, 2.0, True, 1),
    }
    assert selector.update(states, now=1.0)[0] == "left_2"
    assert selector.update({}, now=1.5)[0] == "left_2"
    assert selector.update({}, now=2.1)[0] == "front"


def test_live_snapshot_contains_real_radar_state_without_side_overlay(tmp_path):
    publisher = LiveSnapshotPublisher(tmp_path, max_hz=10)
    radar = _state(left=True, left_m=1.8)

    class Detection:
        track_id = 7
        class_name = "car"
        confidence = 0.9
        bbox = (1, 1, 3, 3)

    publisher.publish(
        image=np.zeros((8, 8, 3), dtype=np.uint8),
        camera_id="left",
        detections=[Detection()],
        objects=[],
        lidar_points=np.zeros((2, 3)),
        radar_state=radar,
        camera_status={"front": True, "left": True, "right": True},
        lidar_ok=True,
        fps=30.0,
        latency_ms=8.0,
        frame_id=3,
        camera_images={
            "front": np.zeros((8, 8, 3), dtype=np.uint8),
            "left_1": np.zeros((8, 8, 3), dtype=np.uint8),
        },
    )
    payload = json.loads((tmp_path / "logs" / "pipeline_snapshot.json").read_text(encoding="utf-8"))
    assert payload["active_camera"] == "left"
    assert payload["side_radar"]["left"] is True
    assert payload["detections"] == []
    assert (tmp_path / "logs" / "camera_front.jpg").is_file()
    assert (tmp_path / "logs" / "camera_left_1.jpg").is_file()
