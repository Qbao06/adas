"""
Python host-pure tests for the JSON v2 transport encoder.

The encoder follows the same slot-based emission contract as the C++
firmware: `snap.radars[0..3]` correspond to the four canonical radar
positions left_front, left_rear, right_front, right_rear.  Tests
construct RadarSnapshot with a list of exactly 4 RadarSample objects
in that order.
"""
from __future__ import annotations

import json
import os
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from firmware.esp32_radar_gateway.tests.jetson_json_transport_py import (  # noqa: E402
    jetson_json_encode_v2,
    RadarSnapshot,
    RadarSample,
    TARGET_STATE_MOVING,
    TARGET_STATE_NONE,
    LD2410_DISTANCE_NONE,
    RADAR_ORDER,
)


def _sample(**overrides):
    s = RadarSample(
        radar_id="left_front",
        target_state=TARGET_STATE_MOVING,
        detected=1,
        valid=1,
        stale=0,
        moving_distance_cm=180,
        stationary_distance_cm=LD2410_DISTANCE_NONE,
        moving_energy=55,
        stationary_energy=0xFF,
        detection_distance_cm=180,
        error_code=0,
        sample_age_ms=20,
    )
    for k, v in overrides.items():
        setattr(s, k, v)
    return s


def _snapshot(*radars: RadarSample) -> RadarSnapshot:
    """Build a RadarSnapshot with exactly four slot-ordered radars."""
    assert len(radars) == 4, "need exactly four radars for the v2 contract"
    return RadarSnapshot(
        sequence=10,
        gateway_ok=1,
        remote_ok=1,
        gateway_boot_id=0xAAAA,
        remote_boot_id=0xBBBB,
        packet_loss=2,
        radars=list(radars),
    )


def test_json_v2_emits_all_four_radars_in_protocol_order():
    snap = _snapshot(
        _sample(radar_id="left_front", moving_distance_cm=180, detection_distance_cm=180),
        _sample(radar_id="left_rear",  moving_distance_cm=100, detection_distance_cm=100),
        _sample(radar_id="right_front", moving_distance_cm=300, detection_distance_cm=300),
        _sample(radar_id="right_rear",  moving_distance_cm=400, detection_distance_cm=400),
    )
    line = jetson_json_encode_v2(snap)
    payload = json.loads(line)
    assert payload["v"] == 2
    assert payload["seq"] == 10
    assert payload["gateway_ok"] == 1
    assert payload["remote_ok"] == 1
    assert payload["gateway_boot_id"] == 0xAAAA
    assert payload["remote_boot_id"] == 0xBBBB
    assert payload["packet_loss"] == 2
    # Order matters: left_front, left_rear, right_front, right_rear.
    keys = list(payload["radars"].keys())
    assert keys == ["left_front", "left_rear", "right_front", "right_rear"]
    assert payload["radars"]["left_front"]["distance_m"] == 1.8
    assert payload["radars"]["left_rear"]["distance_m"] == 1.0
    assert payload["radars"]["right_front"]["distance_m"] == 3.0
    assert payload["radars"]["right_rear"]["distance_m"] == 4.0
    assert payload["radars"]["left_rear"]["moving_distance_m"] == 1.0
    assert payload["radars"]["right_front"]["moving_distance_m"] == 3.0
    assert payload["radars"]["right_rear"]["moving_distance_m"] == 4.0


def test_json_v2_emits_null_for_invalid_distance():
    snap = _snapshot(
        _sample(valid=0),
        _sample(radar_id="left_rear"),
        _sample(radar_id="right_front"),
        _sample(radar_id="right_rear"),
    )
    line = jetson_json_encode_v2(snap)
    payload = json.loads(line)
    assert payload["radars"]["left_front"]["distance_m"] is None
    assert payload["radars"]["left_front"]["valid"] is False
    assert payload["radars"]["left_front"]["stale"] is True


def test_json_v2_never_uses_zero_as_invalid_distance():
    snap = _snapshot(
        _sample(valid=0, moving_distance_cm=0, detection_distance_cm=0),
        _sample(radar_id="left_rear"),
        _sample(radar_id="right_front"),
        _sample(radar_id="right_rear"),
    )
    line = jetson_json_encode_v2(snap)
    payload = json.loads(line)
    # Per the v2 contract: invalid samples must be null, NEVER 0.
    assert payload["radars"]["left_front"]["distance_m"] is None
    assert payload["radars"]["left_front"]["moving_distance_m"] is None
    assert payload["radars"]["left_front"]["moving_energy"] is None


def test_json_v2_handles_just_stale_sample():
    snap = RadarSnapshot(
        sequence=2, gateway_ok=1, remote_ok=0,
        gateway_boot_id=1, remote_boot_id=1, packet_loss=5,
        radars=[
            _sample(stale=1),
            _sample(radar_id="left_rear", stale=1),
            _sample(radar_id="right_front", stale=1),
            _sample(radar_id="right_rear", stale=1),
        ],
    )
    line = jetson_json_encode_v2(snap)
    payload = json.loads(line)
    assert payload["remote_ok"] == 0
    assert payload["packet_loss"] == 5
    for key in ("left_front", "left_rear", "right_front", "right_rear"):
        assert payload["radars"][key]["stale"] is True


def test_json_v2_encodes_target_state_strings():
    snap = _snapshot(
        _sample(target_state=TARGET_STATE_NONE, detected=0, valid=1),
        _sample(radar_id="left_rear"),
        _sample(radar_id="right_front"),
        _sample(radar_id="right_rear"),
    )
    line = jetson_json_encode_v2(snap)
    payload = json.loads(line)
    assert payload["radars"]["left_front"]["target_state"] == "none"
    assert payload["radars"]["left_front"]["detected"] is False
    assert payload["radars"]["left_rear"]["target_state"] == "moving"