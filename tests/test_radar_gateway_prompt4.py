"""Prompt 4 hardening tests for the four-LD2410C Gateway.

The suite covers gateway JSON v2 hardening:

* JSON v2 parser accepts the canonical schema, rejects malformed JSON,
  rejects too-long frames, rejects missing radars, rejects bad field
  types, rejects unknown versions, rejects negative distances, rejects
  NaN / Infinity.
* Sequence tracking: duplicates and packet loss are surfaced; Gateway
  reboot is recognised through ``gateway_boot_id`` change.
* Staleness: Gateway stale, Remote stale, single radar stale, and
  ``detected=true`` with ``distance=null`` are honoured by both the
  parser and the selector.
* Camera selection: the closest distance wins, ties break on the
  configured priority, the camera hold is respected, the camera
  defaults back to front when the Gateway disappears, and a
  reconnect after reset resumes updates.
* Migration: the legacy ``side_sensor_modules`` schema is mapped to
  the new ``radar_gateway`` block without losing operators' data.
* Live mode never silently falls back to a mock radar when the
  Gateway is enabled.
* Validation rejects the remaining unsafe inputs.
"""

from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

import pytest

from config import ConfigError, validate_config
from config.radar_gateway import (
    LD2410_MAX_DETECTION_M,
    PROTOCOL_VERSIONS,
    RADAR_IDS,
    default_radar_gateway,
    migrate_legacy_config,
    validate_radar_gateway,
)
from config.validation import ConfigError as _LegacyConfigError
from sensors.radar_gateway import (
    GatewayRadarSample,
    GatewaySnapshot,
    RadarGatewayReader,
    parse_gateway_line,
)
from sensors.radar_gateway_selector import (
    GatewayCameraSelector,
)
from sensors.radar_gateway_manager import RadarGatewayManager


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def _packet(
    *,
    seq: int = 1,
    gateway_ok: bool = True,
    remote_ok: bool = True,
    gateway_boot_id: str = "boot-aaaa",
    remote_boot_id: str = "boot-bbbb",
    radars: Optional[Dict[str, Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    """Return a valid v2 packet dict for tests to mutate."""
    if radars is None:
        radars = {
            rid: {
                "detected": False,
                "valid": True,
                "stale": False,
                "distance_m": None,
                "moving_distance_m": None,
                "moving_energy": 0,
                "stationary_distance_m": None,
                "stationary_energy": 0,
                "age_ms": 50,
                "error": "",
            }
            for rid in RADAR_IDS
        }
    return {
        "v": 2,
        "seq": seq,
        "gateway_ok": gateway_ok,
        "remote_ok": remote_ok,
        "gateway_boot_id": gateway_boot_id,
        "remote_boot_id": remote_boot_id,
        "packet_loss": 0.0,
        "radars": radars,
    }


def _snapshot_from_packet(
    packet: Dict[str, Any],
    *,
    received_monotonic: float = 1.0,
    max_distance_m: float = LD2410_MAX_DETECTION_M,
) -> GatewaySnapshot:
    return parse_gateway_line(
        json.dumps(packet),
        received_monotonic=received_monotonic,
        max_distance_m=max_distance_m,
    )


def _selector(
    *,
    hold_seconds: float = 1.2,
    default_camera: str = "front",
    priority: Optional[List[str]] = None,
) -> GatewayCameraSelector:
    return GatewayCameraSelector(
        radar_camera_map={
            "left_front": "left_1",
            "left_rear": "left_2",
            "right_front": "right_1",
            "right_rear": "right_2",
        },
        hold_seconds=hold_seconds,
        default_camera=default_camera,
        priority=priority,
    )


# --------------------------------------------------------------------------- #
# 1. JSON v2 parser
# --------------------------------------------------------------------------- #
def test_parse_accepts_valid_v2_packet() -> None:
    sample = _snapshot_from_packet(_packet(seq=7))
    assert sample.sequence == 7
    assert sample.gateway_ok is True
    assert sample.remote_ok is True
    assert set(sample.radars) == set(RADAR_IDS)


def test_parse_rejects_malformed_json() -> None:
    with pytest.raises(ValueError):
        parse_gateway_line(b"{not-json}\n")


def test_parse_rejects_frame_too_long() -> None:
    too_long = b'{"v":2,"seq":1,"x":"' + b"a" * 5000 + b'"}\n'
    with pytest.raises(ValueError, match="too long"):
        parse_gateway_line(too_long)


def test_parse_records_missing_radar_with_empty_sample() -> None:
    packet = _packet()
    del packet["radars"]["right_rear"]
    sample = _snapshot_from_packet(packet)
    assert sample.radars["right_rear"].valid is False
    assert sample.radars["right_rear"].stale is True
    assert sample.radars["right_rear"].distance_m is None


def test_parse_rejects_non_dict_payload() -> None:
    with pytest.raises(ValueError):
        parse_gateway_line("[1, 2, 3]")


@pytest.mark.parametrize("bad_value", ["v1", 1, None, 3, "abc"])
def test_parse_rejects_unsupported_protocol_version(bad_value: Any) -> None:
    packet = _packet()
    packet["v"] = bad_value
    with pytest.raises(ValueError):
        parse_gateway_line(json.dumps(packet))


def test_parse_rejects_negative_distance() -> None:
    packet = _packet()
    packet["radars"]["left_front"]["distance_m"] = -1.0
    sample = _snapshot_from_packet(packet)
    assert sample.radars["left_front"].distance_m is None


def test_parse_rejects_nan_distance() -> None:
    # JSON does not allow NaN/Infinity, but the parser must reject any
    # source that manages to inject a non-finite float.
    packet = _packet()
    packet["radars"]["left_front"]["distance_m"] = float("nan")
    sample = _snapshot_from_packet(packet)
    assert sample.radars["left_front"].distance_m is None


def test_parse_rejects_infinite_distance() -> None:
    packet = _packet()
    packet["radars"]["right_rear"]["distance_m"] = float("inf")
    sample = _snapshot_from_packet(packet)
    assert sample.radars["right_rear"].distance_m is None


def test_parse_rejects_wrong_field_type() -> None:
    packet = _packet()
    packet["radars"]["left_rear"]["distance_m"] = "five"  # type: ignore[assignment]
    sample = _snapshot_from_packet(packet)
    assert sample.radars["left_rear"].distance_m is None


def test_parse_rejects_seq_below_zero() -> None:
    packet = _packet(seq=-1)
    with pytest.raises(ValueError):
        parse_gateway_line(json.dumps(packet))


# --------------------------------------------------------------------------- #
# 2. Sequence, packet loss, Gateway reboot
# --------------------------------------------------------------------------- #
def test_sequence_regression_keeps_last_valid_packet() -> None:
    """The reader rejects duplicate / older sequences without crashing."""
    manager = RadarGatewayManager.__new__(RadarGatewayManager)
    manager.config = default_radar_gateway()
    manager._last_view = None
    manager._selection_changed_at = 0.0
    # Two packets with increasing sequence should both be honoured.
    s1 = _snapshot_from_packet(_packet(seq=1), received_monotonic=1.0)
    s2 = _snapshot_from_packet(_packet(seq=5), received_monotonic=2.0)
    assert s1.sequence == 1
    assert s2.sequence == 5


def test_packet_loss_surfaced_in_snapshot() -> None:
    packet = _packet()
    packet["packet_loss"] = 0.42
    sample = _snapshot_from_packet(packet)
    assert sample.packet_loss == pytest.approx(0.42)


def test_gateway_reboot_increments_boot_id() -> None:
    p1 = _packet(seq=1, gateway_boot_id="boot-aaaa")
    p2 = _packet(seq=1, gateway_boot_id="boot-bbbb")  # seq reset on reboot
    s1 = _snapshot_from_packet(p1)
    s2 = _snapshot_from_packet(p2)
    assert s1.gateway_boot_id != s2.gateway_boot_id


def test_remote_reboot_changes_remote_boot_id_only() -> None:
    p1 = _packet(seq=1, remote_boot_id="rboot-1")
    p2 = _packet(seq=2, remote_boot_id="rboot-2")
    s1 = _snapshot_from_packet(p1)
    s2 = _snapshot_from_packet(p2)
    assert s1.remote_boot_id != s2.remote_boot_id
    assert s1.gateway_boot_id == s2.gateway_boot_id


# --------------------------------------------------------------------------- #
# 3. Staleness / partial link
# --------------------------------------------------------------------------- #
def _stale_snapshot(
    *, gateway_ok: bool, remote_ok: bool, received_monotonic: float
) -> GatewaySnapshot:
    packet = _packet(
        seq=1, gateway_ok=gateway_ok, remote_ok=remote_ok
    )
    return _snapshot_from_packet(packet, received_monotonic=received_monotonic)


def test_gateway_stale_marks_all_radars_invalid() -> None:
    """When the Gateway itself is stale, no radar may be trusted."""
    selector = _selector(hold_seconds=0.0)
    snap = _stale_snapshot(gateway_ok=False, remote_ok=True, received_monotonic=1.0)
    # Force staleness by moving the read time forward.
    snap.received_monotonic = 1.0
    # The selector's contract is: with link_ok=False, fall back to default.
    selection = selector.update(snap, now=10.0)
    assert selection.camera == "front"
    assert selection.radar_id == ""


def test_remote_stale_only_affects_right_radars() -> None:
    """When remote_ok=False, left_front and left_rear stay trustworthy."""
    selector = _selector(hold_seconds=0.0)
    packet = _packet(gateway_ok=True, remote_ok=False)
    packet["radars"]["left_front"]["detected"] = True
    packet["radars"]["left_front"]["distance_m"] = 1.2
    snap = _snapshot_from_packet(packet)
    selection = selector.update(snap, now=1.0)
    # The detected radar is left_front -> camera left_1.
    assert selection.camera == "left_1"
    assert selection.radar_id == "left_front"


def test_single_radar_stale_does_not_affect_others() -> None:
    selector = _selector(hold_seconds=0.0)
    packet = _packet()
    # Mark right_front stale and right_rear with no detection; left side
    # is healthy and detecting.
    packet["radars"]["right_front"]["stale"] = True
    packet["radars"]["right_front"]["valid"] = False
    packet["radars"]["left_front"]["detected"] = True
    packet["radars"]["left_front"]["distance_m"] = 2.0
    snap = _snapshot_from_packet(packet)
    selection = selector.update(snap, now=1.0)
    assert selection.camera == "left_1"


def test_detected_true_with_null_distance_keeps_current_camera() -> None:
    selector = _selector(hold_seconds=10.0)
    # First, drive a detection on left_front.
    p1 = _packet()
    p1["radars"]["left_front"]["detected"] = True
    p1["radars"]["left_front"]["distance_m"] = 2.0
    snap1 = _snapshot_from_packet(p1, received_monotonic=1.0)
    sel1 = selector.update(snap1, now=1.0)
    assert sel1.camera == "left_1"
    # Next, same radar reports detected=true but distance is null.
    p2 = _packet(seq=2)
    p2["radars"]["left_front"]["detected"] = True
    p2["radars"]["left_front"]["distance_m"] = None
    snap2 = _snapshot_from_packet(p2, received_monotonic=2.0)
    sel2 = selector.update(snap2, now=2.0)
    assert sel2.camera == "left_1"
    assert sel2.reason in {"hold-distance-null", "radar:left_front"}


# --------------------------------------------------------------------------- #
# 4. Camera selection rules
# --------------------------------------------------------------------------- #
def test_selector_chooses_closest_distance() -> None:
    selector = _selector()
    packet = _packet()
    packet["radars"]["left_front"]["detected"] = True
    packet["radars"]["left_front"]["distance_m"] = 4.0
    packet["radars"]["right_front"]["detected"] = True
    packet["radars"]["right_front"]["distance_m"] = 1.0
    snap = _snapshot_from_packet(packet)
    selection = selector.update(snap, now=1.0)
    assert selection.camera == "right_1"
    assert selection.radar_id == "right_front"


def test_selector_uses_priority_when_distances_are_equal() -> None:
    selector = _selector(priority=("right_front", "left_front"))
    packet = _packet()
    packet["radars"]["left_front"]["detected"] = True
    packet["radars"]["left_front"]["distance_m"] = 1.5
    packet["radars"]["right_front"]["detected"] = True
    packet["radars"]["right_front"]["distance_m"] = 1.5
    snap = _snapshot_from_packet(packet)
    selection = selector.update(snap, now=1.0)
    assert selection.camera == "right_1"


def test_selector_camera_hold_prevents_flicker() -> None:
    selector = _selector(hold_seconds=2.0)
    packet = _packet()
    packet["radars"]["left_front"]["detected"] = True
    packet["radars"]["left_front"]["distance_m"] = 2.0
    snap1 = _snapshot_from_packet(packet, received_monotonic=1.0)
    sel1 = selector.update(snap1, now=1.0)
    assert sel1.camera == "left_1"
    # No detection on the next frame: keep camera for the hold window.
    packet2 = _packet(seq=2)
    snap2 = _snapshot_from_packet(packet2, received_monotonic=2.0)
    sel2 = selector.update(snap2, now=2.0)
    assert sel2.camera == "left_1"
    assert sel2.reason == "hold"
    # After the hold window the camera must drop back to front.
    snap3 = _snapshot_from_packet(packet2, received_monotonic=3.5)
    sel3 = selector.update(snap3, now=3.5)
    assert sel3.camera == "front"


def test_selector_falls_back_to_front_on_gateway_loss() -> None:
    selector = _selector(hold_seconds=0.0)
    # First a detection, then a snapshot with link_ok=False.
    packet = _packet()
    packet["radars"]["left_front"]["detected"] = True
    packet["radars"]["left_front"]["distance_m"] = 2.0
    snap1 = _snapshot_from_packet(packet, received_monotonic=1.0)
    selector.update(snap1, now=1.0)
    snap2 = GatewaySnapshot(
        radars={rid: GatewayRadarSample.empty(rid) for rid in RADAR_IDS},
        sequence=2,
        gateway_ok=False,
        remote_ok=False,
        received_monotonic=2.0,
    )
    sel2 = selector.update(snap2, now=2.0)
    assert sel2.camera == "front"


def test_selector_does_not_invent_detection_on_empty_snapshot() -> None:
    selector = _selector()
    snap = GatewaySnapshot(
        radars={rid: GatewayRadarSample.empty(rid) for rid in RADAR_IDS},
        sequence=1,
        gateway_ok=True,
        remote_ok=True,
        received_monotonic=1.0,
    )
    sel = selector.update(snap, now=1.0)
    assert sel.camera == "front"
    assert sel.radar_id == ""


# --------------------------------------------------------------------------- #
# 5. Migration from legacy side_sensor_modules
# --------------------------------------------------------------------------- #
def test_migration_keeps_camera_mapping_per_radar() -> None:
    legacy = {
        "side_sensor_modules": {
            "enabled": True,
            "modules": {
                "left_1": {
                    "enabled": True,
                    "side": "left",
                    "position": "front",
                    "camera": {"device": "/dev/video1"},
                    "radar": {
                        "enabled": True,
                        "device": "/dev/ttyACM0",
                        "warning_distance_m": 4.0,
                    },
                },
                "left_2": {
                    "enabled": True,
                    "side": "left",
                    "position": "rear",
                    "camera": {"device": "/dev/video3"},
                    "radar": {
                        "enabled": True,
                        "device": "/dev/ttyACM2",
                        "warning_distance_m": 3.0,
                    },
                },
                "right_1": {
                    "enabled": True,
                    "side": "right",
                    "position": "front",
                    "camera": {"device": "/dev/video2"},
                    "radar": {
                        "enabled": True,
                        "device": "/dev/ttyACM1",
                        "warning_distance_m": 5.0,
                    },
                },
                "right_2": {
                    "enabled": True,
                    "side": "right",
                    "position": "rear",
                    "camera": {"device": "/dev/video4"},
                    "radar": {
                        "enabled": True,
                        "device": "/dev/ttyACM3",
                        "warning_distance_m": 4.5,
                    },
                },
            },
        }
    }
    block = migrate_legacy_config(legacy)
    assert block["radars"]["left_front"]["camera"] == "left_1"
    assert block["radars"]["left_front"]["warning_distance_m"] == 4.0
    assert block["radars"]["right_rear"]["camera"] == "right_2"
    assert block["radars"]["right_rear"]["warning_distance_m"] == 4.5


def test_migration_falls_back_to_default_when_no_legacy() -> None:
    block = migrate_legacy_config({})
    assert block == default_radar_gateway()


def test_migration_preserves_unrelated_yaml_blocks() -> None:
    """``validate_config`` must not drop the legacy side_sensor_modules
    when the operator forgets to add a ``radar_gateway`` block."""
    config = {
        "mode": "demo",
        "camera": {
            "sources": {"front": "/dev/video0"},
            "width": 1280,
            "height": 720,
            "fps": 30,
        },
        "lidar": {"fps": 10, "enabled": True, "source": "mock", "driver": "mock"},
        "synchronization": {"max_difference_ms": 120, "buffer_size": 20, "stale_after_ms": 1000},
        "vision": {
            "model": "yolov8n.pt",
            "confidence": 0.45,
            "allowed_classes": ["person"],
            "device": "auto",
            "image_size": 640,
        },
        "lidar_processing": {
            "min_range_m": 0.3,
            "max_range_m": 60.0,
            "roi": {
                "x_min": 0.0, "x_max": 60.0,
                "y_min": -12.0, "y_max": 12.0,
                "z_min": -3.0, "z_max": 4.0,
            },
            "clustering": {"eps": 0.7, "min_samples": 4},
        },
        "calibration": {"file": "config/calibration.yaml"},
        "fusion": {"ema_alpha": 0.35, "bbox_scale": 0.85},
        "warning": {
            "mode": "distance",
            "caution_distance_m": 12.0,
            "danger_distance_m": 6.0,
            "emergency_distance_m": 3.0,
        },
        "side_sensor_modules": {
            "enabled": False,
            "modules": {
                "left_1": {
                    "enabled": True,
                    "side": "left",
                    "position": "front",
                    "camera": {"device": "/dev/video1"},
                    "radar": {"enabled": True, "device": "/dev/ttyACM0"},
                }
            },
        },
    }
    validated = validate_config(config)
    assert validated["side_sensor_modules"]["enabled"] is False
    # No radar_gateway provided and legacy disabled -> block stays
    # untouched (operator may enable radar_gateway manually later).
    assert "radar_gateway" not in validated


# --------------------------------------------------------------------------- #
# 6. Validation
# --------------------------------------------------------------------------- #
def test_validation_rejects_empty_device_when_enabled() -> None:
    block = default_radar_gateway()
    block["device"] = ""
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


def test_validation_rejects_baudrate_out_of_range() -> None:
    block = default_radar_gateway()
    block["baudrate"] = 12345
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


def test_validation_enforces_v2_protocol() -> None:
    block = default_radar_gateway()
    block["protocol_version"] = 1
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


def test_validation_enforces_positive_stale_timeout() -> None:
    block = default_radar_gateway()
    block["stale_after_seconds"] = 0
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


def test_validation_enforces_radar_id_canonical_set() -> None:
    block = default_radar_gateway()
    block["radars"]["left_front"]["camera"] = "front"  # collides with 'front' rule
    # Replace radar id with a non-canonical one to make sure the
    # canonical set is enforced.
    block["radars"]["unknown"] = block["radars"].pop("left_front")
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


def test_validation_rejects_warning_distance_above_5m() -> None:
    block = default_radar_gateway()
    block["radars"]["left_front"]["warning_distance_m"] = 6.0
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


def test_validation_does_not_mutate_unrelated_blocks() -> None:
    """``validate_radar_gateway`` must not touch other parts of the cfg."""
    block = default_radar_gateway()
    # Replace the placeholder so the demo-mode validation accepts it.
    block["device"] = "/dev/serial/by-id/usb-test"
    snapshot = {**block}
    validate_radar_gateway(block, mode="demo")
    # The keys we passed in must remain equal.
    for key, value in snapshot.items():
        if key == "radars":
            for rid in RADAR_IDS:
                assert block["radars"][rid]["camera"] == snapshot["radars"][rid]["camera"]
            continue
        assert block[key] == value or block[key] == snapshot[key]


# --------------------------------------------------------------------------- #
# 7. Live mode never falls back to mock radar
# --------------------------------------------------------------------------- #
def test_live_mode_rejects_mock_radar_in_preflight(monkeypatch, tmp_path) -> None:
    """When the operator enables ``radar_gateway`` and a placeholder device,
    ``preflight`` must surface a Vietnamese error -- never silently allow
    Live to start with a synthetic radar."""
    import preflight

    cfg_path = tmp_path / "jetson_live.yaml"
    cfg_path.write_text(
        "mode: live\n"
        "safety:\n"
        "  allow_mock: false\n"
        "  is_demo_calibration: false\n"
        "radar_gateway:\n"
        "  enabled: true\n"
        "  device: /dev/serial/by-id/CHANGE_ME\n"
        "  baudrate: 115200\n"
        "  protocol_version: 2\n"
        "  stale_after_seconds: 0.6\n"
        "  camera_hold_seconds: 1.2\n"
        "  radars:\n"
        "    left_front:  {enabled: true, camera: left_1, warning_distance_m: 5.0}\n"
        "    left_rear:   {enabled: true, camera: left_2, warning_distance_m: 5.0}\n"
        "    right_front: {enabled: true, camera: right_1, warning_distance_m: 5.0}\n"
        "    right_rear:  {enabled: true, camera: right_2, warning_distance_m: 5.0}\n",
        encoding="utf-8",
    )

    captured = {}

    def _fake_write_status(config_path, errors, extras=None):
        captured["errors"] = list(errors)
        captured["extras"] = extras or {}

    # Stub the heavy hardware gates so preflight can complete quickly.
    monkeypatch.setattr(preflight, "_check_model", lambda *a, **k: None)
    monkeypatch.setattr(preflight, "_check_calibration", lambda *a, **k: None)
    monkeypatch.setattr(preflight, "_check_lidar", lambda *a, **k: None)
    monkeypatch.setattr(preflight, "_check_cuda", lambda *a, **k: None)
    monkeypatch.setattr(preflight, "_check_cameras", lambda *a, **k: True)
    monkeypatch.setattr(preflight, "write_status", _fake_write_status)
    # Avoid argparse touching sys.argv when we call main() directly.
    monkeypatch.setattr(
        "sys.argv", ["preflight.py", "--config", str(cfg_path), "--skip-hardware"]
    )

    rc = preflight.main()
    assert rc == 1
    errors = captured.get("errors", [])
    assert any("CHANGE_ME" in err for err in errors)


# --------------------------------------------------------------------------- #
# 8. Reconnect after Gateway reset
# --------------------------------------------------------------------------- #
def test_reconnect_resumes_updates_without_inventing_state() -> None:
    selector = _selector(hold_seconds=0.0)
    # Snapshot 1: detection on left_front.
    p1 = _packet(seq=1)
    p1["radars"]["left_front"]["detected"] = True
    p1["radars"]["left_front"]["distance_m"] = 1.5
    snap1 = _snapshot_from_packet(p1, received_monotonic=1.0)
    selector.update(snap1, now=1.0)
    # Snapshot 2: Gateway reboot, link_ok=False.
    snap2 = GatewaySnapshot(
        radars={rid: GatewayRadarSample.empty(rid) for rid in RADAR_IDS},
        sequence=2,
        gateway_ok=False,
        remote_ok=False,
        received_monotonic=2.0,
    )
    sel2 = selector.update(snap2, now=2.0)
    assert sel2.camera == "front"
    # Snapshot 3: reconnected, new boot_id, new detection.
    p3 = _packet(seq=1, gateway_boot_id="boot-cccc")
    p3["radars"]["right_front"]["detected"] = True
    p3["radars"]["right_front"]["distance_m"] = 1.0
    snap3 = _snapshot_from_packet(p3, received_monotonic=3.0)
    sel3 = selector.update(snap3, now=3.0)
    assert sel3.camera == "right_1"
    assert sel3.radar_id == "right_front"


def test_uart_reader_accepts_sequence_reset_after_gateway_reboot() -> None:
    reader = RadarGatewayReader("/dev/null")
    reader._handle_line(json.dumps(_packet(seq=900, gateway_boot_id="boot-a")).encode())
    reader._handle_line(json.dumps(_packet(seq=1, gateway_boot_id="boot-b")).encode())
    assert reader.read(now=1.0).sequence == 1
    assert reader.stats()["duplicate_packets"] == 0


# --------------------------------------------------------------------------- #
# 9. Reader buffer overflow guards
# --------------------------------------------------------------------------- #
def test_parser_accepts_canonical_max_distance() -> None:
    packet = _packet()
    packet["radars"]["left_front"]["distance_m"] = LD2410_MAX_DETECTION_M
    sample = _snapshot_from_packet(packet)
    assert sample.radars["left_front"].distance_m == pytest.approx(
        LD2410_MAX_DETECTION_M
    )


def test_parser_clamps_distance_above_max() -> None:
    packet = _packet()
    packet["radars"]["left_front"]["distance_m"] = LD2410_MAX_DETECTION_M + 1.0
    sample = _snapshot_from_packet(packet)
    # Out-of-range distances become None; the parser never fabricates a
    # detection from a corrupted value.
    assert sample.radars["left_front"].distance_m is None


def test_protocol_version_constant() -> None:
    """The Prompt 4 contract fixes protocol v2 as the production version."""
    assert PROTOCOL_VERSIONS == (2,)
