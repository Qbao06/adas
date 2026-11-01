"""Prompt 4 — dashboard & preflight hardening for the single-Gateway radar.

These tests cover the security and persistence guarantees declared by Prompt 4:

* ``config.radar_gateway`` allowlist rejects path traversal, control chars,
  non-/dev paths, ``..`` segments and bare ``CHANGE_ME`` placeholders.
* ``save_radar_gateway`` writes the YAML atomically (uses ``os.replace`` on
  a ``.tmp`` file) and never mutates unrelated YAML blocks.
* Dashboard API endpoints (``GET /api/radar/config``, ``PUT``,
  ``GET /api/devices``, ``POST /api/radar/test``) validate payloads, refuse
  out-of-allowlist devices, surface 409 when busy and never touch hardware
  in demo mode.
* ``probe_gateway`` returns stable ``category`` identifiers for every failure
  mode: device_missing, permission_denied, port_busy, bad_baudrate,
  version_mismatch, json_invalid, timeout, disallowed_device.
* ``preflight.main`` blocks live mode on ``CHANGE_ME``, forbids mock radar,
  surfaces Vietnamese errors and writes a structured status JSON.
"""
from __future__ import annotations

import json
import socket
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from config import (
    ConfigError,
    is_allowed_device,
    sanitize_device_path,
    save_radar_gateway,
    validate_radar_gateway,
)
from dashboard.server import create_dashboard_server
from sensors import FakeGatewaySource, probe_gateway

ROOT = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------------------
# Path-traversal & allowlist safety
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("path", [
    "",
    "/etc/passwd",
    "/root/.ssh/id_rsa",
    "/dev/../etc/passwd",
    "/dev/../../etc/passwd",
    "/dev/serial/by-id/../../etc/passwd",
    "../../etc/passwd",
    "..%2F..%2F",
    "..%252F..%252F",
    "/dev/ttyACM0; rm -rf /",
    "/dev/serial/by-id/\x00malicious",
    "/dev/serial/by-id/usb\x07bell",
    "/dev/random",
    "C:\\Windows\\System32",
])
def test_is_allowed_device_rejects_unsafe_paths(path: str) -> None:
    """The allowlist must reject anything outside the read-only /dev prefixes.

    The Prompt 4 contract forbids path traversal, control characters and
    arbitrary file reads; ``is_allowed_device`` is the single guard used by
    every endpoint and probe.
    """
    assert is_allowed_device(path) is False


@pytest.mark.parametrize("path", [
    "/dev/serial/by-id/usb-ESP32_Gateway_AAAA-if00",
    "/dev/ttyACM0",
    "/dev/ttyUSB1",
    "/dev/v4l/by-id/usb-Generic_Camera_200901010001-video-index0",
    "/dev/video0",
])
def test_is_allowed_device_accepts_safe_paths(path: str) -> None:
    assert is_allowed_device(path) is True


@pytest.mark.parametrize("path", [
    "../../etc/passwd",
    "/etc/passwd",
    "/dev/serial/by-id/../../etc/passwd",
    "/dev/random",
    "/dev/serial/by-id/\x00bad",
])
def test_sanitize_device_path_raises(path: str) -> None:
    with pytest.raises(ConfigError):
        sanitize_device_path(path)


def test_sanitize_device_path_returns_normalized_absolute_path() -> None:
    cleaned = sanitize_device_path("/dev/serial/by-id/usb-foo//bar")
    assert cleaned == "/dev/serial/by-id/usb-foo/bar"


# ---------------------------------------------------------------------------
# YAML atomic save + unrelated-key preservation
# ---------------------------------------------------------------------------


def _write_seed_config(path: Path) -> None:
    payload = {
        "mode": "live",
        "camera": {"source": 0, "sources": {"front": "/dev/video0"}},
        "lidar": {"driver": "real", "source": "/dev/ttyUSB1"},
        "side_radar": {"enabled": False},
        "calibration": {"file": "config/calibration_live.yaml"},
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_save_radar_gateway_is_atomic(tmp_path: Path) -> None:
    """``save_radar_gateway`` writes via ``.tmp`` then ``os.replace`` so a
    crash mid-write never produces a half-written YAML file."""
    cfg = tmp_path / "jetson_live.yaml"
    _write_seed_config(cfg)
    block = {
        "enabled": True,
        "device": "/dev/serial/by-id/usb-test",
        "baudrate": 115200,
        "protocol_version": 2,
        "stale_after_seconds": 0.5,
        "camera_hold_seconds": 1.0,
        "radars": {
            "left_front": {"enabled": True, "camera": "left_1", "warning_distance_m": 4.0},
            "left_rear": {"enabled": True, "camera": "left_2", "warning_distance_m": 5.0},
            "right_front": {"enabled": True, "camera": "right_1", "warning_distance_m": 3.5},
            "right_rear": {"enabled": True, "camera": "right_2", "warning_distance_m": 4.5},
        },
    }
    normalised = save_radar_gateway(cfg, block)
    assert normalised["enabled"] is True
    assert normalised["device"] == "/dev/serial/by-id/usb-test"
    # Reload and verify only the radar_gateway block changed.
    import yaml
    reloaded = yaml.safe_load(cfg.read_text(encoding="utf-8"))
    assert reloaded["mode"] == "live"
    assert reloaded["camera"]["sources"]["front"] == "/dev/video0"
    assert reloaded["lidar"]["driver"] == "real"
    assert reloaded["side_radar"]["enabled"] is False
    assert reloaded["calibration"]["file"] == "config/calibration_live.yaml"
    assert reloaded["radar_gateway"]["device"] == "/dev/serial/by-id/usb-test"
    # .tmp sidecar must not linger after a successful save.
    assert not (cfg.with_suffix(cfg.suffix + ".tmp")).exists()


def test_save_radar_gateway_rejects_invalid_block(tmp_path: Path) -> None:
    cfg = tmp_path / "jetson_live.yaml"
    _write_seed_config(cfg)
    with pytest.raises(ConfigError):
        save_radar_gateway(cfg, {"enabled": True, "device": "/etc/passwd"})


def test_save_radar_gateway_rejects_placeholder_in_live(tmp_path: Path) -> None:
    cfg = tmp_path / "jetson_live.yaml"
    _write_seed_config(cfg)
    with pytest.raises(ConfigError):
        save_radar_gateway(cfg, {
            "enabled": True,
            "device": "/dev/serial/by-id/CHANGE_ME",
        })


# ---------------------------------------------------------------------------
# Config validation — schema hardening for Prompt 4
# ---------------------------------------------------------------------------


def _radar_block(**overrides):
    block = {
        "enabled": True,
        "device": "/dev/serial/by-id/usb-test",
        "baudrate": 115200,
        "protocol_version": 2,
        "stale_after_seconds": 0.6,
        "camera_hold_seconds": 1.2,
        "radars": {
            "left_front": {"enabled": True, "camera": "left_1", "warning_distance_m": 5.0},
            "left_rear": {"enabled": True, "camera": "left_2", "warning_distance_m": 5.0},
            "right_front": {"enabled": True, "camera": "right_1", "warning_distance_m": 5.0},
            "right_rear": {"enabled": True, "camera": "right_2", "warning_distance_m": 5.0},
        },
    }
    block.update(overrides)
    return block


def test_validate_rejects_change_me_in_live() -> None:
    block = _radar_block(device="/dev/serial/by-id/CHANGE_ME")
    with pytest.raises(ConfigError):
        validate_radar_gateway(block, mode="live")


def test_validate_accepts_change_me_in_demo() -> None:
    block = _radar_block(device="/dev/serial/by-id/CHANGE_ME")
    normalised = validate_radar_gateway(block, mode="demo")
    assert normalised["device"] == "/dev/serial/by-id/CHANGE_ME"


@pytest.mark.parametrize("baudrate", [110, 1234, 500000, "115200"])
def test_validate_rejects_invalid_baudrate(baudrate) -> None:
    block = _radar_block(baudrate=baudrate)
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


@pytest.mark.parametrize("protocol_version", [0, 1, 3, "2", None])
def test_validate_rejects_non_v2_protocol(protocol_version) -> None:
    block = _radar_block(protocol_version=protocol_version)
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


def test_validate_rejects_warning_distance_over_5m() -> None:
    block = _radar_block()
    block["radars"]["left_front"]["warning_distance_m"] = 6.0
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


def test_validate_rejects_zero_or_negative_warning_distance() -> None:
    block = _radar_block()
    block["radars"]["left_rear"]["warning_distance_m"] = 0.0
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)
    block["radars"]["left_rear"]["warning_distance_m"] = -1.0
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


def test_validate_rejects_duplicate_camera_mapping() -> None:
    block = _radar_block()
    block["radars"]["left_rear"]["camera"] = "left_1"  # already used by left_front
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


def test_validate_rejects_enabled_but_no_radar_active() -> None:
    block = _radar_block()
    for radar in block["radars"].values():
        radar["enabled"] = False
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


def test_validate_rejects_negative_stale_timeout() -> None:
    block = _radar_block(stale_after_seconds=-0.5)
    with pytest.raises(ConfigError):
        validate_radar_gateway(block)


def test_validate_rejects_camera_hold_out_of_range() -> None:
    with pytest.raises(ConfigError):
        validate_radar_gateway(_radar_block(camera_hold_seconds=11))
    with pytest.raises(ConfigError):
        validate_radar_gateway(_radar_block(camera_hold_seconds=-1))


# ---------------------------------------------------------------------------
# probe_gateway — error categorisation
# ---------------------------------------------------------------------------


def _good_v2_payload(seq: int = 1, gateway_ok: bool = True, remote_ok: bool = True) -> str:
    return json.dumps({
        "v": 2,
        "seq": seq,
        "gateway_ok": gateway_ok,
        "remote_ok": remote_ok,
        "gateway_boot_id": "boot-aaaa",
        "remote_boot_id": "boot-bbbb",
        "packet_loss": 0.0,
        "radars": {
            rid: {"detected": False, "valid": True, "stale": False,
                  "distance_m": None, "moving_energy": 0, "stationary_energy": 0,
                  "age_ms": 20, "error": ""}
            for rid in ("left_front", "left_rear", "right_front", "right_rear")
        },
    })


def test_probe_gateway_disallowed_device_has_category() -> None:
    report = probe_gateway("/etc/passwd")
    assert report["ok"] is False
    assert report["category"] == "disallowed_device"


def test_probe_gateway_bad_baudrate_has_category() -> None:
    report = probe_gateway("/dev/ttyACM0", baudrate=12345)
    assert report["ok"] is False
    assert report["category"] == "bad_baudrate"


def test_probe_gateway_missing_device_has_category() -> None:
    report = probe_gateway("/dev/serial/by-id/no-such-device-aaaa")
    assert report["ok"] is False
    assert report["category"] in {"device_missing", "permission_denied"}


def test_probe_gateway_version_mismatch_category() -> None:
    bad_lines = [json.dumps({"v": 1, "seq": 1, "radars": {}})]
    report = probe_gateway("/dev/null", source=FakeGatewaySource(bad_lines), duration=0.2)
    assert report["ok"] is False
    assert report["category"] == "version_mismatch"


def test_probe_gateway_json_invalid_category() -> None:
    bad_lines = ["{not json}\n", "<<<broken>>>\n"]
    report = probe_gateway("/dev/null", source=FakeGatewaySource(bad_lines), duration=0.2)
    assert report["ok"] is False
    assert report["category"] == "json_invalid"


def test_probe_gateway_timeout_category_when_silent() -> None:
    report = probe_gateway("/dev/null", source=FakeGatewaySource([]), duration=0.2)
    assert report["ok"] is False
    assert report["category"] == "timeout"


def test_probe_gateway_returns_seq_and_4_radars_on_success() -> None:
    lines = [_good_v2_payload(seq=1), _good_v2_payload(seq=2)]
    report = probe_gateway("/dev/null", source=FakeGatewaySource(lines), duration=0.5)
    assert report["ok"] is True
    assert report["seq"] == 2
    assert report["seq_increasing"] is True
    assert set(report["radars"]) == {"left_front", "left_rear", "right_front", "right_rear"}


# ---------------------------------------------------------------------------
# Dashboard API — Prompt 4 endpoints
# ---------------------------------------------------------------------------


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def radar_dashboard(tmp_path):
    status = tmp_path / "status.json"
    status.write_text(json.dumps({"ready": False, "errors": ["Khởi tạo"]}),
                      encoding="utf-8")
    cfg = tmp_path / "jetson_live.yaml"
    cfg.write_text(
        "mode: live\n"
        "radar_gateway:\n"
        "  enabled: true\n"
        "  device: /dev/serial/by-id/CHANGE_ME\n"
        "  baudrate: 115200\n"
        "  protocol_version: 2\n"
        "  stale_after_seconds: 0.6\n"
        "  camera_hold_seconds: 1.2\n"
        "  radars:\n"
        "    left_front:\n"
        "      enabled: true\n"
        "      camera: left_1\n"
        "      warning_distance_m: 5.0\n"
        "    left_rear:\n"
        "      enabled: true\n"
        "      camera: left_2\n"
        "      warning_distance_m: 5.0\n"
        "    right_front:\n"
        "      enabled: true\n"
        "      camera: right_1\n"
        "      warning_distance_m: 5.0\n"
        "    right_rear:\n"
        "      enabled: true\n"
        "      camera: right_2\n"
        "      warning_distance_m: 5.0\n",
        encoding="utf-8",
    )
    port = _free_port()
    server = create_dashboard_server(
        host="127.0.0.1",
        port=port,
        status_path=status,
        config_path=cfg,
        demo=False,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.1)
    yield {"server": server, "port": port, "cfg": cfg, "status": status}
    server.shutdown()
    server.server_close()


def _put(url: str, payload: dict) -> urllib.request.Request:
    return urllib.request.Request(
        url, data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="PUT",
    )


def _post(url: str, payload: dict) -> urllib.request.Request:
    return urllib.request.Request(
        url, data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST",
    )


def test_dashboard_get_radar_config_returns_block(radar_dashboard) -> None:
    port = radar_dashboard["port"]
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/radar/config") as resp:
        assert resp.status == 200
        data = json.loads(resp.read().decode("utf-8"))
    assert data["ok"] is True
    assert data["config"]["enabled"] is True
    assert data["config"]["device"].endswith("CHANGE_ME")


def test_dashboard_get_devices_returns_allowlisted_only(radar_dashboard) -> None:
    port = radar_dashboard["port"]
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/devices") as resp:
        data = json.loads(resp.read().decode("utf-8"))
    assert data["ok"] is True
    # Every returned path must pass the allowlist guard.
    for label, items in data["devices"].items():
        assert label in {"by_id", "ttyacm", "ttyusb", "v4l_by_id", "video"}
        for path in items:
            assert is_allowed_device(path), f"{label} returned disallowed path {path}"


def test_dashboard_radar_status_empty_when_no_log(radar_dashboard) -> None:
    port = radar_dashboard["port"]
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/radar/status") as resp:
        data = json.loads(resp.read().decode("utf-8"))
    assert data["ok"] is True
    assert data["gateway_online"] is False
    assert data["remote_online"] is False
    assert set(data["radars"]) == {"left_front", "left_rear", "right_front", "right_rear"}


def test_dashboard_put_radar_config_rejects_path_traversal(radar_dashboard) -> None:
    port = radar_dashboard["port"]
    req = _put(f"http://127.0.0.1:{port}/api/radar/config", {
        "config": {"enabled": True, "device": "/etc/passwd"},
    })
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(req)
    assert exc.value.code == 400


def test_dashboard_put_radar_config_rejects_change_me_in_live(radar_dashboard) -> None:
    port = radar_dashboard["port"]
    req = _put(f"http://127.0.0.1:{port}/api/radar/config", {
        "config": {"enabled": True, "device": "/dev/serial/by-id/CHANGE_ME"},
    })
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(req)
    assert exc.value.code == 400
    body = json.loads(exc.value.read().decode("utf-8"))
    assert "CHANGE_ME" in body["error"]


def test_dashboard_put_radar_config_atomic_save(radar_dashboard) -> None:
    port = radar_dashboard["port"]
    cfg_path: Path = radar_dashboard["cfg"]
    req = _put(f"http://127.0.0.1:{port}/api/radar/config", {
        "config": {
            "enabled": True,
            "device": "/dev/serial/by-id/usb-ESP32_A_Gateway-if00",
            "baudrate": 115200,
            "protocol_version": 2,
            "stale_after_seconds": 0.5,
            "camera_hold_seconds": 1.0,
            "radars": {
                "left_front": {"enabled": True, "camera": "left_1", "warning_distance_m": 5.0},
                "left_rear":  {"enabled": True, "camera": "left_2", "warning_distance_m": 5.0},
                "right_front":{"enabled": True, "camera": "right_1", "warning_distance_m": 5.0},
                "right_rear": {"enabled": True, "camera": "right_2", "warning_distance_m": 5.0},
            },
        }
    })
    with urllib.request.urlopen(req) as resp:
        body = json.loads(resp.read().decode("utf-8"))
    assert body["ok"] is True
    assert body["restart_required"] is True
    assert body["config"]["device"] == "/dev/serial/by-id/usb-ESP32_A_Gateway-if00"
    # The file on disk was updated, and the .tmp sidecar is gone.
    text = cfg_path.read_text(encoding="utf-8")
    assert "usb-ESP32_A_Gateway-if00" in text
    assert not (cfg_path.with_suffix(cfg_path.suffix + ".tmp")).exists()


def test_dashboard_radar_test_uses_fake_source(radar_dashboard) -> None:
    port = radar_dashboard["port"]
    payload = _good_v2_payload(seq=10)
    req = _post(f"http://127.0.0.1:{port}/api/radar/test", {
        "device": "/dev/null",
        "baudrate": 115200,
        "duration": 0.5,
        "fake_lines": [payload],
    })
    with urllib.request.urlopen(req) as resp:
        body = json.loads(resp.read().decode("utf-8"))
    assert body["ok"] is True
    assert body["device"] == "<fake>"
    assert body["seq"] == 10


def test_dashboard_radar_test_rejects_path_traversal(radar_dashboard) -> None:
    port = radar_dashboard["port"]
    req = _post(f"http://127.0.0.1:{port}/api/radar/test", {"device": "/etc/passwd"})
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(req)
    assert exc.value.code == 400


def test_dashboard_radar_test_rejects_payload_too_large(radar_dashboard) -> None:
    port = radar_dashboard["port"]
    big = "x" * 16384
    req = _post(f"http://127.0.0.1:{port}/api/radar/test", {"device": big})
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(req)
    assert exc.value.code == 400


# ---------------------------------------------------------------------------
# Preflight — Prompt 4 hardening
# ---------------------------------------------------------------------------


def test_preflight_demo_passes_when_config_demo(tmp_path, monkeypatch) -> None:
    cfg = tmp_path / "jetson_live.yaml"
    cfg.write_text(
        "mode: demo\n"
        "safety:\n"
        "  allow_mock: true\n"
        "  is_demo_calibration: true\n",
        encoding="utf-8",
    )
    monkeypatch.setattr("sys.argv", ["preflight.py", "--config", str(cfg)])
    import preflight
    rc = preflight.main()
    assert rc == 0


def test_preflight_live_blocks_change_me(tmp_path, monkeypatch) -> None:
    cfg = tmp_path / "jetson_live.yaml"
    cfg.write_text(
        "mode: live\n"
        "safety:\n"
        "  allow_mock: false\n"
        "  is_demo_calibration: true\n"
        "camera:\n"
        "  sources: {front: /dev/video0}\n"
        "vision:\n"
        "  model: models/none.pt\n"
        "  allowed_classes: []\n"
        "lidar:\n"
        "  driver: mock\n"
        "radar_gateway:\n"
        "  enabled: true\n"
        "  device: /dev/serial/by-id/CHANGE_ME\n"
        "  baudrate: 115200\n"
        "  protocol_version: 2\n",
        encoding="utf-8",
    )
    monkeypatch.setattr("sys.argv", ["preflight.py", "--config", str(cfg)])
    import preflight
    rc = preflight.main()
    assert rc == 1
    payload = json.loads(preflight.STATUS_PATH.read_text(encoding="utf-8"))
    assert payload["ready"] is False
    assert any("CHANGE_ME" in err for err in payload["errors"])


def test_preflight_live_blocks_disallowed_device(tmp_path, monkeypatch) -> None:
    cfg = tmp_path / "jetson_live.yaml"
    cfg.write_text(
        "mode: live\n"
        "safety:\n"
        "  allow_mock: false\n"
        "  is_demo_calibration: true\n"
        "camera:\n"
        "  sources: {front: /dev/video0}\n"
        "vision:\n"
        "  model: models/none.pt\n"
        "  allowed_classes: []\n"
        "lidar:\n"
        "  driver: mock\n"
        "radar_gateway:\n"
        "  enabled: true\n"
        "  device: /etc/passwd\n"
        "  baudrate: 115200\n"
        "  protocol_version: 2\n",
        encoding="utf-8",
    )
    monkeypatch.setattr("sys.argv", ["preflight.py", "--config", str(cfg)])
    import preflight
    rc = preflight.main()
    assert rc == 1
    payload = json.loads(preflight.STATUS_PATH.read_text(encoding="utf-8"))
    assert any("allowlist" in err or "CHANGE_ME" in err or "live" in err.lower()
               for err in payload["errors"])


def test_preflight_writes_status_json(tmp_path, monkeypatch) -> None:
    cfg = tmp_path / "jetson_live.yaml"
    cfg.write_text(
        "mode: demo\n"
        "safety:\n"
        "  allow_mock: true\n"
        "  is_demo_calibration: true\n",
        encoding="utf-8",
    )
    monkeypatch.setattr("sys.argv", ["preflight.py", "--config", str(cfg)])
    import preflight
    rc = preflight.main()
    assert rc == 0
    status = json.loads(preflight.STATUS_PATH.read_text(encoding="utf-8"))
    assert status["ready"] is True
    assert status["errors"] == []
    assert status["config"] == str(cfg)
