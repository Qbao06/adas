"""Hardware assignment: camera slots, COM ports, LiDAR, dashboard API."""
from __future__ import annotations

import json
import socket
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest
import yaml

from config import (
    ConfigError,
    hardware_from_config,
    is_allowed_camera_source,
    is_allowed_device,
    is_com_port,
    list_hardware_devices,
    sanitize_device_path,
    save_hardware,
    validate_hardware,
)
from dashboard.server import create_dashboard_server


def _payload(**overrides) -> dict:
    base = {
        "camera": {
            "enabled": True,
            "width": 1280,
            "height": 720,
            "fps": 30,
            "slots": {
                "front": {"enabled": True, "device": "/dev/video0"},
                "left_1": {"enabled": True, "device": "/dev/video1"},
                "left_2": {"enabled": False, "device": ""},
                "right_1": {"enabled": True, "device": "/dev/video2"},
                "right_2": {"enabled": False, "device": ""},
                "rear": {"enabled": True, "device": "/dev/video4"},
            },
        },
        "radar_gateway": {
            "enabled": True,
            "device": "/dev/serial/by-id/CHANGE_ME",
            "baudrate": 115200,
        },
        "lidar": {
            "enabled": True,
            "driver": "CHANGE_ME",
            "source": "CHANGE_ME",
            "fps": 10,
            "driver_module": "drivers.real_lidar",
        },
    }
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merged = dict(base[key])
            merged.update(value)
            base[key] = merged
        else:
            base[key] = value
    return base


def test_is_com_port_accepts_com1_to_com256() -> None:
    assert is_com_port("COM1") is True
    assert is_com_port("com12") is True
    assert is_com_port("COM256") is True
    assert is_com_port("COM0") is False
    assert is_com_port("COM257") is False
    assert is_com_port("COM03") is False
    assert is_com_port("COM3;calc") is False


def test_is_allowed_device_accepts_com_and_linux_paths() -> None:
    assert is_allowed_device("COM3") is True
    assert is_allowed_device("com8") is True
    assert is_allowed_device("/dev/ttyACM0") is True
    assert is_allowed_device("/dev/video0") is True
    assert is_allowed_device("C:\\Windows\\System32") is False
    assert is_allowed_device("/etc/passwd") is False


def test_sanitize_com_port_uppercases() -> None:
    assert sanitize_device_path("com3") == "COM3"
    assert sanitize_device_path("/dev/serial/by-id/usb-foo//bar") == "/dev/serial/by-id/usb-foo/bar"


def test_camera_source_allowlist() -> None:
    assert is_allowed_camera_source(0) is True
    assert is_allowed_camera_source("7") is True
    assert is_allowed_camera_source("/dev/video0") is True
    assert is_allowed_camera_source("/dev/v4l/by-id/usb-cam") is True
    assert is_allowed_camera_source("/dev/ttyACM0") is False
    assert is_allowed_camera_source("COM3") is False
    assert is_allowed_camera_source("/etc/passwd") is False
    assert is_allowed_camera_source(99) is False


def test_validate_hardware_rejects_duplicate_cameras() -> None:
    payload = _payload()
    payload["camera"]["slots"]["left_1"]["device"] = "/dev/video0"
    with pytest.raises(ConfigError, match="trùng"):
        validate_hardware(payload)


def test_validate_hardware_requires_front_when_camera_enabled() -> None:
    payload = _payload()
    payload["camera"]["slots"]["front"]["device"] = ""
    with pytest.raises(ConfigError, match="trước"):
        validate_hardware(payload)


def test_validate_hardware_allows_change_me_gateway() -> None:
    normalized = validate_hardware(_payload())
    assert normalized["radar_gateway"]["device"].endswith("CHANGE_ME")
    assert normalized["camera"]["slots"]["front"]["device"] == "/dev/video0"


def test_validate_hardware_accepts_com_gateway() -> None:
    payload = _payload(radar_gateway={"enabled": True, "device": "COM3", "baudrate": 115200})
    normalized = validate_hardware(payload)
    assert normalized["radar_gateway"]["device"] == "COM3"


def test_validate_hardware_rejects_passwd_camera() -> None:
    payload = _payload()
    payload["camera"]["slots"]["front"]["device"] = "/etc/passwd"
    with pytest.raises(ConfigError):
        validate_hardware(payload)


def test_validate_hardware_rejects_bad_lidar_path() -> None:
    payload = _payload(lidar={
        "enabled": True, "driver": "real", "source": "/etc/passwd",
        "fps": 10, "driver_module": "drivers.real_lidar",
    })
    with pytest.raises(ConfigError, match="allowlist"):
        validate_hardware(payload)


def test_hardware_from_config_hydrates_front_from_source() -> None:
    cfg = {
        "camera": {"enabled": True, "width": 640, "height": 480, "fps": 15, "source": 0, "sources": {}},
        "lidar": {"enabled": True, "driver": "mock", "source": "mock", "fps": 10},
        "radar_gateway": {"enabled": False, "device": "/dev/serial/by-id/CHANGE_ME", "baudrate": 115200},
    }
    hw = hardware_from_config(cfg)
    assert hw["camera"]["slots"]["front"]["device"] == "0"
    assert hw["camera"]["width"] == 640
    assert set(hw["camera"]["slots"]) == {"front", "left_1", "left_2", "right_1", "right_2", "rear"}


def test_save_hardware_is_atomic_and_preserves_unrelated_keys(tmp_path: Path) -> None:
    cfg = tmp_path / "live.yaml"
    cfg.write_text(
        yaml.safe_dump({
            "mode": "live",
            "camera": {"source": 0, "sources": {"front": "/dev/video0"}, "width": 1280, "height": 720, "fps": 30},
            "lidar": {"driver": "mock", "source": "mock", "fps": 10},
            "radar_gateway": {
                "enabled": True,
                "device": "/dev/serial/by-id/CHANGE_ME",
                "baudrate": 115200,
                "protocol_version": 2,
                "radars": {
                    "left_front": {"enabled": True, "camera": "left_1", "warning_distance_m": 5.0},
                    "left_rear": {"enabled": True, "camera": "left_2", "warning_distance_m": 5.0},
                    "right_front": {"enabled": True, "camera": "right_1", "warning_distance_m": 5.0},
                    "right_rear": {"enabled": True, "camera": "right_2", "warning_distance_m": 5.0},
                },
            },
            "warning": {"mode": "distance", "danger_distance_m": 6.0},
            "keep_me": "yes",
        }, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    payload = _payload(radar_gateway={"enabled": True, "device": "COM4", "baudrate": 115200})
    saved = save_hardware(cfg, payload)
    text = cfg.read_text(encoding="utf-8")
    data = yaml.safe_load(text)
    assert data["keep_me"] == "yes"
    assert data["warning"]["danger_distance_m"] == 6.0
    assert data["camera"]["sources"]["front"] == "/dev/video0"
    assert data["camera"]["sources"]["left_1"] == "/dev/video1"
    assert "left_2" not in data["camera"]["sources"]
    assert data["radar_gateway"]["device"] == "COM4"
    assert data["radar_gateway"]["radars"]["left_front"]["camera"] == "left_1"
    assert data["lidar"]["driver_module"] == "drivers.real_lidar"
    assert saved["radar_gateway"]["device"] == "COM4"
    assert not (cfg.with_suffix(cfg.suffix + ".tmp")).exists()


def test_save_hardware_syncs_side_module_camera_device(tmp_path: Path) -> None:
    cfg = tmp_path / "live.yaml"
    cfg.write_text(
        yaml.safe_dump({
            "camera": {"sources": {}, "width": 1280, "height": 720, "fps": 30},
            "lidar": {"driver": "mock", "source": "mock", "fps": 10},
            "side_sensor_modules": {
                "enabled": False,
                "modules": {
                    "left_1": {"camera": {"device": "/dev/video9", "width": 1280, "height": 720, "fps": 30}},
                },
            },
        }, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    save_hardware(cfg, _payload())
    data = yaml.safe_load(cfg.read_text(encoding="utf-8"))
    assert data["side_sensor_modules"]["modules"]["left_1"]["camera"]["device"] == "/dev/video1"


def test_list_hardware_devices_shape() -> None:
    devices = list_hardware_devices()
    assert "cameras" in devices
    assert "camera_indices" in devices
    assert set(devices["serial"]) == {"by_id", "ttyacm", "ttyusb", "com"}
    assert "platform" in devices
    assert devices["camera_indices"][0] == "0"


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


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


@pytest.fixture
def hardware_dashboard(tmp_path):
    status = tmp_path / "status.json"
    status.write_text(json.dumps({"ready": False, "errors": []}), encoding="utf-8")
    cfg = tmp_path / "jetson_live.yaml"
    cfg.write_text(
        yaml.safe_dump({
            "mode": "live",
            "camera": {
                "enabled": True, "source": 0, "width": 1280, "height": 720, "fps": 30,
                "sources": {"front": "/dev/video0"},
            },
            "lidar": {"enabled": True, "driver": "mock", "source": "mock", "fps": 10},
            "radar_gateway": {
                "enabled": True,
                "device": "/dev/serial/by-id/CHANGE_ME",
                "baudrate": 115200,
            },
            "warning": {"keep": True},
        }, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    port = _free_port()
    server = create_dashboard_server(
        host="127.0.0.1", port=port, status_path=status, config_path=cfg, demo=False,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.1)
    yield {"port": port, "cfg": cfg, "server": server}
    server.shutdown()
    server.server_close()


@pytest.fixture
def hardware_demo_dashboard(tmp_path):
    status = tmp_path / "status.json"
    cfg = tmp_path / "demo.yaml"
    cfg.write_text("mode: demo\ncamera: {source: mock}\n", encoding="utf-8")
    port = _free_port()
    server = create_dashboard_server(
        host="127.0.0.1", port=port, status_path=status, config_path=cfg, demo=True,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.1)
    yield {"port": port, "server": server}
    server.shutdown()
    server.server_close()


def test_dashboard_get_hardware_config(hardware_dashboard) -> None:
    port = hardware_dashboard["port"]
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/hardware/config") as resp:
        data = json.loads(resp.read().decode("utf-8"))
    assert data["ok"] is True
    assert data["config"]["camera"]["slots"]["front"]["device"] == "/dev/video0"
    assert "left_1" in data["config"]["camera"]["slots"]


def test_dashboard_get_hardware_devices(hardware_dashboard) -> None:
    port = hardware_dashboard["port"]
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/hardware/devices") as resp:
        data = json.loads(resp.read().decode("utf-8"))
    assert data["ok"] is True
    assert "cameras" in data["devices"]
    assert "com" in data["devices"]["serial"]


def test_dashboard_put_hardware_config_saves_yaml(hardware_dashboard) -> None:
    port = hardware_dashboard["port"]
    cfg_path: Path = hardware_dashboard["cfg"]
    req = _put(f"http://127.0.0.1:{port}/api/hardware/config", {"config": _payload(
        radar_gateway={"enabled": True, "device": "COM5", "baudrate": 115200},
    )})
    with urllib.request.urlopen(req) as resp:
        body = json.loads(resp.read().decode("utf-8"))
    assert body["ok"] is True
    assert body["restart_required"] is True
    data = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
    assert data["radar_gateway"]["device"] == "COM5"
    assert data["camera"]["sources"]["rear"] == "/dev/video4"
    assert data["warning"]["keep"] is True


def test_dashboard_put_hardware_rejects_path_traversal(hardware_dashboard) -> None:
    port = hardware_dashboard["port"]
    req = _put(f"http://127.0.0.1:{port}/api/hardware/config", {
        "config": _payload(camera={
            "enabled": True, "width": 1280, "height": 720, "fps": 30,
            "slots": {
                "front": {"enabled": True, "device": "/etc/passwd"},
                "left_1": {"enabled": False, "device": ""},
                "left_2": {"enabled": False, "device": ""},
                "right_1": {"enabled": False, "device": ""},
                "right_2": {"enabled": False, "device": ""},
                "rear": {"enabled": False, "device": ""},
            },
        }),
    })
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(req)
    assert exc.value.code == 400


def test_dashboard_demo_forbids_hardware_put_and_test(hardware_demo_dashboard) -> None:
    port = hardware_demo_dashboard["port"]
    put_req = _put(f"http://127.0.0.1:{port}/api/hardware/config", {"config": _payload()})
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(put_req)
    assert exc.value.code == 403
    post_req = _post(f"http://127.0.0.1:{port}/api/hardware/test", {"kind": "serial", "device": "COM3"})
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(post_req)
    assert exc.value.code == 403


def test_dashboard_hardware_test_rejects_bad_kind(hardware_dashboard) -> None:
    port = hardware_dashboard["port"]
    req = _post(f"http://127.0.0.1:{port}/api/hardware/test", {"kind": "nuke", "device": "COM3"})
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(req)
    assert exc.value.code == 400


def test_index_html_contains_hardware_tab() -> None:
    html = (Path(__file__).resolve().parents[1] / "dashboard" / "static" / "index.html").read_text(encoding="utf-8")
    assert 'id="viewHardware"' in html
    assert 'data-tab="hardware"' in html
    assert "CẤU HÌNH PHẦN CỨNG" in html
    assert 'id="hwVehicle"' in html
    assert 'id="hwRadarDevice"' in html
    assert 'id="hwLidarSource"' in html
