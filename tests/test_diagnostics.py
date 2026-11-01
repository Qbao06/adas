import json

from system.diagnostics import write_runtime_manifest


def test_runtime_manifest_contains_nonsecret_metadata(tmp_path):
    cfg = {
        "mode": "demo",
        "vision": {"model": "model.pt"},
        "calibration": {"file": "calibration.yaml"},
        "lidar": {"driver": "mock", "password": "do-not-copy"},
    }
    path = write_runtime_manifest(tmp_path, cfg)
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["mode"] == "demo"
    assert data["lidar_driver"] == "mock"
    assert "do-not-copy" not in path.read_text(encoding="utf-8")
    assert len(data["config_sha256"]) == 64
