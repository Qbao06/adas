from __future__ import annotations

import copy

import pytest
import yaml

from config import ConfigError
from config.side_modules import (
    apply_side_modules, default_side_modules, save_side_modules, validate_side_modules,
)


def _valid_modules():
    value = default_side_modules()
    value["modules"]["left_1"]["camera"]["device"] = "/dev/video1"
    value["modules"]["left_1"]["radar"]["device"] = "/dev/ttyACM0"
    value["modules"]["right_1"]["camera"]["device"] = "/dev/video2"
    value["modules"]["right_1"]["radar"]["device"] = "/dev/ttyACM1"
    return value


def test_validate_four_module_schema_and_derive_enabled_camera_sources():
    modules = _valid_modules()
    modules["modules"]["left_2"]["enabled"] = True
    modules["modules"]["left_2"]["camera"]["device"] = "/dev/video3"
    modules["modules"]["left_2"]["radar"]["device"] = "/dev/ttyACM2"
    normalized = validate_side_modules(modules)
    config = {"camera": {"sources": {"front": "/dev/video0"}}}
    apply_side_modules(config, normalized)
    assert config["camera"]["sources"]["left_2"]["device"] == "/dev/video3"
    assert config["camera"]["sources"]["left_2"]["fps"] == 30
    assert "right_2" not in config["camera"]["sources"]


def test_reject_duplicate_uart_and_camera():
    modules = _valid_modules()
    modules["modules"]["right_1"]["radar"]["device"] = "/dev/ttyACM0"
    with pytest.raises(ConfigError, match="UART"):
        validate_side_modules(modules)
    modules = _valid_modules()
    modules["modules"]["right_1"]["camera"]["device"] = "/dev/video1"
    with pytest.raises(ConfigError, match="Camera"):
        validate_side_modules(modules)


def test_atomic_save_preserves_unrelated_config(tmp_path):
    path = tmp_path / "live.yaml"
    path.write_text(yaml.safe_dump({"mode": "live", "lidar": {"driver": "real"}}), encoding="utf-8")
    saved = save_side_modules(path, _valid_modules())
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert payload["mode"] == "live"
    assert payload["lidar"]["driver"] == "real"
    assert saved["modules"]["left_1"]["enabled"] is True
