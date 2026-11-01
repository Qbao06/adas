import copy

import pytest

from config import ConfigError, load_config, validate_config
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _config():
    return load_config(ROOT / "config" / "adas.yaml")


def test_default_config_validates():
    assert validate_config(_config())["mode"] == "demo"


def test_invalid_distance_order_rejected():
    cfg = _config()
    cfg["warning"]["emergency_distance_m"] = 20
    with pytest.raises(ConfigError):
        validate_config(cfg)


def test_live_mock_policy_rejected():
    cfg = _config()
    cfg["mode"] = "live"
    cfg["safety"]["allow_mock"] = True
    with pytest.raises(ConfigError):
        validate_config(cfg)


def test_bad_confidence_rejected():
    cfg = _config()
    cfg["vision"]["confidence"] = 1.5
    with pytest.raises(ConfigError):
        validate_config(cfg)
