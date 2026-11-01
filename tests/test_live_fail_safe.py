"""Fail-safe tests: live must never invent mock camera / lidar / detector."""
from __future__ import annotations

import pytest
from pathlib import Path

from app import ADASPipeline
from config import load_config, coerce_config
from sensors.lidar_reader import LidarReaderFactory, MockLidarReader
from vision.detector import build_detector


ROOT = Path(__file__).resolve().parents[1]


def _demo_config() -> dict:
    cfg = load_config(ROOT / "config" / "adas.yaml")
    coerce_config(cfg)
    return cfg


def test_lidar_factory_unknown_falls_back_when_allowed():
    reader = LidarReaderFactory.create(
        "no_such_driver",
        allow_mock=True,
        fps=10,
        config={"path": "/tmp/x"},
    )
    assert isinstance(reader, MockLidarReader)


def test_lidar_factory_refuses_unknown_when_strict():
    with pytest.raises(RuntimeError, match="chưa đăng ký"):
        LidarReaderFactory.create("no_such_driver", allow_mock=False, fps=10)


def test_lidar_factory_refuses_mock_name_when_strict():
    with pytest.raises(RuntimeError, match="MockLidarReader"):
        LidarReaderFactory.create("mock", allow_mock=False, fps=10)


def test_build_detector_refuses_mock_when_disallowed():
    with pytest.raises(RuntimeError, match="MockDetector|Ultralytics|Không tải"):
        build_detector(
            {
                "model": str(ROOT / "models" / "does_not_exist.engine"),
                "validate_model_on_startup": True,
            },
            allow_mock=False,
        )


def test_pipeline_unknown_mode_strict_does_not_become_demo():
    cfg = _demo_config()
    cfg["mode"] = "not-a-mode"
    cfg["safety"]["allow_mock"] = False
    with pytest.raises(RuntimeError, match="không hỗ trợ"):
        ADASPipeline(cfg, root=ROOT)


def test_pipeline_live_rejects_mock_camera_source():
    cfg = _demo_config()
    cfg["mode"] = "live"
    cfg["safety"]["allow_mock"] = False
    cfg["safety"]["is_demo_calibration"] = False
    cfg["camera"]["source"] = "mock"
    cfg["camera"].pop("sources", None)
    cfg["lidar"]["enabled"] = False
    with pytest.raises(RuntimeError, match="MockCameraReader"):
        ADASPipeline(cfg, root=ROOT)


def test_pipeline_live_rejects_mock_lidar():
    cfg = _demo_config()
    cfg["mode"] = "live"
    cfg["safety"]["allow_mock"] = False
    cfg["safety"]["is_demo_calibration"] = False
    cfg["camera"]["enabled"] = False
    cfg["lidar"]["driver"] = "mock"
    cfg["lidar"]["source"] = "mock"
    with pytest.raises(RuntimeError, match="MockLidarReader"):
        ADASPipeline(cfg, root=ROOT)
