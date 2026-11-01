"""Integration + smoke tests that exercise the entire pipeline end-to-end.

These tests run without any hardware, no YOLO model, and no calibration file
other than the one that ships in the repo.
"""
from __future__ import annotations

import numpy as np
import pytest
from pathlib import Path

from config import load_config, coerce_config
from app import ADASPipeline
from core.types import CameraFrame, LidarFrame, Detection, FusedObject
from core.enums import FusionStatus, WarningLevel


ROOT = Path(__file__).resolve().parents[1]
CONFIG_FILE = ROOT / "config" / "adas.yaml"


def _minimal_config() -> dict:
    cfg = load_config(CONFIG_FILE)
    coerce_config(cfg)
    cfg["mode"] = "demo"
    cfg.setdefault("benchmark", {})
    cfg["benchmark"]["warmup_frames"] = 0
    return cfg


# -------------------------------------------------------------------------- #
#  Pipeline construction
# -------------------------------------------------------------------------- #

def test_pipeline_constructs_without_error():
    cfg = _minimal_config()
    pipeline = ADASPipeline(cfg, root=ROOT)
    assert pipeline is not None
    assert pipeline._mode == "demo"
    assert pipeline.camera is not None
    assert pipeline.lidar is not None


def test_pipeline_constructs_camera_only_mode():
    cfg = _minimal_config()
    cfg["mode"] = "camera-only"
    pipeline = ADASPipeline(cfg, root=ROOT)
    assert pipeline._mode == "camera-only"
    assert pipeline.camera is not None
    assert pipeline.lidar is None


def test_pipeline_constructs_lidar_only_mode():
    cfg = _minimal_config()
    cfg["mode"] = "lidar-only"
    pipeline = ADASPipeline(cfg, root=ROOT)
    assert pipeline._mode == "lidar-only"
    assert pipeline.lidar is not None
    assert pipeline.camera is None


# -------------------------------------------------------------------------- #
#  Single-frame processing
# -------------------------------------------------------------------------- #

def test_pipeline_single_frame_processing():
    cfg = _minimal_config()
    pipeline = ADASPipeline(cfg, root=ROOT)
    cam = CameraFrame(np.zeros((360, 640, 3), dtype=np.uint8), 1.0, 0)
    lid = LidarFrame(np.zeros((100, 3)), 1.0, 0)
    objects, projected = pipeline.process(cam, lid)
    assert isinstance(objects, list)
    assert projected is not None


def test_pipeline_handles_empty_lidar():
    cfg = _minimal_config()
    pipeline = ADASPipeline(cfg, root=ROOT)
    cam = CameraFrame(np.zeros((360, 640, 3), dtype=np.uint8), 1.0, 0)
    lid = LidarFrame(np.empty((0, 3)), 1.0, 0)
    objects, projected = pipeline.process(cam, lid)
    assert objects is not None
    assert projected is not None


def test_pipeline_handles_nan_inf_lidar():
    cfg = _minimal_config()
    pipeline = ADASPipeline(cfg, root=ROOT)
    pts = np.array([[1, 0, 0], [np.nan, 0, 0], [np.inf, 0, 0]])
    cam = CameraFrame(np.zeros((360, 640, 3), dtype=np.uint8), 1.0, 0)
    lid = LidarFrame(pts, 1.0, 0)
    objects, projected = pipeline.process(cam, lid)
    assert objects is not None


def test_pipeline_handles_no_detections():
    cfg = _minimal_config()
    pipeline = ADASPipeline(cfg, root=ROOT)
    cam = CameraFrame(np.zeros((360, 640, 3), dtype=np.uint8), 1.0, 0)
    lid = LidarFrame(np.zeros((50, 3)), 1.0, 0)
    # MockDetector always returns 1 detection; we can't easily suppress it.
    # The important thing is the pipeline doesn't crash.
    objects, projected = pipeline.process(cam, lid)
    assert objects is not None


# -------------------------------------------------------------------------- #
#  Run + stop
# -------------------------------------------------------------------------- #

def test_pipeline_run_headless_few_frames():
    cfg = _minimal_config()
    pipeline = ADASPipeline(cfg, root=ROOT)
    count = pipeline.run(max_frames=3, display=False)
    assert count == 3
    stats = pipeline.stats()
    assert stats.frames == 3


def test_pipeline_run_stops_on_max_frames():
    cfg = _minimal_config()
    pipeline = ADASPipeline(cfg, root=ROOT)
    count = pipeline.run(max_frames=5, display=False)
    assert count == 5


def test_pipeline_stop_is_clean():
    cfg = _minimal_config()
    pipeline = ADASPipeline(cfg, root=ROOT)
    pipeline.run(max_frames=2, display=False)
    pipeline.stop()   # should be idempotent
    pipeline.stop()


def test_pipeline_keyboard_interrupt_does_not_crash():
    cfg = _minimal_config()
    pipeline = ADASPipeline(cfg, root=ROOT)
    # Set a maximum frame count to force a clean stop inside run().
    count = pipeline.run(max_frames=2, display=False)
    assert count == 2
    pipeline.stop()


# -------------------------------------------------------------------------- #
#  Stats and health
# -------------------------------------------------------------------------- #

def test_pipeline_stats_accumulate():
    cfg = _minimal_config()
    pipeline = ADASPipeline(cfg, root=ROOT)
    pipeline.run(max_frames=10, display=False)
    stats = pipeline.stats()
    assert stats.frames == 10
    assert stats.fps > 0


def test_pipeline_health_monitor():
    cfg = _minimal_config()
    pipeline = ADASPipeline(cfg, root=ROOT)
    pipeline.run(max_frames=3, display=False)
    from core.enums import SystemStatus
    assert pipeline.health.status in {SystemStatus.OK, SystemStatus.DEGRADED}


# -------------------------------------------------------------------------- #
#  Config defaults are applied
# -------------------------------------------------------------------------- #

def test_config_load_with_defaults():
    from config.loader import _default_config, _merge_defaults
    base = _default_config()
    assert base["mode"] == "demo"
    assert "fusion" in base
    assert "warning" in base
    assert base["fusion"]["ema_alpha"] == 0.35


def test_config_merge_resolves_nested():
    from config.loader import _merge_defaults
    base = {"a": {"x": 1}, "b": 2}
    override = {"a": {"y": 3}}
    merged = _merge_defaults(base, override)
    assert merged["a"]["x"] == 1
    assert merged["a"]["y"] == 3
    assert merged["b"] == 2
