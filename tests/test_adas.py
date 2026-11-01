import math

import numpy as np
import pytest

from adas.motion import MotionEstimator
from adas.warning import CollisionWarning
from core.enums import FusionStatus, WarningLevel
from core.types import FusedObject


def _obj(tid: int, distance: float | None, ttc: float | None, position=None, ts: float = 0.0):
    obj = FusedObject(
        track_id=tid,
        class_name="car",
        confidence=0.9,
        bbox=(0, 0, 10, 10),
        timestamp=ts,
        position_3d=position,
        raw_distance=distance,
        filtered_distance=distance,
        fusion_status=FusionStatus.FUSED if distance is not None else FusionStatus.NO_POINTS,
    )
    obj.ttc = ttc
    return obj


def test_motion_estimator_positive_when_closing():
    m = MotionEstimator(alpha=1.0, min_closing_mps=0.0)
    obj1 = _obj(1, distance=20.0, ttc=None)
    m.update([obj1], 0.0)
    obj2 = _obj(1, distance=18.0, ttc=None)
    m.update([obj2], 1.0)
    assert obj2.relative_speed == pytest.approx(2.0, abs=1e-6)
    assert obj2.ttc == pytest.approx(9.0, abs=1e-6)


def test_motion_estimator_negative_when_moving_away():
    m = MotionEstimator(alpha=1.0, min_closing_mps=0.0)
    m.update([_obj(1, 10.0, None)], 0.0)
    obj2 = _obj(1, 12.0, None)
    m.update([obj2], 1.0)
    assert obj2.relative_speed == pytest.approx(-2.0, abs=1e-6)
    assert math.isinf(obj2.ttc)


def test_motion_estimator_ttc_capped():
    # Distance shrinks at 1 m/s from 100 → 90. Raw TTC = 90. Cap = 5.
    m = MotionEstimator(alpha=1.0, min_closing_mps=0.0, ttc_cap_s=5.0)
    m.update([_obj(1, 100.0, None)], 0.0)
    obj = _obj(1, 90.0, None)
    m.update([obj], 1.0)
    assert obj.ttc == pytest.approx(5.0, abs=1e-6)


def test_motion_estimator_handles_zero_closing_speed():
    m = MotionEstimator(min_closing_mps=0.1)
    m.update([_obj(1, 10.0, None)], 0.0)
    obj = _obj(1, 10.0, None)
    m.update([obj], 1.0)
    assert math.isinf(obj.ttc)


def test_motion_estimator_ignores_none_distance():
    m = MotionEstimator()
    m.update([_obj(1, None, None)], 0.0)
    assert m.history == {}


def test_warning_classification_thresholds():
    w = CollisionWarning(
        {
            "lane_half_width_m": 1.5,
            "caution_ttc": 4.0,
            "danger_ttc": 2.5,
            "emergency_ttc": 1.5,
            "hysteresis_seconds": 0.0,
            "hysteresis_levels": 1,
        }
    )
    pos = np.array([10.0, 0.5, 0.0])
    safe = _obj(1, 10.0, 5.0, position=pos)
    caution = _obj(2, 10.0, 3.0, position=pos)
    danger = _obj(3, 10.0, 2.0, position=pos)
    emerg = _obj(4, 10.0, 1.0, position=pos)
    w.evaluate([safe, caution, danger, emerg])
    assert safe.warning_level == WarningLevel.SAFE
    assert caution.warning_level == WarningLevel.CAUTION
    assert danger.warning_level == WarningLevel.DANGER
    assert emerg.warning_level == WarningLevel.EMERGENCY


def test_warning_off_lane_is_safe():
    w = CollisionWarning(
        {
            "lane_half_width_m": 1.5,
            "caution_ttc": 4.0,
            "danger_ttc": 2.5,
            "emergency_ttc": 1.5,
            "hysteresis_seconds": 0.0,
            "hysteresis_levels": 1,
        }
    )
    pos = np.array([10.0, 5.0, 0.0])
    obj = _obj(1, 10.0, 1.0, position=pos)
    w.evaluate([obj])
    assert obj.warning_level == WarningLevel.SAFE


def test_warning_no_distance_unknown():
    w = CollisionWarning(
        {
            "lane_half_width_m": 1.5,
            "caution_ttc": 4.0,
            "danger_ttc": 2.5,
            "emergency_ttc": 1.5,
            "hysteresis_seconds": 0.0,
            "hysteresis_levels": 1,
        }
    )
    obj = _obj(1, None, None, position=None)
    w.evaluate([obj])
    assert obj.warning_level == WarningLevel.UNKNOWN


def test_warning_hysteresis_delays_escalation():
    w = CollisionWarning(
        {
            "lane_half_width_m": 1.5,
            "caution_ttc": 4.0,
            "danger_ttc": 2.5,
            "emergency_ttc": 1.5,
            "hysteresis_seconds": 10.0,   # very slow
            "hysteresis_levels": 1,
        }
    )
    pos = np.array([10.0, 0.5, 0.0])
    obj = _obj(1, 10.0, 3.0, position=pos)
    w.evaluate([obj])
    assert obj.warning_level == WarningLevel.SAFE
    obj.ttc = 1.0
    w.evaluate([obj])
    # Still SAFE because hysteresis_seconds is 10s.
    assert obj.warning_level == WarningLevel.SAFE
