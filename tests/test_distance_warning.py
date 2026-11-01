import numpy as np
import pytest

from adas.distance_warning import DistanceWarning
from core.enums import FusionStatus, WarningLevel
from core.types import FusedObject


def _obj(distance, lateral=0.0, tid=1):
    return FusedObject(
        track_id=tid, class_name="car", confidence=0.9,
        bbox=(0, 0, 10, 10), timestamp=0.0,
        position_3d=np.array([distance or 0.0, lateral, 0.0]) if distance is not None else None,
        raw_distance=distance, filtered_distance=distance,
        fusion_status=FusionStatus.FUSED if distance is not None else FusionStatus.NO_POINTS,
    )


def _warning():
    return DistanceWarning({
        "emergency_distance_m": 3.0,
        "danger_distance_m": 6.0,
        "caution_distance_m": 12.0,
        "hysteresis_distance_m": 0.5,
        "lane_half_width_m": 1.75,
    })


@pytest.mark.parametrize("distance,expected", [
    (2.0, WarningLevel.EMERGENCY),
    (5.0, WarningLevel.DANGER),
    (10.0, WarningLevel.CAUTION),
    (20.0, WarningLevel.SAFE),
])
def test_distance_warning_thresholds(distance, expected):
    obj = _obj(distance)
    _warning().evaluate([obj])
    assert obj.warning_level == expected


def test_distance_warning_off_lane_safe():
    obj = _obj(2.0, lateral=4.0)
    _warning().evaluate([obj])
    assert obj.warning_level == WarningLevel.SAFE


def test_distance_warning_missing_distance_unknown():
    obj = _obj(None)
    _warning().evaluate([obj])
    assert obj.warning_level == WarningLevel.UNKNOWN


def test_distance_warning_deescalation_hysteresis():
    warning = _warning()
    first = _obj(2.9)
    warning.evaluate([first])
    assert first.warning_level == WarningLevel.EMERGENCY
    near_boundary = _obj(3.2)
    warning.evaluate([near_boundary])
    assert near_boundary.warning_level == WarningLevel.EMERGENCY
    clear = _obj(3.6)
    warning.evaluate([clear])
    assert clear.warning_level == WarningLevel.DANGER


def test_distance_warning_rejects_bad_threshold_order():
    with pytest.raises(ValueError):
        DistanceWarning({
            "emergency_distance_m": 8,
            "danger_distance_m": 6,
            "caution_distance_m": 12,
        })
