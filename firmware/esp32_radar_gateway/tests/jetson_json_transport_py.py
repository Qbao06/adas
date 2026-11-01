"""
Python mirror of firmware/esp32_radar_gateway/src/jetson_json_transport.{h,cpp}.

The encoder MUST emit radars in the fixed protocol order
(left_front, left_rear, right_front, right_rear) so the Jetson can
index by slot.  Building a dict keyed by radar_id would collapse
duplicate ids; the C++ side walks a static order[] table and that
contract is mirrored here.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import List, Optional

LD2410_DISTANCE_NONE = 0xFFFF
LD2410_ENERGY_NONE = 0xFF

TARGET_STATE_NONE = 0x00
TARGET_STATE_MOVING = 0x01
TARGET_STATE_STATIONARY = 0x02
TARGET_STATE_BOTH = 0x03

ADAS_MAX_RADARS = 4

# Fixed protocol order.  Mirrors the `order[]` table in
# jetson_json_transport.cpp.
RADAR_ORDER = ("left_front", "left_rear", "right_front", "right_rear")


@dataclass
class RadarSample:
    radar_id: str = ""
    target_state: int = TARGET_STATE_NONE
    detected: int = 0
    valid: int = 0
    stale: int = 1
    moving_distance_cm: int = LD2410_DISTANCE_NONE
    stationary_distance_cm: int = LD2410_DISTANCE_NONE
    moving_energy: int = LD2410_ENERGY_NONE
    stationary_energy: int = LD2410_ENERGY_NONE
    detection_distance_cm: int = LD2410_DISTANCE_NONE
    error_code: int = 0
    sample_age_ms: int = 0xFFFF
    last_update_ms: int = 0


@dataclass
class RadarSnapshot:
    sequence: int = 0
    gateway_ok: int = 0
    remote_ok: int = 0
    gateway_boot_id: int = 0
    remote_boot_id: int = 0
    packet_loss: int = 0
    radars: List[RadarSample] = field(default_factory=list)


_TARGET_STATE_STRINGS = {
    TARGET_STATE_MOVING: "moving",
    TARGET_STATE_STATIONARY: "stationary",
    TARGET_STATE_BOTH: "both",
}


def _distance_m_or_null(cm: int, valid: bool) -> Optional[float]:
    if not valid or cm == LD2410_DISTANCE_NONE:
        return None
    whole = cm // 100
    frac = cm % 100
    return round(whole + frac / 100.0, 2)


def _uint_or_null(valid: bool, value: int) -> Optional[int]:
    if not valid or value == LD2410_ENERGY_NONE:
        return None
    return int(value)


def _radar_dict(s: RadarSample) -> dict:
    valid = bool(s.valid)
    # An invalid radar is by definition stale too: the Jetson must drop
    # its distance fields regardless of how stale the sample is.
    stale = bool(s.stale) or not valid
    return {
        "detected": bool(s.detected),
        "target_state": _TARGET_STATE_STRINGS.get(s.target_state, "none"),
        "distance_m": _distance_m_or_null(s.detection_distance_cm, valid),
        "moving_distance_m": _distance_m_or_null(s.moving_distance_cm, valid),
        "stationary_distance_m": _distance_m_or_null(s.stationary_distance_cm, valid),
        "moving_energy": _uint_or_null(valid, s.moving_energy),
        "stationary_energy": _uint_or_null(valid, s.stationary_energy),
        "valid": bool(s.valid),
        "stale": stale,
        "age_ms": int(s.sample_age_ms),
        "error": int(s.error_code),
    }


def jetson_json_encode_v2(snap: RadarSnapshot) -> str:
    """Produce one JSON v2 line.

    Mirrors the C++ encoder: walks the four canonical slots in protocol
    order and reads `snap.radars[i]`.  If `snap.radars` has fewer than
    4 entries the missing slot is filled with an empty
    valid=False / stale=True sample, exactly like the C++ encoder
    does after `radar_snapshot_reset()`.  This slot-based emission
    matches the C++ `order[]` table so the wire format is identical.
    """
    radars_ordered: List[dict] = []
    for i, rid in enumerate(RADAR_ORDER):
        if i < len(snap.radars):
            s = snap.radars[i]
            # Stamp radar_id from the slot so the wire payload always
            # carries the canonical name regardless of what the caller
            # put on the sample object.
            s.radar_id = rid
        else:
            s = RadarSample(radar_id=rid)
        radars_ordered.append(_radar_dict(s))

    payload = {
        "v": 2,
        "seq": snap.sequence,
        "gateway_ok": snap.gateway_ok,
        "remote_ok": snap.remote_ok,
        "gateway_boot_id": snap.gateway_boot_id,
        "remote_boot_id": snap.remote_boot_id,
        "packet_loss": snap.packet_loss,
        "radars": {name: radars_ordered[i] for i, name in enumerate(RADAR_ORDER)},
    }
    line = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
    return line + "\n"