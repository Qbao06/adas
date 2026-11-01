"""Python mirror of ``firmware/esp32_radar_gateway/src/jetson_json_transport.{h,cpp}``.

The Gateway sends one JSON v2 object per line over USB Serial:

.. code-block:: json

    {"v":2,"seq":N,"gateway_ok":1,"remote_ok":0|1,
     "gateway_boot_id":N,"remote_boot_id":N,"packet_loss":N,
     "radars":{
        "left_front":{...}, "left_rear":{...},
        "right_front":{...}, "right_rear":{...}
     }}\\n

The contract (Prompt 1):

* Invalid data MUST be serialised as JSON ``null``, never as 0.
* ``radars`` always contains exactly four entries in the order
  ``left_front``, ``left_rear``, ``right_front``, ``right_rear``.
* ``target_state`` is encoded as a string (``"none"``, ``"moving"``,
  ``"stationary"``, ``"both"``) for human readability; the firmware C++
  also encodes it as a number for backwards compatibility, the Python
  encoder always emits the string form.
* Distances are emitted in **metres** with two decimal places, derived
  from centimetres.

The Python encoder mirrors the C++ encoder byte for byte so a packet
captured on-device replays cleanly here and vice versa.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import List

from .protocol import (
    LD2410_DISTANCE_NONE,
    LD2410_ENERGY_NONE,
    TARGET_STATE_MOVING,
    TARGET_STATE_NONE,
    TARGET_STATE_STATIONARY,
    TARGET_STATE_BOTH,
)

JSON_PROTOCOL_VERSION: int = 2
ADAS_MAX_RADARS: int = 4

RADAR_ORDER: tuple = (
    "left_front",
    "left_rear",
    "right_front",
    "right_rear",
)

_TARGET_STATE_STRINGS = {
    TARGET_STATE_MOVING: "moving",
    TARGET_STATE_STATIONARY: "stationary",
    TARGET_STATE_BOTH: "both",
}


@dataclass
class RadarSample:
    """Host-side radar representation used by the JSON encoder."""

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
    """Aggregated four-radar snapshot serialised to JSON v2."""

    sequence: int = 0
    gateway_ok: int = 0
    remote_ok: int = 0
    gateway_boot_id: int = 0
    remote_boot_id: int = 0
    packet_loss: int = 0
    radars: List[RadarSample] = field(default_factory=list)


# --- Helpers -------------------------------------------------------------- #
def _distance_m_or_null(cm: int, valid: bool):
    """Render centimetres as metres or None.

    The encoder never emits 0.0 for an invalid sample: ``valid=False``
    or the LD2410 "no target" sentinel always becomes JSON ``null``.
    """
    if not valid or cm == LD2410_DISTANCE_NONE:
        return None
    whole = cm // 100
    frac = cm % 100
    return round(whole + frac / 100.0, 2)


def _uint_or_none(valid: bool, value: int):
    if not valid or value == LD2410_ENERGY_NONE:
        return None
    return int(value)


def _radar_dict(s: RadarSample) -> dict:
    valid = bool(s.valid)
    return {
        "detected": bool(s.detected),
        "target_state": _TARGET_STATE_STRINGS.get(s.target_state, "none"),
        "distance_m": _distance_m_or_null(s.detection_distance_cm, valid),
        "moving_distance_m": _distance_m_or_null(s.moving_distance_cm, valid),
        "stationary_distance_m": _distance_m_or_null(s.stationary_distance_cm, valid),
        "moving_energy": _uint_or_none(valid, s.moving_energy),
        "stationary_energy": _uint_or_none(valid, s.stationary_energy),
        "valid": valid,
        "stale": bool(s.stale),
        "age_ms": int(s.sample_age_ms),
        "error": int(s.error_code),
    }


# --- Encoder -------------------------------------------------------------- #
def jetson_json_encode_v2(snap: RadarSnapshot) -> str:
    """Produce one newline-terminated JSON v2 line."""
    radars_dict = {r.radar_id: _radar_dict(r) for r in snap.radars}
    payload = {
        "v": JSON_PROTOCOL_VERSION,
        "seq": int(snap.sequence),
        "gateway_ok": int(snap.gateway_ok),
        "remote_ok": int(snap.remote_ok),
        "gateway_boot_id": int(snap.gateway_boot_id),
        "remote_boot_id": int(snap.remote_boot_id),
        "packet_loss": int(snap.packet_loss),
        "radars": radars_dict,
    }
    return json.dumps(payload, separators=(",", ":"), ensure_ascii=False) + "\n"


def jetson_json_encode_v2_bytes(snap: RadarSnapshot) -> bytes:
    """Same as :func:`jetson_json_encode_v2` but returning bytes."""
    return jetson_json_encode_v2(snap).encode("utf-8")


__all__ = [
    "JSON_PROTOCOL_VERSION",
    "ADAS_MAX_RADARS",
    "RADAR_ORDER",
    "RadarSample",
    "RadarSnapshot",
    "jetson_json_encode_v2",
    "jetson_json_encode_v2_bytes",
]
