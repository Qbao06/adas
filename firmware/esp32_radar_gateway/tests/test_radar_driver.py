"""Host tests for the RadarDriver interface.

These tests cover:

* :class:`MockRadarDriver` -- deterministic, no hardware required.
* :class:`RealRadarDriver` -- host stub that consumes a fake
  ``HardwareSerial``-like port and feeds bytes through the parser.
* The full driver contract: ``begin`` + ``read`` returning samples
  with all canonical fields populated.

The tests deliberately do NOT exercise any real HardwareSerial; the
production C++ driver lives in ``ld2410_uart_driver.cpp`` and is
covered by the firmware build (Prompt 2 will exercise that path).
"""
from __future__ import annotations

import os
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from firmware.esp32_radar_gateway.src.radar_driver import (  # noqa: E402
    MockRadarDriver,
    RadarDriverConfig,
    RealRadarDriver,
)
from firmware.esp32_radar_gateway.tests.ld2410_parser_py import (  # noqa: E402
    TARGET_STATE_MOVING,
    TARGET_STATE_NONE,
    LD2410_DISTANCE_NONE,
    Ld2410Parser,
    make_mock_frame,
)


def test_mock_driver_returns_valid_samples_within_first_read():
    cfg = RadarDriverConfig(radar_id="left_front", slot=0, baudrate=256000, seed=1)
    driver = MockRadarDriver(cfg)
    driver.begin()
    sample = driver.read(now_ms=10)
    assert sample.radar_id == "left_front"
    assert sample.valid == 1
    assert sample.stale == 0
    assert sample.target_state == TARGET_STATE_MOVING
    assert sample.detected == 1
    assert 30 <= sample.detection_distance_cm <= 500


def test_mock_driver_produces_independent_sweeps_per_slot():
    a = MockRadarDriver(RadarDriverConfig("left_front", slot=0, seed=1))
    b = MockRadarDriver(RadarDriverConfig("left_rear", slot=1, seed=1))
    a.begin()
    b.begin()
    sa = a.read(now_ms=2000)
    sb = b.read(now_ms=2000)
    # Same seed, different slot -> different distance (phase shifted).
    assert sa.moving_distance_cm != sb.moving_distance_cm


def test_mock_driver_handles_now_ms_wraparound_safely():
    cfg = RadarDriverConfig("right_front", slot=2, seed=42)
    driver = MockRadarDriver(cfg)
    driver.begin()
    s1 = driver.read(now_ms=10)
    s2 = driver.read(now_ms=4_000_000_000)  # well past millis wrap point
    assert s1.valid == 1 and s2.valid == 1
    assert s1.target_state == TARGET_STATE_MOVING
    assert s2.target_state == TARGET_STATE_MOVING


# --------------------------------------------------------------------------- #
# RealRadarDriver (host stub).
# --------------------------------------------------------------------------- #


class _FakePort:
    """Mimic Arduino's ``HardwareSerial.read``/``available`` shape."""

    def __init__(self, data: bytes) -> None:
        self._buf = bytearray(data)
        self._reads = 0

    def available(self) -> int:
        return len(self._buf)

    def read(self) -> int:
        self._reads += 1
        if not self._buf:
            return -1
        return self._buf.pop(0)


def test_real_driver_drains_uart_and_returns_first_valid_sample():
    cfg = RadarDriverConfig("left_front", slot=0, baudrate=256000)
    frame = make_mock_frame(
        target_state=TARGET_STATE_MOVING,
        moving_distance_cm=180,
        stationary_distance_cm=LD2410_DISTANCE_NONE,
        moving_energy=42,
        stationary_energy=0xFF,
    )
    port = _FakePort(frame + make_mock_frame(
        target_state=TARGET_STATE_MOVING,
        moving_distance_cm=240,
        stationary_distance_cm=LD2410_DISTANCE_NONE,
        moving_energy=55,
        stationary_energy=0xFF,
    ))
    captured = []

    def on_sample(slot, sample, user):
        captured.append((slot, sample))

    driver = RealRadarDriver(cfg, port=port, on_sample=on_sample)
    driver.begin()
    sample = driver.read(now_ms=123)
    assert sample.valid == 1
    assert sample.radar_id == "left_front"
    # Two frames were emitted through the callback.
    assert len(captured) == 2
    assert captured[0][1].detection_distance_cm == 180
    assert captured[1][1].detection_distance_cm == 240
    # The driver's ``last_sample`` matches the last frame.
    assert sample.detection_distance_cm == 240


def test_real_driver_with_no_port_returns_zero_sample():
    cfg = RadarDriverConfig("right_rear", slot=1)
    driver = RealRadarDriver(cfg, port=None, on_sample=None)
    driver.begin()
    sample = driver.read(now_ms=0)
    # No port bound, no bytes fed -> empty sample, valid=False, stale=True.
    assert sample.radar_id == "right_rear"
    assert sample.valid == 0
    assert sample.stale == 1


def test_real_driver_survives_garbage_then_valid_frame():
    cfg = RadarDriverConfig("left_rear", slot=1)
    garbage_then_frame = b"\x00\x11\x22\x33\x44" + make_mock_frame(
        target_state=TARGET_STATE_MOVING,
        moving_distance_cm=200,
        stationary_distance_cm=LD2410_DISTANCE_NONE,
        moving_energy=33,
        stationary_energy=0xFF,
    )
    driver = RealRadarDriver(cfg, port=_FakePort(garbage_then_frame), on_sample=None)
    driver.begin()
    sample = driver.read(now_ms=42)
    assert sample.valid == 1
    assert sample.detection_distance_cm == 200


def test_real_driver_rejects_corrupt_checksum_and_recovers():
    cfg = RadarDriverConfig("left_front", slot=0)
    good = make_mock_frame(
        target_state=TARGET_STATE_MOVING,
        moving_distance_cm=200,
        stationary_distance_cm=LD2410_DISTANCE_NONE,
        moving_energy=33,
        stationary_energy=0xFF,
    )
    # Corrupt the last byte (checksum).
    corrupted = bytes(good[:-1]) + bytes([good[-1] ^ 0xFF])
    driver = RealRadarDriver(cfg, port=_FakePort(corrupted + good), on_sample=None)
    driver.begin()
    sample = driver.read(now_ms=99)
    # The corrupt frame is rejected and the second (good) frame survives.
    assert sample.valid == 1
    assert sample.detection_distance_cm == 200


def test_real_driver_never_returns_distance_when_target_is_none():
    cfg = RadarDriverConfig("right_front", slot=2)
    frame = make_mock_frame(
        target_state=TARGET_STATE_NONE,
        moving_distance_cm=LD2410_DISTANCE_NONE,
        stationary_distance_cm=LD2410_DISTANCE_NONE,
        moving_energy=0xFF,
        stationary_energy=0xFF,
    )
    driver = RealRadarDriver(cfg, port=_FakePort(frame), on_sample=None)
    driver.begin()
    sample = driver.read(now_ms=10)
    # The parser resets target_state and keeps distance sentinel on no target.
    assert sample.target_state == TARGET_STATE_NONE
    assert sample.detected == 0
    assert sample.detection_distance_cm == LD2410_DISTANCE_NONE
