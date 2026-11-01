"""
Python host-pure tests for the LD2410 frame parser.

These tests intentionally mirror the C++ parser byte-for-byte so the
two implementations must agree.  When the user verifies the LD2410
datasheet against real hardware and updates the parser, the same byte
sequence in this file must be updated together with the C++ source.
"""
from __future__ import annotations

import os
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from firmware.esp32_radar_gateway.tests.ld2410_parser_py import (  # noqa: E402
    LD2410_DISTANCE_NONE,
    LD2410_ENERGY_NONE,
    TARGET_STATE_NONE,
    TARGET_STATE_MOVING,
    RADAR_ERROR_FRAME_CRC,
    Ld2410Parser,
    make_mock_frame,
)


def _build_basic_frame(
    *,
    target_state: int = TARGET_STATE_MOVING,
    detection_distance_cm: int = 150,
    bad_checksum: bool = False,
    bad_footer: bool = False,
    bad_header: bool = False,
    extra_garbage: bytes = b"",
) -> bytes:
    """Build a minimal LD2410 basic-mode frame.

    Layout (mirrors the C++ parser exactly):
        header(4) | length(2) | payload(6) | footer(4) | checksum(1)
    where payload = frame_type(1) + head_marker(1) + tail_marker(1)
                    + target_state(1) + detection_distance(2 LE).
    Wire indices:
        target_state at byte 9   (LD2410_OFFSET_TARGET_STATE = 9)
        distance_lo  at byte 10  (LD2410_OFFSET_DETECTION_DIST_LO = 10)
        distance_hi  at byte 11  (LD2410_OFFSET_DETECTION_DIST_HI = 11)
    Total length on wire = 4 + 2 + 6 + 4 + 1 = 17 bytes; the parser
    computes expected_total = 11 + length_field.
    """
    buf = bytearray()
    buf += bytes([0xF4, 0xF3, 0xF2, 0xF1])
    lf = 6  # payload length: type(1) + head(1) + tail(1) + target(1) + dist(2)
    buf += bytes([lf & 0xFF, (lf >> 8) & 0xFF])
    buf += bytes([0x02])                # byte 6: frame type
    buf += bytes([0xAA])                # byte 7: head marker
    buf += bytes([0xAA])                # byte 8: tail marker
    buf += bytes([target_state & 0x03]) # byte 9: target_state
    buf += bytes([detection_distance_cm & 0xFF, (detection_distance_cm >> 8) & 0xFF])  # bytes 10..11
    if bad_header:
        buf[0] = 0x00
    buf += bytes([0xF8, 0xF7, 0xF6, 0xF5])
    if bad_footer:
        buf[-4] = 0x00
    # Checksum: sum of length bytes + payload + footer, masked to 8 bits.
    csum_range = buf[4:4 + 2 + lf + 4]
    csum = sum(csum_range) & 0xFF
    if bad_checksum:
        csum ^= 0xFF
    buf += bytes([csum])
    return bytes(buf) + extra_garbage


def test_parser_accepts_well_formed_basic_frame():
    parser = Ld2410Parser()
    sample = None
    for byte in _build_basic_frame(target_state=TARGET_STATE_MOVING, detection_distance_cm=150):
        out = parser.feed(byte, "left_front", now_ms=10)
        if out is not None:
            sample = out
    assert sample is not None
    assert sample.valid is True or sample.valid == 1
    assert sample.stale is False or sample.stale == 0
    assert sample.target_state == TARGET_STATE_MOVING
    assert sample.detection_distance_cm == 150
    assert sample.radar_id == "left_front"
    assert sample.error_code == 0
    assert sample.last_update_ms == 10


def test_parser_resyncs_after_garbage_then_accepts_frame():
    parser = Ld2410Parser()
    sample = None
    garbage = b"\x00\x11\x22\x33\x44" + _build_basic_frame()
    for byte in garbage:
        out = parser.feed(byte, "left_rear", now_ms=20)
        if out is not None:
            sample = out
    assert sample is not None
    assert sample.radar_id == "left_rear"
    assert sample.valid == 1 or sample.valid is True


def test_parser_handles_two_back_to_back_frames():
    parser = Ld2410Parser()
    stream = _build_basic_frame(detection_distance_cm=100) + _build_basic_frame(detection_distance_cm=200)
    seen = []
    for byte in stream:
        out = parser.feed(byte, "right_front", now_ms=30)
        if out is not None:
            seen.append(out)
    assert len(seen) == 2
    assert seen[0].detection_distance_cm == 100
    assert seen[1].detection_distance_cm == 200


def test_parser_rejects_bad_checksum():
    parser = Ld2410Parser()
    surfaced = 0
    for byte in _build_basic_frame(bad_checksum=True):
        out = parser.feed(byte, "left_front", now_ms=40)
        if out is not None:
            surfaced += 1
    assert surfaced == 0  # no good frame ever surfaced


def test_parser_rejects_bad_footer():
    parser = Ld2410Parser()
    surfaced = 0
    for byte in _build_basic_frame(bad_footer=True):
        out = parser.feed(byte, "left_front", now_ms=50)
        if out is not None:
            surfaced += 1
    assert surfaced == 0


def test_parser_rejects_bad_header():
    parser = Ld2410Parser()
    surfaced = 0
    for byte in _build_basic_frame(bad_header=True):
        out = parser.feed(byte, "left_front", now_ms=60)
        if out is not None:
            surfaced += 1
    assert surfaced == 0


def test_parser_does_not_invent_distance_on_no_target():
    parser = Ld2410Parser()
    sample = None
    frame = _build_basic_frame(target_state=TARGET_STATE_NONE, detection_distance_cm=LD2410_DISTANCE_NONE)
    for byte in frame:
        out = parser.feed(byte, "left_front", now_ms=70)
        if out is not None:
            sample = out
    # With no target the parser must reset target_state and leave the
    # detection distance sentinel so the JSON encoder emits null.
    assert sample is not None
    assert sample.target_state == TARGET_STATE_NONE
    assert sample.detection_distance_cm == LD2410_DISTANCE_NONE


def test_parser_drops_overlong_garbage_burst():
    parser = Ld2410Parser()
    burst = bytes(range(256)) * 4  # 1024 bytes of pure garbage
    surfaced = 0
    for byte in burst:
        out = parser.feed(byte, "left_front", now_ms=80)
        if out is not None:
            surfaced += 1
    assert surfaced == 0
    # After the burst, a valid frame must still parse.
    sample = None
    for byte in _build_basic_frame():
        out = parser.feed(byte, "left_front", now_ms=90)
        if out is not None:
            sample = out
    assert sample is not None


def test_mock_frame_round_trips_through_parser():
    parser = Ld2410Parser()
    frame = make_mock_frame(
        target_state=TARGET_STATE_MOVING,
        moving_distance_cm=300,
        stationary_distance_cm=LD2410_DISTANCE_NONE,
        moving_energy=42,
        stationary_energy=LD2410_ENERGY_NONE,
    )
    sample = None
    for byte in frame:
        out = parser.feed(byte, "right_rear", now_ms=100)
        if out is not None:
            sample = out
    assert sample is not None
    assert sample.valid == 1 or sample.valid is True
    assert sample.detection_distance_cm == 300