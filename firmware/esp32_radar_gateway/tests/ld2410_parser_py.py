"""
Python mirror of firmware/esp32_radar_gateway/src/ld2410_parser.{h,cpp}.

This file is intentionally written to match the C++ parser byte for
byte.  When the user validates the LD2410 datasheet and adjusts the
C++ offsets, the same constants MUST be updated here so the host
tests stay authoritative.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

LD2410_DISTANCE_NONE = 0xFFFF
LD2410_ENERGY_NONE = 0xFF

TARGET_STATE_NONE = 0x00
TARGET_STATE_MOVING = 0x01
TARGET_STATE_STATIONARY = 0x02
TARGET_STATE_BOTH = 0x03

RADAR_ERROR_NONE = 0x00
RADAR_ERROR_NO_FRAME = 0x01
RADAR_ERROR_FRAME_CRC = 0x02
RADAR_ERROR_LENGTH_MISMATCH = 0x03
RADAR_ERROR_BAD_HEADER = 0x04
RADAR_ERROR_BAD_FOOTER = 0x05

LD2410_HEADER = (0xF4, 0xF3, 0xF2, 0xF1)
LD2410_FOOTER = (0xF8, 0xF7, 0xF6, 0xF5)

LD2410_MAX_FRAME_LEN = 64

# Offsets used by the LD2410 basic report frame (must mirror C++ source).
LD2410_OFFSET_TARGET_STATE = 9
LD2410_OFFSET_DETECTION_DIST_LO = 10
LD2410_OFFSET_DETECTION_DIST_HI = 11


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
    error_code: int = RADAR_ERROR_NO_FRAME
    sample_age_ms: int = 0xFFFF
    last_update_ms: int = 0


class Ld2410Parser:
    def __init__(self) -> None:
        self.buf: bytearray = bytearray()
        self.expected_len: int = 0
        self.sync_lost: int = 1

    def _reset_sample(self, sample: RadarSample) -> None:
        sample.target_state = TARGET_STATE_NONE
        sample.detected = 0
        sample.valid = 0
        sample.stale = 1
        sample.moving_distance_cm = LD2410_DISTANCE_NONE
        sample.stationary_distance_cm = LD2410_DISTANCE_NONE
        sample.moving_energy = LD2410_ENERGY_NONE
        sample.stationary_energy = LD2410_ENERGY_NONE
        sample.detection_distance_cm = LD2410_DISTANCE_NONE
        sample.error_code = RADAR_ERROR_NO_FRAME
        sample.sample_age_ms = 0xFFFF
        sample.last_update_ms = 0

    @staticmethod
    def _checksum(data: bytes) -> int:
        return sum(data) & 0xFF

    def feed(
        self,
        byte_in: int,
        radar_id: str,
        now_ms: int = 0,
    ) -> Optional[RadarSample]:
        if len(self.buf) >= LD2410_MAX_FRAME_LEN:
            self.buf.clear()
            self.sync_lost = 1

        self.buf.append(byte_in & 0xFF)

        if self.sync_lost or self.expected_len == 0:
            # Scan forward for header.
            while (
                len(self.buf) >= 4
                and tuple(self.buf[:4]) != LD2410_HEADER
            ):
                del self.buf[0]
            if len(self.buf) < 4:
                return None
            if len(self.buf) >= 6:
                lf = self.buf[4] | (self.buf[5] << 8)
                total = 11 + lf
                if lf < 2 or total > LD2410_MAX_FRAME_LEN:
                    del self.buf[0]
                    self.sync_lost = 1
                    return None
                self.expected_len = total
                self.sync_lost = 0
            else:
                return None

        if len(self.buf) < self.expected_len:
            return None

        sample = RadarSample()
        consumed = self._try_parse(self.buf[: self.expected_len], radar_id, now_ms, sample)
        if consumed == 0:
            if len(self.buf) > 1:
                del self.buf[0]
            else:
                self.buf.clear()
            self.expected_len = 0
            self.sync_lost = 1
            return None

        # Drop consumed bytes; keep any remainder for the next call.
        del self.buf[:consumed]
        self.expected_len = 0
        self.sync_lost = 1 if self.buf else 0
        return sample

    def _try_parse(
        self,
        buf: bytes,
        radar_id: str,
        now_ms: int,
        sample: RadarSample,
    ) -> int:
        if len(buf) < 14:
            return 0
        if tuple(buf[:4]) != LD2410_HEADER:
            return 0
        if tuple(buf[-5:-1]) != LD2410_FOOTER:
            return 0
        lf = buf[4] | (buf[5] << 8)
        if len(buf) != 11 + lf:
            return 0
        expected_csum = self._checksum(buf[4:4 + 2 + lf + 4])
        if expected_csum != buf[-1]:
            return 0
        self._reset_sample(sample)
        sample.radar_id = radar_id[:15]
        # C++ ld2410_parser.cpp uses:
        #   LD2410_OFFSET_TARGET_STATE       = 9
        #   LD2410_OFFSET_DETECTION_DIST_LO  = 10
        #   LD2410_OFFSET_DETECTION_DIST_HI  = 11
        # The Python mirror MUST stay byte-for-byte identical to the C++.
        sample.target_state = buf[LD2410_OFFSET_TARGET_STATE] & 0x03
        sample.detection_distance_cm = (
            buf[LD2410_OFFSET_DETECTION_DIST_LO]
            | (buf[LD2410_OFFSET_DETECTION_DIST_HI] << 8)
        )
        if sample.detection_distance_cm == LD2410_DISTANCE_NONE:
            sample.target_state = TARGET_STATE_NONE
        sample.detected = 1 if sample.target_state != TARGET_STATE_NONE else 0
        sample.valid = 1
        sample.stale = 0
        sample.error_code = RADAR_ERROR_NONE
        sample.last_update_ms = now_ms
        return len(buf)


def make_mock_frame(
    *,
    target_state: int,
    moving_distance_cm: int,
    stationary_distance_cm: int,
    moving_energy: int,
    stationary_energy: int,
) -> bytes:
    """Mirror of ld2410_parser_make_mock_frame() in C++.

    Wire layout (basic mode):
        header(4) + length(2) + frame_type(1) + head_marker(1) + tail_marker(1)
            + target_state(1) + detection_distance(2 LE)
            + footer(4) + checksum(1)
        = 17 bytes total
    The parser reads:
        target_state at index 9 (LD2410_OFFSET_TARGET_STATE)
        distance_lo at index 10, distance_hi at index 11
    """
    buf = bytearray()
    buf += bytes(LD2410_HEADER)
    payload_len = 6  # frame_type + head_marker + tail_marker + target_state + distance(2)
    buf += bytes([payload_len & 0xFF, (payload_len >> 8) & 0xFF])
    # byte 6: frame_type
    buf += bytes([0x02])
    # byte 7: head_marker
    buf += bytes([0xAA])
    # byte 8: tail_marker
    buf += bytes([0xAA])
    # byte 9: target_state
    buf += bytes([target_state & 0x03])
    # bytes 10..11: detection_distance_cm little-endian
    buf += bytes([moving_distance_cm & 0xFF, (moving_distance_cm >> 8) & 0xFF])
    # footer + checksum
    buf += bytes(LD2410_FOOTER)
    csum = sum(buf[4:]) & 0xFF
    buf += bytes([csum])
    return bytes(buf)
