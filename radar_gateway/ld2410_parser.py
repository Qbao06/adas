"""Python mirror of ``firmware/esp32_radar_gateway/src/ld2410_parser.{h,cpp}``.

The HLK-LD2410C is a 24 GHz FMCW presence sensor with a binary UART
protocol.  The header magic ``F4 F3 F2 F1`` and footer magic
``F8 F7 F6 F5`` are part of the publicly documented protocol.  The
internal offsets for moving/stationary gate distances and energies are
**not** yet validated against the official datasheet -- the firmware
MUST treat parsed values as untrusted until they are cross-checked on
real hardware.

To keep the contract safe, this parser:

* Verifies header, footer, length and the LD2410 internal frame
  checksum (last byte == sum of bytes from length field through
  footer, masked to 8 bits).
* Extracts only the two bytes that the LD2410 basic report places at
  known fixed offsets: target state (byte 8 of payload) and combined
  detection distance (bytes 9-10 little-endian).
* Leaves moving/stationary gate distance+energy at the no-target
  sentinel unless ``LD2410_PARSE_FULL_GATES`` is enabled AND the exact
  gate layout has been validated against the datasheet.
* Never fabricates a non-zero distance on a malformed frame.

Stream behaviour:

* We accumulate bytes until the header is found; garbage before the
  header is silently dropped.
* After the header comes a 2-byte little-endian length field.  The full
  frame size is ``length_field + 9``.  We use that as the expected
  length and refuse to buffer more than ``LD2410_MAX_FRAME_LEN`` bytes.
* Once we have the expected length, we wait for the remaining bytes,
  then verify the footer and the LD2410 internal checksum.
* A failed verification clears the buffer, sets ``sync_lost``, and the
  next call resumes scanning for a new header.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

# Reuse the protocol-level constants so the two modules stay in sync.
from .protocol import (
    LD2410_DISTANCE_NONE,
    LD2410_ENERGY_NONE,
    RADAR_ERROR_BAD_FOOTER,
    RADAR_ERROR_BAD_HEADER,
    RADAR_ERROR_FRAME_CRC,
    RADAR_ERROR_LENGTH_MISMATCH,
    RADAR_ERROR_NO_FRAME,
    RADAR_ERROR_NONE,
    TARGET_STATE_MOVING,
    TARGET_STATE_NONE,
    TARGET_STATE_STATIONARY,
)

LD2410_HEADER: tuple = (0xF4, 0xF3, 0xF2, 0xF1)
LD2410_FOOTER: tuple = (0xF8, 0xF7, 0xF6, 0xF5)

LD2410_MAX_FRAME_LEN: int = 64

# Offsets used by the basic report frame.  See the C++ header for the
# full byte-by-byte layout.
LD2410_OFFSET_TARGET_STATE: int = 9
LD2410_OFFSET_DETECTION_DIST_LO: int = 10
LD2410_OFFSET_DETECTION_DIST_HI: int = 11
LD2410_BASIC_PAYLOAD_LEN: int = 6


@dataclass
class RadarSample:
    """Result of parsing a single LD2410 frame."""

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


@dataclass
class Ld2410Parser:
    """Incremental byte-by-byte LD2410 frame parser.

    Usage::

        parser = Ld2410Parser()
        sample = None
        for byte in serial_port:
            out = parser.feed(byte, "left_front", now_ms=millis())
            if out is not None:
                sample = out
    """

    buf: bytearray = None  # type: ignore[assignment]
    expected_len: int = 0
    sync_lost: int = 1

    def __init__(self) -> None:
        self.buf = bytearray()
        self.expected_len = 0
        self.sync_lost = 1  # start in resync state

    # ------------------------------------------------------------- helpers
    @staticmethod
    def _checksum(data: bytes) -> int:
        return sum(data) & 0xFF

    @staticmethod
    def _reset_sample(sample: RadarSample) -> None:
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

    # ------------------------------------------------------------- feed API
    def feed(
        self,
        byte_in: int,
        radar_id: str,
        now_ms: int = 0,
    ) -> Optional[RadarSample]:
        """Feed one UART byte into the parser.

        Returns a populated :class:`RadarSample` when a complete valid
        frame is consumed, otherwise ``None``.
        """
        if len(self.buf) >= LD2410_MAX_FRAME_LEN:
            self.buf.clear()
            self.sync_lost = 1

        self.buf.append(byte_in & 0xFF)

        if self.sync_lost or self.expected_len == 0:
            # Scan forward for header.
            while len(self.buf) >= 4 and tuple(self.buf[:4]) != LD2410_HEADER:
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
        consumed = self._try_parse(bytes(self.buf[: self.expected_len]), radar_id, now_ms, sample)
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

    # ------------------------------------------------------------- private
    def _try_parse(
        self,
        buf: bytes,
        radar_id: str,
        now_ms: int,
        sample: RadarSample,
    ) -> int:
        """Return the number of bytes consumed on success, 0 on failure."""
        if len(buf) < 14:
            return 0
        if tuple(buf[:4]) != LD2410_HEADER:
            return 0
        if tuple(buf[-5:-1]) != LD2410_FOOTER:
            return 0
        lf = buf[4] | (buf[5] << 8)
        if len(buf) != 11 + lf:
            return 0
        expected_csum = self._checksum(buf[4 : 4 + 2 + lf + 4])
        if expected_csum != buf[-1]:
            return 0

        self._reset_sample(sample)
        sample.radar_id = (radar_id or "")[:15]
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
        sample.last_update_ms = int(now_ms) & 0xFFFFFFFF
        return len(buf)


def make_mock_frame(
    *,
    target_state: int,
    moving_distance_cm: int,
    stationary_distance_cm: int,
    moving_energy: int,
    stationary_energy: int,
) -> bytes:
    """Build a minimal LD2410 basic-mode frame that round-trips the parser.

    Wire layout (basic mode):
        header(4) + length(2) + frame_type(1) + head_marker(1) + tail_marker(1)
            + target_state(1) + detection_distance(2 LE)
            + footer(4) + checksum(1)

    The parser reads:
        target_state at index 9 (LD2410_OFFSET_TARGET_STATE)
        distance_lo at index 10, distance_hi at index 11

    This is a Python counterpart to ``ld2410_parser_make_mock_frame`` in
    the firmware.
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
    buf += bytes(
        [moving_distance_cm & 0xFF, (moving_distance_cm >> 8) & 0xFF]
    )
    buf += bytes(LD2410_FOOTER)
    csum = sum(buf[4:]) & 0xFF
    buf += bytes([csum])
    return bytes(buf)


__all__ = [
    "LD2410_HEADER",
    "LD2410_FOOTER",
    "LD2410_MAX_FRAME_LEN",
    "LD2410_OFFSET_TARGET_STATE",
    "LD2410_OFFSET_DETECTION_DIST_LO",
    "LD2410_OFFSET_DETECTION_DIST_HI",
    "LD2410_BASIC_PAYLOAD_LEN",
    "RadarSample",
    "Ld2410Parser",
    "make_mock_frame",
]
