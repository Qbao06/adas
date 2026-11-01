"""Python mirror of ``firmware/esp32_radar_gateway/src/espnow_protocol.{h,cpp}``.

This module is intentionally written to match the C++ codec byte for byte.
It allows:

* the firmware host tests (``firmware/esp32_radar_gateway/tests/``) to be
  authored against the same encoder/decoder used on-device,
* the CLI monitor, soak and fault-injection tools to construct synthetic
  ESP-NOW frames without writing C++ test rigs.

Wire format
-----------

``offset size  field``

* 0  4     magic            = ``ESPNOW_MAGIC`` (``"RDN1"`` as little-endian uint32)
* 4  1     protocol_version
* 5  1     node_id          (1 = gateway, 2 = remote)
* 6  4     boot_id
* 10 4     sequence         (monotonic per boot_id)
* 14 4     timestamp_ms
* 18 2     packet_size      (bytes from ``header[0]`` through checksum)
* 20 2     radar_count      (always 2 for the Remote)
* 22 N     radar_samples    (see :class:`EspNowRadar`)
* ... 4    checksum         (FNV-1a 32 bit)

The Gateway MUST verify (per the prompt-1 contract):

* magic,
* ``protocol_version == ESPNOW_PROTOCOL_VERSION``,
* ``radar_count == 2``,
* ``packet_size`` matches the actual received length,
* checksum matches the recomputed value,
* ``boot_id`` changes are accepted as a reboot (sequence resets),
* sequence is monotonic per ``boot_id``; duplicates / older seqs are dropped.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import List, Tuple

# --- Wire constants ------------------------------------------------------- #
ESPNOW_MAGIC: int = 0x52444E31  # "RDN1" little-endian
ESPNOW_PROTOCOL_VERSION: int = 2
ESPNOW_NODE_GATEWAY: int = 1
ESPNOW_NODE_REMOTE: int = 2

ESPNOW_HEADER_FIELDS_SIZE: int = 22  # magic..radar_count
ESPNOW_CHECKSUM_SIZE: int = 4
ESPNOW_RADARS_PER_PACKET: int = 2  # one Remote = 2 radars

# --- Header wire format ---------------------------------------------------- #
# Layout (little-endian, fixed-size, no padding):
#   offset  size  field
#     0      4    magic           (u32 LE)
#     4      1    protocol_version (u8)
#     5      1    node_id         (u8)
#     6      4    boot_id         (u32 LE)
#    10      4    sequence        (u32 LE)
#    14      4    timestamp_ms    (u32 LE)
#    18      2    packet_size     (u16 LE)
#    20      2    radar_count     (u16 LE)
# Total: 22 bytes.  Matches ``firmware/esp32_radar_gateway/src/espnow_protocol.h``.
_ESPNOW_HEADER_FMT: str = "<IBBIIIHH"
assert struct.calcsize(_ESPNOW_HEADER_FMT) == ESPNOW_HEADER_FIELDS_SIZE, (
    f"wire header mismatch: expected {ESPNOW_HEADER_FIELDS_SIZE} bytes, "
    f"got {struct.calcsize(_ESPNOW_HEADER_FMT)}"
)

# LD2410 sentinels and target-state constants.
LD2410_DISTANCE_NONE: int = 0xFFFF
LD2410_ENERGY_NONE: int = 0xFF
TARGET_STATE_NONE: int = 0x00
TARGET_STATE_MOVING: int = 0x01
TARGET_STATE_STATIONARY: int = 0x02
TARGET_STATE_BOTH: int = 0x03

RADAR_ERROR_NONE: int = 0x00
RADAR_ERROR_NO_FRAME: int = 0x01
RADAR_ERROR_FRAME_CRC: int = 0x02
RADAR_ERROR_LENGTH_MISMATCH: int = 0x03
RADAR_ERROR_BAD_HEADER: int = 0x04
RADAR_ERROR_BAD_FOOTER: int = 0x05
RADAR_ERROR_DROP: int = 0x06

# On-the-wire radar layout: see ``espnow_protocol.h`` in the firmware.
# Fields (in order, 11 packed values + 2 reserved bytes):
#   target_state(u8) detected(u8) valid(u8) stale(u8)
#   moving_distance_cm(u16) stationary_distance_cm(u16)
#   moving_energy(u8) stationary_energy(u8)
#   detection_distance_cm(u16) error_code(u8) sample_age_ms(u16)
#   reserved[2] -- packed as ``2s`` so total struct size == 17 bytes.
_ESPNOW_RADAR_FMT: str = "<BBBBHHBBHBH2s"
assert struct.calcsize(_ESPNOW_RADAR_FMT) == 17, (
    f"radar layout mismatch: expected 17 bytes, got {struct.calcsize(_ESPNOW_RADAR_FMT)}"
)

ESPNOW_PAYLOAD_SIZE: int = (
    ESPNOW_HEADER_FIELDS_SIZE
    + struct.calcsize(_ESPNOW_RADAR_FMT) * ESPNOW_RADARS_PER_PACKET
    + ESPNOW_CHECKSUM_SIZE
)


# --- Helpers --------------------------------------------------------------- #
def espnow_checksum(data: bytes) -> int:
    """FNV-1a 32 bit checksum; identical to the C++ implementation."""
    h = 0x811C9DC5
    for b in data:
        h ^= b
        h = (h * 0x01000193) & 0xFFFFFFFF
    return h


def _parse_error(code: int) -> str:
    return {
        2: "bad magic",
        3: "bad protocol_version",
        4: "bad radar_count",
        5: "packet_size mismatch",
        6: "checksum mismatch",
    }.get(code, f"decode error {code}")


# --- Data classes ---------------------------------------------------------- #
@dataclass
class RadarSample:
    """Host-side radar representation.  Same fields as the C++ struct."""

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
class EspNowRadar:
    """On-the-wire radar sample."""

    target_state: int = 0
    detected: int = 0
    valid: int = 0
    stale: int = 0
    moving_distance_cm: int = 0
    stationary_distance_cm: int = 0
    moving_energy: int = 0
    stationary_energy: int = 0
    detection_distance_cm: int = 0
    error_code: int = 0
    sample_age_ms: int = 0
    reserved: bytes = b"\x00\x00"


@dataclass
class EspNowPacket:
    magic: int = 0
    protocol_version: int = 0
    node_id: int = 0
    boot_id: int = 0
    sequence: int = 0
    timestamp_ms: int = 0
    packet_size: int = 0
    radar_count: int = 0
    radars: List[EspNowRadar] = field(default_factory=list)
    checksum: int = 0


# --- Pack / unpack --------------------------------------------------------- #
def espnow_pack_radar(sample: RadarSample) -> EspNowRadar:
    return EspNowRadar(
        target_state=sample.target_state & 0x03,
        detected=sample.detected & 0x01,
        valid=sample.valid & 0x01,
        stale=sample.stale & 0x01,
        moving_distance_cm=sample.moving_distance_cm & 0xFFFF,
        stationary_distance_cm=sample.stationary_distance_cm & 0xFFFF,
        moving_energy=sample.moving_energy & 0xFF,
        stationary_energy=sample.stationary_energy & 0xFF,
        detection_distance_cm=sample.detection_distance_cm & 0xFFFF,
        error_code=sample.error_code & 0xFF,
        sample_age_ms=sample.sample_age_ms & 0xFFFF,
    )


def espnow_unpack_radar(r: EspNowRadar, radar_id: str, now_ms: int) -> RadarSample:
    return RadarSample(
        radar_id=radar_id[:15],
        target_state=r.target_state & 0x03,
        detected=r.detected & 0x01,
        valid=r.valid & 0x01,
        stale=r.stale & 0x01,
        moving_distance_cm=r.moving_distance_cm,
        stationary_distance_cm=r.stationary_distance_cm,
        moving_energy=r.moving_energy,
        stationary_energy=r.stationary_energy,
        detection_distance_cm=r.detection_distance_cm,
        error_code=r.error_code,
        sample_age_ms=r.sample_age_ms,
        last_update_ms=int(now_ms) & 0xFFFFFFFF,
    )


def _pack_radar_bytes(r: EspNowRadar) -> bytes:
    # Format <BBBBHHBBHBHxx: 11 type slots + 2 padding bytes = 17 wire
    # bytes (matches sizeof(EspNowRadar) on the C side).  Field order:
    #   0 target_state    1 detected       2 valid          3 stale
    #   4 moving_cm       5 stationary_cm  6 moving_energy  7 stat_energy
    #   8 detection_cm    9 error_code     10 sample_age_ms
    #   11 reserved (2 bytes, packed as ``2s``)
    reserved = bytes(r.reserved) if r.reserved is not None else b"\x00\x00"
    if len(reserved) != 2:
        reserved = (reserved + b"\x00\x00")[:2]
    return struct.pack(
        _ESPNOW_RADAR_FMT,
        r.target_state,        # B
        r.detected,            # B
        r.valid,               # B
        r.stale,               # B
        r.moving_distance_cm,  # H
        r.stationary_distance_cm,  # H
        r.moving_energy,       # B
        r.stationary_energy,   # B
        r.detection_distance_cm,  # H
        r.error_code,          # B
        r.sample_age_ms,       # H
        reserved,              # 2s
    )


def _unpack_radar_bytes(buf: bytes) -> EspNowRadar:
    # The format's 2s slot is the trailing ``reserved`` bytes; we feed
    # them straight back into the dataclass so callers can introspect.
    fields = struct.unpack(_ESPNOW_RADAR_FMT, buf)
    return EspNowRadar(
        target_state=fields[0],
        detected=fields[1],
        valid=fields[2],
        stale=fields[3],
        moving_distance_cm=fields[4],
        stationary_distance_cm=fields[5],
        moving_energy=fields[6],
        stationary_energy=fields[7],
        detection_distance_cm=fields[8],
        error_code=fields[9],
        sample_age_ms=fields[10],
        reserved=fields[11],
    )


# --- Encode / decode ------------------------------------------------------- #
def espnow_encode(
    *,
    node_id: int,
    boot_id: int,
    sequence: int,
    timestamp_ms: int,
    radars: List[RadarSample],
) -> bytes:
    """Encode one ESP-NOW packet.  Returns the wire bytes.

    Raises :class:`ValueError` if ``radars`` does not contain exactly two
    samples (the protocol v2 contract for the Remote).
    """
    if radars is None or len(radars) != ESPNOW_RADARS_PER_PACKET:
        raise ValueError(
            f"radars must be a list of {ESPNOW_RADARS_PER_PACKET} RadarSample"
        )
    packed_radars = b"".join(
        _pack_radar_bytes(espnow_pack_radar(r)) for r in radars
    )
    # Header layout (22 bytes) -- matches firmware espnow_protocol.h:
    #   magic(4) | version(1) | node_id(1) | boot_id(4)
    #   sequence(4) | timestamp_ms(4) | packet_size(2) | radar_count(2)
    # The format string matches what espnow_decode unpacks and the wire
    # layout in the firmware.
    placeholder = struct.pack(
        _ESPNOW_HEADER_FMT,
        ESPNOW_MAGIC,
        ESPNOW_PROTOCOL_VERSION,
        int(node_id) & 0xFF,
        int(boot_id) & 0xFFFFFFFF,
        int(sequence) & 0xFFFFFFFF,
        int(timestamp_ms) & 0xFFFFFFFF,
        0,  # placeholder for packet_size (filled in below)
        ESPNOW_RADARS_PER_PACKET,
    )
    # Replace the packet_size field at offset 18 (uint16 LE).
    header = placeholder[:18] + struct.pack("<H", ESPNOW_PAYLOAD_SIZE) + placeholder[20:]
    body = header + packed_radars
    assert len(body) == ESPNOW_HEADER_FIELDS_SIZE + len(packed_radars)
    checksum = espnow_checksum(body)
    return body + struct.pack("<I", checksum)


def espnow_decode(buf: bytes) -> EspNowPacket:
    """Decode one ESP-NOW packet.  Raises :class:`ValueError` on any fault.

    Error categories:
        * bad magic,
        * unsupported ``protocol_version`` or ``node_id``,
        * bad ``radar_count``,
        * ``packet_size`` mismatch (buffer too short / too long),
        * checksum mismatch.
    """
    if len(buf) != ESPNOW_PAYLOAD_SIZE:
        # The firmware requires an exact-length match; report this as a
        # packet_size mismatch so the diagnostic matches the on-wire field.
        raise ValueError(
            f"espnow: packet_size mismatch (got {len(buf)}, expected {ESPNOW_PAYLOAD_SIZE})"
        )

    magic, version, node_id, boot_id, sequence, timestamp_ms, packet_size, radar_count = (
        struct.unpack_from(_ESPNOW_HEADER_FMT, buf, 0)
    )
    # Verify the packet_size field carried inside the header before any
    # other validation.  This way truncation / length-mismatch failures
    # always surface as ``packet_size`` in the error message.
    if packet_size != ESPNOW_PAYLOAD_SIZE:
        raise ValueError(
            f"espnow: packet_size mismatch (got {packet_size}, expected {ESPNOW_PAYLOAD_SIZE})"
        )
    if magic != ESPNOW_MAGIC:
        raise ValueError(f"espnow: {_parse_error(2)} (0x{magic:08X})")
    if version != ESPNOW_PROTOCOL_VERSION:
        raise ValueError(f"espnow: {_parse_error(3)} ({version})")
    if node_id not in (ESPNOW_NODE_GATEWAY, ESPNOW_NODE_REMOTE):
        raise ValueError(f"espnow: bad node_id ({node_id})")
    if radar_count != ESPNOW_RADARS_PER_PACKET:
        raise ValueError(f"espnow: {_parse_error(4)} ({radar_count})")

    radar_size = struct.calcsize(_ESPNOW_RADAR_FMT)
    radars: List[EspNowRadar] = []
    for i in range(ESPNOW_RADARS_PER_PACKET):
        start = ESPNOW_HEADER_FIELDS_SIZE + i * radar_size
        radars.append(_unpack_radar_bytes(buf[start : start + radar_size]))

    checksum = struct.unpack_from("<I", buf, len(buf) - 4)[0]
    expected_csum = espnow_checksum(buf[: len(buf) - 4])
    if checksum != expected_csum:
        raise ValueError(
            f"espnow: {_parse_error(6)} (got 0x{checksum:08X}, expected 0x{expected_csum:08X})"
        )

    return EspNowPacket(
        magic=magic,
        protocol_version=version,
        node_id=node_id,
        boot_id=boot_id,
        sequence=sequence,
        timestamp_ms=timestamp_ms,
        packet_size=packet_size,
        radar_count=radar_count,
        radars=radars,
        checksum=checksum,
    )


# --- Convenience helpers --------------------------------------------------- #
def detect_sequence_wrap(
    current: int,
    previous: int,
) -> bool:
    """Return True when ``current`` looks like a 32-bit wrap of ``previous``.

    The firmware uses an unsigned 32-bit sequence number.  When it
    wraps, ``current < previous`` is normal.  Callers should also
    observe ``boot_id`` changes to fully disambiguate reboots.
    """
    if previous is None or current is None:
        return False
    # Strict monotonic (current > previous) is not a wrap.
    if current > previous:
        return False
    # Equal is a duplicate; not a wrap.  Strictly decreasing without
    # crossing the 2^32 boundary is also not a wrap (regression).
    if current == previous:
        return False
    # Wrap candidates: current < previous AND the gap is large enough
    # that we must have crossed 2^32.
    if (previous - current) >= (1 << 31):
        return True
    return False


__all__: Tuple[str, ...] = (
    "ESPNOW_MAGIC",
    "ESPNOW_PROTOCOL_VERSION",
    "ESPNOW_NODE_GATEWAY",
    "ESPNOW_NODE_REMOTE",
    "ESPNOW_HEADER_FIELDS_SIZE",
    "ESPNOW_CHECKSUM_SIZE",
    "ESPNOW_RADARS_PER_PACKET",
    "ESPNOW_PAYLOAD_SIZE",
    "LD2410_DISTANCE_NONE",
    "LD2410_ENERGY_NONE",
    "TARGET_STATE_NONE",
    "TARGET_STATE_MOVING",
    "TARGET_STATE_STATIONARY",
    "TARGET_STATE_BOTH",
    "RADAR_ERROR_NONE",
    "RADAR_ERROR_NO_FRAME",
    "RADAR_ERROR_FRAME_CRC",
    "RADAR_ERROR_LENGTH_MISMATCH",
    "RADAR_ERROR_BAD_HEADER",
    "RADAR_ERROR_BAD_FOOTER",
    "RADAR_ERROR_DROP",
    "RadarSample",
    "EspNowRadar",
    "EspNowPacket",
    "espnow_checksum",
    "espnow_pack_radar",
    "espnow_unpack_radar",
    "espnow_encode",
    "espnow_decode",
    "detect_sequence_wrap",
)
