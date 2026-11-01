"""
Python mirror of firmware/esp32_radar_gateway/src/espnow_protocol.{h,cpp}.

Kept byte-for-byte compatible with the C++ implementation.  The test
suite relies on this agreement so a single change can be verified in
both worlds.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import List

ESPNOW_MAGIC = 0x52444E31  # "RDN1"
ESPNOW_PROTOCOL_VERSION = 2
ESPNOW_NODE_GATEWAY = 1
ESPNOW_NODE_REMOTE = 2

ESPNOW_RADARS_PER_PACKET = 2

LD2410_DISTANCE_NONE = 0xFFFF
LD2410_ENERGY_NONE = 0xFF
TARGET_STATE_NONE = 0x00
TARGET_STATE_MOVING = 0x01
TARGET_STATE_STATIONARY = 0x02

RADAR_ERROR_NONE = 0x00
RADAR_ERROR_NO_FRAME = 0x01


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


@dataclass
class EspNowRadar:
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


def espnow_checksum(data: bytes) -> int:
    """FNV-1a 32."""
    h = 0x811C9DC5
    for b in data:
        h ^= b
        h = (h * 0x01000193) & 0xFFFFFFFF
    return h


def espnow_pack_radar(sample: RadarSample) -> EspNowRadar:
    return EspNowRadar(
        target_state=sample.target_state,
        detected=sample.detected,
        valid=sample.valid,
        stale=sample.stale,
        moving_distance_cm=sample.moving_distance_cm,
        stationary_distance_cm=sample.stationary_distance_cm,
        moving_energy=sample.moving_energy,
        stationary_energy=sample.stationary_energy,
        detection_distance_cm=sample.detection_distance_cm,
        error_code=sample.error_code,
        sample_age_ms=sample.sample_age_ms,
    )


def espnow_unpack_radar(r: EspNowRadar, radar_id: str, now_ms: int) -> RadarSample:
    return RadarSample(
        radar_id=radar_id[:15],
        target_state=r.target_state,
        detected=r.detected,
        valid=r.valid,
        stale=r.stale,
        moving_distance_cm=r.moving_distance_cm,
        stationary_distance_cm=r.stationary_distance_cm,
        moving_energy=r.moving_energy,
        stationary_energy=r.stationary_energy,
        detection_distance_cm=r.detection_distance_cm,
        error_code=r.error_code,
        sample_age_ms=r.sample_age_ms,
        last_update_ms=now_ms,
    )


_ESPNOW_RADAR_FMT = "<BBBBHHBBBHBBxx"


def _pack_radar_bytes(r: EspNowRadar) -> bytes:
    # Format <BBBBHHBBBHBBxx: 11 fields + 1 padding block.
    return struct.pack(
        _ESPNOW_RADAR_FMT,
        r.target_state,
        r.detected,
        r.valid,
        r.stale,
        r.moving_distance_cm,
        r.stationary_distance_cm,
        r.moving_energy,
        r.stationary_energy,
        r.detection_distance_cm,
        r.error_code,
        r.sample_age_ms,
        0,  # reserved (xx in the format pads 2 bytes)
    )


def _unpack_radar_bytes(b: bytes) -> EspNowRadar:
    fields = struct.unpack(_ESPNOW_RADAR_FMT, b)
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
    )


def espnow_encode(*, node_id: int, boot_id: int, sequence: int,
                  timestamp_ms: int, radars: List[RadarSample]) -> bytes:
    if radars is None or len(radars) != ESPNOW_RADARS_PER_PACKET:
        raise ValueError("radars must be a list of 2 RadarSample")
    # Match the C++ `uint32_t` truncation: anything above 2^32 - 1 is
    # masked to fit the on-wire field.  The Gateway aggregator detects
    # a real wrap via boot_id change, not by sequence alone.
    sequence = sequence & 0xFFFFFFFF
    boot_id = boot_id & 0xFFFFFFFF
    timestamp_ms = timestamp_ms & 0xFFFFFFFF
    packed_radars = [_pack_radar_bytes(espnow_pack_radar(r)) for r in radars]
    radar_count = ESPNOW_RADARS_PER_PACKET
    header = struct.pack(
        "<IBBI I I H H",
        ESPNOW_MAGIC,
        ESPNOW_PROTOCOL_VERSION,
        node_id,
        boot_id,
        sequence,
        timestamp_ms,
        0,  # placeholder for packet_size
        radar_count,
    )
    body = header + b"".join(packed_radars)
    packet_size = len(body) + 4  # +4 for the trailing checksum
    # Replace placeholder with the real size.
    body = body[:18] + struct.pack("<H", packet_size) + body[20:]
    checksum = espnow_checksum(body)
    return body + struct.pack("<I", checksum)


def espnow_decode(buf: bytes) -> EspNowPacket:
    if len(buf) < 22 + 2 * 17 + 4:
        raise ValueError("espnow: buffer too short")
    magic, version, node_id, boot_id, sequence, timestamp_ms, packet_size, radar_count = struct.unpack_from(
        "<IBBI I I H H", buf, 0
    )
    if magic != ESPNOW_MAGIC:
        raise ValueError(f"espnow: bad magic 0x{magic:08X}")
    if version != ESPNOW_PROTOCOL_VERSION:
        raise ValueError(f"espnow: bad protocol_version {version}")
    if node_id not in (ESPNOW_NODE_GATEWAY, ESPNOW_NODE_REMOTE):
        raise ValueError(f"espnow: bad node_id {node_id}")
    if radar_count != ESPNOW_RADARS_PER_PACKET:
        raise ValueError(f"espnow: bad radar_count {radar_count}")
    expected_size = 22 + ESPNOW_RADARS_PER_PACKET * 17 + 4
    if packet_size != expected_size or len(buf) != packet_size:
        raise ValueError(f"espnow: packet_size mismatch (got {packet_size}, expected {expected_size})")
    radars = []
    for i in range(ESPNOW_RADARS_PER_PACKET):
        start = 22 + i * 17
        radars.append(_unpack_radar_bytes(buf[start:start + 17]))
    checksum = struct.unpack_from("<I", buf, len(buf) - 4)[0]
    expected_csum = espnow_checksum(buf[: len(buf) - 4])
    if checksum != expected_csum:
        raise ValueError(f"espnow: checksum mismatch (got 0x{checksum:08X}, expected 0x{expected_csum:08X})")
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
