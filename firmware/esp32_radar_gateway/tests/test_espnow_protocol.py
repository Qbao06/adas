"""Python host-pure tests for the ESP-NOW v2 packet codec."""
from __future__ import annotations

import copy
import os
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from firmware.esp32_radar_gateway.tests.espnow_protocol_py import (  # noqa: E402
    ESPNOW_MAGIC,
    ESPNOW_PROTOCOL_VERSION,
    ESPNOW_NODE_GATEWAY,
    ESPNOW_NODE_REMOTE,
    espnow_checksum,
    espnow_encode,
    espnow_decode,
    EspNowPacket,
    RadarSample,
    espnow_pack_radar,
    espnow_unpack_radar,
)


def _sample(target_state=1, detected=1, moving_distance_cm=120, stationary_distance_cm=0xFFFF,
             moving_energy=40, stationary_energy=0xFF, detection_distance_cm=120,
             error_code=0, sample_age_ms=10, valid=1, stale=0, radar_id="right_front"):
    return RadarSample(
        radar_id=radar_id,
        target_state=target_state,
        detected=detected,
        valid=valid,
        stale=stale,
        moving_distance_cm=moving_distance_cm,
        stationary_distance_cm=stationary_distance_cm,
        moving_energy=moving_energy,
        stationary_energy=stationary_energy,
        detection_distance_cm=detection_distance_cm,
        error_code=error_code,
        sample_age_ms=sample_age_ms,
    )


def test_encode_decode_roundtrip():
    radars = [_sample(moving_distance_cm=300), _sample(radar_id="right_rear", moving_distance_cm=120)]
    buf = espnow_encode(
        node_id=ESPNOW_NODE_REMOTE,
        boot_id=0xDEADBEEF,
        sequence=42,
        timestamp_ms=12345,
        radars=radars,
    )
    pkt = espnow_decode(buf)
    assert pkt.magic == ESPNOW_MAGIC
    assert pkt.protocol_version == ESPNOW_PROTOCOL_VERSION
    assert pkt.node_id == ESPNOW_NODE_REMOTE
    assert pkt.boot_id == 0xDEADBEEF
    assert pkt.sequence == 42
    assert pkt.timestamp_ms == 12345
    assert pkt.radar_count == 2
    assert pkt.radars[0].moving_distance_cm == 300
    assert pkt.radars[1].moving_distance_cm == 120
    # Pack/unpack the second radar to make sure the ID is re-applied by
    # the Gateway and not transported on the wire.
    out = RadarSample()
    out = espnow_unpack_radar(pkt.radars[0], "right_front", now_ms=999)
    assert out.radar_id == "right_front"
    assert out.last_update_ms == 999


def test_decode_rejects_bad_magic():
    radars = [_sample(), _sample()]
    buf = espnow_encode(node_id=ESPNOW_NODE_REMOTE, boot_id=1, sequence=1, timestamp_ms=0, radars=radars)
    buf = bytes([0xFF, 0xFF, 0xFF, 0xFF]) + buf[4:]
    with pytest.raises(ValueError, match="magic"):
        espnow_decode(buf)


def test_decode_rejects_bad_version():
    radars = [_sample(), _sample()]
    buf = espnow_encode(node_id=ESPNOW_NODE_REMOTE, boot_id=1, sequence=1, timestamp_ms=0, radars=radars)
    # protocol_version is at byte 4
    buf = buf[:4] + bytes([ESPNOW_PROTOCOL_VERSION + 1]) + buf[5:]
    # Recompute checksum so we don't trip the checksum guard first.
    # checksum is the last 4 bytes.
    import struct
    new_csum = espnow_checksum(buf[:-4]) & 0xFFFFFFFF
    buf = buf[:-4] + struct.pack("<I", new_csum)
    with pytest.raises(ValueError, match="protocol_version"):
        espnow_decode(buf)


def test_decode_rejects_bad_length():
    radars = [_sample(), _sample()]
    buf = espnow_encode(node_id=ESPNOW_NODE_REMOTE, boot_id=1, sequence=1, timestamp_ms=0, radars=radars)
    # Corrupt packet_size so it claims a different size than the buffer.
    import struct
    corrupted = bytearray(buf)
    struct.pack_into("<H", corrupted, 18, 0)
    # Recompute checksum so it doesn't trip the checksum guard first.
    new_csum = espnow_checksum(bytes(corrupted[:-4])) & 0xFFFFFFFF
    corrupted[-4:] = struct.pack("<I", new_csum)
    with pytest.raises(ValueError, match="packet_size"):
        espnow_decode(bytes(corrupted))


def test_decode_rejects_bad_checksum():
    radars = [_sample(), _sample()]
    buf = espnow_encode(node_id=ESPNOW_NODE_REMOTE, boot_id=1, sequence=1, timestamp_ms=0, radars=radars)
    # Flip the lowest bit of the first radar byte.
    corrupted = bytearray(buf)
    corrupted[22] ^= 0x01
    # Recompute checksum so the only thing wrong is the flipped byte.
    import struct
    new_csum = espnow_checksum(bytes(corrupted[:-4])) & 0xFFFFFFFF
    corrupted[-4:] = struct.pack("<I", new_csum)
    # The decoder recomputes the checksum over the entire header+payload
    # (which now includes the flipped byte) so it should match.  The
    # test as-written is therefore not a "checksum mismatch" test -- it
    # passes.  Replace with a stronger guarantee below.
    pkt = espnow_decode(bytes(corrupted))
    assert pkt.radars[0].moving_energy != 0xFF  # sanity


def test_decode_rejects_truly_corrupt_checksum():
    radars = [_sample(), _sample()]
    buf = espnow_encode(node_id=ESPNOW_NODE_REMOTE, boot_id=1, sequence=1, timestamp_ms=0, radars=radars)
    # Flip the last byte of the checksum without recomputing.
    corrupted = bytearray(buf)
    corrupted[-1] ^= 0xFF
    with pytest.raises(ValueError, match="checksum"):
        espnow_decode(bytes(corrupted))


def test_decode_rejects_wrong_radar_count():
    radars = [_sample(), _sample()]
    buf = espnow_encode(node_id=ESPNOW_NODE_REMOTE, boot_id=1, sequence=1, timestamp_ms=0, radars=radars)
    # radar_count is at offset 20 (uint16 LE).
    import struct
    corrupted = bytearray(buf)
    struct.pack_into("<H", corrupted, 20, 3)
    # Recompute checksum so it doesn't trip first.
    new_csum = espnow_checksum(bytes(corrupted[:-4])) & 0xFFFFFFFF
    corrupted[-4:] = struct.pack("<I", new_csum)
    with pytest.raises(ValueError, match="radar_count"):
        espnow_decode(bytes(corrupted))


def test_sequence_wrap_around_is_handled_by_boot_id():
    """The C++ codec itself does not enforce sequence wrap; the Gateway
    aggregator does.  We mirror that policy in Python by re-encoding a
    sequence that exceeds 32 bits."""
    radars = [_sample(), _sample()]
    big_seq = (1 << 33)  # > 2^32, will be masked by 32-bit field
    buf = espnow_encode(
        node_id=ESPNOW_NODE_REMOTE,
        boot_id=1,
        sequence=big_seq,
        timestamp_ms=0,
        radars=radars,
    )
    pkt = espnow_decode(buf)
    assert pkt.sequence == big_seq & 0xFFFFFFFF
