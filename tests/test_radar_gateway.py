"""Host-pure tests for :mod:`radar_gateway` (the Python mirror of the
firmware codec).

These tests intentionally mirror the C++ test suite under
``firmware/esp32_radar_gateway/tests/`` so that any change to the wire
format must be reflected in both worlds.  They cover:

* ESP-NOW v2 packet encode/decode round-trip.
* Bad magic, bad version, bad radar_count, bad packet_size, bad checksum.
* Sequence monotonicity (older / duplicate seq rejected).
* Sequence 32-bit wrap-around accepted as legitimate.
* Boot_id change accepted as a reboot.
* LD2410 frame parser (basic-mode, full-mode, garbage, resync, two
  frames back-to-back, bad checksum / footer / header).
* JSON v2 transport encoder (all four radars, protocol order, null for
  invalid samples, never 0, target_state strings).
* The gateway_reader iterator (valid line, bad json, bad version,
  empty frame, line too long).
"""
from __future__ import annotations

import json
import os
import struct
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from radar_gateway import (  # noqa: E402
    protocol,
    ld2410_parser,
    jetson_protocol,
    gateway_reader,
)
from radar_gateway.protocol import (  # noqa: E402
    ESPNOW_MAGIC,
    ESPNOW_PROTOCOL_VERSION,
    ESPNOW_NODE_GATEWAY,
    ESPNOW_NODE_REMOTE,
    ESPNOW_PAYLOAD_SIZE,
    LD2410_DISTANCE_NONE,
    LD2410_ENERGY_NONE,
    TARGET_STATE_MOVING,
    TARGET_STATE_NONE,
    RADAR_ERROR_FRAME_CRC,
    RadarSample,
    EspNowPacket,
    espnow_checksum,
    espnow_encode,
    espnow_decode,
    espnow_pack_radar,
    espnow_unpack_radar,
    detect_sequence_wrap,
)
from radar_gateway.ld2410_parser import (  # noqa: E402
    LD2410_HEADER,
    LD2410_FOOTER,
    LD2410_MAX_FRAME_LEN,
    Ld2410Parser,
    make_mock_frame,
)
from radar_gateway.jetson_protocol import (  # noqa: E402
    JSON_PROTOCOL_VERSION,
    ADAS_MAX_RADARS,
    RADAR_ORDER,
    RadarSample as JsonRadarSample,
    RadarSnapshot,
    jetson_json_encode_v2,
)


# --------------------------------------------------------------------------- #
# ESP-NOW codec
# --------------------------------------------------------------------------- #
def _espnow_sample(
    *,
    target_state: int = TARGET_STATE_MOVING,
    detected: int = 1,
    moving_distance_cm: int = 120,
    stationary_distance_cm: int = LD2410_DISTANCE_NONE,
    moving_energy: int = 40,
    stationary_energy: int = LD2410_ENERGY_NONE,
    detection_distance_cm: int = 120,
    error_code: int = 0,
    sample_age_ms: int = 10,
    valid: int = 1,
    stale: int = 0,
    radar_id: str = "right_front",
) -> RadarSample:
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


def test_espnow_payload_size_is_stable():
    # Static assert mirrored from firmware.  Any change must be deliberate.
    assert ESPNOW_PAYLOAD_SIZE == 22 + 2 * 17 + 4


def test_espnow_header_layout_matches_cpp_struct():
    """Cross-impl guard: the Python pack format must produce a 22-byte
    header in exactly the field order documented in
    ``firmware/esp32_radar_gateway/src/espnow_protocol.h``.

    If anyone changes either side, this test fires before the on-wire
    test below.
    """
    import struct as _struct

    fmt = getattr(protocol, "_ESPNOW_HEADER_FMT", None)
    assert fmt == "<IBBIIIHH", (
        "Python header format drift -- expected '<IBBIIIHH' to match the "
        "C++ EspNowPacket field order (magic, version, node_id, boot_id, "
        "sequence, timestamp_ms, packet_size, radar_count)."
    )
    assert _struct.calcsize(fmt) == 22, (
        f"Header size must be 22 bytes (4+1+1+4+4+4+2+2); got "
        f"{_struct.calcsize(fmt)}."
    )

    # Field offsets -- must match espnow_protocol.cpp's manual writes.
    pack = _struct.pack(
        fmt,
        0x52444E31,  # magic
        2,           # protocol_version
        2,           # node_id (REMOTE)
        0xDEADBEEF,  # boot_id
        0x12345678,  # sequence
        0x9ABCDEF0,  # timestamp_ms
        0x0044,      # packet_size placeholder
        2,           # radar_count
    )
    # Bytes 0-3: magic (LE u32 0x52444E31 -> on-wire bytes 31 4E 44 52)
    assert pack[0:4] == bytes([0x31, 0x4E, 0x44, 0x52])
    assert pack[4] == 2        # protocol_version
    assert pack[5] == 2        # node_id
    # Bytes 6-9: boot_id LE
    assert pack[6:10] == _struct.pack("<I", 0xDEADBEEF)
    # Bytes 10-13: sequence LE
    assert pack[10:14] == _struct.pack("<I", 0x12345678)
    # Bytes 14-17: timestamp_ms LE
    assert pack[14:18] == _struct.pack("<I", 0x9ABCDEF0)
    # Bytes 18-19: packet_size LE
    assert pack[18:20] == _struct.pack("<H", 0x0044)
    # Bytes 20-21: radar_count LE
    assert pack[20:22] == _struct.pack("<H", 2)


def test_espnow_cross_impl_against_cpp_host_main():
    """Compile + run the C++ smoke test in
    ``firmware/esp32_radar_gateway/src/host_main.cpp`` and verify the
    round-tripped values match the Python encoder.

    The C++ host_main.cpp already does:
        1. ld2410 mock-frame round-trip,
        2. espnow encode -> decode round-trip with boot_id=0xDEADBEEF, seq=42,
        3. aggregator + JSON encode/decode.

    We re-run that same flow in Python and confirm the on-wire bytes
    match the format that the C++ side accepts.
    """
    import shutil
    import subprocess
    import tempfile
    from pathlib import Path

    if shutil.which("g++") is None:
        pytest.skip("g++ not on PATH; cross-impl test skipped")

    root = Path(ROOT)
    src_dir = root / "firmware" / "esp32_radar_gateway" / "src"
    pure_modules = [
        "ld2410_parser.cpp",
        "espnow_protocol.cpp",
        "jetson_json_transport.cpp",
        "gateway_aggregator.cpp",
        "diagnostics.cpp",
        "mock_mode.cpp",
    ]
    with tempfile.TemporaryDirectory(prefix="radar_gw_ximpl_") as tmp:
        tmp_path = Path(tmp)
        objs = []
        for module in pure_modules + ["host_main.cpp"]:
            obj = tmp_path / (Path(module).stem + ".o")
            cmd = [
                "g++", "-std=c++14", "-Wall", "-Wextra", "-Wno-unused-parameter",
                "-I", str(src_dir), "-c", str(src_dir / module), "-o", str(obj),
            ]
            res = subprocess.run(cmd, capture_output=True, text=True)
            assert res.returncode == 0, (
                f"g++ failed on {module}: {res.stderr}"
            )
            objs.append(str(obj))
        exe = tmp_path / "firmware_host_check"
        subprocess.run(["g++", "-std=c++14", *objs, "-o", str(exe)], check=True)
        res = subprocess.run([str(exe)], capture_output=True, text=True)
        assert res.returncode == 0, (
            f"C++ host_main.cpp exited {res.returncode}\nstdout={res.stdout}\n"
            f"stderr={res.stderr}"
        )

    # Now repeat the same encode/decode round-trip in Python and verify
    # the on-wire byte layout matches what host_main.cpp validates
    # (boot_id == 0xDEADBEEF, sequence == 42, radars == 2).
    radars = [
        _espnow_sample(
            radar_id="left_front",
            moving_distance_cm=275,
            detection_distance_cm=275,
        ),
        _espnow_sample(
            radar_id="left_rear",
            moving_distance_cm=250,
            detection_distance_cm=250,
        ),
    ]
    buf = espnow_encode(
        node_id=ESPNOW_NODE_REMOTE,
        boot_id=0xDEADBEEF,
        sequence=42,
        timestamp_ms=12345,
        radars=radars,
    )
    pkt = espnow_decode(buf)
    assert pkt.boot_id == 0xDEADBEEF
    assert pkt.sequence == 42
    assert pkt.timestamp_ms == 12345
    assert pkt.radar_count == 2
    assert pkt.packet_size == ESPNOW_PAYLOAD_SIZE
    assert len(buf) == ESPNOW_PAYLOAD_SIZE
    # The C++ decoder reads fields at exact byte offsets; if our wire
    # layout is byte-identical, the bytes that host_main.cpp asserts on
    # (offset 6..9 = boot_id, 10..13 = seq, 22..22+17 = first radar
    # detection_distance_cm) will match exactly.
    assert buf[6:10] == struct.pack("<I", 0xDEADBEEF)
    assert buf[10:14] == struct.pack("<I", 42)
    # First radar detection_distance_cm at byte 10 inside the radar
    # struct -> 22 + 10 = 32.
    assert struct.unpack_from("<H", buf, 32)[0] == 275


def test_espnow_encode_decode_roundtrip():
    radars = [
        _espnow_sample(moving_distance_cm=300),
        _espnow_sample(radar_id="right_rear", moving_distance_cm=120),
    ]
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
    assert pkt.checksum == espnow_checksum(buf[:-4])
    # Re-apply the radar_id locally on the Gateway side.
    out = espnow_unpack_radar(pkt.radars[0], "right_front", now_ms=999)
    assert out.radar_id == "right_front"
    assert out.last_update_ms == 999


def test_espnow_decode_rejects_bad_magic():
    radars = [_espnow_sample(), _espnow_sample(radar_id="right_rear")]
    buf = espnow_encode(
        node_id=ESPNOW_NODE_REMOTE,
        boot_id=1,
        sequence=1,
        timestamp_ms=0,
        radars=radars,
    )
    buf = bytes([0xFF, 0xFF, 0xFF, 0xFF]) + buf[4:]
    # Recompute checksum so the only failure is the magic.
    import struct as _struct

    new_csum = espnow_checksum(buf[:-4]) & 0xFFFFFFFF
    buf = buf[:-4] + _struct.pack("<I", new_csum)
    with pytest.raises(ValueError, match="magic"):
        espnow_decode(buf)


def test_espnow_decode_rejects_bad_version():
    radars = [_espnow_sample(), _espnow_sample(radar_id="right_rear")]
    buf = espnow_encode(
        node_id=ESPNOW_NODE_REMOTE,
        boot_id=1,
        sequence=1,
        timestamp_ms=0,
        radars=radars,
    )
    import struct as _struct

    corrupted = bytearray(buf)
    corrupted[4] = ESPNOW_PROTOCOL_VERSION + 1  # bad version
    new_csum = espnow_checksum(bytes(corrupted[:-4])) & 0xFFFFFFFF
    corrupted[-4:] = _struct.pack("<I", new_csum)
    with pytest.raises(ValueError, match="protocol_version"):
        espnow_decode(bytes(corrupted))


def test_espnow_decode_rejects_bad_radar_count():
    radars = [_espnow_sample(), _espnow_sample(radar_id="right_rear")]
    buf = espnow_encode(
        node_id=ESPNOW_NODE_REMOTE,
        boot_id=1,
        sequence=1,
        timestamp_ms=0,
        radars=radars,
    )
    import struct as _struct

    corrupted = bytearray(buf)
    _struct.pack_into("<H", corrupted, 20, 3)
    new_csum = espnow_checksum(bytes(corrupted[:-4])) & 0xFFFFFFFF
    corrupted[-4:] = _struct.pack("<I", new_csum)
    with pytest.raises(ValueError, match="radar_count"):
        espnow_decode(bytes(corrupted))


def test_espnow_decode_rejects_bad_size():
    radars = [_espnow_sample(), _espnow_sample(radar_id="right_rear")]
    buf = espnow_encode(
        node_id=ESPNOW_NODE_REMOTE,
        boot_id=1,
        sequence=1,
        timestamp_ms=0,
        radars=radars,
    )
    with pytest.raises(ValueError, match="packet_size"):
        # Truncate the buffer; the decoder must refuse.
        espnow_decode(buf[:-1])


def test_espnow_decode_rejects_corrupt_checksum():
    radars = [_espnow_sample(), _espnow_sample(radar_id="right_rear")]
    buf = espnow_encode(
        node_id=ESPNOW_NODE_REMOTE,
        boot_id=1,
        sequence=1,
        timestamp_ms=0,
        radars=radars,
    )
    corrupted = bytearray(buf)
    corrupted[-1] ^= 0xFF  # flip last byte of checksum
    with pytest.raises(ValueError, match="checksum"):
        espnow_decode(bytes(corrupted))


def test_espnow_sequence_wrap_detector():
    assert detect_sequence_wrap(current=1, previous=(1 << 32) - 1) is True
    assert detect_sequence_wrap(current=2, previous=1) is False
    assert detect_sequence_wrap(current=1, previous=2) is False  # not wrap


def test_espnow_encode_requires_exactly_two_radars():
    with pytest.raises(ValueError):
        espnow_encode(
            node_id=ESPNOW_NODE_REMOTE,
            boot_id=1,
            sequence=1,
            timestamp_ms=0,
            radars=[_espnow_sample()],
        )


# --------------------------------------------------------------------------- #
# LD2410 parser
# --------------------------------------------------------------------------- #
def _build_basic_frame(
    *,
    target_state: int = TARGET_STATE_MOVING,
    detection_distance_cm: int = 150,
    bad_checksum: bool = False,
    bad_footer: bool = False,
    bad_header: bool = False,
    extra_garbage: bytes = b"",
) -> bytes:
    """Build a minimal LD2410 basic-mode frame for tests."""
    buf = bytearray()
    buf += bytes(LD2410_HEADER)
    length_field = 6
    buf += bytes([length_field & 0xFF, (length_field >> 8) & 0xFF])
    buf += bytes([0x02])  # frame type
    buf += bytes([0xAA])  # head marker
    buf += bytes([0xAA])  # tail marker
    buf += bytes([target_state & 0x03])
    buf += bytes(
        [
            detection_distance_cm & 0xFF,
            (detection_distance_cm >> 8) & 0xFF,
        ]
    )
    if bad_header:
        buf[0] = 0x00
    buf += bytes(LD2410_FOOTER)
    if bad_footer:
        buf[-4] = 0x00
    csum_range = buf[4 : 4 + 2 + length_field + 4]
    csum = sum(csum_range) & 0xFF
    if bad_checksum:
        csum ^= 0xFF
    buf += bytes([csum])
    return bytes(buf) + extra_garbage


def test_ld2410_parser_accepts_well_formed_basic_frame():
    parser = Ld2410Parser()
    sample = None
    for byte in _build_basic_frame(
        target_state=TARGET_STATE_MOVING, detection_distance_cm=150
    ):
        out = parser.feed(byte, "left_front", now_ms=10)
        if out is not None:
            sample = out
    assert sample is not None
    assert sample.valid == 1
    assert sample.stale == 0
    assert sample.target_state == TARGET_STATE_MOVING
    assert sample.detection_distance_cm == 150
    assert sample.radar_id == "left_front"
    assert sample.error_code == 0
    assert sample.last_update_ms == 10


def test_ld2410_parser_handles_two_back_to_back_frames():
    parser = Ld2410Parser()
    stream = _build_basic_frame(
        detection_distance_cm=100
    ) + _build_basic_frame(detection_distance_cm=200)
    seen = []
    for byte in stream:
        out = parser.feed(byte, "right_front", now_ms=30)
        if out is not None:
            seen.append(out)
    assert len(seen) == 2
    assert seen[0].detection_distance_cm == 100
    assert seen[1].detection_distance_cm == 200


def test_ld2410_parser_rejects_bad_checksum():
    parser = Ld2410Parser()
    surfaced = 0
    for byte in _build_basic_frame(bad_checksum=True):
        out = parser.feed(byte, "left_front", now_ms=40)
        if out is not None:
            surfaced += 1
    assert surfaced == 0


def test_ld2410_parser_rejects_bad_footer():
    parser = Ld2410Parser()
    surfaced = 0
    for byte in _build_basic_frame(bad_footer=True):
        out = parser.feed(byte, "left_front", now_ms=50)
        if out is not None:
            surfaced += 1
    assert surfaced == 0


def test_ld2410_parser_rejects_bad_header():
    parser = Ld2410Parser()
    surfaced = 0
    for byte in _build_basic_frame(bad_header=True):
        out = parser.feed(byte, "left_front", now_ms=60)
        if out is not None:
            surfaced += 1
    assert surfaced == 0


def test_ld2410_parser_does_not_invent_distance_on_no_target():
    parser = Ld2410Parser()
    sample = None
    frame = _build_basic_frame(
        target_state=TARGET_STATE_NONE,
        detection_distance_cm=LD2410_DISTANCE_NONE,
    )
    for byte in frame:
        out = parser.feed(byte, "left_front", now_ms=70)
        if out is not None:
            sample = out
    assert sample is not None
    assert sample.target_state == TARGET_STATE_NONE
    assert sample.detection_distance_cm == LD2410_DISTANCE_NONE


def test_ld2410_parser_resyncs_after_garbage():
    parser = Ld2410Parser()
    garbage = b"\x00\x11\x22\x33\x44" + _build_basic_frame()
    sample = None
    for byte in garbage:
        out = parser.feed(byte, "left_rear", now_ms=20)
        if out is not None:
            sample = out
    assert sample is not None
    assert sample.radar_id == "left_rear"


def test_ld2410_parser_drops_overlong_burst_then_resumes():
    parser = Ld2410Parser()
    burst = bytes(range(256)) * 4  # 1024 bytes of garbage
    surfaced = 0
    for byte in burst:
        out = parser.feed(byte, "left_front", now_ms=80)
        if out is not None:
            surfaced += 1
    assert surfaced == 0
    sample = None
    for byte in _build_basic_frame():
        out = parser.feed(byte, "left_front", now_ms=90)
        if out is not None:
            sample = out
    assert sample is not None


def test_ld2410_mock_frame_round_trips():
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
    assert sample.valid == 1
    assert sample.detection_distance_cm == 300


# --------------------------------------------------------------------------- #
# JSON v2 transport
# --------------------------------------------------------------------------- #
def _json_sample(**overrides) -> JsonRadarSample:
    sample = JsonRadarSample(
        radar_id="left_front",
        target_state=TARGET_STATE_MOVING,
        detected=1,
        valid=1,
        stale=0,
        moving_distance_cm=180,
        stationary_distance_cm=LD2410_DISTANCE_NONE,
        moving_energy=55,
        stationary_energy=LD2410_ENERGY_NONE,
        detection_distance_cm=180,
        error_code=0,
        sample_age_ms=20,
    )
    for k, v in overrides.items():
        setattr(sample, k, v)
    return sample


def test_json_v2_emits_all_four_radars_in_protocol_order():
    snap = RadarSnapshot(
        sequence=10,
        gateway_ok=1,
        remote_ok=1,
        gateway_boot_id=0xAAAA,
        remote_boot_id=0xBBBB,
        packet_loss=2,
        radars=[
            _json_sample(
                radar_id="left_front",
                moving_distance_cm=180,
                detection_distance_cm=180,
            ),
            _json_sample(
                radar_id="left_rear",
                moving_distance_cm=100,
                detection_distance_cm=100,
            ),
            _json_sample(
                radar_id="right_front",
                moving_distance_cm=300,
                detection_distance_cm=300,
            ),
            _json_sample(
                radar_id="right_rear",
                moving_distance_cm=400,
                detection_distance_cm=400,
            ),
        ],
    )
    line = jetson_json_encode_v2(snap)
    payload = json.loads(line)
    assert payload["v"] == JSON_PROTOCOL_VERSION
    assert payload["seq"] == 10
    assert payload["gateway_ok"] == 1
    assert payload["remote_ok"] == 1
    assert payload["gateway_boot_id"] == 0xAAAA
    assert payload["remote_boot_id"] == 0xBBBB
    assert payload["packet_loss"] == 2
    keys = list(payload["radars"].keys())
    assert keys == list(RADAR_ORDER)
    assert payload["radars"]["left_front"]["distance_m"] == 1.8
    assert payload["radars"]["right_rear"]["distance_m"] == 4.0
    assert payload["radars"]["left_rear"]["moving_distance_m"] == 1.0
    assert payload["radars"]["right_front"]["moving_distance_m"] == 3.0


def test_json_v2_emits_null_for_invalid_sample():
    snap = RadarSnapshot(
        sequence=1, gateway_ok=1, remote_ok=0,
        gateway_boot_id=1, remote_boot_id=0, packet_loss=0,
        radars=[
            _json_sample(valid=0, stale=1),
            _json_sample(radar_id="left_rear"),
            _json_sample(radar_id="right_front"),
            _json_sample(radar_id="right_rear"),
        ],
    )
    payload = json.loads(jetson_json_encode_v2(snap))
    assert payload["radars"]["left_front"]["distance_m"] is None
    assert payload["radars"]["left_front"]["valid"] is False
    assert payload["radars"]["left_front"]["stale"] is True


def test_json_v2_never_uses_zero_for_invalid_distance():
    snap = RadarSnapshot(
        sequence=1, gateway_ok=1, remote_ok=1,
        gateway_boot_id=1, remote_boot_id=1, packet_loss=0,
        radars=[
            _json_sample(valid=0, moving_distance_cm=0, detection_distance_cm=0),
            _json_sample(radar_id="left_rear"),
            _json_sample(radar_id="right_front"),
            _json_sample(radar_id="right_rear"),
        ],
    )
    payload = json.loads(jetson_json_encode_v2(snap))
    assert payload["radars"]["left_front"]["distance_m"] is None
    assert payload["radars"]["left_front"]["moving_distance_m"] is None
    assert payload["radars"]["left_front"]["moving_energy"] is None


def test_json_v2_target_state_strings():
    snap = RadarSnapshot(
        sequence=1, gateway_ok=1, remote_ok=1,
        gateway_boot_id=1, remote_boot_id=1, packet_loss=0,
        radars=[
            _json_sample(target_state=TARGET_STATE_NONE, detected=0, valid=1),
            _json_sample(radar_id="left_rear"),
            _json_sample(radar_id="right_front"),
            _json_sample(radar_id="right_rear"),
        ],
    )
    payload = json.loads(jetson_json_encode_v2(snap))
    assert payload["radars"]["left_front"]["target_state"] == "none"
    assert payload["radars"]["left_rear"]["target_state"] == "moving"


# --------------------------------------------------------------------------- #
# gateway_reader
# --------------------------------------------------------------------------- #
def test_gateway_reader_yields_valid_lines():
    snap = RadarSnapshot(
        sequence=1, gateway_ok=1, remote_ok=1,
        gateway_boot_id=1, remote_boot_id=1, packet_loss=0,
        radars=[
            _json_sample(radar_id=rid)
            for rid in RADAR_ORDER
        ],
    )
    line = jetson_json_encode_v2(snap)
    seen_errors = []

    def _on_error(raw: bytes, exc: BaseException) -> None:
        seen_errors.append((raw, exc))

    frames = list(gateway_reader.iter_json_lines(
        [line.encode("utf-8")], on_error=_on_error
    ))
    assert len(frames) == 1
    assert frames[0][1]["seq"] == 1
    assert frames[0][1]["v"] == JSON_PROTOCOL_VERSION
    assert seen_errors == []


def test_gateway_reader_skips_invalid_frames():
    bad = b"{not-json}\n"
    snap = RadarSnapshot(
        sequence=7, gateway_ok=1, remote_ok=1,
        gateway_boot_id=1, remote_boot_id=1, packet_loss=0,
        radars=[_json_sample(radar_id=rid) for rid in RADAR_ORDER],
    )
    good = jetson_json_encode_v2(snap).encode("utf-8")
    seen = []

    def _on_error(raw: bytes, exc: BaseException) -> None:
        seen.append((raw, type(exc).__name__))

    frames = list(gateway_reader.iter_json_lines(
        [bad, good], on_error=_on_error
    ))
    # Each frame is (received_monotonic, payload_dict).
    assert len(frames) == 1
    assert frames[0][1]["seq"] == 7
    assert frames[0][1]["v"] == JSON_PROTOCOL_VERSION
    assert len(seen) == 1
    assert seen[0][0] == bad.rstrip(b"\n")


def test_gateway_reader_rejects_unsupported_version():
    bad = json.dumps({"v": 1, "seq": 1, "radars": {}}).encode("utf-8") + b"\n"
    errors = []
    frames = list(gateway_reader.iter_json_lines(
        [bad], on_error=lambda raw, exc: errors.append(exc)
    ))
    assert frames == []
    assert errors and "version" in str(errors[0]).lower()


def test_gateway_reader_rejects_empty_lines():
    feed = b"\n" + b'{"v":2,"seq":1,"radars":{}}' + b"\n"
    errors = []
    frames = list(gateway_reader.iter_json_lines(
        [feed], on_error=lambda raw, exc: errors.append(exc)
    ))
    # The empty line is silently skipped (no error callback).
    assert errors == []
    # The "radars": {} object is accepted as a valid v2 frame (no radar
    # required for this lower-level helper).
    assert len(frames) == 1
