"""Aggregator-level ESP-NOW tests (stale, duplicate, wraparound).

These tests exercise the freshness / boot-id / packet-loss policy that
the Gateway firmware applies to the ESP-NOW link.  The codec itself
already has its own tests in ``test_espnow_protocol.py``; this file
focuses on the policy that wraps the codec.
"""
from __future__ import annotations

import os
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from firmware.esp32_radar_gateway.tests.espnow_protocol_py import (  # noqa: E402
    ESPNOW_NODE_REMOTE,
    RadarSample,
    espnow_encode,
)
from firmware.esp32_radar_gateway.tests.gateway_aggregator_py import (  # noqa: E402
    GatewayAggregator,
    RADAR_SLOTS,
    empty_sample,
)


def _packet(*, boot_id: int, sequence: int, timestamp_ms: int = 0,
            right_front_distance: int = 150, right_rear_distance: int = 90):
    radars = [
        RadarSample(
            radar_id="right_front",
            target_state=1,
            detected=1,
            valid=1,
            stale=0,
            moving_distance_cm=right_front_distance,
            detection_distance_cm=right_front_distance,
            sample_age_ms=10,
        ),
        RadarSample(
            radar_id="right_rear",
            target_state=1,
            detected=1,
            valid=1,
            stale=0,
            moving_distance_cm=right_rear_distance,
            detection_distance_cm=right_rear_distance,
            sample_age_ms=10,
        ),
    ]
    return espnow_encode(
        node_id=ESPNOW_NODE_REMOTE,
        boot_id=boot_id,
        sequence=sequence,
        timestamp_ms=timestamp_ms,
        radars=radars,
    )


def _decode(buf):
    from firmware.esp32_radar_gateway.tests.espnow_protocol_py import espnow_decode
    return espnow_decode(buf)


# --------------------------------------------------------------------------- #
# Stale-packet policy.
# --------------------------------------------------------------------------- #


def test_packet_older_than_last_received_is_rejected_and_loss_grows():
    ag = GatewayAggregator(stale_after_ms=600)
    ag.set_local_boot_id(1, now_ms=0)
    ag.set_remote_boot_id(100, now_ms=0)

    accepted1 = ag.feed_remote_packet(_decode(_packet(boot_id=100, sequence=10)), now_ms=100)
    accepted2 = ag.feed_remote_packet(_decode(_packet(boot_id=100, sequence=8)), now_ms=200)
    assert accepted1 is True
    assert accepted2 is False
    # Out-of-order packet pushes loss upward (gap = 10 - 8 - 1 = 1 lost).
    assert ag.remote_loss > 0.0
    assert ag.last_remote_seq == 10  # never moved backwards.


def test_duplicate_sequence_is_rejected_and_does_not_update_sample():
    ag = GatewayAggregator(stale_after_ms=600)
    ag.set_remote_boot_id(1, now_ms=0)

    first = _decode(_packet(boot_id=1, sequence=42, right_front_distance=180))
    first.radars[0].moving_energy = 33
    first.radars[0].moving_distance_cm = 180
    assert ag.feed_remote_packet(first, now_ms=100) is True
    # Re-apply a "duplicate" packet with the same sequence and a clearly
    # different sample value, then check the snapshot did NOT overwrite.
    dup = _decode(_packet(boot_id=1, sequence=42, right_front_distance=180))
    dup.radars[0].moving_energy = 77
    accepted = ag.feed_remote_packet(dup, now_ms=200)
    assert accepted is False
    snap = ag.tick(now_ms=300)
    right_front = snap.radars[2]
    assert right_front.moving_distance_cm == 180
    assert right_front.moving_energy == 33  # original, not 77


def test_stale_sample_is_marked_when_local_feed_stops():
    ag = GatewayAggregator(stale_after_ms=600)
    ag.set_local_boot_id(1, now_ms=0)
    ag.feed_local(0, RadarSample(
        radar_id="left_front",
        target_state=1, detected=1, valid=1, stale=0,
        moving_distance_cm=150, detection_distance_cm=150,
        sample_age_ms=10,
    ), now_ms=1000)
    snap1 = ag.tick(now_ms=1100)
    assert snap1.radars[0].stale == 0
    assert snap1.radars[0].valid == 1
    # 800ms later (past stale_after_ms=600) the sample is marked stale.
    snap2 = ag.tick(now_ms=1900)
    assert snap2.radars[0].stale == 1
    # gateway_ok reflects that none of the local samples are valid+!stale.
    assert snap2.gateway_ok is False


# --------------------------------------------------------------------------- #
# Sequence wraparound policy.
# --------------------------------------------------------------------------- #


def test_sequence_wraparound_is_handled_by_boot_id_change():
    """At 2^32 the on-wire sequence wraps to 0; a wrap is only legal
    if the Remote also changes boot_id (reboot / re-flash).  The
    aggregator MUST use boot_id to detect the wrap, never the
    sequence value alone."""
    ag = GatewayAggregator(stale_after_ms=600)
    ag.set_remote_boot_id(1, now_ms=0)

    near_max = 0xFFFFFFFF - 1
    assert ag.feed_remote_packet(_decode(_packet(boot_id=1, sequence=near_max)), now_ms=100) is True
    # Same boot_id, sequence that wrapped (without a reboot).  Must be
    # treated as a duplicate / out-of-order because 0 < near_max.
    assert ag.feed_remote_packet(_decode(_packet(boot_id=1, sequence=0)), now_ms=200) is False

    # Different boot_id with sequence=0 is a fresh link and IS accepted.
    ag2 = GatewayAggregator(stale_after_ms=600)
    ag2.set_remote_boot_id(2, now_ms=0)
    assert ag2.feed_remote_packet(_decode(_packet(boot_id=2, sequence=0)), now_ms=10) is True
    assert ag2.last_remote_seq == 0


def test_32_bit_sequence_increments_pass_the_codec():
    """Smoke test: feeding seq=0xFFFFFFFF then seq=0xFFFFFFF0 with a
    same boot_id marks the second one as duplicate (loss).  This is
    the case the firmware aggregator will see in real life if the
    Remote forgets to bump boot_id on wrap."""
    ag = GatewayAggregator(stale_after_ms=600)
    ag.set_remote_boot_id(7, now_ms=0)
    assert ag.feed_remote_packet(_decode(_packet(boot_id=7, sequence=0xFFFFFFFF)), now_ms=10) is True
    # Same boot_id, smaller sequence -> rejected.
    assert ag.feed_remote_packet(_decode(_packet(boot_id=7, sequence=0xFFFFFFF0)), now_ms=20) is False
    # The aggregator should never regress.
    assert ag.last_remote_seq == 0xFFFFFFFF


# --------------------------------------------------------------------------- #
# Loss bookkeeping.
# --------------------------------------------------------------------------- #


def test_packet_loss_is_smoothed_exponentially():
    ag = GatewayAggregator(stale_after_ms=600, packet_loss_alpha=0.5)
    ag.set_remote_boot_id(1, now_ms=0)
    for seq in range(1, 11):
        ag.feed_remote_packet(_decode(_packet(boot_id=1, sequence=seq)), now_ms=10 * seq)
    # No packets were lost yet: loss == 0.
    assert ag.remote_loss == 0.0
    # Skip seq=15 -> gap=4.
    ag.feed_remote_packet(_decode(_packet(boot_id=1, sequence=15)), now_ms=200)
    # EMA: loss = 0.5*0 + 0.5*4 = 2.0
    assert ag.remote_loss == pytest.approx(2.0)
    # Skip seq=20 -> gap=4 again.
    ag.feed_remote_packet(_decode(_packet(boot_id=1, sequence=20)), now_ms=300)
    # EMA: loss = 0.5*2.0 + 0.5*4 = 3.0
    assert ag.remote_loss == pytest.approx(3.0)


def test_remote_ok_false_when_no_packets_yet():
    ag = GatewayAggregator(stale_after_ms=600)
    ag.set_local_boot_id(1, now_ms=0)
    snap = ag.tick(now_ms=0)
    assert snap.remote_ok is False
    assert snap.gateway_ok is False


def test_remote_ok_true_after_first_valid_packet():
    ag = GatewayAggregator(stale_after_ms=600)
    ag.set_remote_boot_id(1, now_ms=0)
    ag.feed_remote_packet(_decode(_packet(boot_id=1, sequence=1)), now_ms=10)
    snap = ag.tick(now_ms=20)
    assert snap.remote_ok is True
    assert snap.remote_boot_id == 1


def test_remote_loss_resets_on_boot_id_change():
    ag = GatewayAggregator(stale_after_ms=600, packet_loss_alpha=0.5)
    ag.set_remote_boot_id(1, now_ms=0)
    # Build up loss.
    for seq in range(1, 20):
        ag.feed_remote_packet(_decode(_packet(boot_id=1, sequence=seq)), now_ms=10 * seq)
    ag.feed_remote_packet(_decode(_packet(boot_id=1, sequence=100)), now_ms=1000)
    assert ag.remote_loss > 0
    # Remote reboots with a fresh boot_id.
    ag.set_remote_boot_id(2, now_ms=2000)
    assert ag.remote_loss == 0.0
    assert ag.last_remote_seq == -1


def test_aggregator_handles_local_and_remote_in_one_snapshot():
    ag = GatewayAggregator(stale_after_ms=600)
    ag.set_local_boot_id(11, now_ms=0)
    ag.set_remote_boot_id(22, now_ms=0)
    ag.feed_local(0, RadarSample(
        radar_id="left_front",
        target_state=1, detected=1, valid=1, stale=0,
        moving_distance_cm=80, detection_distance_cm=80,
    ), now_ms=100)
    ag.feed_local(1, RadarSample(
        radar_id="left_rear",
        target_state=0, detected=0, valid=1, stale=0,
        moving_distance_cm=0xFFFF, detection_distance_cm=0xFFFF,
    ), now_ms=100)
    ag.feed_remote_packet(_decode(_packet(boot_id=22, sequence=5, right_front_distance=60)), now_ms=100)
    snap = ag.tick(now_ms=200)
    assert [r.radar_id for r in snap.radars] == list(RADAR_SLOTS)
    assert snap.gateway_ok is True
    assert snap.remote_ok is True
    assert snap.gateway_boot_id == 11
    assert snap.remote_boot_id == 22
    assert snap.radars[0].moving_distance_cm == 80
    assert snap.radars[2].moving_distance_cm == 60
