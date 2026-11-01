"""
Python mirror of firmware/esp32_radar_gateway/src/gateway_aggregator.{h,cpp}.

Kept behaviour-for-behaviour compatible with the C++ implementation so
host tests can exercise the same freshness / boot_id / packet-loss
policy that the Gateway firmware runs on the ESP32.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from firmware.esp32_radar_gateway.tests.espnow_protocol_py import (
    ESPNOW_RADARS_PER_PACKET,
    ESPNOW_NODE_GATEWAY,
    ESPNOW_NODE_REMOTE,
    EspNowPacket,
    RadarSample,
    espnow_unpack_radar,
)


# Radar slot identifiers — Gateway owns local slot 0/1, Remote owns
# slot 2/3.  Order matches RADAR_IDS in config/radar_gateway.py.
RADAR_SLOTS = ("left_front", "left_rear", "right_front", "right_rear")


def empty_sample(radar_id: str) -> RadarSample:
    return RadarSample(
        radar_id=radar_id,
        valid=0,
        stale=1,
        sample_age_ms=0xFFFF,
    )


@dataclass
class AggregatedSnapshot:
    """One aggregator tick: a snapshot the Gateway will JSON-encode."""

    radars: List[RadarSample] = field(default_factory=list)
    sequence: int = 0
    gateway_ok: bool = False
    remote_ok: bool = False
    gateway_boot_id: int = 0
    remote_boot_id: int = 0
    packet_loss: float = 0.0
    received_monotonic_ms: int = 0


class GatewayAggregator:
    """Tracks the freshness of four radars + the Remote link.

    The aggregator holds two independent sources of truth:

    * The local Gateway slots (left_front / left_rear), which are fed
      by the firmware's own LD2410C parsers.  These are kept "fresh"
      as long as the host poll keeps feeding samples with monotonically
      increasing ``last_update_ms``.
    * The Remote link (right_front / right_rear), which is fed by the
      ESP-NOW packets arriving from ESP32 B.  The aggregator compares
      the on-wire ``sequence`` against the last received one and
      records the loss as a moving exponential moving average.

    Boot-id handling mirrors the C++ implementation: a new boot_id from
    either side resets the per-link loss counter and clears the stale
    flag.  Duplicates and out-of-order packets are counted but never
    replace the latest snapshot.
    """

    def __init__(
        self,
        *,
        stale_after_ms: int = 600,
        packet_loss_alpha: float = 0.2,
        initial_remote_seq: int = -1,
    ) -> None:
        self.stale_after_ms = int(stale_after_ms)
        self.packet_loss_alpha = float(packet_loss_alpha)
        # Local Gateway samples: keep the latest per-slot.
        self._local: List[Optional[RadarSample]] = [None, None]
        # Remote (ESP-NOW) samples + metadata.
        self._remote: List[Optional[RadarSample]] = [None, None]
        self._last_remote_seq: int = int(initial_remote_seq)
        self._remote_boot_id: int = 0
        self._remote_loss: float = 0.0
        self._remote_received: int = 0
        self._remote_ok: bool = False
        self._local_boot_id: int = 0
        self._gateway_ok: bool = False
        self._last_sequence_emitted: int = -1
        self._tick_counter: int = 0
        self._received_ms: int = 0

    # ----------------------------------------------------------------- API

    def set_local_boot_id(self, boot_id: int, *, now_ms: int) -> None:
        if int(boot_id) != self._local_boot_id:
            # Reset Gateway-local state on a reboot.
            self._local_boot_id = int(boot_id) & 0xFFFFFFFF
            self._gateway_ok = False
            self._local = [None, None]

    def set_remote_boot_id(self, boot_id: int, *, now_ms: int) -> None:
        if int(boot_id) != self._remote_boot_id:
            self._remote_boot_id = int(boot_id) & 0xFFFFFFFF
            self._last_remote_seq = -1
            self._remote_loss = 0.0
            self._remote_ok = False
            self._remote = [None, None]

    def feed_local(self, slot: int, sample: RadarSample, *, now_ms: int) -> None:
        if slot not in (0, 1):
            raise ValueError(f"Gateway aggregator: bad local slot {slot}")
        sample.last_update_ms = int(now_ms)
        self._local[slot] = sample
        self._received_ms = int(now_ms)

    def feed_remote_packet(self, pkt: EspNowPacket, *, now_ms: int) -> bool:
        """Apply an ESP-NOW packet to the Remote slots.

        Returns ``True`` when the packet was accepted (newer sequence
        or new boot_id).  Out-of-order / duplicate / wraparound
        packets are counted via :attr:`remote_loss`.
        """
        if pkt.node_id != ESPNOW_NODE_REMOTE:
            return False
        # Boot-id changes always reset the link state.
        if pkt.boot_id != self._remote_boot_id:
            self._remote_boot_id = pkt.boot_id & 0xFFFFFFFF
            self._last_remote_seq = -1
            self._remote_loss = 0.0
            self._remote = [None, None]
        # Duplicate / out-of-order detection.
        if self._last_remote_seq >= 0 and pkt.sequence <= self._last_remote_seq:
            gap = (self._last_remote_seq + 1) - pkt.sequence
            self._remote_loss = (
                (1.0 - self.packet_loss_alpha) * self._remote_loss
                + self.packet_loss_alpha * max(0, gap)
            )
            return False
        if self._last_remote_seq >= 0:
            gap = pkt.sequence - self._last_remote_seq - 1
            if gap > 0:
                self._remote_loss = (
                    (1.0 - self.packet_loss_alpha) * self._remote_loss
                    + self.packet_loss_alpha * gap
                )
        self._last_remote_seq = pkt.sequence & 0xFFFFFFFF
        self._remote_received += 1
        self._remote_ok = True
        for i, r in enumerate(pkt.radars):
            sample = espnow_unpack_radar(r, RADAR_SLOTS[2 + i], int(now_ms))
            sample.last_update_ms = int(now_ms)
            self._remote[i] = sample
        self._received_ms = int(now_ms)
        return True

    def tick(self, *, now_ms: int) -> AggregatedSnapshot:
        """Build a snapshot to emit over JSON v2."""
        radars: List[RadarSample] = []
        local_ok = False
        remote_ok = False
        # Local slots 0, 1
        for slot in (0, 1):
            sample = self._local[slot]
            if sample is None:
                radars.append(empty_sample(RADAR_SLOTS[slot]))
                continue
            clone = RadarSample(**{**vars(sample)})
            if (int(now_ms) - int(clone.last_update_ms)) > self.stale_after_ms:
                clone.stale = 1
            radars.append(clone)
            if clone.valid and not clone.stale:
                local_ok = True
        # Remote slots 2, 3
        for slot in (0, 1):
            sample = self._remote[slot]
            if sample is None:
                radars.append(empty_sample(RADAR_SLOTS[2 + slot]))
                continue
            clone = RadarSample(**{**vars(sample)})
            if (int(now_ms) - int(clone.last_update_ms)) > self.stale_after_ms:
                clone.stale = 1
            radars.append(clone)
            if clone.valid and not clone.stale:
                remote_ok = True
        self._tick_counter += 1
        self._gateway_ok = local_ok
        return AggregatedSnapshot(
            radars=radars,
            sequence=self._tick_counter,
            gateway_ok=local_ok,
            remote_ok=self._remote_ok and remote_ok,
            gateway_boot_id=self._local_boot_id,
            remote_boot_id=self._remote_boot_id,
            packet_loss=float(self._remote_loss),
            received_monotonic_ms=int(now_ms),
        )

    # ------------------------------------------------------------- counters

    @property
    def remote_loss(self) -> float:
        return float(self._remote_loss)

    @property
    def last_remote_seq(self) -> int:
        return int(self._last_remote_seq)

    @property
    def remote_received(self) -> int:
        return int(self._remote_received)


__all__ = [
    "AggregatedSnapshot",
    "GatewayAggregator",
    "RADAR_SLOTS",
    "empty_sample",
]
