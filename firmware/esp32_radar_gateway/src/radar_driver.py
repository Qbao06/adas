"""RadarDriver interface, MockRadarDriver and RealRadarDriver stub.

This module is the **Prompt-1 contract** for the firmware side of the
four-LD2410C architecture.  It defines:

* :class:`RadarDriver` -- a pure Python interface every radar driver must
  satisfy.  The interface is intentionally tiny so the parser, the
  UART driver glue and the aggregator can swap drivers without touching
  the rest of the firmware.
* :class:`MockRadarDriver` -- a deterministic generator used by host
  tests and by ``MOCK_MODE=1`` builds on real hardware.
* :class:`RealRadarDriver` -- the production UART reader.  This is a
  **stub** that documents the full interface; the live implementation
  lives in ``ld2410_uart_driver.cpp`` because Arduino's
  ``HardwareSerial`` is only available inside the Arduino framework.

The contract:

    driver = RealRadarDriver(radar_id="left_front", port=Serial1)
    driver.begin(baudrate=256000)
    while True:
        sample = driver.read(now_ms=millis())
        if sample.valid and not sample.stale:
            ...consume...

The C++ side of :class:`RealRadarDriver` is intentionally NOT
implemented here.  Prompt 2 will pick up the production version; this
file exists so the parser, MockRadarDriver and aggregator can be
exercised end-to-end on the host without any hardware.  The host tests
that drive the parser + ESP-NOW + JSON encoders use :class:`MockRadarDriver`
exclusively.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Protocol

from firmware.esp32_radar_gateway.tests.ld2410_parser_py import (
    Ld2410Parser,
    RadarSample,
    TARGET_STATE_MOVING,
    TARGET_STATE_NONE,
    LD2410_DISTANCE_NONE,
    LD2410_ENERGY_NONE,
)


@dataclass(slots=True)
class RadarDriverConfig:
    """Shared configuration consumed by every RadarDriver."""

    radar_id: str
    slot: int = 0
    baudrate: int = 256000
    seed: int = 0


class RadarDriver(Protocol):
    """Minimum surface every radar driver must expose.

    The C++ implementation in ``ld2410_uart_driver.cpp`` provides the same
    contract: ``begin``, ``poll``, ``on_sample`` callback.
    """

    def begin(self, baudrate: int) -> None:
        ...

    def read(self, *, now_ms: int) -> RadarSample:
        ...


# --------------------------------------------------------------------------- #
# MockRadarDriver -- deterministic, used by host tests and MOCK_MODE builds.
# --------------------------------------------------------------------------- #


class MockRadarDriver:
    """Deterministic mock driver.

    Produces a sweep between 30 cm and 500 cm with a sinusoidal phase so
    the host tests have a stable, easily-checked signature.  Two
    instances with different seeds produce two independent sweeps so the
    Gateway and Remote paths can both be exercised.
    """

    def __init__(self, config: RadarDriverConfig) -> None:
        self.config = config
        self._seed = int(config.seed) & 0xFFFFFFFF
        self._started = False

    def begin(self, baudrate: int = 256000) -> None:
        self.config.baudrate = int(baudrate)
        self._started = True

    def read(self, *, now_ms: int) -> RadarSample:
        if not self._started:
            self.begin(self.config.baudrate)
        sample = RadarSample(radar_id=self.config.radar_id)
        sample.last_update_ms = int(now_ms)
        sample.stale = 0
        sample.valid = 1
        sample.error_code = 0
        # Phase shift per slot so four radars don't move in lockstep.
        phase = (self.config.slot * 7000 + (self._seed & 0xFFFF)) & 0xFFFFFFFF
        period = 6000
        pos = (int(now_ms) + phase) % period
        cm = 30 + (pos * 470) // period
        sample.target_state = TARGET_STATE_MOVING
        sample.detected = 1
        sample.moving_distance_cm = cm
        sample.moving_energy = 20 + ((now_ms * 1103515245 + self._seed) % 40)
        sample.detection_distance_cm = cm
        sample.sample_age_ms = 0
        return sample


# --------------------------------------------------------------------------- #
# RealRadarDriver -- the production UART reader, documented as a stub.
# --------------------------------------------------------------------------- #


class RealRadarDriver:
    """Production UART reader for an LD2410C radar.

    This class is a **stub** that documents the host-testable surface.
    The actual implementation on the ESP32 lives in
    ``firmware/esp32_radar_gateway/src/ld2410_uart_driver.cpp`` because
    ``HardwareSerial`` is only available inside the Arduino framework.

    On the host (no Arduino) the stub lets tests construct the object,
    register a callback and feed synthetic bytes through the parser.
    Production builds rely on the C++ side; the host stub's role is to
    keep the Python mirror testable.

    Wire contract:

        * ``port`` -- a file-like object exposing ``read(n)`` and
          ``available()`` (the Arduino ``HardwareSerial`` shape).  When
          ``None`` the driver is a no-op that always returns an empty
          sample (used when the host wants to exercise the aggregator
          without touching any device).
        * The driver never blocks on ``port.read()``; it drains whatever
          is currently available and returns the most recent valid
          sample via ``on_sample`` (Python) / the C++ callback.
    """

    def __init__(
        self,
        config: RadarDriverConfig,
        *,
        port: Optional[object] = None,
        on_sample=None,
    ) -> None:
        self.config = config
        self._port = port
        self._on_sample = on_sample
        self._parser = Ld2410Parser()
        self._last_sample: RadarSample = RadarSample(radar_id=config.radar_id)

    # ----------------------------------------------------------- lifecycle

    def begin(self, baudrate: int = 256000) -> None:
        self.config.baudrate = int(baudrate)
        # Real HardwareSerial.begin(...) lives on the C++ side.  The host
        # stub does nothing here -- callers feed bytes via ``feed_bytes``.

    def feed_bytes(self, data: bytes, *, now_ms: int) -> Optional[RadarSample]:
        """Host-only helper: feed raw bytes as if they arrived from UART.

        Returns the first valid sample parsed out of the byte stream
        (or ``None`` when no complete frame was found yet).
        """
        out: Optional[RadarSample] = None
        for byte in data:
            sample = self._parser.feed(
                byte, self.config.radar_id, int(now_ms)
            )
            if sample is not None:
                sample.radar_id = self.config.radar_id
                self._last_sample = sample
                if self._on_sample is not None:
                    self._on_sample(self.config.slot, sample, None)
                if out is None:
                    out = sample
        return out

    # ------------------------------------------------------- production API

    def read(self, *, now_ms: int) -> RadarSample:
        if self._port is None:
            # No port bound: hand back the last sample unchanged.
            return self._last_sample
        # Real driver drains `port.available()` and feeds the parser.
        # On the host this path is unreachable (we use feed_bytes).
        try:
            available_fn = self._port.available  # type: ignore[attr-defined]
        except AttributeError:
            return self._last_sample
        out: Optional[RadarSample] = None
        while available_fn() > 0:
            byte = self._port.read()  # type: ignore[attr-defined]
            sample = self._parser.feed(
                byte & 0xFF, self.config.radar_id, int(now_ms)
            )
            if sample is not None:
                sample.radar_id = self.config.radar_id
                self._last_sample = sample
                if self._on_sample is not None:
                    self._on_sample(self.config.slot, sample, None)
                out = sample
        if out is None:
            return self._last_sample
        return out


__all__ = [
    "RadarDriver",
    "RadarDriverConfig",
    "MockRadarDriver",
    "RealRadarDriver",
]
