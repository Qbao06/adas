"""Python mirrors of the ESP32 Radar Gateway firmware codec.

The firmware under ``firmware/esp32_radar_gateway/src`` is written in C++
so it can be unit-tested on a host.  To keep the wire format
authoritative we ship matching Python implementations here so the
test suite and any third-party tooling (replay, soak, monitor) can
speak the same protocol.

Modules
-------

* :mod:`radar_gateway.protocol` -- ESP-NOW v2 packet codec used between
  ESP32 A (Gateway) and ESP32 B (Remote).
* :mod:`radar_gateway.ld2410_parser` -- HLK-LD2410C frame parser.
* :mod:`radar_gateway.jetson_protocol` -- JSON v2 transport sent by the
  Gateway to the Jetson over USB Serial.
* :mod:`radar_gateway.gateway_reader` -- POSIX UART reader for the
  Gateway stream, used by the CLI monitor and tests.

All public symbols mirror the firmware names so the C++ and Python
implementations can be diffed one-to-one.
"""
from __future__ import annotations

from . import (
    gateway_reader,
    jetson_protocol,
    ld2410_parser,
    protocol,
)

__all__ = [
    "protocol",
    "ld2410_parser",
    "jetson_protocol",
    "gateway_reader",
]
__version__ = "0.1.0"
