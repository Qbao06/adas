"""Thin CLI entrypoint for ADAS hardware-readiness & cockpit dashboard."""
from __future__ import annotations

import sys
from dashboard.server import main

if __name__ == "__main__":
    raise SystemExit(main())
