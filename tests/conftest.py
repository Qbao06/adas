"""Make `pytest -q` work even when the system has ROS/ament pytest plugins.

The system ROS (Robot Operating System) installation injects a number of
pytest plugins (ament_lint, launch_testing_ros, ...) that depend on
lark / pyyaml / ... versions we do not control. On hosts that have ROS
installed under /opt/ros, those plugins get auto-discovered and crash
our pytest session.

This conftest strips /opt/ros paths from ``sys.path`` before any test
imports run. Users can still call ``pytest -q`` directly.
"""
from __future__ import annotations

import sys

# Strip ROS /opt/ros paths from sys.path before pytest plugin discovery.
sys.path[:] = [p for p in sys.path if "/opt/ros" not in p]

# Make sure the project root is importable regardless of cwd.
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
    