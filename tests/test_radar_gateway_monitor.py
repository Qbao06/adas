"""Unit tests for ``tools/radar_gateway_monitor.py``.

These tests cover:
* v2 protocol parser (valid, missing fields, bad version, negative / NaN
  distance, malformed JSON);
* CLI mode dispatch (``--list-faults``, ``--fault``, ``--soak``, ``--raw``);
* fault scenarios (each scenario must mark the right radars stale and must
  not fabricate detections).

The tests deliberately do NOT claim the system is vehicle-safe.  They
verify the parser + state machine honour the safety contract.
"""
from __future__ import annotations

import csv
import json
import os
import random
import subprocess
import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.radar_gateway_monitor import (  # noqa: E402
    FaultState,
    SCENARIOS,
    Stats,
    parse_gateway_line,
    _base_packet,
    _distance_or_none,
)


def _good_packet(seq: int = 1) -> dict:
    return {
        "v": 2,
        "seq": seq,
        "gateway_ok": True,
        "remote_ok": True,
        "gateway_boot_id": 1,
        "remote_boot_id": 1,
        "packet_loss": 0.0,
        "radars": {
            "left_front": {
                "detected": False,
                "valid": True,
                "stale": False,
                "target_state": 0,
                "distance_m": None,
                "moving_distance_m": None,
                "moving_energy": 0,
                "stationary_distance_m": None,
                "stationary_energy": 0,
                "age_ms": 50,
                "error": None,
            },
            "left_rear": {
                "detected": False,
                "valid": True,
                "stale": False,
                "target_state": 0,
                "distance_m": None,
                "moving_distance_m": None,
                "moving_energy": 0,
                "stationary_distance_m": None,
                "stationary_energy": 0,
                "age_ms": 50,
                "error": None,
            },
            "right_front": {
                "detected": False,
                "valid": True,
                "stale": False,
                "target_state": 0,
                "distance_m": None,
                "moving_distance_m": None,
                "moving_energy": 0,
                "stationary_distance_m": None,
                "stationary_energy": 0,
                "age_ms": 50,
                "error": None,
            },
            "right_rear": {
                "detected": False,
                "valid": True,
                "stale": False,
                "target_state": 0,
                "distance_m": None,
                "moving_distance_m": None,
                "moving_energy": 0,
                "stationary_distance_m": None,
                "stationary_energy": 0,
                "age_ms": 50,
                "error": None,
            },
        },
    }


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------


def test_parser_accepts_valid_v2_packet():
    sample = parse_gateway_line(json.dumps(_good_packet(seq=7)))
    assert sample.sequence == 7
    assert sample.gateway_ok is True
    assert sample.remote_ok is True
    assert set(sample.radars) == {"left_front", "left_rear", "right_front", "right_rear"}


def test_parser_rejects_v1():
    with pytest.raises(ValueError, match="unsupported"):
        parse_gateway_line(json.dumps({"v": 1, "seq": 1, "radars": {}}))


def test_parser_rejects_malformed_json():
    with pytest.raises(ValueError):
        parse_gateway_line(b"{not json}\n")


def test_parser_records_errors_for_missing_radars():
    payload = _good_packet()
    payload["radars"] = {"left_front": payload["radars"]["left_front"]}
    sample = parse_gateway_line(json.dumps(payload))
    assert "missing radar left_rear" in sample.errors
    assert "missing radar right_front" in sample.errors
    assert "missing radar right_rear" in sample.errors


def test_parser_records_negative_distance_without_crashing():
    payload = _good_packet()
    payload["radars"]["left_front"]["distance_m"] = -1.0
    sample = parse_gateway_line(json.dumps(payload))
    assert any("outside 0..6.0m" in err for err in sample.errors)


def test_parser_records_nan_distance():
    payload = json.loads(json.dumps(_good_packet()).replace('"distance_m": null', '"distance_m": NaN'))
    sample = parse_gateway_line(json.dumps(payload))
    assert any("outside 0..6.0m" in err for err in sample.errors)


def test_parser_rejects_seq_below_zero():
    payload = _good_packet(seq=-1)
    with pytest.raises(ValueError, match="seq"):
        parse_gateway_line(json.dumps(payload))


def test_distance_helper_accepts_none_and_rejects_negative():
    assert _distance_or_none(None, "x", 5.0) is None
    with pytest.raises(ValueError):
        _distance_or_none(-0.1, "x", 5.0)


# ---------------------------------------------------------------------------
# Stats
# ---------------------------------------------------------------------------


def test_stats_packet_rate_window():
    stats = Stats(started_at=0.0)
    # 10 samples at 50ms intervals = 0.45s span -> 10 / 0.45 ~ 22.2 Hz.
    for i in range(10):
        stats.note_line(0.0 + i * 0.05)
    assert stats.packet_rate(0.5) == pytest.approx(22.222, rel=0.05)


def test_stats_packet_rate_single_sample():
    stats = Stats(started_at=0.0)
    stats.note_line(0.0)
    assert stats.packet_rate(1.0) == 0.0  # need at least 2 samples


def test_stats_average_loss():
    stats = Stats(started_at=0.0)
    stats.note_loss(0.1)
    stats.note_loss(0.3)
    assert stats.average_loss() == pytest.approx(0.2)


# ---------------------------------------------------------------------------
# Synthetic packet generator
# ---------------------------------------------------------------------------


def test_synthetic_packet_has_four_radars_and_sequence():
    state = FaultState(rng=random.Random(0), started_at=0.0)
    pkt = _base_packet(state)
    assert pkt["v"] == 2
    assert pkt["seq"] == 1
    assert set(pkt["radars"]) == {"left_front", "left_rear", "right_front", "right_rear"}


# ---------------------------------------------------------------------------
# Fault scenarios
# ---------------------------------------------------------------------------


def test_scenario_esp32_b_power_loss_only_affects_remote_radars():
    state = FaultState(rng=random.Random(0), started_at=0.0)
    cfg = SCENARIOS["esp32_b_power_loss"]
    pkt = cfg.schedule(8.0, state)
    assert pkt is not None
    sample = parse_gateway_line(json.dumps(pkt))
    assert sample.remote_ok is False
    assert sample.radars["right_front"]["valid"] is False
    assert sample.radars["right_front"]["stale"] is True
    assert sample.radars["right_rear"]["valid"] is False
    assert sample.radars["right_rear"]["stale"] is True
    assert sample.radars["left_front"]["valid"] is True
    assert sample.radars["left_rear"]["valid"] is True
    # No detection fabrication on either side.
    assert sample.radars["right_front"]["detected"] is False
    assert sample.radars["right_rear"]["detected"] is False


def test_scenario_one_ld2410_uart_loss_marks_only_one_radar_stale():
    state = FaultState(rng=random.Random(0), started_at=0.0)
    cfg = SCENARIOS["one_ld2410_uart_loss"]
    pkt = cfg.schedule(6.0, state)
    assert pkt is not None
    sample = parse_gateway_line(json.dumps(pkt))
    assert sample.radars["right_front"]["valid"] is False
    assert sample.radars["right_front"]["stale"] is True
    assert sample.radars["left_front"]["valid"] is True
    assert sample.radars["left_rear"]["valid"] is True
    assert sample.radars["right_rear"]["valid"] is True


def test_scenario_esp32_a_reset_increments_boot_id():
    state = FaultState(rng=random.Random(0), started_at=0.0)
    cfg = SCENARIOS["esp32_a_reset"]
    # Advance past the reset window.
    pkt_pre = cfg.schedule(4.0, state)
    assert pkt_pre is not None
    pre_sample = parse_gateway_line(json.dumps(pkt_pre))
    # Walk through the disconnect + reboot phases.
    cfg.schedule(5.5, state)
    cfg.schedule(6.4, state)
    pkt_post = cfg.schedule(7.0, state)
    assert pkt_post is not None
    post_sample = parse_gateway_line(json.dumps(pkt_post))
    assert post_sample.gateway_boot_id > pre_sample.gateway_boot_id


def test_scenario_gateway_usb_unplug_emits_nothing_while_offline():
    state = FaultState(rng=random.Random(0), started_at=0.0)
    cfg = SCENARIOS["gateway_usb_unplug_replug"]
    pkt = cfg.schedule(6.0, state)
    assert pkt is None


def test_scenario_corrupt_json_emits_bytes():
    state = FaultState(rng=random.Random(0), started_at=0.0)
    cfg = SCENARIOS["corrupt_json"]
    # The scenario emits a corrupt bytes frame every 7th call and a valid
    # JSON-bytes frame otherwise.  Walk enough iterations to see both.
    seen_json = False
    seen_corrupt = False
    for tick in range(20):
        emitted = cfg.schedule(0.1 + tick * 0.01, state)
        assert emitted is not None
        assert isinstance(emitted, bytes)
        stripped = emitted.rstrip(b"\n")
        try:
            import json as _json
            parsed = _json.loads(stripped)
        except _json.JSONDecodeError:
            seen_corrupt = True
        else:
            assert isinstance(parsed, dict)
            seen_json = True
        if seen_json and seen_corrupt:
            break
    assert seen_json
    assert seen_corrupt


# ---------------------------------------------------------------------------
# CLI integration
# ---------------------------------------------------------------------------


def _run_cli(*args: str) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"
    return subprocess.run(
        [sys.executable, "tools/radar_gateway_monitor.py", *args],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=30,
        env=env,
    )


def test_cli_list_faults():
    proc = _run_cli("--list-faults")
    assert proc.returncode == 0
    assert "esp32_b_power_loss" in proc.stdout


def test_cli_fault_writes_csv_and_report(tmp_path):
    csv_path = tmp_path / "fault.csv"
    json_path = tmp_path / "fault.json"
    proc = _run_cli(
        "--fault",
        "esp32_b_power_loss",
        "--duration",
        "7",
        "--no-color",
        "--csv",
        str(csv_path),
        "--json-report",
        str(json_path),
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert csv_path.exists()
    with csv_path.open() as fp:
        rows = list(csv.DictReader(fp))
    assert rows, "fault CSV should contain at least one row"
    payload = json.loads(json_path.read_text())
    assert payload["scenario"] == "esp32_b_power_loss"
    assert "right_front" in payload["faulty_radars"]
    assert "right_rear" in payload["faulty_radars"]


def test_cli_raw_replay_emits_lines(tmp_path):
    capture = tmp_path / "capture.jsonl"
    state = FaultState(rng=random.Random(1), started_at=0.0)
    capture.write_text(
        "\n".join(json.dumps(_base_packet(state)) for _ in range(20)) + "\n",
        encoding="utf-8",
    )
    proc = _run_cli(
        "--raw",
        str(capture),
        "--no-color",
        "--duration",
        "1",
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "[monitor]" in proc.stdout


def test_cli_soak_writes_csv_and_json(tmp_path):
    capture = tmp_path / "capture.jsonl"
    state = FaultState(rng=random.Random(2), started_at=0.0)
    capture.write_text(
        "\n".join(json.dumps(_base_packet(state)) for _ in range(40)) + "\n",
        encoding="utf-8",
    )
    csv_path = tmp_path / "soak.csv"
    json_path = tmp_path / "soak.json"
    proc = _run_cli(
        "--soak",
        "--raw",
        str(capture),
        "--duration",
        "3",
        "--no-color",
        "--csv",
        str(csv_path),
        "--json-report",
        str(json_path),
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert csv_path.exists()
    assert json_path.exists()
    payload = json.loads(json_path.read_text())
    assert payload["packet_count"] > 0
    assert "samples" in payload


def test_cli_help_exits_zero():
    proc = _run_cli("--help")
    assert proc.returncode == 0
    assert "--fault" in proc.stdout
