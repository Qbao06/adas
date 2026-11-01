"""Tests for ADAS Hardware Readiness & Full Cockpit Web Dashboard."""
from __future__ import annotations

import json
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch

import pytest

from dashboard.server import create_dashboard_server, DashboardServer

ROOT = Path(__file__).resolve().parents[1]


def _get_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def live_server(tmp_path):
    port = _get_free_port()
    status_file = tmp_path / "status.json"
    status_file.write_text(
        json.dumps({
            "ready": False,
            "demo": True,  # intentionally poisoned to test sanitization
            "checked_at": datetime.now(timezone.utc).isoformat(),
            "errors": ["Không đọc được camera source=0", "Chưa cấu hình driver LiDAR thật"],
        }),
        encoding="utf-8",
    )
    cfg_file = ROOT / "config" / "jetson_live.yaml"
    server = create_dashboard_server(
        host="127.0.0.1",
        port=port,
        status_path=status_file,
        config_path=cfg_file,
        demo=False,
    )
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    time.sleep(0.1)
    yield server, port, status_file
    server.shutdown()
    server.server_close()


@pytest.fixture
def demo_server(tmp_path):
    port = _get_free_port()
    status_file = tmp_path / "demo_status.json"
    cfg_file = ROOT / "config" / "jetson_live.yaml"
    server = create_dashboard_server(
        host="127.0.0.1",
        port=port,
        status_path=status_file,
        config_path=cfg_file,
        demo=True,
    )
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    time.sleep(0.1)
    yield server, port, status_file
    server.shutdown()
    server.server_close()


# -------------------------------------------------------------------------- #
# 1. Healthz & Root endpoints
# -------------------------------------------------------------------------- #

def test_healthz_endpoint(live_server):
    _, port, _ = live_server
    url = f"http://127.0.0.1:{port}/healthz"
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req) as resp:
        assert resp.status == 200
        assert "no-store" in resp.headers.get("Cache-Control", "")
        assert "application/json" in resp.headers.get("Content-Type", "")
        data = json.loads(resp.read().decode("utf-8"))
        assert data["status"] == "ok"
        assert data["demo"] is False


def test_index_serves_cockpit_html(live_server):
    _, port, _ = live_server
    url = f"http://127.0.0.1:{port}/"
    with urllib.request.urlopen(url) as resp:
        assert resp.status == 200
        assert "text/html" in resp.headers.get("Content-Type", "")
        assert "no-store" in resp.headers.get("Cache-Control", "")
        content = resp.read().decode("utf-8")
        assert "ADAS PRO" in content
        assert "COCKPIT" in content
        assert 'id="primaryCanvas"' in content
        assert 'id="bevCanvas"' in content
        assert 'id="distanceBanner"' in content
        assert 'id="readinessGrid"' in content
        assert 'id="recheckBtn"' in content


# -------------------------------------------------------------------------- #
# 2. Static assets: MIME types & no-store
# -------------------------------------------------------------------------- #

def test_static_assets_mime_and_cache(live_server):
    _, port, _ = live_server
    # CSS
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/static/styles.css") as resp:
        assert resp.status == 200
        assert "text/css" in resp.headers.get("Content-Type", "")
        assert "no-store" in resp.headers.get("Cache-Control", "")
        assert ":root" in resp.read().decode("utf-8")

    # JS
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/static/app.js") as resp:
        assert resp.status == 200
        assert "application/javascript" in resp.headers.get("Content-Type", "")
        assert "no-store" in resp.headers.get("Cache-Control", "")
        assert "primaryCanvas" in resp.read().decode("utf-8")

    # Missing static asset returns 404
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(f"http://127.0.0.1:{port}/static/non_existent.js")
    assert exc.value.code == 404


# -------------------------------------------------------------------------- #
# 3. Live vs Demo status isolation
# -------------------------------------------------------------------------- #

def test_live_status_never_claims_demo(live_server):
    _, port, _ = live_server
    url = f"http://127.0.0.1:{port}/api/status"
    with urllib.request.urlopen(url) as resp:
        assert resp.status == 200
        data = json.loads(resp.read().decode("utf-8"))
        assert data["demo"] is False
        assert data["ready"] is False
        assert len(data["errors"]) == 2


def test_demo_status_always_claims_demo(demo_server):
    _, port, _ = demo_server
    url = f"http://127.0.0.1:{port}/api/status"
    with urllib.request.urlopen(url) as resp:
        assert resp.status == 200
        data = json.loads(resp.read().decode("utf-8"))
        assert data["demo"] is True
        assert data["ready"] is True
        assert "snapshot" in data
        assert "warning" in data["snapshot"]
        assert "detections" in data["snapshot"]


def test_recheck_forbidden_in_demo(demo_server):
    _, port, _ = demo_server
    url = f"http://127.0.0.1:{port}/api/recheck"
    req = urllib.request.Request(url, data=b"{}", headers={"Content-Type": "application/json"})
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(req)
    assert exc.value.code == 403


# -------------------------------------------------------------------------- #
# 4. Recheck Concurrency & Execution
# -------------------------------------------------------------------------- #

def test_recheck_prevents_concurrent_runs(live_server):
    server, port, _ = live_server
    url = f"http://127.0.0.1:{port}/api/recheck"

    assert server.recheck_lock.acquire(blocking=False)
    try:
        req = urllib.request.Request(url, data=b"{}", headers={"Content-Type": "application/json"})
        with pytest.raises(urllib.error.HTTPError) as exc:
            urllib.request.urlopen(req)
        assert exc.value.code == 409
        body = json.loads(exc.value.read().decode("utf-8"))
        assert body["busy"] is True
    finally:
        server.recheck_lock.release()


def test_recheck_executes_and_returns_result(live_server):
    _, port, _ = live_server
    url = f"http://127.0.0.1:{port}/api/recheck"
    req = urllib.request.Request(url, data=b"{}", headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req) as resp:
        assert resp.status == 200
        body = json.loads(resp.read().decode("utf-8"))
        assert "ok" in body


def test_recheck_handles_timeout_gracefully(live_server):
    _, port, _ = live_server
    url = f"http://127.0.0.1:{port}/api/recheck"

    with patch("dashboard.server.subprocess.run", side_effect=subprocess.TimeoutExpired(cmd="preflight", timeout=60)):
        req = urllib.request.Request(url, data=b"{}", headers={"Content-Type": "application/json"})
        with pytest.raises(urllib.error.HTTPError) as exc:
            urllib.request.urlopen(req)
        assert exc.value.code == 504
        body = json.loads(exc.value.read().decode("utf-8"))
        assert body["ok"] is False
        assert "timeout" in body["error"].lower()


# -------------------------------------------------------------------------- #
# 5. UI Policy: No Mock button in Live, Demo Banner in Demo, XSS Safety
# -------------------------------------------------------------------------- #

def test_no_mock_controls_in_html():
    html = (ROOT / "dashboard" / "static" / "index.html").read_text(encoding="utf-8")
    assert 'id="demoBanner"' in html
    assert "DEMO UI — NOT LIVE" in html
    assert "chuyển sang mock" not in html.lower()
    assert "chạy mock" not in html.lower()
    assert "allow_mock" not in html.lower()


def test_xss_protection_in_app_js():
    js = (ROOT / "dashboard" / "static" / "app.js").read_text(encoding="utf-8")
    assert "textEl.textContent = err;" in js


# -------------------------------------------------------------------------- #
# 6. Demo Actions: In-memory simulation and 404 in Live
# -------------------------------------------------------------------------- #

def test_demo_action_endpoint_returns_404_in_live(live_server):
    _, port, _ = live_server
    url = f"http://127.0.0.1:{port}/api/demo/action"
    req = urllib.request.Request(
        url,
        data=json.dumps({"action": "normal"}).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(req)
    assert exc.value.code == 404


def test_demo_actions_execute_in_memory_without_hardware(demo_server):
    _, port, _ = demo_server
    url = f"http://127.0.0.1:{port}/api/demo/action"

    actions_to_test = [
        ("pedestrian", lambda snap: snap["scenario"] == "pedestrian"),
        ("danger", lambda snap: snap["scenario"] == "danger"),
        ("lost_cam", lambda snap: snap["health"]["camera"] == "error"),
        ("lost_lidar", lambda snap: snap["health"]["lidar"] == "error"),
        ("lost_ai", lambda snap: snap["health"]["ai"] == "error"),
        ("reverse", lambda snap: snap["vehicle"]["gear"] == "R" and snap["active_camera"] == "rear"),
        ("normal", lambda snap: snap["scenario"] == "normal" and snap["health"]["camera"] == "ok"),
        ("reset", lambda snap: snap["scenario"] == "normal"),
    ]

    for action, validator in actions_to_test:
        req = urllib.request.Request(
            url,
            data=json.dumps({"action": action}).encode("utf-8"),
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req) as resp:
            assert resp.status == 200
            data = json.loads(resp.read().decode("utf-8"))
            assert data["ok"] is True
            assert data["action"] == action
            assert validator(data["snapshot"])


def test_demo_action_rejects_disallowed_actions(demo_server):
    _, port, _ = demo_server
    url = f"http://127.0.0.1:{port}/api/demo/action"
    req = urllib.request.Request(
        url,
        data=json.dumps({"action": "malicious_command_or_hack"}).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(req)
    assert exc.value.code == 400
