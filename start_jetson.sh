#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PY="$PROJECT_DIR/.venv/bin/python"
CONFIG_PATH="${ADAS_CONFIG:-$PROJECT_DIR/config/jetson_live.yaml}"

export PYTHONPATH=""
unset ROS_DISTRO ROS_VERSION ROS_ROOT ROS_PACKAGE_PATH ROS_LOCATIONS
cd "$PROJECT_DIR"

if [ ! -x "$VENV_PY" ]; then
    echo "[start_jetson] Chưa cài môi trường. Chạy ./setup_jetson.sh một lần." >&2
    exit 1
fi

set +e
"$VENV_PY" "$PROJECT_DIR/preflight.py" --config "$CONFIG_PATH"
PREFLIGHT_STATUS=$?
set -e
if [ "$PREFLIGHT_STATUS" -ne 0 ]; then
    DASHBOARD_PORT="${ADAS_DASHBOARD_PORT:-8765}"
    echo "[start_jetson] Phần cứng chưa sẵn sàng. Mở dashboard tại http://127.0.0.1:$DASHBOARD_PORT"
    "$VENV_PY" "$PROJECT_DIR/web_dashboard.py" \
        --host 0.0.0.0 --port "$DASHBOARD_PORT" \
        --status logs/preflight_status.json --config "$CONFIG_PATH" &
    DASHBOARD_PID=$!
    if [ -n "${DISPLAY:-}" ] && command -v xdg-open >/dev/null 2>&1; then
        sleep 0.6
        xdg-open "http://127.0.0.1:$DASHBOARD_PORT" >/dev/null 2>&1 || true
    fi
    wait "$DASHBOARD_PID"
    exit "$PREFLIGHT_STATUS"
fi

DASHBOARD_PORT="${ADAS_DASHBOARD_PORT:-8765}"
"$VENV_PY" "$PROJECT_DIR/web_dashboard.py" \
    --host 127.0.0.1 --port "$DASHBOARD_PORT" \
    --status logs/preflight_status.json --config "$CONFIG_PATH" &
DASHBOARD_PID=$!
trap 'kill "$DASHBOARD_PID" >/dev/null 2>&1 || true' EXIT INT TERM
if [ -n "${DISPLAY:-}" ] && command -v xdg-open >/dev/null 2>&1; then
    sleep 0.6
    xdg-open "http://127.0.0.1:$DASHBOARD_PORT" >/dev/null 2>&1 || true
fi
set +e
"$VENV_PY" "$PROJECT_DIR/run.py" --config "$CONFIG_PATH" --mode live --device cuda --headless "$@"
PIPELINE_STATUS=$?
set -e
exit "$PIPELINE_STATUS"
