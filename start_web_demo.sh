#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${ADAS_DEMO_DASHBOARD_PORT:-8766}"

if [ -x "$PROJECT_DIR/.venv/bin/python" ]; then
    PYTHON_BIN="$PROJECT_DIR/.venv/bin/python"
elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3)"
else
    echo "Không tìm thấy Python 3. Ubuntu/JetPack thường đã cài sẵn Python 3." >&2
    exit 1
fi

"$PYTHON_BIN" "$PROJECT_DIR/web_dashboard.py" \
    --host 127.0.0.1 --port "$PORT" \
    --status logs/demo_dashboard_status.json --demo &
SERVER_PID=$!
trap 'kill "$SERVER_PID" >/dev/null 2>&1 || true' EXIT INT TERM
sleep 0.6
if [ -n "${DISPLAY:-}" ] && command -v xdg-open >/dev/null 2>&1; then
    xdg-open "http://127.0.0.1:$PORT" >/dev/null 2>&1 || true
fi
echo "ADAS Web Demo: http://127.0.0.1:$PORT"
wait "$SERVER_PID"
