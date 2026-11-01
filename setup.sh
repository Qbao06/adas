#!/usr/bin/env bash
# One-time environment bootstrap for Linux / Raspberry Pi.
# Creates .venv, installs requirements.

set -e

export PYTHONPATH=""
unset ROS_DISTRO
unset ROS_VERSION
unset ROS_ROOT
unset ROS_PACKAGE_PATH
unset ROS_LOCATIONS

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_ROOT"

if [ -f /etc/nv_tegra_release ]; then
    echo "[setup.sh] Đây là Jetson. Không dùng setup.sh (sẽ cài opencv-python pip)." >&2
    echo "Chạy: ./setup_jetson.sh" >&2
    exit 1
fi

PYTHON_BIN="${PYTHON:-python3}"
if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    PYTHON_BIN="python"
fi

VENV_DIR="$PROJECT_ROOT/.venv"
VENV_PY="$VENV_DIR/bin/python"

if [ ! -d "$VENV_DIR" ] || [ ! -x "$VENV_PY" ]; then
    echo "[setup.sh] Tao moi truong ao tai $VENV_DIR"
    rm -rf "$VENV_DIR"
    "$PYTHON_BIN" -m venv --system-site-packages "$VENV_DIR"
fi

# shellcheck disable=SC1091
if [ -f "$VENV_DIR/bin/activate" ]; then
    source "$VENV_DIR/bin/activate"
fi

"$VENV_PY" -m pip install --upgrade pip wheel
"$VENV_PY" -m pip install -r "$PROJECT_ROOT/requirements.txt"

mkdir -p "$PROJECT_ROOT/logs" "$PROJECT_ROOT/output" "$PROJECT_ROOT/data" "$PROJECT_ROOT/models"

echo "[setup.sh] Hoan tat. Chay './run.sh' de khoi dong pipeline mac dinh."
