#!/usr/bin/env bash
# Single command launcher for the ADAS pipeline on Linux / Raspberry Pi.
# Detects the project root, sets up .venv if needed, installs dependencies,
# then forwards all arguments to ``python run.py``.
#
# IMPORTANT: PYTHONPATH is explicitly cleared so that system ROS installations
# (which set PYTHONPATH=/opt/ros/...) cannot pollute our isolated venv.

set -e

# -------------------------------------------------------------------------- #
# 1. Environment isolation — prevent ROS / system Python from leaking in
# -------------------------------------------------------------------------- #
export PYTHONPATH=""
unset ROS_DISTRO
unset ROS_VERSION
unset ROS_ROOT
unset ROS_PACKAGE_PATH
unset ROS_LOCATIONS
# Keep LD_LIBRARY_PATH: Jetson uses it for CUDA/TensorRT runtime libraries.

# -------------------------------------------------------------------------- #
# 2. Determine project root
# -------------------------------------------------------------------------- #
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_ROOT"

# -------------------------------------------------------------------------- #
# 3. Find Python
# -------------------------------------------------------------------------- #
PYTHON_BIN="${PYTHON:-python3}"
if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    PYTHON_BIN="python"
fi
if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    echo "[run.sh] Loi: Khong tim thay Python 3.10+. Cai Python va thu lai." >&2
    exit 1
fi

# -------------------------------------------------------------------------- #
# 4. Virtual environment
# -------------------------------------------------------------------------- #
VENV_DIR="$PROJECT_ROOT/.venv"
VENV_PY="$VENV_DIR/bin/python"

if [ ! -d "$VENV_DIR" ] || [ ! -x "$VENV_PY" ]; then
    echo "[run.sh] Tao moi truong ao tai $VENV_DIR ..."
    rm -rf "$VENV_DIR"
    "$PYTHON_BIN" -m venv --system-site-packages "$VENV_DIR"
fi

# Activate (paths only in bash; does NOT inherit PYTHONPATH)
# shellcheck disable=SC1091
if [ -f "$VENV_DIR/bin/activate" ]; then
    source "$VENV_DIR/bin/activate"
fi

# -------------------------------------------------------------------------- #
# 5. Dependencies must be installed once by setup.sh
# -------------------------------------------------------------------------- #
if ! "$VENV_PY" -c "import numpy, yaml" >/dev/null 2>&1; then
    echo "[run.sh] Thieu dependency. Chay './setup.sh' mot lan truoc." >&2
    exit 1
fi

# -------------------------------------------------------------------------- #
# 6. Ensure runtime directories exist
# -------------------------------------------------------------------------- #
mkdir -p "$PROJECT_ROOT/logs" "$PROJECT_ROOT/output" "$PROJECT_ROOT/data" "$PROJECT_ROOT/models"

# -------------------------------------------------------------------------- #
# 7. Launch camera-only without the full ADAS preflight
# -------------------------------------------------------------------------- #
for arg in "$@"; do
    if [[ "$arg" == "camera-only" || "$arg" == "--mode=camera-only" ]]; then
        exec "$VENV_PY" "$PROJECT_ROOT/run.py" "$@"
    fi
done

# 7. Production entrypoint: preflight + live only
# -------------------------------------------------------------------------- #
echo "[run.sh] Khoi dong ADAS LIVE (khong cho phep demo/mock) ..."
"$PROJECT_ROOT/start_jetson.sh" "$@"
STATUS=$?

if [ $STATUS -ne 0 ]; then
    echo "[run.sh] Pipeline ket thuc voi ma loi $STATUS" >&2
fi

exit $STATUS
