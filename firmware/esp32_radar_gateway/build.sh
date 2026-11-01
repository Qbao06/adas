#!/usr/bin/env bash
# Build the ESP32 Radar Gateway firmware for one or both roles.
#
# Usage:
#   ./build.sh gateway      # build ROLE_GATEWAY (ESP32 A)
#   ./build.sh remote       # build ROLE_REMOTE  (ESP32 B)
#   ./build.sh all          # build both
#   ./build.sh host         # host-only compile check (no Arduino toolchain)
#
# Requires arduino-cli with the esp32:esp32 core installed.  See README
# step 1.  Override FQBN or PORT via the environment if needed.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/src"
INC_FLAG="-I${SRC}"
FQBN="${FQBN:-esp32:esp32:esp32}"

if ! command -v arduino-cli >/dev/null 2>&1; then
    echo "[ERR] arduino-cli not on PATH.  Install it from https://arduino.github.io/arduino-cli/" >&2
    exit 2
fi

build_role() {
    local role="$1" upper
    upper="$(echo "$role" | tr '[:lower:]' '[:upper:]')"
    echo "==> Building ROLE_${upper}"
    arduino-cli compile \
        --fqbn "$FQBN" \
        --build-property "build.extra_flags=-DROLE_${upper} ${INC_FLAG}" \
        --warnings none \
        "$HERE"
}

case "${1:-all}" in
    gateway) build_role gateway ;;
    remote)  build_role remote ;;
    all)
             python3 "$HERE/../tools/firmware_preflight.py" || exit 1
             build_role gateway
             echo
             build_role remote ;;
    host)
        python3 "$HERE/../tools/firmware_preflight.py" || true
        python3 "$HERE/../tools/firmware_host_build.py"
        ;;
    *)
        echo "usage: $0 {gateway|remote|all|host}" >&2
        exit 1
        ;;
esac