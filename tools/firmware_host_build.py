"""Compile-check the pure C++ modules of the ESP32 firmware.

The firmware is split so the parser, ESP-NOW codec, JSON transport,
gateway aggregator, diagnostics and mock mode contain ZERO Arduino
dependencies.  This script compiles them with plain g++ on a host so
we catch syntax / type / wire-format regressions without needing the
full Arduino toolchain.  It also links a small host_main.cpp smoke
test that exercises the whole pure pipeline end-to-end.

Run from the repo root:

    python tools/firmware_host_build.py

Exits 0 on success, non-zero with diagnostics on failure.
"""
from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = ROOT / "firmware" / "esp32_radar_gateway" / "src"

PURE_MODULES = [
    "ld2410_parser.cpp",
    "espnow_protocol.cpp",
    "jetson_json_transport.cpp",
    "gateway_aggregator.cpp",
    "diagnostics.cpp",
    "mock_mode.cpp",
]

HOST_MAIN = "host_main.cpp"


def main() -> int:
    if shutil.which("g++") is None:
        print(
            "[FAIL] g++ not found on PATH.  Install build-essential "
            "(apt: sudo apt install build-essential) before running "
            "tools/firmware_host_build.py.",
            file=sys.stderr,
        )
        return 2

    if not SRC_DIR.is_dir():
        print(f"[ERR] firmware src dir not found: {SRC_DIR}", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory(prefix="firmware_host_") as tmp:
        tmp_path = Path(tmp)
        objs = []
        for module in PURE_MODULES + [HOST_MAIN]:
            obj = tmp_path / (Path(module).stem + ".o")
            cmd = [
                "g++",
                "-std=c++14",
                "-Wall",
                "-Wextra",
                "-Wno-unused-parameter",
                "-I",
                str(SRC_DIR),
                "-c",
                str(SRC_DIR / module),
                "-o",
                str(obj),
            ]
            print("[CC]", module)
            rc = subprocess.run(cmd).returncode
            if rc != 0:
                print(f"[FAIL] g++ exited with {rc} while compiling {module}", file=sys.stderr)
                return rc
            objs.append(str(obj))

        exe = tmp_path / "firmware_host_check"
        link_cmd = ["g++", "-std=c++14", *objs, "-o", str(exe)]
        print("[LD]", " ".join(link_cmd))
        rc = subprocess.run(link_cmd).returncode
        if rc != 0:
            print(f"[FAIL] linker exited with {rc}", file=sys.stderr)
            return rc

        print("[RUN]", str(exe))
        rc = subprocess.run([str(exe)]).returncode
        if rc != 0:
            print(f"[FAIL] host smoke test exited with {rc}", file=sys.stderr)
            return rc
        print(f"[OK] compiled + linked {len(PURE_MODULES)} modules, smoke test passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())