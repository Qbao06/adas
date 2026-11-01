"""Compatibility entrypoint — prefer ``python run.py``.

Both ``python main.py`` and ``python run.py`` should work. The newer
``run.py`` provides a richer CLI (mode flags, --video, --lidar, etc.) so
we forward to it from here.
"""
from __future__ import annotations

import sys
from pathlib import Path


def main() -> int:
    root = Path(__file__).resolve().parent
    runner = root / "run.py"
    if runner.exists():
        sys.argv[0] = str(runner)
        try:
            runpy = __import__("runpy")
            runpy.run_path(str(runner), run_name="__main__")
            return 0
        except SystemExit as exc:
            return int(exc.code or 0)
    print("Không tìm thấy run.py — hãy dùng 'python run.py'.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())



    