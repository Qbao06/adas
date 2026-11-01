"""Kiểm tra thư viện runtime trên Jetson trước khi chạy live.

Không thay preflight.py: file này chỉ kiểm tra Python/CUDA/OpenCV/Ultralytics.
Phần cứng (camera, UART, calibration) do preflight.py đảm nhiệm.
"""
from __future__ import annotations

import argparse
import importlib.metadata
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IS_JETSON = Path("/etc/nv_tegra_release").exists()

REQUIRED_IMPORTS = (
    ("numpy", "numpy"),
    ("yaml", "PyYAML"),
    ("scipy", "scipy"),
    ("sklearn", "scikit-learn"),
    ("PIL", "pillow"),
    ("requests", "requests"),
    ("matplotlib", "matplotlib"),
    ("pandas", "pandas"),
    ("tqdm", "tqdm"),
    ("seaborn", "seaborn"),
    ("psutil", "psutil"),
    ("cpuinfo", "py-cpuinfo"),
    ("thop", "ultralytics-thop"),
    ("ultralytics", "ultralytics"),
)

POISON_DISTS = (
    "opencv-python",
    "opencv-python-headless",
    "opencv-contrib-python",
    "onnxruntime",
    "onnxruntime-gpu",
)


def _dist_names() -> set[str]:
    names: set[str] = set()
    for dist in importlib.metadata.distributions():
        try:
            name = dist.metadata["Name"]
        except Exception:
            continue
        if name:
            names.add(str(name).lower().replace("_", "-"))
    return names


def check(strict_jetson: bool) -> list[str]:
    errors: list[str] = []
    if sys.version_info < (3, 10):
        errors.append(f"Cần Python >= 3.10 (đang {sys.version.split()[0]})")

    for module, package in REQUIRED_IMPORTS:
        try:
            __import__(module)
        except Exception as exc:
            errors.append(f"Thiếu {package} (import {module}: {exc})")

    try:
        import cv2  # type: ignore

        cv_path = Path(getattr(cv2, "__file__", "") or "").resolve()
        print(f"[OK] cv2 {cv2.__version__}  path={cv_path}")
        if ".venv" in cv_path.parts and "site-packages" in cv_path.parts:
            errors.append(
                "cv2 đang lấy từ pip trong .venv. Gỡ opencv-python rồi dùng "
                "sudo apt install python3-opencv"
            )
    except Exception as exc:
        errors.append(f"Thiếu OpenCV (cv2): {exc}. Cài: sudo apt install python3-opencv")

    dists = _dist_names()
    for poison in POISON_DISTS:
        if poison in dists:
            errors.append(
                f"Gói độc trên Jetson: {poison}. Gỡ: pip uninstall -y {poison}"
            )

    try:
        import torch  # type: ignore

        print(f"[OK] torch {torch.__version__}")
        if not torch.cuda.is_available():
            errors.append(
                "torch.cuda.is_available()=False. Cài wheel PyTorch NVIDIA đúng JetPack, "
                "không pip install torch từ PyPI"
            )
        else:
            print(f"[OK] CUDA {torch.version.cuda}  GPU={torch.cuda.get_device_name(0)}")
    except Exception as exc:
        errors.append(f"Không import được torch: {exc}")

    try:
        import torchvision  # type: ignore

        print(f"[OK] torchvision {torchvision.__version__}")
    except Exception as exc:
        msg = (
            f"Không import được torchvision: {exc}. "
            "Cài wheel NVIDIA cùng phiên bản với torch."
        )
        if strict_jetson:
            errors.append(msg)
        else:
            print(f"[WARN] {msg}")

    try:
        import tensorrt  # type: ignore

        print(f"[OK] tensorrt {getattr(tensorrt, '__version__', '?')}")
    except Exception as exc:
        msg = (
            f"Không import được tensorrt: {exc}. "
            "Cài python3-libnvinfer từ JetPack (không pip tensorrt)."
        )
        if strict_jetson:
            errors.append(msg)
        else:
            print(f"[WARN] {msg}")

    if strict_jetson and not IS_JETSON:
        errors.append("Không thấy /etc/nv_tegra_release — máy này không phải Jetson Linux")

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description="Kiểm tra thư viện ADAS trên Jetson")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Bắt buộc Jetson + torchvision + TensorRT (dùng trong setup_jetson.sh)",
    )
    args = parser.parse_args()
    print(f"[INFO] python={sys.version.split()[0]}  jetson={IS_JETSON}")
    errors = check(strict_jetson=args.strict)
    for item in errors:
        print(f"[FAIL] {item}", file=sys.stderr)
    if errors:
        print(f"[RESULT] Thiếu {len(errors)} mục thư viện", file=sys.stderr)
        return 1
    print("[RESULT] Thư viện runtime đủ cho live")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
