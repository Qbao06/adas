"""Export an Ultralytics checkpoint to a Jetson-local TensorRT engine."""
from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/best.pt")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--workspace", type=float, default=2.0)
    args = parser.parse_args()
    model_path = Path(args.model).resolve()
    if not model_path.exists():
        raise SystemExit(f"Không có model: {model_path}")
    import torch
    if not torch.cuda.is_available():
        raise SystemExit("CUDA không hoạt động; chỉ export engine trên Jetson đã cấu hình đúng")
    from ultralytics import YOLO
    model = YOLO(str(model_path))
    print(f"Classes: {model.names}")
    result = model.export(
        format="engine",
        device=0,
        half=True,
        imgsz=args.imgsz,
        workspace=args.workspace,
        simplify=False,
    )
    print(f"TensorRT engine: {result}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
