"""Cross-platform launcher for the ADAS pipeline.

Resolves paths via the project root — independent of the user's current
working directory. Supports ``--mode`` (demo, live, playback, ...), feeding
in custom video / LiDAR files, recording output, headless runs and
benchmarking.

This file is intentionally framework-free (no click / typer) so it works on
a clean install.
"""
from __future__ import annotations

import argparse
import io
import logging
import sys
from pathlib import Path
from typing import Iterable, Optional


# Windows consoles (cp1252) crash on Vietnamese argparse text: force UTF-8.
if sys.platform == "win32":  # pragma: no cover - host specific
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")


PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from config import ConfigError, coerce_config, load_config, validate_config  # noqa: E402
from system import write_runtime_manifest  # noqa: E402
from system.logging import configure_logging  # noqa: E402


def _parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="adas",
        description="ADAS LiDAR-Camera pipeline (không dùng ROS).",
    )
    parser.add_argument("--config", default="config/adas.yaml")
    parser.add_argument(
        "--mode",
        choices=["demo", "live", "playback", "camera-only", "lidar-only", "fusion-test", "benchmark"],
        help="live: toan bo ADAS; camera-only: camera that, khong AI/LiDAR.",
    )
    parser.add_argument("--port", type=int, default=8765, help="Cổng dashboard camera-only.")
    parser.add_argument("--video", help="Đường dẫn video đầu vào (camera.source).")
    parser.add_argument("--lidar", help="Đường dẫn file LiDAR playback.")
    parser.add_argument("--headless", action="store_true", help="Không mở cửa sổ OpenCV.")
    parser.add_argument("--max-frames", type=int, default=None, help="Giới hạn số frame xử lý.")
    parser.add_argument("--record", action="store_true", help="Ghi video kết quả ra output/result.mp4.")
    parser.add_argument(
        "--record-path",
        help="Đường dẫn file video output (mặc định: output/result.mp4).",
    )
    parser.add_argument(
        "--save-frame",
        action="store_true",
        help="Lưu frame đầu tiên của demo ra output/demo_first_frame.png.",
    )
    parser.add_argument(
        "--device",
        choices=["auto", "cpu", "cuda"],
        help="Thiết bị chạy YOLO (mặc định: auto).",
    )
    parser.add_argument(
        "--model",
        help="Đường dẫn model YOLO (vd: yolov8n.pt hoặc model đã fine-tune).",
    )
    parser.add_argument(
        "--camera-source",
        help="Chỉ định nguồn camera (vd: 0, /dev/video0, video.mp4).",
    )
    parser.add_argument(
        "--lidar-driver",
        help="Tên driver LiDAR đã đăng ký (vd: mock, playback, livox).",
    )
    parser.add_argument(
        "--log-level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default="INFO",
    )
    return parser.parse_args(list(argv) if argv is not None else None)


def _build_overrides(args: argparse.Namespace) -> dict:
    overrides: dict = {}
    if args.mode:
        overrides.setdefault("mode", args.mode)
    if args.video:
        camera = overrides.setdefault("camera", {})
        camera["source"] = args.video
        camera["video_path"] = args.video
    if args.camera_source:
        camera = overrides.setdefault("camera", {})
        camera["source"] = args.camera_source
    if args.lidar:
        lidar = overrides.setdefault("lidar", {})
        lidar["playback_path"] = args.lidar
        lidar["source"] = "playback"
        lidar["driver"] = "playback"
        if overrides.get("mode") in (None, "demo"):
            overrides["mode"] = "playback"
    if args.lidar_driver:
        lidar = overrides.setdefault("lidar", {})
        lidar["driver"] = args.lidar_driver
        lidar["source"] = args.lidar_driver
    if args.device:
        vision = overrides.setdefault("vision", {})
        vision["device"] = args.device
    if args.model:
        vision = overrides.setdefault("vision", {})
        vision["model"] = args.model
    if args.record_path or args.record:
        outputs = overrides.setdefault("outputs", {})
        visualization = overrides.setdefault("visualization", {})
        visualization["record_video"] = True
        path = args.record_path or "output/result.mp4"
        outputs["video_path"] = path
        visualization["record_path"] = path
    if args.log_level:
        outputs = overrides.setdefault("outputs", {})
        outputs["log_level"] = args.log_level
    return overrides


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = _parse_args(argv)
    if args.mode == "camera-only":
        from dashboard.camera_live import run_camera_dashboard
        return run_camera_dashboard(PROJECT_ROOT, port=args.port, source=args.camera_source)
    from app import ADASPipeline
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = PROJECT_ROOT / config_path
    overrides = _build_overrides(args)
    config = load_config(config_path, overrides=overrides)
    coerce_config(config)
    try:
        validate_config(config)
    except ConfigError as exc:
        print(f"Cấu hình không hợp lệ: {exc}", file=sys.stderr)
        return 2
    logger = configure_logging(
        PROJECT_ROOT / config.get("outputs", {}).get("log_directory", "logs"),
        level=config.get("outputs", {}).get("log_level", args.log_level),
        console=True,
    )
    logger.info("Khởi động ADAS — mode=%s", config.get("mode"))
    manifest = write_runtime_manifest(PROJECT_ROOT, config)
    logger.info("Runtime manifest: %s", manifest)

    pipeline = ADASPipeline(config, root=PROJECT_ROOT, cli_overrides=overrides)
    try:
        frames = pipeline.run(
            max_frames=args.max_frames,
            display=not args.headless,
            save_first_frame=args.save_frame,
        )
    except KeyboardInterrupt:
        logger.info("Người dùng đã hủy (Ctrl+C)")
        return 0
    except Exception as exc:
        logger.exception("Pipeline lỗi không mong muốn: %s", exc)
        return 1
    logger.info(
        "Hoàn thành — frames=%d fps=%.1f avg_latency=%.1fms",
        frames,
        pipeline.stats().fps,
        pipeline.stats().avg_latency_ms,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
