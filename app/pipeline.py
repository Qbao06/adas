"""Top-level ADAS pipeline.

Composes:
    camera/lidar readers → vision (detector + tracker) → LiDAR filter+cluster
    → projection → fusion (per-track distance) → EMA filter
    → motion (TTC) → warning → visualizer → health.

The pipeline is mode-driven. Supported modes:

    demo          : mock camera + mock LiDAR, no hardware required.
    live          : camera from ``camera.source`` + LiDAR chosen via ``lidar.driver``.
    playback      : video file + LiDAR .npy/.npz/.csv file.
    camera-only   : only camera path, LiDAR is bypassed (pure vision run).
    lidar-only    : only LiDAR path, no detector (raw clusters remain).
    fusion-test   : synthetic data exercising every fusion component.
    benchmark     : runs N frames and prints timing stats.
"""
from __future__ import annotations

import logging
import importlib
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, List, Optional, Sequence

import numpy as np

from adas import CollisionWarning, DistanceWarning, MotionEstimator
from calibration import LidarCameraProjector
from config import apply_side_modules, modules_from_config, resolve_path
from core.enums import SystemStatus
from core.types import CameraFrame, LidarFrame
from fusion import FusionEngine, TemporalFilter
from lidar import LidarProcessor
from sensors import (
    AsyncFrameReader,
    CameraReader,
    ESP32RadarReader,
    LidarReaderFactory,
    MockCameraReader,
    MockLidarReader,
    MultiCameraReader,
    PlaybackLidarReader,
    RadarCameraSelector,
    ModuleCameraSelector,
    SideModuleManager,
    RadarGatewayManager,
    RadarGatewayView,
)
from synchronization import SensorSynchronizer
from system import HealthMonitor, LiveSnapshotPublisher, PerformanceMonitor, configure_logging, get_logger
from vision import MockDetector, VisionProcessor, build_detector, build_tracker
from visualization import Visualizer

LOGGER = get_logger("adas.pipeline")


@dataclass
class PipelineStats:
    started_at: float = field(default_factory=time.perf_counter)
    frames: int = 0
    total_frames: int = 0
    detection_count: int = 0
    fused_count: int = 0
    emergency_count: int = 0
    danger_count: int = 0
    caution_count: int = 0
    safe_count: int = 0
    last_latency_ms: float = 0.0
    avg_latency_ms: float = 0.0
    p95_latency_ms: float = 0.0
    fps: float = 0.0

    def update(self, latency_ms: float, detections: int, fused: Sequence) -> None:
        self.frames += 1
        self.detection_count += detections
        self.fused_count += sum(1 for o in fused if getattr(o, "fusion_status", None))
        for obj in fused:
            level = getattr(obj, "warning_level", None)
            if level is None:
                continue
            if level.value == "emergency":
                self.emergency_count += 1
            elif level.value == "danger":
                self.danger_count += 1
            elif level.value == "caution":
                self.caution_count += 1
            elif level.value == "safe":
                self.safe_count += 1


class ADASPipeline:
    """High-level ADAS orchestration."""

    SUPPORTED_MODES = (
        "demo",
        "live",
        "playback",
        "camera-only",
        "lidar-only",
        "fusion-test",
        "benchmark",
    )

    def __init__(
        self,
        config: dict,
        root: Optional[Path] = None,
        *,
        cli_overrides: Optional[dict] = None,
    ) -> None:
        self.config = config or {}
        self.root = Path(root or Path(__file__).resolve().parents[1])
        self.cli_overrides = dict(cli_overrides or {})

        log_dir = resolve_path(self.root, self.config.get("outputs", {}).get("log_directory", "logs"))
        log_level = self.config.get("outputs", {}).get("log_level", "INFO")
        configure_logging(log_dir, level=log_level)

        self._normalise_paths()
        self.running = False
        self._stats = PipelineStats()
        self._latencies: List[float] = []
        self._build()
        self._last_detections: list = []
        self._last_lidar_points = np.empty((0, 3), dtype=float)
        web_cfg = self.config.get("web_dashboard", {})
        self.snapshot_publisher = (
            LiveSnapshotPublisher(
                self.root,
                max_hz=float(web_cfg.get("snapshot_hz", 10.0)),
                jpeg_quality=int(web_cfg.get("jpeg_quality", 75)),
                secondary_hz=float(web_cfg.get("secondary_hz", 2.0)),
                secondary_quality=int(web_cfg.get("secondary_quality", 60)),
                async_encode=bool(web_cfg.get("async_encode", False)),
            )
            if self._mode == "live" and web_cfg.get("enabled", True)
            else None
        )

    def _normalise_paths(self) -> None:
        """Resolve every relative path used at runtime against ``self.root``."""
        outputs = self.config.setdefault("outputs", {})
        out_dir = resolve_path(self.root, outputs.get("directory", "output"))
        outputs["directory"] = str(out_dir)
        if outputs.get("video_path"):
            outputs["video_path"] = str(resolve_path(self.root, outputs["video_path"]))
        vision = self.config.setdefault("vision", {})
        model = vision.get("model")
        if isinstance(model, str) and model and not Path(model).is_absolute():
            vision["model"] = str(self.root / model)
        lidar = self.config.setdefault("lidar", {})
        playback = lidar.get("playback_path")
        if isinstance(playback, str) and playback:
            lidar["playback_path"] = str(resolve_path(self.root, playback))
        camera = self.config.setdefault("camera", {})
        if camera.get("video_path") and camera.get("video_path") != "mock":
            camera["video_path"] = str(resolve_path(self.root, camera["video_path"]))
        calibration = self.config.setdefault("calibration", {})
        if calibration.get("file"):
            calibration["file"] = str(resolve_path(self.root, calibration["file"]))
        visualization = self.config.setdefault("visualization", {})
        if visualization.get("record_path") and not Path(str(visualization["record_path"])).is_absolute():
            visualization["record_path"] = str(
                resolve_path(self.root, visualization["record_path"])
            )
        elif visualization.get("record_path") and not outputs.get("video_path"):
            outputs["video_path"] = str(out_dir / "result.mp4")
            visualization["record_path"] = outputs["video_path"]

    # --------------------------------------------------------- builders

    def _build(self) -> None:
        cfg = self.config
        mode = (cfg.get("mode") or "demo").lower()
        if mode not in self.SUPPORTED_MODES:
            if not cfg.get("safety", {}).get("allow_mock", True):
                raise RuntimeError(
                    f"Mode '{mode}' không hỗ trợ. Live không được fallback sang demo."
                )
            LOGGER.warning("Mode '%s' không hỗ trợ, fallback sang demo", mode)
            mode = "demo"
            cfg["mode"] = mode

        self._mode = mode
        if cfg.get("side_sensor_modules"):
            apply_side_modules(cfg, modules_from_config(cfg))
        self.synchronizer = SensorSynchronizer(
            max_difference_ms=cfg["synchronization"]["max_difference_ms"],
            buffer_size=cfg["synchronization"]["buffer_size"],
            stale_after_ms=cfg["synchronization"]["stale_after_ms"],
        )
        self.synchronizer.sorted_search = bool(
            cfg.get("synchronization", {}).get("sorted_search", True)
        )

        # Camera.
        camera_cfg = cfg["camera"]
        if mode in {"camera-only", "lidar-only"}:
            camera_cfg = dict(camera_cfg)
            camera_cfg["enabled"] = (mode == "camera-only")
        if not camera_cfg.get("enabled", True):
            self.camera = None
            LOGGER.info("[%s] Camera bị tắt trong config — pipeline chạy không camera", mode)
        else:
            self.camera = self._build_camera(camera_cfg, mode)
        self.radar = None
        self.side_modules = None
        self.camera_selector = None
        self._radar_state = None
        self._module_states = {}
        self._gateway_view: RadarGatewayView | None = None
        self._last_gateway_view: RadarGatewayView | None = None
        self._radar_gateway = None
        module_cfg = cfg.get("side_sensor_modules", {})
        gateway_cfg = cfg.get("radar_gateway")
        gateway_enabled = bool(gateway_cfg.get("enabled", False)) if isinstance(gateway_cfg, dict) else False
        # C4: when the new Gateway schema is enabled, the legacy single
        # ``side_radar`` block is incompatible (different protocol, no
        # aggregated 4-radar picture).  Reject the combination loudly so
        # the user knows to drop one of the two blocks.
        radar_cfg = cfg.get("side_radar", {})
        legacy_radar_enabled = bool(radar_cfg.get("enabled", False))
        if (
            mode == "live"
            and gateway_enabled
            and legacy_radar_enabled
        ):
            raise RuntimeError(
                "Cấu hình xung đột: radar_gateway.enabled=true và "
                "side_radar.enabled=true cùng lúc. Khi dùng ESP32 Gateway "
                "thì tắt side_radar (chỉ giữ radar_gateway) hoặc ngược lại."
            )
        if gateway_enabled and legacy_radar_enabled is False:
            # Gateway path: force-disable the legacy block so any downstream
            # code that reads side_radar still sees a clean "disabled" config.
            radar_cfg = dict(radar_cfg)
            radar_cfg["enabled"] = False
            cfg["side_radar"] = radar_cfg
            LOGGER.info(
                "radar_gateway.enabled=true → force tắt side_radar để "
                "tránh ESP32RadarReader được mở cùng lúc."
            )
            legacy_radar_enabled = False
        # Prefer the new gateway schema when enabled. Old modules stay disabled.
        if mode == "live" and gateway_enabled:
            try:
                self._radar_gateway = RadarGatewayManager(gateway_cfg)
                self.camera_selector = self._radar_gateway.selector
                LOGGER.info(
                    "RadarGateway đang dùng device=%s baud=%d protocol_v=%s",
                    gateway_cfg.get("device"),
                    int(gateway_cfg.get("baudrate", 115200)),
                    int(gateway_cfg.get("protocol_version", 2)),
                )
            except Exception as exc:
                # Live must NEVER silently fall back to mock radar. Surface the error.
                raise RuntimeError(f"Không khởi tạo được RadarGateway: {exc}") from exc
        elif mode == "live" and module_cfg.get("enabled", False):
            self.side_modules = SideModuleManager(module_cfg)
            self.camera_selector = ModuleCameraSelector(
                hold_seconds=float(module_cfg.get("camera_hold_seconds", 1.2))
            )
        radar_cfg = cfg.get("side_radar", {})
        if mode == "live" and self.side_modules is None and radar_cfg.get("enabled", False):
            self.radar = ESP32RadarReader(
                radar_cfg.get("device", "/dev/ttyACM0"),
                baudrate=int(radar_cfg.get("baudrate", 115200)),
                stale_after_seconds=float(radar_cfg.get("stale_after_seconds", 0.6)),
                max_distance_m=float(radar_cfg.get("max_distance_m", 30.0)),
            )
            self.camera_selector = RadarCameraSelector(
                hold_seconds=float(radar_cfg.get("camera_hold_seconds", 1.2))
            )
        # Note: when radar_gateway.enabled=true the C4 block above
        # force-disables the legacy side_radar block (and raises a
        # RuntimeError if both are enabled together), so this branch
        # never executes in the Gateway path.
        # LiDAR.
        lidar_cfg = cfg["lidar"]
        if mode in {"camera-only", "lidar-only"}:
            lidar_cfg = dict(lidar_cfg)
            lidar_cfg["enabled"] = (mode == "lidar-only")
        if not lidar_cfg.get("enabled", True):
            self.lidar = None
            LOGGER.info("[%s] LiDAR bị tắt trong config — pipeline chạy không LiDAR", mode)
        else:
            self.lidar = self._build_lidar(lidar_cfg, mode)

        # Vision / detection.
        allow_mock = bool(cfg.get("safety", {}).get("allow_mock", mode != "live"))
        if self.camera is not None:
            detector = build_detector(cfg["vision"], allow_mock=allow_mock)
            if mode == "live" and not allow_mock and isinstance(detector, MockDetector):
                raise RuntimeError("LIVE từ chối MockDetector: kiểm tra model/CUDA")
            tracker = build_tracker(cfg["vision"])
            self.vision = VisionProcessor(
                detector,
                tracker,
                input_order=str(cfg["vision"].get("input_order", "bgr")),
            )
        else:
            self.vision = None

        # LiDAR processing.
        if self.lidar is not None:
            self.lidar_processor = LidarProcessor(cfg["lidar_processing"])
            if mode == "live" and cfg.get("safety", {}).get("is_demo_calibration", True):
                raise RuntimeError("LIVE từ chối calibration mô phỏng; hãy calibrate phần cứng thật")
            self.projector = LidarCameraProjector(resolve_path(self.root, cfg["calibration"]["file"]))
        else:
            self.lidar_processor = None
            self.projector = None

        if self.vision is not None and self.lidar_processor is not None and self.projector is not None:
            self.fusion = FusionEngine(self.projector, cfg["fusion"])
            self.temporal = TemporalFilter(
                alpha=cfg["fusion"].get("ema_alpha", 0.35),
                stale_seconds=cfg["fusion"].get("track_stale_seconds", 1.5),
                max_hold=cfg["fusion"].get("track_max_hold", 10),
            )
        else:
            self.fusion = None
            self.temporal = None

        warning_mode = str(cfg["warning"].get("mode", "ttc")).lower()
        self._warning_mode = warning_mode
        if warning_mode == "distance":
            self.motion = None
            self.warning = DistanceWarning(cfg["warning"])
        else:
            self.motion = MotionEstimator(
                alpha=cfg["adas"]["alpha"],
                stale_seconds=cfg["adas"]["stale_seconds"],
                min_closing_mps=cfg["adas"]["min_closing_mps"],
                ttc_cap_s=cfg["adas"]["ttc_cap_s"],
                max_speed_mps=cfg["adas"].get("max_speed_mps", 60.0),
                ego_speed_mps=cfg["adas"].get("ego_speed_mps", 0.0),
            )
            self.warning = CollisionWarning(cfg["warning"])

        self.visualizer = Visualizer(cfg["visualization"])
        self.health = HealthMonitor(timeout_seconds=cfg.get("safety", {}).get("heartbeat_timeout", 1.5))
        self.performance = PerformanceMonitor()
        if self.vision is not None:
            try:
                self.vision.warmup()
            except Exception as exc:
                LOGGER.warning("Vision warmup loi: %s", exc)

    def _build_camera(self, camera_cfg: dict, mode: str):
        sources = camera_cfg.get("sources")
        if mode == "live" and isinstance(sources, dict) and sources:
            reader = MultiCameraReader(
                sources,
                width=int(camera_cfg.get("width", 1280)),
                height=int(camera_cfg.get("height", 720)),
                fps=int(camera_cfg.get("fps", 30)),
                reconnect_seconds=float(camera_cfg.get("reconnect_seconds", 2.0)),
                shared_usb_hub=bool(camera_cfg.get("shared_usb_hub", False)),
                hub_keep_open=bool(camera_cfg.get("hub_keep_open", False)),
            )
            return reader
        # playback mode or explicit video_path → file-based CameraReader.
        if mode == "playback" or (camera_cfg.get("video_path") and camera_cfg.get("source") in (None, "", "video")):
            path = camera_cfg.get("video_path") or camera_cfg.get("source")
            if isinstance(path, (str, Path)) and path and not str(path).isdigit():
                if str(path).lower() == "mock":
                    LOGGER.info("Camera dùng MockCameraReader trong chế độ playback")
                    return MockCameraReader(
                        width=int(camera_cfg.get("width", 1280)),
                        height=int(camera_cfg.get("height", 720)),
                        fps=int(camera_cfg.get("fps", 30)),
                    )
                LOGGER.info("Camera lấy từ video file: %s", path)
                return CameraReader(
                    source=str(path),
                    width=int(camera_cfg.get("width", 1280)),
                    height=int(camera_cfg.get("height", 720)),
                    fps=int(camera_cfg.get("fps", 30)),
                    loop_video=bool(camera_cfg.get("loop_video", True)),
                    reconnect_seconds=float(camera_cfg.get("reconnect_seconds", 2.0)),
                )
        source = camera_cfg.get("source", "mock")
        if str(source).lower() == "mock" or (
            mode in ("demo", "fusion-test")
            and not self.cli_overrides.get("camera", {}).get("source")
        ):
            if mode == "live" and not self.config.get("safety", {}).get("allow_mock", False):
                raise RuntimeError(
                    "LIVE từ chối MockCameraReader: cấu hình camera.source / camera.sources thật"
                )
            LOGGER.info("Mode %s: dùng MockCameraReader", mode)
            return MockCameraReader(
                width=int(camera_cfg.get("width", 1280)),
                height=int(camera_cfg.get("height", 720)),
                fps=int(camera_cfg.get("fps", 30)),
            )
        try:
            reader = CameraReader(
                source=source,
                width=int(camera_cfg.get("width", 1280)),
                height=int(camera_cfg.get("height", 720)),
                fps=int(camera_cfg.get("fps", 30)),
                loop_video=bool(camera_cfg.get("loop_video", True)),
                reconnect_seconds=float(camera_cfg.get("reconnect_seconds", 2.0)),
            )
            if mode in ("demo", "fusion-test"):
                reader.open()
            return reader
        except Exception as exc:
            if mode == "live" and not self.config.get("safety", {}).get("allow_mock", False):
                raise RuntimeError(f"LIVE không mở được camera {source}: {exc}") from exc
            LOGGER.warning("Không mở được camera thật (%s) — fallback mock: %s", source, exc)
            return MockCameraReader(
                width=int(camera_cfg.get("width", 1280)),
                height=int(camera_cfg.get("height", 720)),
                fps=int(camera_cfg.get("fps", 30)),
            )

    def _build_lidar(self, lidar_cfg: dict, mode: str):
        source = (lidar_cfg.get("source") or "mock").lower()
        driver = (lidar_cfg.get("driver") or source or "mock").lower()
        driver_module = lidar_cfg.get("driver_module")
        if driver_module:
            importlib.import_module(str(driver_module))
        if mode == "live" and not self.config.get("safety", {}).get("allow_mock", False):
            if driver == "mock" or source == "mock":
                raise RuntimeError("LIVE từ chối MockLidarReader: cấu hình driver LiDAR thật")
            if driver not in LidarReaderFactory.available():
                raise RuntimeError(
                    f"Driver LiDAR '{driver}' chưa đăng ký; available={LidarReaderFactory.available()}"
                )
        try:
            if mode == "playback" and lidar_cfg.get("playback_path"):
                path = lidar_cfg["playback_path"]
                LOGGER.info("Playback LiDAR từ: %s", path)
                return PlaybackLidarReader(
                    path=path,
                    fps=int(lidar_cfg.get("fps", 10)),
                    playback_format=lidar_cfg.get("playback_format", "auto"),
                    loop=True,
                )
            if driver == "mock" or source == "mock" or mode == "demo" or mode == "fusion-test":
                return MockLidarReader(
                    objects=lidar_cfg.get("mock_objects"),
                    fps=int(lidar_cfg.get("fps", 10)),
                    seed=int(lidar_cfg.get("mock_seed", 7)),
                    points_per_object=int(lidar_cfg.get("mock_points_per_object", 40)),
                    jitter=tuple(lidar_cfg.get("mock_jitter", [0.25, 0.25, 0.35])),
                )
            live_strict = mode == "live" and not self.config.get("safety", {}).get(
                "allow_mock", False
            )
            return LidarReaderFactory.create(
                driver,
                allow_mock=not live_strict,
                config=lidar_cfg,
                fps=int(lidar_cfg.get("fps", 10)),
            )
        except FileNotFoundError as exc:
            if mode == "live" and not self.config.get("safety", {}).get("allow_mock", False):
                raise RuntimeError(f"LIVE không mở được file/driver LiDAR: {exc}") from exc
            LOGGER.warning("Playback LiDAR không có sẵn (%s) — fallback mock", exc)
            return MockLidarReader(
                objects=lidar_cfg.get("mock_objects"),
                fps=int(lidar_cfg.get("fps", 10)),
                seed=int(lidar_cfg.get("mock_seed", 7)),
                points_per_object=int(lidar_cfg.get("mock_points_per_object", 40)),
                jitter=tuple(lidar_cfg.get("mock_jitter", [0.25, 0.25, 0.35])),
            )

    # ------------------------------------------------------- main loop

    def process(self, camera_frame, lidar_frame):
        started = time.perf_counter()
        detections: list = []
        objects: list = []
        projected = None
        if bool(self.config.get("runtime", {}).get("parallel_perception", False)):
            detections, points = self._process_parallel(camera_frame, lidar_frame)
        else:
            detections = self._run_vision(camera_frame)
            points = self._run_lidar(lidar_frame)
        self._last_detections = detections
        self._last_lidar_points = points

        try:
            if (
                self.fusion is not None
                and camera_frame is not None
                and self.projector is not None
                and getattr(camera_frame, "camera_id", "front") == "front"
            ):
                objects, projected = self.fusion.fuse(
                    detections,
                    points,
                    camera_frame.image.shape[:2],
                    camera_frame.timestamp,
                )
            else:
                objects = []
                projected = None
        except Exception as exc:
            LOGGER.warning("Fusion lỗi: %s", exc)
            objects = []

        try:
            if self.temporal is not None and camera_frame is not None:
                self.temporal.update(objects, camera_frame.timestamp)
        except Exception as exc:
            LOGGER.warning("Temporal filter lỗi: %s", exc)
        try:
            if camera_frame is not None:
                if self.motion is not None:
                    self.motion.update(objects, camera_frame.timestamp)
                self.warning.evaluate(objects)
        except Exception as exc:
            LOGGER.warning("Motion/warning lỗi: %s", exc)

        self.performance.mark(started)
        latency_ms = self.performance.last_latency_ms
        self._stats.update(latency_ms, len(detections), objects)
        self._latencies.append(latency_ms)
        return objects, projected

    def _run_vision(self, camera_frame):
        try:
            if self.vision is not None and camera_frame is not None:
                return self.vision.process(camera_frame)
        except Exception as exc:
            LOGGER.warning("Vision lỗi: %s", exc)
        return []

    def _run_lidar(self, lidar_frame):
        try:
            if self.lidar_processor is not None and lidar_frame is not None:
                points, clusters = self.lidar_processor.process(lidar_frame)
                if self.config.get("fusion", {}).get("use_clusters", False):
                    if clusters:
                        points = np.vstack([cluster.points for cluster in clusters])
                    elif not self.config.get("fusion", {}).get(
                        "fallback_to_filtered_points", False
                    ):
                        points = np.empty((0, 3), dtype=float)
                return points
        except Exception as exc:
            LOGGER.warning("LiDAR processing lỗi: %s", exc)
        return np.empty((0, 3), dtype=float)

    def _process_parallel(self, camera_frame, lidar_frame):
        """Run vision and LiDAR in parallel threads; fusion waits for both."""
        import concurrent.futures

        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            fut_vision = pool.submit(self._run_vision, camera_frame)
            fut_lidar = pool.submit(self._run_lidar, lidar_frame)
            return fut_vision.result(), fut_lidar.result()

    def render(self, image, objects, projected, *, sensor_summary=None):
        try:
            draw = (self.visualizer.draw_async
                    if bool(self.config.get("visualization", {}).get("async_render", False))
                    and hasattr(self.visualizer, "draw_async")
                    else self.visualizer.draw)
            return draw(
                image,
                objects,
                projected,
                fps=self.performance.fps(),
                latency_ms=self.performance.last_latency_ms,
                status=self.health.evaluate().value,
                lidar_status=(sensor_summary or {}).get("lidar", "ok"),
                camera_status=(sensor_summary or {}).get("camera", "ok"),
                sensor_summary=sensor_summary,
            )
        except Exception as exc:
            LOGGER.warning("Visualizer lỗi: %s", exc)
            return image

    # -------------------------------------------------- public API

    def run(
        self,
        max_frames: Optional[int] = None,
        display: bool = True,
        *,
        save_first_frame: bool = False,
        record_video: Optional[bool] = None,
    ) -> int:
        """Run the main loop. Returns the number of frames processed."""
        if self._mode == "benchmark" and max_frames is None:
            benchmark = self.config.get("benchmark", {})
            max_frames = int(benchmark.get("warmup_frames", 0)) + int(
                benchmark.get("measure_frames", 50)
            )
        if record_video is not None:
            visualization = self.config.setdefault("visualization", {})
            visualization["record_video"] = bool(record_video)
            if record_video and not visualization.get("record_path"):
                visualization["record_path"] = self.config.get("outputs", {}).get(
                    "video_path", str(self.root / "output" / "result.mp4")
                )
        self._open_inputs()
        camera_worker = None
        lidar_worker = None
        async_enabled = bool(
            self.config.get("runtime", {}).get("async_sensors", True)
            and self.camera is not None
            and self.lidar is not None
        )
        if async_enabled:
            camera_worker = AsyncFrameReader(self.camera, "camera", queue_size=4)
            lidar_worker = AsyncFrameReader(self.lidar, "lidar", queue_size=4)
            camera_worker.start()
            lidar_worker.start()
        camera_sequence = 0
        lidar_sequence = 0
        last_input_at = time.monotonic()
        no_data_timeout = float(self.config.get("runtime", {}).get("no_data_timeout_seconds", 3.0))
        sensor_timeout = float(self.config.get("runtime", {}).get("sensor_timeout_seconds", 2.0))
        sync_timeout = float(self.config.get("runtime", {}).get("sync_timeout_seconds", 2.0))
        last_camera_at = last_input_at
        last_lidar_at = last_input_at
        last_pair_at = last_input_at
        self.health.set_status(SystemStatus.STARTING)
        self.running = True
        count = 0
        warmup_frames = max(0, int(self.config.get("benchmark", {}).get("warmup_frames", 0)))
        try:
            while self.running:
                if self._radar_gateway is not None:
                    try:
                        self._gateway_view = self._radar_gateway.read()
                    except Exception as exc:
                        LOGGER.warning("RadarGateway read lỗi: %s", exc)
                        self._gateway_view = None
                    self._last_gateway_view = self._gateway_view
                    if self._gateway_view is not None:
                        selection = self._gateway_view.selection
                        if isinstance(self.camera, MultiCameraReader):
                            self.camera.select(selection.camera)
                        # Gateway heartbeat: keep separate slots so the health
                        # monitor can distinguish them from the legacy
                        # side_radar slot.
                        if self._gateway_view.snapshot.link_ok:
                            self.health.heartbeat("radar_gateway")
                            if not self._gateway_view.snapshot.gateway_ok:
                                # gateway_ok=false must always raise a health error.
                                self.health.set_status(SystemStatus.ERROR)
                            elif (
                                not self._gateway_view.snapshot.remote_ok
                                and self._any_remote_radar_enabled()
                            ):
                                self.health.set_status(SystemStatus.DEGRADED)
                elif self.side_modules is not None and self.camera_selector is not None:
                    self._module_states = self.side_modules.read()
                    selection_camera, selection_reason, selection_changed = self.camera_selector.update(self._module_states)
                    if isinstance(self.camera, MultiCameraReader):
                        self.camera.select(selection_camera)
                    if selection_changed:
                        LOGGER.info("Module camera select: %s (reason=%s)", selection_camera, selection_reason)
                    if any(state.link_ok for state in self._module_states.values()):
                        self.health.heartbeat("side_radar")
                elif self.radar is not None and self.camera_selector is not None:
                    self._radar_state = self.radar.read()
                    selection = self.camera_selector.update(self._radar_state)
                    if isinstance(self.camera, MultiCameraReader):
                        self.camera.select(selection.camera)
                    if selection.changed:
                        LOGGER.info(
                            "Radar camera select: %s (reason=%s left=%s right=%s)",
                            selection.camera,
                            selection.reason,
                            self._radar_state.left_detected,
                            self._radar_state.right_detected,
                        )
                    if self._radar_state.link_ok:
                        self.health.heartbeat("side_radar")
                camera_frame: Optional[CameraFrame] = None
                lidar_frame: Optional[LidarFrame] = None
                try:
                    if camera_worker is not None:
                        camera_sequence, camera_frame = camera_worker.get_latest(camera_sequence)
                    elif self.camera is not None:
                        camera_frame = self.camera.read()
                except Exception as exc:
                    LOGGER.warning("Camera read lỗi: %s", exc)
                try:
                    if lidar_worker is not None:
                        lidar_sequence, lidar_frame = lidar_worker.get_latest(lidar_sequence)
                    elif self.lidar is not None:
                        lidar_frame = self.lidar.read()
                except Exception as exc:
                    LOGGER.warning("LiDAR read lỗi: %s", exc)
                if camera_frame is None and lidar_frame is None:
                    if async_enabled:
                        worker_error = camera_worker.error or lidar_worker.error
                        if worker_error is not None:
                            raise RuntimeError(f"Luồng cảm biến lỗi: {worker_error}") from worker_error
                        if time.monotonic() - last_input_at > no_data_timeout:
                            raise RuntimeError("LIVE không nhận được dữ liệu cảm biến trong thời gian cho phép")
                        continue
                    LOGGER.info("Cả camera lẫn LiDAR không có dữ liệu — dừng")
                    break
                last_input_at = time.monotonic()
                if camera_frame is not None:
                    # Health uses local monotonic receipt time. Sensor timestamps
                    # may be PTP/epoch values and are reserved for synchronization.
                    self.health.heartbeat("camera")
                    last_camera_at = time.monotonic()
                if lidar_frame is not None:
                    self.health.heartbeat("lidar")
                    last_lidar_at = time.monotonic()
                if self._mode == "live":
                    now = time.monotonic()
                    if self.camera is not None and now - last_camera_at > sensor_timeout:
                        raise RuntimeError("LIVE mất dữ liệu camera")
                    if self.lidar is not None and now - last_lidar_at > sensor_timeout:
                        raise RuntimeError("LIVE mất dữ liệu LiDAR")
                # Pair real dual-sensor frames by timestamp before fusion.
                # Single-sensor modes bypass the synchronizer and use the
                # placeholder path below.
                if self.camera is not None and self.lidar is not None:
                    if camera_frame is not None:
                        self.synchronizer.add_camera(camera_frame)
                    if lidar_frame is not None:
                        self.synchronizer.add_lidar(lidar_frame)
                    pair = self.synchronizer.get_pair()
                    if pair is None:
                        if self._mode == "live" and time.monotonic() - last_pair_at > sync_timeout:
                            raise RuntimeError("LIVE không đồng bộ được camera và LiDAR")
                        continue
                    camera_frame, lidar_frame = pair.camera, pair.lidar
                    last_pair_at = time.monotonic()
                if lidar_frame is None:
                    lidar_frame = LidarFrame(
                        points=np.empty((0, 3), dtype=float),
                        timestamp=time.monotonic(),
                        frame_id=-1,
                    )
                if camera_frame is None:
                    placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
                    placeholder[:] = (20, 20, 20)
                    camera_frame = CameraFrame(
                        image=placeholder,
                        timestamp=lidar_frame.timestamp,
                        frame_id=-1,
                    )
                objects, projected = self.process(camera_frame, lidar_frame)
                if self.camera is None:
                    image = camera_frame.image
                else:
                    image = camera_frame.image
                sensor_summary = {
                    "camera": "disabled" if self.camera is None else ("ok" if self.camera.is_connected() else "lost"),
                    "lidar": "disabled" if self.lidar is None else ("ok" if self.lidar.is_connected() else "lost"),
                }
                output = self.render(image, objects, projected, sensor_summary=sensor_summary)
                if self.snapshot_publisher is not None:
                    display_frame = camera_frame
                    camera_id = getattr(camera_frame, "camera_id", "front")
                    if isinstance(self.camera, MultiCameraReader):
                        camera_statuses = self.camera.connection_status()
                        camera_id = self.camera.active_camera
                        selected_frame = self.camera.read_display()
                        if selected_frame is not None:
                            display_frame = selected_frame
                        camera_images = {
                            name: frame.image
                            for name, frame in self.camera.read_views().items()
                        }
                    else:
                        camera_statuses = {camera_id: bool(self.camera and self.camera.is_connected())}
                        camera_images = {camera_id: display_frame.image}
                    gateway_view = self._gateway_view if self._radar_gateway is not None else None
                    self.snapshot_publisher.publish(
                        image=display_frame.image,
                        camera_id=camera_id,
                        detections=self._last_detections,
                        objects=objects,
                        lidar_points=self._last_lidar_points,
                        radar_state=self._radar_state,
                        module_states=self._module_states,
                        gateway_view=gateway_view,
                        gateway_stats=(self._radar_gateway.stats() if self._radar_gateway is not None else None),
                        camera_images=camera_images,
                        camera_status=camera_statuses,
                        lidar_ok=bool(self.lidar and self.lidar.is_connected()),
                        fps=self.performance.fps(),
                        latency_ms=self.performance.last_latency_ms,
                        frame_id=camera_frame.frame_id,
                    )

                if save_first_frame and count == 0:
                    out_path = (
                        Path(self.config.get("outputs", {}).get("directory", "output"))
                        / "demo_first_frame.png"
                    )
                    self.visualizer.save(output, out_path)
                    LOGGER.info("Đã lưu frame mẫu: %s", out_path)
                if display:
                    key = self.visualizer.show(
                        output, window_name=self.config.get("visualization", {}).get("window_name", "ADAS LiDAR-Camera")
                    )
                    if key in (ord("q"), 27):
                        break
                count += 1
                if max_frames is not None and count >= max_frames:
                    break
                if warmup_frames and count == warmup_frames:
                    # Reset performance counters after warmup.
                    self.performance = PerformanceMonitor()
                    self._latencies.clear()
                    self._stats = PipelineStats()
                    warmup_frames = 0
        except KeyboardInterrupt:
            LOGGER.info("Nhận KeyboardInterrupt — dừng pipeline")
        finally:
            if camera_worker is not None:
                camera_worker.stop()
            if lidar_worker is not None:
                lidar_worker.stop()
            self._stats.total_frames = count
            self.stop()
        return count

    def _any_remote_radar_enabled(self) -> bool:
        gateway_cfg = self.config.get("radar_gateway", {})
        if not isinstance(gateway_cfg, dict):
            return False
        for radar_id, item in gateway_cfg.get("radars", {}).items():
            if not isinstance(item, dict):
                continue
            if not item.get("enabled", False):
                continue
            if radar_id in {"right_front", "right_rear"}:
                return True
        return False

    def _open_inputs(self) -> None:
        if self._radar_gateway is not None:
            try:
                self._radar_gateway.connect()
            except Exception as exc:
                if self._mode == "live":
                    raise RuntimeError(f"LIVE không kết nối được Radar Gateway: {exc}") from exc
                LOGGER.warning("Không kết nối được Radar Gateway: %s", exc)
        if self.side_modules is not None:
            try:
                self.side_modules.connect()
            except Exception as exc:
                if self._mode == "live":
                    raise RuntimeError(f"LIVE không kết nối được module radar ESP32: {exc}") from exc
                LOGGER.warning("Không kết nối được module radar: %s", exc)
        if self.radar is not None:
            try:
                self.radar.connect()
            except Exception as exc:
                if self._mode == "live":
                    raise RuntimeError(f"LIVE không kết nối được ESP32 radar UART: {exc}") from exc
                LOGGER.warning("Không kết nối được ESP32 radar: %s", exc)
        if self.camera is not None:
            try:
                self.camera.open()
            except Exception as exc:
                if self._mode == "live":
                    raise RuntimeError(f"LIVE không mở được camera: {exc}") from exc
                LOGGER.warning("Không mở được camera: %s", exc)
        if self.lidar is not None:
            try:
                self.lidar.connect()
            except Exception as exc:
                if self._mode == "live":
                    raise RuntimeError(f"LIVE không kết nối được LiDAR: {exc}") from exc
                LOGGER.warning("Không kết nối được LiDAR: %s", exc)

    def stop(self) -> None:
        self.running = False
        if self._radar_gateway is not None:
            try:
                self._radar_gateway.disconnect()
            except Exception as exc:  # pragma: no cover
                LOGGER.debug("Radar Gateway disconnect lỗi: %s", exc)
        if self.side_modules is not None:
            self.side_modules.disconnect()
        if self.radar is not None:
            try:
                self.radar.disconnect()
            except Exception as exc:  # pragma: no cover
                LOGGER.debug("ESP32 radar disconnect lỗi: %s", exc)
        if self.camera is not None:
            try:
                self.camera.close()
            except Exception as exc:  # pragma: no cover
                LOGGER.debug("Camera close lỗi: %s", exc)
        if self.lidar is not None:
            try:
                self.lidar.disconnect()
            except Exception as exc:  # pragma: no cover
                LOGGER.debug("LiDAR disconnect lỗi: %s", exc)
        try:
            self.visualizer.close()
        except Exception:  # pragma: no cover
            pass
        # Final stats.
        self._stats.fps = self.performance.fps()
        if self._latencies:
            arr = np.asarray(self._latencies)
            self._stats.avg_latency_ms = float(arr.mean())
            self._stats.p95_latency_ms = float(np.percentile(arr, 95))
        LOGGER.info(
            "Pipeline dừng — total_frames=%d measured_frames=%d  fps=%.1f  latency avg=%.1fms p95=%.1fms",
            self._stats.total_frames,
            self._stats.frames,
            self._stats.fps,
            self._stats.avg_latency_ms,
            self._stats.p95_latency_ms,
        )

    def stats(self) -> PipelineStats:
        return self._stats


__all__ = ["ADASPipeline", "PipelineStats"]
