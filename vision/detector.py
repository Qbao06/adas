"""Object detection wrappers.

We default to Ultralytics YOLO when available; otherwise we fall back to a
lightweight heuristic detector that produces plausible bounding boxes from
motion in the image. The tracker only sees ``Detection`` objects, so the
fallback is fully transparent.
"""
from __future__ import annotations

import logging
from typing import Any, Iterable, Optional

import numpy as np

from core.types import CameraFrame, Detection

LOGGER = logging.getLogger(__name__)

# Cache YOLO/model instances per path so a rebuild reuses the loaded engine
# instead of reloading per frame or per pipeline rebuild.
_MODEL_CACHE: dict = {}

try:  # Heavy optional dependency.
    from ultralytics import YOLO  # type: ignore

    _HAVE_YOLO = True
except Exception:  # pragma: no cover - import fallback
    YOLO = None  # type: ignore
    _HAVE_YOLO = False


class NullDetector:
    """Detector that returns no detections (used in lidar-only mode)."""

    name = "null"

    def detect(self, frame: CameraFrame) -> list[Detection]:  # noqa: D401
        return []

    def warmup(self) -> None:  # pragma: no cover
        return None

    def close(self) -> None:  # pragma: no cover
        return None


class MockDetector:
    """Heuristic detector used when YOLO is unavailable.

    Generates deterministic detections based on image size so downstream
    components still receive data to exercise. Real YOLO can be plugged in
    later by setting ``vision.model`` to a path that exists.
    """

    name = "mock"

    def __init__(self, allowed_classes: Optional[Iterable[str]] = None) -> None:
        self.allowed = {c.lower() for c in (allowed_classes or [])}

    def detect(self, frame: CameraFrame) -> list[Detection]:
        h, w = frame.image.shape[:2]
        cx, cy = w // 2, h // 2
        drift = int(40 * np.sin(frame.frame_id * 0.1))
        bbox_main = (cx - 120 + drift, cy - 130, cx + 120 + drift, cy + 170)
        bbox_side = (w - 280, cy - 90, w - 100, cy + 110)
        output = [
            Detection(
                class_id=2,
                class_name="car",
                confidence=0.92,
                bbox=bbox_main,
                timestamp=frame.timestamp,
            ),
            Detection(
                class_id=2,
                class_name="car",
                confidence=0.85,
                bbox=bbox_side,
                timestamp=frame.timestamp,
            ),
        ]
        if self.allowed and "car" not in self.allowed:
            return []
        return output

    def warmup(self) -> None:
        return None

    def close(self) -> None:
        return None


class YoloDetector:
    """Ultralytics YOLO detector with safe startup.

    The constructor does not throw when the model file is missing or
    Ultralytics is not installed — it logs a warning and exposes
    :py:meth:`is_real` so the caller can decide to switch to a mock.
    """

    name = "yolo"

    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        confidence: float = 0.45,
        allowed_classes: Optional[Iterable[str]] = None,
        device: str = "auto",
        image_size: int = 640,
    ) -> None:
        if not _HAVE_YOLO:
            raise RuntimeError(
                "Ultralytics chưa được cài — chuyển sang MockDetector hoặc "
                "cài 'ultralytics' và torch."
            )
        self.confidence = float(confidence)
        self.allowed = {c.lower() for c in (allowed_classes or [])}
        self.device = None if device == "auto" else device
        self.image_size = int(image_size)
        self.model: Optional[Any] = None
        self._names: dict[int, str] = {}
        self._open_error: Optional[str] = None
        cache_key = f"{model_path}|{device}|{image_size}"
        if cache_key in _MODEL_CACHE:
            self.model, self._names = _MODEL_CACHE[cache_key]
            return
        try:
            self.model = YOLO(model_path)
            self._names = dict(self.model.names) if hasattr(self.model, "names") else {}
            _MODEL_CACHE[cache_key] = (self.model, self._names)
        except Exception as exc:  # pragma: no cover - depends on host environment
            self._open_error = str(exc)
            LOGGER.warning("Không tải được YOLO '%s': %s", model_path, exc)

    @property
    def is_real(self) -> bool:
        return self.model is not None

    @property
    def class_names(self) -> set[str]:
        return {str(name).lower() for name in self._names.values()}

    def detect(self, frame: CameraFrame) -> list[Detection]:
        if self.model is None:
            return []
        try:
            result = self.model.predict(
                frame.image,
                conf=self.confidence,
                device=self.device,
                imgsz=self.image_size,
                verbose=False,
            )[0]
        except Exception as exc:  # pragma: no cover - depends on host environment
            LOGGER.warning("YOLO detect lỗi, fallback frame này: %s", exc)
            return []
        names = result.names or self._names
        if result.boxes is None:
            return []
        out: list[Detection] = []
        for box in result.boxes:
            try:
                class_id = int(box.cls.item())
                conf = float(box.conf.item())
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
            except Exception:  # pragma: no cover - malformed boxes
                continue
            class_name = str(names.get(class_id, class_id))
            if self.allowed and class_name.lower() not in self.allowed:
                continue
            out.append(
                Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=conf,
                    bbox=(x1, y1, x2, y2),
                    timestamp=frame.timestamp,
                )
            )
        return out

    def warmup(self) -> None:
        if self.model is None:
            return
        try:
            dummy = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
            self.model.predict(
                dummy,
                conf=self.confidence,
                device=self.device,
                imgsz=self.image_size,
                verbose=False,
            )
        except Exception as exc:  # pragma: no cover
            LOGGER.debug("YOLO warmup thất bại: %s", exc)

    def close(self) -> None:
        # Ultralytics releases resources on its own; nothing to do.
        return None


def build_detector(vision_cfg: dict, *, allow_mock: bool = True) -> Any:
    """Factory that picks YOLO/Mock/Null based on availability and config.

    ``allow_mock=False`` (live) never returns :class:`MockDetector`.
    ``vision.validate_model_on_startup`` also forbids the mock fallback.
    """
    model_name = vision_cfg.get("model", "yolov8n.pt")
    confidence = float(vision_cfg.get("confidence", 0.45))
    device = vision_cfg.get("device", "auto")
    allowed = vision_cfg.get("allowed_classes", [])
    image_size = int(vision_cfg.get("image_size", 640))
    refuse_mock = (not allow_mock) or bool(vision_cfg.get("validate_model_on_startup", False))
    if not _HAVE_YOLO:
        if refuse_mock:
            raise RuntimeError(
                "Ultralytics chưa được cài — live không được dùng MockDetector"
            )
        LOGGER.warning("Không có ultralytics — sử dụng MockDetector")
        return MockDetector(allowed_classes=allowed)
    engine_path = vision_cfg.get("engine_path", "")
    if engine_path and str(engine_path).endswith((".engine", ".plan")):
        # TensorRT engine on Jetson: prefer it when present, else fall back.
        from pathlib import Path as _Path

        if _Path(str(engine_path)).exists():
            model_name = str(engine_path)
            LOGGER.info("Dùng TensorRT engine FP16: %s", model_name)
        else:
            LOGGER.warning("engine_path không tồn tại (%s) — dùng model=%s",
                           engine_path, model_name)
    detector = YoloDetector(
        model_path=model_name,
        confidence=confidence,
        allowed_classes=allowed,
        device=device,
        image_size=image_size,
    )
    if detector.is_real:
        if bool(vision_cfg.get("async_detect", False)):
            try:
                detector = AsyncDetectWrapper(detector)
                LOGGER.info("Bật async YOLO worker (latest-wins)")
            except Exception as exc:  # pragma: no cover - defensive
                LOGGER.warning("Không bật được async_detect: %s", exc)
        missing = detector.allowed - detector.class_names
        if missing:
            LOGGER.warning("allowed_classes không có trong model: %s", sorted(missing))
        if detector.allowed and not (detector.allowed & detector.class_names):
            raise RuntimeError(
                "Không class nào trong allowed_classes khớp model; "
                f"model_classes={sorted(detector.class_names)}"
            )
        return detector
    if refuse_mock:
        detail = detector._open_error or "model=None"
        raise RuntimeError(
            f"Không tải được model YOLO '{model_name}' ({detail}) — "
            "live không fallback MockDetector"
        )
    LOGGER.warning(
        "Không tải được model YOLO '%s' — fallback sang MockDetector", model_name
    )
    return MockDetector(allowed_classes=allowed)


__all__ = [
    "AsyncDetectWrapper",
    "YoloDetector",
    "MockDetector",
    "NullDetector",
    "build_detector",
    "_HAVE_YOLO",
]


class AsyncDetectWrapper:
    """Non-blocking YOLO wrapper: worker thread runs detect, latest wins.

    Keeps :meth:`VisionProcessor.process` synchronous semantics: ``detect``
    returns the most recent finished result (or runs inline until the first
    result arrives). Disabled by default; enable via
    ``vision.async_detect=true``.
    """

    name = "async-yolo"

    def __init__(self, detector, *, max_queue: int = 1) -> None:
        import queue
        import threading

        self.detector = detector
        self._queue: "queue.Queue" = queue.Queue(maxsize=max(1, int(max_queue)))
        self._result = None
        self._result_lock = threading.Lock()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        for attr in ("allowed", "confidence", "image_size", "device",
                     "_names", "model", "is_real"):
            if hasattr(detector, attr):
                try:
                    setattr(self, attr, getattr(detector, attr))
                except Exception:
                    pass

    def _loop(self) -> None:
        while True:
            frame = self._queue.get()
            if frame is None:
                return
            try:
                out = self.detector.detect(frame)
            except Exception as exc:  # pragma: no cover - defensive
                LOGGER.warning("AsyncDetect lỗi, giữ kết quả cũ: %s", exc)
                continue
            with self._result_lock:
                self._result = out

    def detect(self, frame):
        try:
            if self._queue.full():
                try:
                    self._queue.get_nowait()  # drop stale, keep newest
                except Exception:
                    pass
            self._queue.put_nowait(frame)
        except Exception:
            return self.detector.detect(frame)
        with self._result_lock:
            if self._result is not None:
                return self._result
        # First frame: run inline so behaviour matches sync detector.
        try:
            out = self.detector.detect(frame)
        except Exception as exc:
            LOGGER.warning("YOLO detect lỗi, fallback frame này: %s", exc)
            return []
        with self._result_lock:
            self._result = out
        return out

    def warmup(self) -> None:
        warmup = getattr(self.detector, "warmup", None)
        if callable(warmup):
            warmup()

    def close(self) -> None:
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass
        close = getattr(self.detector, "close", None)
        if callable(close):
            close()
