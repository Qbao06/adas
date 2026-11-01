"""Camera readers: real device, video file, mock.

Each reader exposes the same surface:

    open() -> None
    read() -> CameraFrame | None
    is_connected() -> bool
    close() -> None

The mock reader is used by default so the rest of the pipeline always has
something valid to look at, regardless of whether a real camera exists.
"""
from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Any, Mapping, Optional

import numpy as np

from core.types import CameraFrame

LOGGER = logging.getLogger(__name__)

try:  # OpenCV is optional — only required for non-mock modes.
    import cv2

    _HAVE_CV2 = True
except Exception:  # pragma: no cover - import fallback
    cv2 = None  # type: ignore[assignment]
    _HAVE_CV2 = False


class _BaseCamera:
    """Common interface. The base does not implement real IO."""

    name = "base"

    def open(self) -> None:  # pragma: no cover - abstract
        raise NotImplementedError

    def read(self) -> Optional[CameraFrame]:  # pragma: no cover - abstract
        raise NotImplementedError

    def is_connected(self) -> bool:
        raise NotImplementedError

    def close(self) -> None:
        return None


def _draw_moving_box(
    image: np.ndarray,
    frame_id: int,
    target_box: tuple[int, int, int, int],
    side_box: tuple[int, int, int, int],
) -> None:
    """Draw a moving car and a side panel to provide texture for detection."""
    if not _HAVE_CV2:
        return
    import cv2 as _cv2

    h, w = image.shape[:2]
    road_h = int(h * 0.7)
    _cv2.rectangle(image, (0, road_h), (w, h), (60, 60, 60), thickness=-1)
    for i in range(0, w, 80):
        _cv2.line(
            image,
            (i, int(h * 0.85)),
            (i + 40, int(h * 0.85)),
            (200, 200, 200),
            thickness=2,
        )

    drift = (frame_id * 4) % 40
    x1, y1, x2, y2 = target_box
    x1 = max(0, min(w - 1, x1 + drift))
    x2 = max(0, min(w, x2 + drift))
    _cv2.rectangle(image, (x1, y1), (x2, y2), (90, 170, 230), thickness=-1)
    _cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 255), thickness=2)
    sx1, sy1, sx2, sy2 = side_box
    _cv2.rectangle(image, (sx1, sy1), (sx2, sy2), (200, 200, 90), thickness=-1)
    _cv2.rectangle(image, (sx1, sy1), (sx2, sy2), (255, 255, 255), thickness=2)


class MockCameraReader(_BaseCamera):
    """Generate synthetic frames with simple moving shapes.

    The mock is deterministic enough for tests yet diverse enough for demos.
    """

    name = "mock"

    def __init__(self, width: int = 1280, height: int = 720, fps: int = 30) -> None:
        self.width = int(width)
        self.height = int(height)
        self.period = 1.0 / max(float(fps), 0.1)
        self.frame_id = 0
        self._connected = False
        self._lock = threading.Lock()

    def open(self) -> None:
        self._connected = True
        LOGGER.info("MockCameraReader đã mở (%dx%d)", self.width, self.height)

    def read(self) -> Optional[CameraFrame]:
        with self._lock:
            if not self._connected:
                self.open()
            image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            image[:] = (35, 35, 35)
            cy = self.height // 2
            cx = self.width // 2
            target_box = (
                cx - 120 + int(60 * np.sin(self.frame_id * 0.1)),
                cy - 130,
                cx + 120 + int(60 * np.sin(self.frame_id * 0.1)),
                cy + 170,
            )
            side_box = (
                self.width - 280,
                cy - 90,
                self.width - 100,
                cy + 110,
            )
            _draw_moving_box(image, self.frame_id, target_box, side_box)
            time.sleep(self.period)
            item = CameraFrame(
                image=image,
                timestamp=time.monotonic(),
                frame_id=self.frame_id,
            )
            self.frame_id += 1
            return item

    def is_connected(self) -> bool:
        return self._connected

    def close(self) -> None:
        self._connected = False
        LOGGER.info("MockCameraReader đã đóng")


class CameraReader(_BaseCamera):
    """Open a real camera index or a video file (mp4/avi/...)."""

    name = "camera"

    def __init__(
        self,
        source,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        loop_video: bool = False,
        reconnect_seconds: float = 2.0,
        max_reconnect_attempts: int = 3,
        usb_mjpeg: bool = False,
    ) -> None:
        self.source = source
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.loop_video = bool(loop_video)
        self.reconnect_seconds = float(reconnect_seconds)
        self.max_reconnect_attempts = int(max_reconnect_attempts)
        self._reconnect_attempts = 0
        self.capture = None
        self._video_path: Optional[Path] = None
        self.frame_id = 0
        self._lock = threading.Lock()
        self._warned = False
        self.usb_mjpeg = bool(usb_mjpeg)

    def open(self) -> None:
        if not _HAVE_CV2:
            raise RuntimeError(
                "OpenCV (cv2) chưa được cài — không thể mở camera/video thật"
            )
        if isinstance(self.source, str) and not self.source.isdigit():
            path = Path(self.source)
            if not path.exists():
                raise FileNotFoundError(f"Không tìm thấy video: {path}")
            self._video_path = path
            self.capture = cv2.VideoCapture(str(path))
        else:
            idx = int(self.source)
            backend = cv2.CAP_V4L2 if self.usb_mjpeg and hasattr(cv2, "CAP_V4L2") else cv2.CAP_ANY
            self.capture = cv2.VideoCapture(idx, backend)
        if self.capture is None or not self.capture.isOpened():
            raise RuntimeError(f"Không mở được camera/video: {self.source}")
        if self.usb_mjpeg:
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps > 0:
            self.capture.set(cv2.CAP_PROP_FPS, float(self.fps))
        LOGGER.info(
            "Đã mở nguồn camera/video: %s (loop=%s)",
            self.source,
            self.loop_video,
        )

    def _try_reopen(self) -> None:
        """Best-effort reopen after a read failure (e.g. USB camera unplugged)."""
        if self.reconnect_seconds <= 0 or self._reconnect_attempts >= self.max_reconnect_attempts:
            return
        self._reconnect_attempts += 1
        if not self._warned:
            LOGGER.warning(
                "Camera mất tín hiệu, thử kết nối lại (%d/%d)...",
                self._reconnect_attempts,
                self.max_reconnect_attempts,
            )
            self._warned = True
        try:
            if self.capture is not None:
                self.capture.release()
        except Exception:  # pragma: no cover
            pass
        time.sleep(self.reconnect_seconds)
        try:
            self.open()
            self._reconnect_attempts = 0
            self._warned = False
        except Exception as exc:  # pragma: no cover - network dependent
            LOGGER.debug("Reopen thất bại: %s", exc)

    def read(self) -> Optional[CameraFrame]:
        with self._lock:
            if self.capture is None:
                try:
                    self.open()
                except Exception as exc:
                    if not self._warned:
                        LOGGER.warning("Không thể mở camera (%s): %s", self.source, exc)
                        self._warned = True
                    return None
            ok, image = self.capture.read()
            if not ok:
                if self.loop_video and self._video_path is not None:
                    self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, image = self.capture.read()
                if not ok:
                    self._try_reopen()
                    return None
            item = CameraFrame(
                image=image,
                timestamp=time.monotonic(),
                frame_id=self.frame_id,
            )
            self.frame_id += 1
            return item

    def is_connected(self) -> bool:
        return self.capture is not None and self.capture.isOpened()

    def close(self) -> None:
        with self._lock:
            if self.capture is not None:
                try:
                    self.capture.release()
                except Exception:  # pragma: no cover
                    pass
                self.capture = None
        LOGGER.info("CameraReader đã đóng")


class MultiCameraReader(_BaseCamera):
    """Continuously capture multiple cameras and expose the selected view.

    Every physical device has its own acquisition thread, so switching does
    not display old buffered USB frames.  Selection affects the HMI/process
    view only; it never changes radar state.
    """

    name = "multi-camera"
    CAMERA_IDS = ("front", "rear", "left", "right", "left_1", "left_2", "right_1", "right_2")

    def __init__(
        self,
        sources: Mapping[str, Any],
        *,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        reconnect_seconds: float = 2.0,
        required: tuple[str, ...] = ("front",),
        shared_usb_hub: bool = False,
        hub_keep_open: bool = False,
    ) -> None:
        clean = {str(k).lower(): v for k, v in sources.items() if str(k).lower() in self.CAMERA_IDS}
        missing = [name for name in required if name not in clean]
        if missing:
            raise ValueError(f"Thiếu nguồn camera bắt buộc: {', '.join(missing)}")
        self.sources = clean
        self.required = tuple(required)
        self.shared_usb_hub = bool(shared_usb_hub)
        self.hub_keep_open = bool(hub_keep_open)
        self._multiplex_opened = False
        self._multiplex_frames: dict[str, CameraFrame] = {}
        self._readers = {}
        for name, source_config in clean.items():
            profile = source_config if isinstance(source_config, Mapping) else {}
            source = profile.get("device", profile.get("source")) if profile else source_config
            if source in (None, ""):
                raise ValueError(f"Nguồn camera {name} không được để trống")
            self._readers[name] = CameraReader(
                source=source,
                width=352 if self.shared_usb_hub else int(profile.get("width", width)),
                height=288 if self.shared_usb_hub else int(profile.get("height", height)),
                fps=int(profile.get("fps", fps)),
                reconnect_seconds=float(profile.get("reconnect_seconds", reconnect_seconds)),
                usb_mjpeg=self.shared_usb_hub,
            )
        self._workers: dict[str, Any] = {}
        self._sequences = {name: 0 for name in clean}
        self._display_sequences = {name: 0 for name in clean}
        self._view_sequences = {name: 0 for name in clean}
        self._selected = "front"
        self._selection_lock = threading.Lock()

    @property
    def active_camera(self) -> str:
        with self._selection_lock:
            return self._selected

    def select(self, camera_id: str) -> bool:
        camera_id = str(camera_id).lower()
        if camera_id not in self._readers:
            return False
        with self._selection_lock:
            self._selected = camera_id
        return True

    def open(self) -> None:
        from .async_reader import AsyncFrameReader

        if self.shared_usb_hub:
            self._multiplex_opened = True
            LOGGER.info("MultiCameraReader dùng đọc luân phiên cho shared USB hub")
            return
        if self._workers:
            return
        opened: list[CameraReader] = []
        try:
            for camera_id, reader in self._readers.items():
                reader.open()
                opened.append(reader)
                worker = AsyncFrameReader(reader, f"camera-{camera_id}")
                worker.start()
                self._workers[camera_id] = worker
        except Exception:
            for worker in self._workers.values():
                worker.stop()
            self._workers.clear()
            for reader in opened:
                reader.close()
            raise
        LOGGER.info("MultiCameraReader mở camera: %s", sorted(self._readers))

    def _read_multiplexed(self, camera_id: str) -> Optional[CameraFrame]:
        """Round-robin capture without open/close churn when configured.

        ``hub_keep_open=true`` keeps each UVC handle open and skips grab
        failures cheaply instead of releasing the device every frame.
        Default (False) preserves the old close-after-read behaviour.
        """
        reader = self._readers.get(camera_id)
        if reader is None:
            return None
        keep_open = bool(getattr(self, "hub_keep_open", False))
        try:
            frame = reader.read()
        finally:
            if not keep_open:
                reader.close()
        if frame is not None:
            frame.camera_id = camera_id
            self._multiplex_frames[camera_id] = frame
            return frame
        return self._multiplex_frames.get(camera_id) if keep_open else None

    def read(self) -> Optional[CameraFrame]:
        """Return the front camera used by perception/fusion.

        Radar selection deliberately does not change this stream because the
        LiDAR calibration belongs to the front camera.
        """
        if self.shared_usb_hub:
            if not self._multiplex_opened:
                self.open()
            return self._read_multiplexed("front")
        if not self._workers:
            self.open()
        camera_id = "front"
        worker = self._workers.get(camera_id)
        if worker is None:
            return None
        seq, frame = worker.get_latest(self._sequences[camera_id], timeout=0.1)
        if frame is None:
            return None
        self._sequences[camera_id] = seq
        frame.camera_id = camera_id
        return frame

    def read_display(self) -> Optional[CameraFrame]:
        """Return the newest frame for the HMI-selected camera."""
        if self.shared_usb_hub:
            if not self._multiplex_opened:
                self.open()
            return self._read_multiplexed(self.active_camera) or self._multiplex_frames.get(self.active_camera)
        if not self._workers:
            return None
        camera_id = self.active_camera
        worker = self._workers.get(camera_id)
        if worker is None:
            return None
        seq, frame = worker.get_latest(self._display_sequences[camera_id], timeout=0.0)
        if frame is None:
            return None
        self._display_sequences[camera_id] = seq
        frame.camera_id = camera_id
        return frame

    def read_views(self) -> dict[str, CameraFrame]:
        """Return newly captured frames for every physical camera.

        This is a non-blocking HMI path.  It has independent sequence cursors,
        so publishing the 360-degree view never consumes perception or active
        display frames.
        """
        if self.shared_usb_hub:
            if not self._multiplex_opened:
                self.open()
            for camera_id in self._readers:
                self._read_multiplexed(camera_id)
            return dict(self._multiplex_frames)
        if not self._workers:
            return {}
        frames: dict[str, CameraFrame] = {}
        for camera_id, worker in self._workers.items():
            sequence, frame = worker.get_latest(
                self._view_sequences[camera_id], timeout=0.0
            )
            if frame is None:
                continue
            self._view_sequences[camera_id] = sequence
            frame.camera_id = camera_id
            frames[camera_id] = frame
        return frames

    def is_connected(self) -> bool:
        if self.shared_usb_hub:
            return self._multiplex_opened
        reader = self._readers.get("front")
        return bool(reader and reader.is_connected())

    def connection_status(self) -> dict[str, bool]:
        if self.shared_usb_hub:
            return {name: name in self._multiplex_frames for name in self._readers}
        return {name: reader.is_connected() for name, reader in self._readers.items()}

    def close(self) -> None:
        for worker in self._workers.values():
            worker.stop()
        self._workers.clear()
        self._multiplex_opened = False
        for reader in self._readers.values():
            reader.close()


__all__ = [
    "CameraReader",
    "MultiCameraReader",
    "MockCameraReader",
    "_HAVE_CV2",
]
