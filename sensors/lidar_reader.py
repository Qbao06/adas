"""LiDAR readers: mock, playback (.npy/.npz/.csv), and a pluggable driver slot.

All readers share the same surface:

    connect()  -> None
    read()     -> LidarFrame | None
    is_connected() -> bool
    disconnect()  -> None

Adding a real driver means subclassing ``_BaseLidarReader`` and registering
it in ``LidarReaderFactory``. Until then the system falls back to mock and
logs a clear warning.
"""
from __future__ import annotations

import csv
import logging
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from core.types import LidarFrame

LOGGER = logging.getLogger(__name__)


class _BaseLidarReader:
    name = "base"

    def connect(self) -> None:
        raise NotImplementedError

    def read(self) -> Optional[LidarFrame]:
        raise NotImplementedError

    def is_connected(self) -> bool:
        raise NotImplementedError

    def disconnect(self) -> None:
        return None


class MockLidarReader(_BaseLidarReader):
    """Generate synthetic point clouds around a configurable list of centres."""

    name = "mock"

    def __init__(
        self,
        objects: Optional[List[List[float]]] = None,
        fps: int = 10,
        seed: int = 7,
        points_per_object: int = 40,
        jitter: tuple[float, float, float] = (0.25, 0.25, 0.35),
    ) -> None:
        self.objects = list(objects) if objects else [[12.0, 0.0, 0.0]]
        self.period = 1.0 / max(float(fps), 0.1)
        self.rng = np.random.default_rng(int(seed))
        self.points_per_object = max(1, int(points_per_object))
        self.jitter = tuple(jitter)
        self.frame_id = 0
        self._connected = False

    def connect(self) -> None:
        self._connected = True
        LOGGER.info("MockLidarReader đã kết nối với %d vật", len(self.objects))

    def read(self) -> Optional[LidarFrame]:
        if not self._connected:
            self.connect()
        clouds = []
        for center in self.objects:
            c = np.asarray(center, dtype=float)
            pts = c + self.rng.normal(
                0.0, list(self.jitter), (self.points_per_object, 3)
            )
            clouds.append(pts)
        points = (
            np.vstack(clouds).astype(float) if clouds else np.empty((0, 3), dtype=float)
        )
        time.sleep(self.period)
        item = LidarFrame(points=points, timestamp=time.monotonic(), frame_id=self.frame_id)
        self.frame_id += 1
        return item

    def is_connected(self) -> bool:
        return self._connected

    def disconnect(self) -> None:
        self._connected = False
        LOGGER.info("MockLidarReader đã ngắt kết nối")


class PlaybackLidarReader(_BaseLidarReader):
    """Replay LiDAR frames from .npy/.npz/.csv files."""

    name = "playback"

    def __init__(
        self,
        path: str | Path,
        fps: int = 10,
        playback_format: str = "auto",
        loop: bool = True,
    ) -> None:
        self.path = Path(path)
        self.period = 1.0 / max(float(fps), 0.1)
        self.format = playback_format
        self.loop = bool(loop)
        self.frames: Optional[List[np.ndarray]] = None
        self.frame_id = 0
        self._connected = False

    def _load(self) -> List[np.ndarray]:
        if not self.path.exists():
            raise FileNotFoundError(f"Không tìm thấy file playback: {self.path}")
        suffix = self.path.suffix.lower()
        fmt = self.format
        if fmt == "auto":
            if suffix == ".npy":
                fmt = "npy"
            elif suffix == ".npz":
                fmt = "npz"
            elif suffix == ".csv":
                fmt = "csv"
            else:
                fmt = "npy"
        if fmt == "npy":
            arr = np.load(self.path, allow_pickle=True)
            return _split_frames(arr)
        if fmt == "npz":
            with np.load(self.path, allow_pickle=True) as npz:
                key = "frames" if "frames" in npz.files else npz.files[0]
                arr = npz[key]
                return _split_frames(arr)
        if fmt == "csv":
            return _read_csv_frames(self.path)
        raise ValueError(f"Playback format không hỗ trợ: {fmt}")

    def connect(self) -> None:
        self.frames = self._load()
        self._connected = True
        LOGGER.info(
            "PlaybackLidarReader đã nạp %d frame từ %s", len(self.frames), self.path
        )

    def read(self) -> Optional[LidarFrame]:
        if not self._connected:
            self.connect()
        if not self.frames:
            return None
        if self.frame_id >= len(self.frames):
            if self.loop:
                self.frame_id = 0
            else:
                return None
        points = np.asarray(self.frames[self.frame_id], dtype=float)
        time.sleep(self.period)
        item = LidarFrame(
            points=points,
            timestamp=time.monotonic(),
            frame_id=self.frame_id,
        )
        self.frame_id += 1
        return item

    def is_connected(self) -> bool:
        return self._connected

    def disconnect(self) -> None:
        self._connected = False
        self.frames = None


def _split_frames(arr) -> List[np.ndarray]:
    """Convert a stacked/ndarray file into a list of (N, 3) frames."""
    arr = np.asarray(arr)
    if arr.dtype == object:
        return [np.asarray(f, dtype=float) for f in arr]
    if arr.ndim == 3 and arr.shape[-1] == 3:
        return [arr[i].astype(float) for i in range(arr.shape[0])]
    if arr.ndim == 2 and arr.shape[-1] == 3:
        # Single frame in the file.
        return [arr.astype(float)]
    raise ValueError(
        f"Playback array không hợp lệ (shape={arr.shape}, dtype={arr.dtype})"
    )


def _read_csv_frames(path: Path) -> List[np.ndarray]:
    """Read a CSV where each block separated by a blank line is one frame.

    Each non-empty line is parsed as ``x,y,z[,intensity]`` and aggregated into
    a per-frame numpy array. The function is forgiving about trailing spaces
    and comments.
    """
    frames: List[np.ndarray] = []
    current: List[List[float]] = []
    with path.open("r", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row:
                if current:
                    frames.append(np.asarray(current, dtype=float))
                    current = []
                continue
            if row[0].startswith("#"):
                continue
            try:
                values = [float(v) for v in row[:3]]
            except ValueError:
                continue
            current.append(values)
    if current:
        frames.append(np.asarray(current, dtype=float))
    if not frames:
        LOGGER.warning("CSV playback trống: %s", path)
    return frames


class LidarReaderFactory:
    """Registry of available LiDAR drivers.

    New real drivers can be registered from elsewhere without editing this
    file:

        LidarReaderFactory.register("velodyne", VelodyneDriver)
    """

    _registry: Dict[str, Callable[..., _BaseLidarReader]] = {
        "mock": lambda **kw: MockLidarReader(**kw),
        "playback": lambda **kw: PlaybackLidarReader(**kw),
    }

    @classmethod
    def register(cls, name: str, factory: Callable[..., _BaseLidarReader]) -> None:
        cls._registry[name.lower()] = factory

    @classmethod
    def available(cls) -> List[str]:
        return sorted(cls._registry)

    @classmethod
    def create(cls, name: str, *, allow_mock: bool = True, **kwargs: Any) -> _BaseLidarReader:
        key = (name or "mock").lower()
        if not allow_mock and key in {"", "mock"}:
            raise RuntimeError("LIVE từ chối MockLidarReader")
        if key not in cls._registry:
            if not allow_mock:
                raise RuntimeError(
                    f"Driver LiDAR '{name}' chưa đăng ký; available={cls.available()}"
                )
            LOGGER.warning(
                "Driver LiDAR '%s' chưa đăng ký, fallback sang mock (drivers=%s)",
                name,
                cls.available(),
            )
            key = "mock"
            # Real drivers receive ``config=``. MockLidarReader does not.
            allowed = {"objects", "fps", "seed", "points_per_object", "jitter"}
            kwargs = {k: v for k, v in kwargs.items() if k in allowed}
        return cls._registry[key](**kwargs)


__all__ = [
    "MockLidarReader",
    "PlaybackLidarReader",
    "LidarReaderFactory",
    "_BaseLidarReader",
]
