from dataclasses import dataclass, field
from typing import Optional
import numpy as np
from .enums import FusionStatus, WarningLevel


@dataclass(slots=True)
class CameraFrame:
    image: np.ndarray
    timestamp: float
    frame_id: int
    camera_id: str = "front"


@dataclass(slots=True)
class LidarFrame:
    points: np.ndarray
    timestamp: float
    frame_id: int


@dataclass(slots=True)
class Detection:
    class_id: int
    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]
    timestamp: float
    track_id: Optional[int] = None


@dataclass(slots=True)
class LidarCluster:
    cluster_id: int
    points: np.ndarray
    center: np.ndarray
    dimensions: np.ndarray
    distance: float
    timestamp: float


@dataclass(slots=True)
class ProjectedPoints:
    pixels: np.ndarray
    camera_points: np.ndarray
    lidar_points: np.ndarray
    source_indices: np.ndarray


@dataclass(slots=True)
class SynchronizedFrame:
    camera: CameraFrame
    lidar: LidarFrame
    time_difference: float


@dataclass(slots=True)
class FusedObject:
    track_id: int
    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]
    timestamp: float
    position_3d: Optional[np.ndarray] = None
    raw_distance: Optional[float] = None
    filtered_distance: Optional[float] = None
    relative_speed: Optional[float] = None
    ttc: Optional[float] = None
    lidar_point_count: int = 0
    fusion_status: FusionStatus = FusionStatus.NO_POINTS
    warning_level: WarningLevel = WarningLevel.UNKNOWN
    metadata: dict = field(default_factory=dict)
