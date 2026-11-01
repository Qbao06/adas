from enum import Enum


class FusionStatus(str, Enum):
    FUSED = "fused"
    NO_POINTS = "no_points"
    INSUFFICIENT_POINTS = "insufficient_points"
    INVALID_DISTANCE = "invalid_distance"


class WarningLevel(str, Enum):
    UNKNOWN = "unknown"
    SAFE = "safe"
    CAUTION = "caution"
    DANGER = "danger"
    EMERGENCY = "emergency"


class SystemStatus(str, Enum):
    STARTING = "starting"
    OK = "ok"
    DEGRADED = "degraded"
    ERROR = "error"
    STOPPED = "stopped"

