"""Time-based synchronisation between camera and LiDAR streams."""
from __future__ import annotations

import logging
from collections import deque
from threading import Lock

from core.types import SynchronizedFrame

LOGGER = logging.getLogger(__name__)


class SensorSynchronizer:
    """Pair camera frames with the LiDAR frame whose timestamp is closest.

    The buffers are bounded (``buffer_size``). If ``|t_camera - t_lidar|``
    exceeds ``max_difference_ms``, we do not emit a pair and instead drop
    stale frames older than ``stale_after_ms``.
    """

    def __init__(
        self,
        max_difference_ms: float = 120.0,
        buffer_size: int = 20,
        stale_after_ms: float = 1000.0,
    ) -> None:
        self.max_diff = float(max_difference_ms) / 1000.0
        self.stale_after = float(stale_after_ms) / 1000.0
        self.cameras: deque = deque(maxlen=int(buffer_size))
        self.lidars: deque = deque(maxlen=int(buffer_size))
        self.lock = Lock()
        # O(C+L) sorted search is the default; set False for legacy scan.
        self.sorted_search = True

    def add_camera(self, frame) -> None:
        with self.lock:
            self.cameras.append(frame)

    def add_lidar(self, frame) -> None:
        with self.lock:
            self.lidars.append(frame)

    def _drop_stale(self, now: float) -> None:
        while self.cameras and now - self.cameras[0].timestamp > self.stale_after:
            self.cameras.popleft()
        while self.lidars and now - self.lidars[0].timestamp > self.stale_after:
            self.lidars.popleft()

    def get_pair(self):
        with self.lock:
            if not self.cameras or not self.lidars:
                return None
            if bool(getattr(self, "sorted_search", True)):
                best_ci, best_li, best_diff = self._closest_pair_sorted()
            else:  # legacy O(C*L) scan, kept behind a flag for safety.
                best_ci, best_li, best_diff = self._closest_pair_scan()
            if best_diff - self.max_diff > 1e-9 or best_ci < 0 or best_li < 0:
                newest = max(self.cameras[-1].timestamp, self.lidars[-1].timestamp)
                self._drop_stale(newest)
                return None
            camera = self.cameras[best_ci]
            lidar = self.lidars[best_li]
            for _ in range(best_ci + 1):
                if self.cameras:
                    self.cameras.popleft()
            for _ in range(best_li + 1):
                if self.lidars:
                    self.lidars.popleft()
            return SynchronizedFrame(camera, lidar, best_diff)

    def _closest_pair_scan(self):
        best_diff = float("inf")
        best_ci = -1
        best_li = -1
        for ci, c in enumerate(self.cameras):
            for li, l in enumerate(self.lidars):
                diff = abs(c.timestamp - l.timestamp)
                if diff < best_diff:
                    best_diff = diff
                    best_ci = ci
                    best_li = li
        return best_ci, best_li, best_diff

    def _closest_pair_sorted(self):
        """Two-pointer merge over timestamp-sorted buffers: O(C+L)."""
        import bisect

        cam_ts = [c.timestamp for c in self.cameras]
        lid_ts = [l.timestamp for l in self.lidars]
        best_diff = float("inf")
        best_ci = -1
        best_li = -1
        # Buffers append in arrival order (non-decreasing timestamps), so
        # one binary search per camera frame finds its nearest LiDAR frame.
        for ci, tc in enumerate(cam_ts):
            pos = bisect.bisect_left(lid_ts, tc)
            for li in (pos - 1, pos):
                if 0 <= li < len(lid_ts):
                    diff = abs(tc - lid_ts[li])
                    if diff < best_diff:
                        best_diff = diff
                        best_ci = ci
                        best_li = li
        return best_ci, best_li, best_diff

    def reset(self) -> None:
        with self.lock:
            self.cameras.clear()
            self.lidars.clear()


__all__ = ["SensorSynchronizer"]
