"""Background acquisition wrapper for blocking hardware readers."""
from __future__ import annotations

import threading
import time
from typing import Any, Optional


class AsyncFrameReader:
    """Continuously read a sensor and expose only its newest frame."""

    def __init__(self, source: Any, name: str, *, queue_size: int = 1) -> None:
        self.source = source
        self.name = name
        self.queue_size = max(1, int(queue_size))
        self._condition = threading.Condition()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame = None
        self._sequence = 0
        self.error: Optional[BaseException] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, name=f"adas-{self.name}", daemon=True
        )
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            try:
                frame = self.source.read()
            except BaseException as exc:  # propagate to orchestrator
                self.error = exc
                self._running = False
                with self._condition:
                    self._condition.notify_all()
                return
            if frame is None:
                time.sleep(0.005)
                continue
            with self._condition:
                self._frame = frame
                self._sequence += 1
                self._condition.notify_all()

    def get_latest(self, after_sequence: int, timeout: float = 0.05):
        """Return ``(sequence, frame)`` or ``(after_sequence, None)``."""
        with self._condition:
            if self._sequence <= after_sequence and self._running:
                self._condition.wait(timeout=max(0.0, float(timeout)))
            if self._sequence <= after_sequence:
                return after_sequence, None
            return self._sequence, self._frame

    def stop(self) -> None:
        self._running = False
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None


__all__ = ["AsyncFrameReader"]
