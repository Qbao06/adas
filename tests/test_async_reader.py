import time

from sensors.async_reader import AsyncFrameReader


class _Source:
    def __init__(self):
        self.value = 0

    def read(self):
        self.value += 1
        time.sleep(0.002)
        return self.value


def test_async_reader_returns_newest_frame():
    worker = AsyncFrameReader(_Source(), "test")
    worker.start()
    sequence, frame = worker.get_latest(0, timeout=0.2)
    worker.stop()
    assert sequence > 0
    assert frame is not None
    assert worker.error is None


class _BrokenSource:
    def read(self):
        raise RuntimeError("sensor failed")


def test_async_reader_exposes_source_error():
    worker = AsyncFrameReader(_BrokenSource(), "broken")
    worker.start()
    for _ in range(50):
        if worker.error is not None:
            break
        time.sleep(0.002)
    worker.stop()
    assert isinstance(worker.error, RuntimeError)
