import numpy as np
import pytest

from vision.tracker import IoUTracker, bbox_iou
from core.types import Detection


def _det(x1, y1, x2, y2, cls="car", conf=0.9, ts=0.0):
    return Detection(class_id=2, class_name=cls, confidence=conf,
                     bbox=(x1, y1, x2, y2), timestamp=ts)


def test_bbox_iou_perfect_overlap():
    assert bbox_iou((0, 0, 10, 10), (0, 0, 10, 10)) == pytest.approx(1.0)


def test_bbox_iou_no_overlap():
    assert bbox_iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0


def test_bbox_iou_partial():
    iou = bbox_iou((0, 0, 10, 10), (5, 5, 15, 15))
    assert 0.1 < iou < 0.5


def test_tracker_single_detection_gets_id():
    t = IoUTracker(iou_threshold=0.3, max_age=15, min_hits=1)
    dets = [_det(10, 10, 50, 50)]
    result = t.update(dets, 0.0)
    assert len(result) == 1
    assert result[0].track_id is not None


def test_tracker_lost_detection_expires():
    t = IoUTracker(iou_threshold=0.3, max_age=2, min_hits=1)
    t.update([_det(10, 10, 50, 50)], 0.0)
    # Each frame the unmatched track ages: 1, 2, 3 → at age=3 > max_age=2 it's removed.
    t.update([], 1.0)
    t.update([], 2.0)
    t.update([], 3.0)
    assert len(t.tracks) == 0


def test_tracker_stable_id_across_frames():
    t = IoUTracker(iou_threshold=0.3, max_age=30, min_hits=1)
    id1 = t.update([_det(10, 10, 50, 50)], 0.0)[0].track_id
    id2 = t.update([_det(12, 12, 52, 52)], 0.1)[0].track_id
    assert id1 == id2


def test_tracker_new_id_for_different_object():
    t = IoUTracker(iou_threshold=0.3, max_age=30, min_hits=1)
    ids = t.update([_det(10, 10, 50, 50)], 0.0)
    ids2 = t.update([_det(200, 200, 300, 300)], 0.1)
    assert ids[0].track_id != ids2[0].track_id


def test_tracker_resets():
    t = IoUTracker()
    t.update([_det(0, 0, 10, 10)], 0.0)
    t.reset()
    assert len(t.tracks) == 0
    assert t.next_id == 1
