"""Perception's event logic with fake detectors: crops, the book-lost confirm, the RESET palm."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen import config as C          # noqa: E402
from lumen import events as E          # noqa: E402
from lumen.machine import Machine      # noqa: E402
from lumen.perception import Perception, grow   # noqa: E402
from lumen.perception.gestures import OPEN_PALM   # noqa: E402

FRAME = np.zeros((480, 640, 3), np.uint8)


class FakeBook:
    def __init__(self):
        self.present, self.rois = True, []

    def detect(self, frame, roi=None):
        self.rois.append(roi)
        return [(300, 200, 100, 80, 0.9)] if self.present else []


class FakeHands:
    def __init__(self):
        self.rois = []

    def detect(self, frame, roi=None):
        self.rois.append(roi)
        return []


class Box:
    def __init__(self, x, y, w, h):
        self.x, self.y, self.w, self.h = x, y, w, h


class FakeFace:
    def __init__(self):
        self.target = Box(280, 100, 80, 80)

    def detect(self, frame, now):
        return [self.target]


def run(p, rates, ctx, t0, t1, fps=15):
    events, t = [], t0
    while t < t1:
        events += [(round(t, 2), e) for e in p.step(FRAME, t, rates, ctx)]
        t += 1 / fps
    return events


class BookLost(unittest.TestCase):
    def setUp(self):
        self.book = FakeBook()
        self.p, self.ctx = Perception(book=self.book), {}
        self.hold = {"book": 0.5}

    def test_one_missed_slow_check_is_not_lost(self):
        run(self.p, self.hold, self.ctx, 0, 4.1)
        self.book.present = False
        events = run(self.p, self.hold, self.ctx, 4.1, 4.3)       # one miss
        self.book.present = True
        events += run(self.p, self.hold, self.ctx, 4.3, 10)
        self.assertNotIn(E.BOOK_LOST, [e for _, e in events])

    def test_gone_is_lost_about_two_seconds_after_last_seen(self):
        run(self.p, self.hold, self.ctx, 0, 4.1)                    # last seen at 4.0
        self.book.present = False
        lost = [t for t, e in run(self.p, self.hold, self.ctx, 4.1, 12) if e == E.BOOK_LOST]
        self.assertEqual(len(lost), 1)
        self.assertLess(lost[0] - 4.0, C.BOOK_LOST_S + 1.0)          # was a full 0.5 Hz period late or early

    def test_book_crop_follows_the_book_and_widens_on_a_miss(self):
        run(self.p, self.hold, self.ctx, 0, 2.1)
        self.assertEqual(self.ctx["book_roi"], grow((300, 200, 100, 80), (0.5,) * 4, FRAME.shape))
        self.book.present = False
        run(self.p, self.hold, self.ctx, 2.1, 4.6)                  # the next 0.5 Hz check misses
        self.assertIsNone(self.ctx["book_roi"])


class HandCrop(unittest.TestCase):
    def test_hand_crop_is_around_and_below_the_face(self):
        hands = FakeHands()
        p, ctx = Perception(face=FakeFace(), hands=hands), {}
        run(p, {"face": 5, "hand": 10}, ctx, 0, 1)
        x, y, w, h = ctx["hand_roi"]
        self.assertLess(x, 280)
        self.assertGreater(y + h, 180 + 2 * 80)                     # reaches well below the chin
        self.assertIn(ctx["hand_roi"], hands.rois)

    def test_grow_clamps_to_the_frame(self):
        self.assertEqual(grow((0, 0, 100, 100), (1, 1, 1, 1), (480, 640)), (0, 0, 200, 200))


class Reset(unittest.TestCase):
    def test_long_palm_resets_a_latched_fault(self):
        p, ctx = Perception(hands=FakeHands()), {}
        import lumen.perception as P
        orig = P.classify
        P.classify = lambda lm: OPEN_PALM
        p.hands.detect = lambda frame, roi=None: [[(0, 0, 0)] * 21]
        try:
            events = [e for _, e in run(p, {"hand": 2}, ctx, 0, C.RESET_HOLD_S + 1)]
        finally:
            P.classify = orig
        self.assertIn(E.RESET, events)
        m = Machine(start="FAULT")
        m.handle(E.RESET, 1.0)
        self.assertEqual(m.state, "BOOT")

    def test_state_change_forgets_the_face(self):
        p, ctx = Perception(), {"face_visible": True}
        p.on_state_change(ctx)
        self.assertFalse(ctx["face_visible"])


if __name__ == "__main__":
    unittest.main()
