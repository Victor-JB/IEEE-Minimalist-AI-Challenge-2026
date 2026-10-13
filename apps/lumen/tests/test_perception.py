"""Perception's event logic with fake detectors: crops, the book-lost confirm, the RESET palm."""
import random
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


class Scripted:
    """A detector whose output is a function of the fake clock."""
    def __init__(self, fn):
        self.fn, self.t = fn, 0.0

    def detect(self, frame, *a):
        return self.fn(self.t)

    def update(self, frame):          # as a motion gate
        return self.fn(self.t)


class NoFace:
    target = None

    def detect(self, frame, now):
        return []


def illuminate(hand_at, motion_at, seconds, fps=10, seed=0):
    """Run Perception + Machine through ILLUMINATE (HOLD with its check-ins, which end after 2 s)
    with no face ever visible. Returns (machine, times NOBODY fired)."""
    hands, gate, book = Scripted(lambda t: [[(320, 300, 0)] * 21] if hand_at(t) else []), \
        Scripted(motion_at), Scripted(lambda t: [(300, 200, 100, 80, 0.9)])
    p = Perception(face=NoFace(), hands=hands, book=book, gate=gate)
    m = Machine(start="HOLD", on_enter=lambda s, m: p.on_state_change(m.ctx), rng=random.Random(seed))
    nobody, t = [], 0.0
    while t < seconds and m.state not in ("GOODBYE",):
        t += 1 / fps
        hands.t = gate.t = book.t = t
        events = p.step(FRAME, t, m.models, m.ctx)
        nobody += [t for e in events if e == E.NOBODY]
        if m.state == "CHECKIN" and t - m.entered_at > 2.0:
            events.append(E.CLIP_DONE)                     # the look-up plan is done
        for e in events:
            m.handle(e, t)
        m.tick(t)
    return m, nobody


class Nobody(unittest.TestCase):
    """A reader has their head down: no face, ever. Only no face, no hand and no motion for
    NOBODY_S ends ILLUMINATE."""
    SPAN = C.NOBODY_S + 20

    def test_reader_with_a_hand_on_the_book_never_triggers_nobody(self):
        m, nobody = illuminate(lambda t: True, lambda t: False, self.SPAN)
        self.assertEqual(nobody, [])
        self.assertGreater(sum(tr.dst == "CHECKIN" for tr in m.history), 8)   # through many check-ins
        self.assertEqual(m.spec.parent, "ILLUMINATE")

    def test_reader_turning_pages_never_triggers_nobody(self):
        page_turn = lambda t: int(t) % 40 == 0 and t % 1 < 0.3      # a little motion every 40 s
        m, nobody = illuminate(lambda t: False, page_turn, self.SPAN)
        self.assertEqual(nobody, [])
        self.assertEqual(m.spec.parent, "ILLUMINATE")

    def test_hand_now_and_then_is_enough(self):
        m, nobody = illuminate(lambda t: int(t) % 60 < 2, lambda t: False, self.SPAN)
        self.assertEqual(nobody, [])

    def test_truly_empty_desk_ends_it_after_three_minutes(self):
        m, nobody = illuminate(lambda t: False, lambda t: False, self.SPAN)
        self.assertEqual(len(nobody), 1)
        self.assertAlmostEqual(nobody[0], C.NOBODY_S, delta=1.0)
        self.assertEqual(m.state, "GOODBYE")

    def test_reader_leaves_then_it_ends_three_minutes_later(self):
        for seed in range(5):
            m, nobody = illuminate(lambda t: t < 30, lambda t: False, C.NOBODY_S + 40, seed=seed)
            self.assertEqual(len(nobody), 1)
            # counted from the last hand seen: up to one check-in (~2 s, no hand model) before 30 s
            self.assertTrue(C.NOBODY_S - 2.5 <= nobody[0] - 30 <= C.NOBODY_S + 1.0, (seed, nobody))


def hand_at(x, y=300, size=40):
    """21 landmarks: wrist at (x, y), middle knuckle `size` above it."""
    lm = [(x, y, 0)] * 21
    lm[9] = (x, y - size, 0)
    return lm


class Wave(unittest.TestCase):
    def feed(self, xs, hz=10, t0=0.0):
        from lumen.perception.gestures import WaveDetector
        w, fired = WaveDetector(C.WAVE_REVERSALS, C.WAVE_WINDOW_S, C.WAVE_SWING), []
        for i, x in enumerate(xs):
            t = t0 + i / hz
            if w.update(x, 40, t):
                fired.append(round(t, 2))
        return fired

    def test_a_quick_wave_fires_once(self):
        xs = [300, 320, 340, 320, 300, 320, 340, 360, 340]      # right, left, right: 2 reversals in 0.8 s
        self.assertEqual(len(self.feed(xs)), 1)

    def test_jitter_and_drift_are_not_waves(self):
        self.assertEqual(self.feed([300, 305, 298, 306, 299, 304, 300, 305] * 3), [])     # < half a hand
        self.assertEqual(self.feed(list(range(200, 400, 10))), [])                        # one way only

    def test_too_slow_is_not_a_wave(self):
        xs = [300] + [340] * 13 + [300] * 13 + [340] * 3        # reversals 1.3 s apart
        self.assertEqual(self.feed(xs), [])

    def test_missing_frames_dont_break_it(self):
        xs = [300, 340, None, 300, None, 340]
        self.assertEqual(len(self.feed(xs)), 1)

    def test_perception_emits_wave_from_hand_landmarks(self):
        xs = [300, 340, 300, 340, 300]
        hands = Scripted(lambda t: [hand_at(xs[min(len(xs) - 1, int(t * 10 + 1e-6))])])
        p, ctx, events, t = Perception(hands=hands), {}, [], 0.0
        for _ in range(6):
            hands.t = t
            events += p.step(FRAME, t, {"hand": 10}, ctx)
            t += 0.1
        self.assertIn(E.WAVE, events)
        self.assertIsNotNone(p.wave_t)
        self.assertIsNone(Machine(start="STANDBY").handle(E.WAVE, 1.0))   # no transition


class GatedFaceCount(unittest.TestCase):
    def test_gated_face_runs_are_not_counted(self):
        class StillGate:
            def update(self, frame):
                return False
        calls = []

        class CountingFace(FakeFace):
            def detect(self, frame, now):
                calls.append(now)
                return []
        p = Perception(face=CountingFace(), gate=StillGate())
        p.last_motion_t = 0.0                                       # one motion at t=0, then stillness
        run(p, {"gate": 10, "face": 1}, {}, 0, 30)
        self.assertEqual(p.sched.counts.get("face", 0), len(calls))   # logged == real model calls
        self.assertLessEqual(len(calls), 2)                           # gated after 1 s of no motion


if __name__ == "__main__":
    unittest.main()
