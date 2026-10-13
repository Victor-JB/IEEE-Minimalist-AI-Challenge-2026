"""Drive the state machine with a fake clock: python -m unittest discover apps/lumen/tests"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen import events as E          # noqa: E402
from lumen.machine import Machine, STATES, TRANSITIONS   # noqa: E402


class HappyPath(unittest.TestCase):
    def setUp(self):
        self.entered = []
        self.m = Machine(on_enter=lambda s, m: self.entered.append(s))
        self.t = 0.0

    def ev(self, event, dt=0.1):
        self.t += dt
        return self.m.handle(event, self.t)

    def test_glance_to_lit_page_and_back(self):
        for event, expect in [(E.READY, "IDLE"), (E.FACE_SEEN, "NOTICE"), (E.FACE_CONFIRMED, "STANDBY"),
                              (E.POINT, "ACK"), (E.CLIP_DONE, "SEARCH"), (E.BOOK_FOUND, "AIM"),
                              (E.AT_POSE, "HOLD"), (E.CLOSER, "ADJUST"), (E.MOVE_DONE, "HOLD"),
                              (E.PALM, "GOODBYE")]:
            self.ev(event)
            self.assertEqual(self.m.state, expect, event)
        self.assertIn("ILLUMINATE", self.entered)          # parent entered once on the way in
        self.m.ctx["face_visible"] = True
        self.ev(E.CLIP_DONE)
        self.assertEqual(self.m.state, "STANDBY")

    def test_book_not_found(self):
        for event in (E.READY, E.FACE_SEEN, E.FACE_CONFIRMED, E.POINT, E.CLIP_DONE, E.SWEEP_DONE):
            self.ev(event)
        self.assertEqual(self.m.state, "NOT_FOUND")
        self.ev(E.CLIP_DONE)
        self.assertEqual(self.m.state, "STANDBY")

    def test_notice_false_alarm_times_out(self):
        self.ev(E.READY); self.ev(E.FACE_SEEN)
        self.assertIsNone(self.m.tick(self.t + 1.0))
        self.m.tick(self.t + 2.0)
        self.assertEqual(self.m.state, "IDLE")

    def test_checkin_timer_fires_inside_illuminate(self):
        for event in (E.READY, E.FACE_SEEN, E.FACE_CONFIRMED, E.POINT, E.CLIP_DONE, E.BOOK_FOUND, E.AT_POSE):
            self.ev(event)
        self.m.tick(self.t + 16)
        self.assertEqual(self.m.state, "CHECKIN")
        self.assertEqual(self.m.path, "ILLUMINATE/CHECKIN")

    def test_exit_works_from_any_illuminate_substate(self):
        for event in (E.READY, E.FACE_SEEN, E.FACE_CONFIRMED, E.POINT, E.CLIP_DONE, E.BOOK_FOUND, E.AT_POSE, E.BOOK_LOST):
            self.ev(event)
        self.assertEqual(self.m.state, "REACQUIRE")
        self.ev(E.PALM)
        self.assertEqual(self.m.state, "GOODBYE")

    def test_boot_times_out_to_fault_and_retries(self):
        self.m.tick(self.t + 30)
        self.assertEqual(self.m.state, "FAULT")
        self.ev(E.RECOVERED)
        self.assertEqual(self.m.state, "BOOT")
        self.ev(E.READY)
        self.assertEqual(self.m.state, "IDLE")

    def test_fault_from_anywhere(self):
        self.ev(E.READY); self.ev(E.FACE_SEEN); self.ev(E.FAULT)
        self.assertEqual(self.m.state, "FAULT")
        self.ev(E.RECOVERED)
        self.assertEqual(self.m.state, "BOOT")

    def test_ignored_events_do_nothing(self):
        self.ev(E.READY)
        self.assertIsNone(self.ev(E.CLOSER))   # no closer/further outside ILLUMINATE
        self.assertEqual(self.m.state, "IDLE")

    def test_previous_states_are_tracked_before_enter(self):
        seen = []
        m = Machine(on_enter=lambda s, m: seen.append((s, m.came_from(1), m.came_from(2))))
        m.handle(E.READY, 1.0)
        m.handle(E.NO_MOTION, 2.0)
        m.handle(E.MOTION, 50.0)
        m.handle(E.FACE_SEEN, 51.0)
        state, last, before = seen[-1]
        self.assertEqual(state, "NOTICE")
        self.assertEqual(last, ("IDLE", 50.0, 51.0, E.FACE_SEEN))
        self.assertEqual(before, ("SLEEP", 2.0, 50.0, E.MOTION))
        for _ in range(10):
            m.handle(E.FAULT, 60.0)
        self.assertLessEqual(len(m.ctx["prev"]), 4)

    def test_table_is_consistent(self):
        for (src, _), dst in TRANSITIONS.items():
            self.assertTrue(src == "*" or src in STATES, src)
            self.assertTrue(callable(dst) or dst in STATES, dst)


if __name__ == "__main__":
    unittest.main()
