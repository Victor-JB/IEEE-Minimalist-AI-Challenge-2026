"""The HUD's "thinking" lines: state + timer, where it can go next, what the arm is doing."""
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen import events as E           # noqa: E402
from lumen import thinking as T         # noqa: E402
from lumen.machine import Machine       # noqa: E402


class Thinking(unittest.TestCase):
    def machine(self, *events):
        m = Machine(rng=random.Random(1))
        for i, e in enumerate(events):
            m.handle(e, 0.1 * (i + 1))
        return m

    def test_standby_lists_its_exits_but_not_the_global_fault(self):
        m = self.machine(E.READY, E.FACE_SEEN, E.FACE_CONFIRMED)
        moves = dict(T.next_moves(m))
        self.assertEqual(moves[E.POINT], "ACK")
        self.assertEqual(moves[E.FACE_LOST], "LOST")
        self.assertNotIn(E.FAULT, moves)

    def test_substate_includes_the_parents_exits(self):
        m = self.machine(E.READY, E.FACE_SEEN, E.FACE_CONFIRMED, E.POINT, E.CLIP_DONE, E.BOOK_FOUND, E.AT_POSE)
        self.assertEqual(m.path, "ILLUMINATE/HOLD")
        moves = dict(T.next_moves(m))
        self.assertEqual(moves[E.BOOK_LOST], "REACQUIRE")
        self.assertEqual(moves[E.PALM], "GOODBYE")          # from ILLUMINATE

    def test_timer_countdown_and_function_target_label(self):
        m = self.machine(E.READY, E.FACE_SEEN)              # NOTICE: 1.5 s timeout -> IDLE
        out = T.lines(m, m.entered_at + 0.5)
        self.assertIn("NOTICE", out[0])
        self.assertIn("IDLE (in 1.0s)", " ".join(out[1:]))
        g = self.machine(E.READY, E.FACE_SEEN, E.FACE_CONFIRMED, E.POINT, E.CLIP_DONE, E.BOOK_FOUND,
                         E.AT_POSE, E.PALM)
        self.assertIn("STANDBY/IDLE (CLIP_DONE)", " ".join(T.lines(g, 1.0)))

    def test_lines_wrap_and_show_the_arm(self):
        m = self.machine(E.READY, E.FACE_SEEN, E.FACE_CONFIRMED)
        out = T.lines(m, 1.0, "clip idle_breathe/alert")
        self.assertTrue(all(len(l) <= 46 for l in out[1:-1]))
        self.assertEqual(out[-1], "arm: clip idle_breathe/alert")
        self.assertEqual(T.lines(m, 1.0)[-1], "arm: hold")


class WorkerDoing(unittest.TestCase):
    def test_worker_reports_the_clip_then_hold(self):
        from lumen.behavior.expressive import Expressive
        from lumen.motion.worker import MotionWorker
        w = MotionWorker(Expressive(random.Random(1)), fake=True)
        seen = []
        w._play = lambda *a: seen.append(w.doing) or True
        w.play("nod_yes/small_calm", 1.0, 1.0, 1)
        orig = w._hold_tick
        w._hold_tick = lambda: (w._stop.set(), orig())
        w._loop()
        self.assertEqual(seen, ["clip nod_yes/small_calm"])
        self.assertEqual(w.doing, "hold")


if __name__ == "__main__":
    unittest.main()
