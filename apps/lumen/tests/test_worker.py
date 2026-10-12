"""Motion worker fault handling with a fake arm: stalls are acknowledged and survived, repeated
stalls become a FAULT, and a faulted worker reconnects when BOOT asks. No board needed."""
import random
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen import events as E                      # noqa: E402
from lumen.behavior.expressive import Expressive   # noqa: E402
from lumen.motion import worker as W               # noqa: E402


class FakeArm:
    """Stands in for ArmMotion: streams fine except on the points listed in stall_at."""
    def __init__(self, stall_at=()):
        self.stall_at, self.points, self.halts, self.closed = set(stall_at), 0, 0, False

    def stream_point(self, pose, vel=None, horizon=0.1):
        self.points += 1
        return {"state": "stalled" if self.points in self.stall_at else "streaming", "angles": pose}

    def halt(self):
        self.halts += 1

    def angles(self):
        return [0.0, 60.0, 5.0]

    def servo_status(self):
        return {"stalled_joint": "head", "stall_angle": 5.1}

    def relax(self):
        pass

    def close(self):
        self.closed = True


class WorkerFaults(unittest.TestCase):
    def setUp(self):
        self.w = W.MotionWorker(Expressive(random.Random(1)), fake=True)
        self.w.arm = FakeArm(stall_at={3})
        self.w.current = [0.0, 60.0, 5.0]

    def test_stall_is_pushed_through_without_ending_the_move(self):
        W.STALL_PAUSE_S = 0
        self.assertTrue(self.w._guard(self.w._goto, [1.0, 61.0, 6.0], 0.2))
        self.assertEqual(self.w.arm.halts, 1)
        self.assertEqual(self.w.stall_count, 1)
        self.assertEqual(self.w.current, [1.0, 61.0, 6.0])      # got there after the retry

    def test_clip_survives_a_stall(self):
        W.STALL_PAUSE_S = 0
        self.w.arm = FakeArm(stall_at={10})
        self.assertTrue(self.w._guard(self.w._play, "nod_yes/small_calm", 1.0, 1.0))
        self.assertEqual(self.w.stall_count, 1)

    def test_unpushable_stall_abandons_then_faults_when_jammed(self):
        W.STALL_PAUSE_S = 0
        self.w.arm = FakeArm(stall_at=range(1, 10**6))
        for _ in range(W.GIVE_UP_LIMIT - 1):
            self.assertIsNone(self.w._guard(self.w._goto, [1.0, 61.0, 6.0], 0.1))
        self.assertEqual(self.w.arm.halts, (W.GIVE_UP_LIMIT - 1) * (W.STALL_RETRIES + 1))
        with self.assertRaises(RuntimeError):
            self.w._guard(self.w._goto, [1.0, 61.0, 6.0], 0.1)

    def test_goto_reports_done(self):
        self.w.cmds.put(("goto", [1.0, 61.0, 6.0], 0.2, 7, E.MOVE_DONE))
        self._run_loop_briefly()
        self.assertIn((E.MOVE_DONE, 7, None), self.w.poll_events())
        self.assertEqual(self.w.base, [1.0, 61.0, 6.0])

    def test_fault_then_reconnect_on_boot(self):
        connects = []
        w = self.w
        w.fake = False

        def connect():
            connects.append(1)
            w.arm = FakeArm()
            return True
        w._connect = connect

        def boom():
            raise RuntimeError("link lost")
        w._hold_tick = boom
        w.start()
        deadline = time.monotonic() + 2
        events = []
        while time.monotonic() < deadline and not any(e[0] == E.FAULT for e in events):
            events += w.poll_events()
            time.sleep(0.01)
        self.assertTrue(any(e[0] == E.FAULT for e in events))
        self.assertIsNone(w.arm)                     # session closed after the fault
        self.assertTrue(w.is_alive())                # but the thread is still there
        del w._hold_tick
        w.connect()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and len(connects) < 2:
            time.sleep(0.01)
        w.stop()
        self.assertEqual(len(connects), 2)

    def _run_loop_briefly(self):
        """Run _loop until the command queue is empty and one hold tick passed."""
        w = self.w
        orig = w._hold_tick

        def tick():
            w._stop.set()
            orig()
        w._hold_tick = tick
        w._loop()


if __name__ == "__main__":
    unittest.main()
