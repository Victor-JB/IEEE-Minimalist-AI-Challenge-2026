"""Lumen with the locked shoulder: re-locked at the measured angle on connect, and pinned in every
streamed setpoint (a setpoint off the real angle would make the firmware push the dead motor)."""
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen.movement import worker as W              # noqa: E402
from lumen.movement.expressive import Expressive    # noqa: E402
import arm_config as A                               # noqa: E402  (on the path via lumen.paths)


class LockedShoulder(unittest.TestCase):
    def setUp(self):
        self.was = A.SHOULDER_LOCK
        A.lock_shoulder(-12.04)

    def tearDown(self):
        A.unlock_shoulder() if self.was is None else A.lock_shoulder(self.was)

    def test_connect_relocks_at_the_measured_angle(self):
        self.assertEqual(W.relock_shoulder(-13.6), -13.6)
        self.assertEqual((A.SHOULDER_LOCK, A.HOME[0], A.SAFE_RANGES[0]), (-13.6, -13.6, (-13.6, -13.6)))
        self.assertIsNone(W.relock_shoulder(-80.0))       # a nonsense reading is not trusted
        self.assertEqual(A.SHOULDER_LOCK, -13.6)

    def test_every_streamed_setpoint_holds_the_shoulder(self):
        W.relock_shoulder(-13.6)

        class Arm:
            points = []

            def stream_point(self, pose, vel=None, horizon=0.1):
                self.points.append(list(pose))
                return {"state": "streaming", "angles": pose}

            def roll(self, angle, speed=None):
                pass
        w = W.MotionWorker(Expressive(random.Random(1)), fake=True)
        w.arm, w.current = Arm(), [-13.6, 63.0, 5.0]
        w._goto([5.0, 50.0, 20.0], 0.3)                    # asks the shoulder to move: it must not
        self.assertTrue(w.arm.points)
        self.assertTrue(all(p[0] == -13.6 for p in w.arm.points))


if __name__ == "__main__":
    unittest.main()
