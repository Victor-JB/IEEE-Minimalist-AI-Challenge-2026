"""The locked shoulder (arm_config.SHOULDER_LOCK): its motor is out, the joint is fixed, the
encoder still reads. Clips fold its motion into the elbow, IK aims with elbow + head only, and every
clamp pins it."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
import arm_config as A                 # noqa: E402
import kinematics as K                 # noqa: E402
from trajectory import fold_locked_shoulder, trajectory   # noqa: E402

LOCK = -12.04


class Locked(unittest.TestCase):
    def setUp(self):
        self.was = A.SHOULDER_LOCK
        A.lock_shoulder(LOCK)

    def tearDown(self):
        A.unlock_shoulder() if self.was is None else A.lock_shoulder(self.was)

    def test_lock_pins_home_and_range_and_unlock_restores(self):
        A.lock_shoulder(-15.0)
        self.assertEqual((A.HOME[0], A.SAFE_RANGES[0], A.SHOULDER_LOCK), (-15.0, (-15.0, -15.0), -15.0))
        A.unlock_shoulder()
        self.assertEqual((A.SAFE_RANGES[0], A.SHOULDER_LOCK), (A.SHOULDER_FREE_RANGE, None))

    def test_fold_keeps_forearm_and_head_world_angles(self):
        delta = np.array([[5.0, -3.0, 2.0], [-4.0, 1.0, -6.0]])
        f = fold_locked_shoulder(delta)
        np.testing.assert_allclose(f[:, 0], 0.0)
        np.testing.assert_allclose(f[:, 0] + f[:, 1], delta[:, 0] + delta[:, 1])           # forearm
        np.testing.assert_allclose(f.sum(1), delta.sum(1))                                    # head / beam

    def test_fold_shrinks_the_shoulder_share_to_keep_the_elbow_in_range(self):
        home = [LOCK, 70.0, 0.0]
        f = fold_locked_shoulder(np.array([[0.0, 0.0, 0.0], [20.0, 0.0, 0.0]]), home)
        self.assertAlmostEqual(home[1] + f[1, 1], A.SAFE_RANGES[1][1])      # 7 of the 20 deg fit

    def test_every_clip_keeps_the_shoulder_still(self):
        from player import list_clips
        for ref in list_clips():
            t, q, vel, scale = trajectory(ref, A.HOME)
            self.assertTrue(np.allclose(q[:, 0], LOCK), ref)
            for j, (lo, hi) in enumerate(A.SAFE_RANGES):
                self.assertTrue(((q[:, j] >= lo - 1e-9) & (q[:, j] <= hi + 1e-9)).all(), (ref, j))

    def test_aim_hits_the_target_with_the_shoulder_fixed(self):
        hit = 0
        for x in (0.40, 0.45, 0.50, 0.55):
            for y in (-0.15, 0.0, 0.15):
                a = K.aim((x, y, 0.02))
                if a is None:
                    continue
                hit += 1
                self.assertEqual(a.q[0], LOCK)
                h = K.fk(a.q, a.roll)
                v = [a.target[i] - h.light[i] for i in range(3)]
                along = sum(v[i] * h.axis[i] for i in range(3))
                self.assertLess(math.sqrt(sum((v[i] - along * h.axis[i]) ** 2 for i in range(3))), 1e-6)
        self.assertGreaterEqual(hit, 10)                # the reachable band is ~0.40-0.55 m

    def test_adjust_further_moves_along_the_beam(self):
        a = K.aim((0.45, 0.0, 0.02))
        m, moved = K.move_along_beam(a.q, a.roll, a.target, -0.03)
        self.assertLess(moved, -0.005)                   # some of the 3 cm (the elbow is near its limit)
        self.assertEqual(m.q[0], LOCK)
        self.assertAlmostEqual(m.standoff, a.standoff - moved, places=2)


if __name__ == "__main__":
    unittest.main()
