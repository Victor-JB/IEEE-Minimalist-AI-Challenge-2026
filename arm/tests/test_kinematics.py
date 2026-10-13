"""kinematics.py: FK against the CAD-exact MuJoCo model, IK round trips, and the reach limits."""
import json
import math
import random
import sys
import unittest
from pathlib import Path

ARM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ARM / 'python'))
import kinematics as K                              # noqa: E402
from arm_config import HOME, ROLL_RANGE, SAFE_RANGES  # noqa: E402

# (shoulder, elbow, head), roll -> lamp light position and direction from MuJoCo 3.15 on
# tools/arm_mujoco/robot_arm.xml (origin/dev c088a9d) with elegnt_demo.py's scene light, after
# mj_kinematics + mj_camlight. Over 500 random poses inside SAFE_RANGES / ROLL_RANGE the largest
# difference from fk() was 3e-16 m (position) and 7e-16 (direction).
MUJOCO_POSES = [
    ((-12.04, 63.46, 5.27), 0.0, (0.18542, 0.0, 0.39747), (0.83571, 0.0, 0.54917)),
    ((0.0, 0.0, 0.0), 0.0, (0.0, 0.0, 0.514), (0.0, 0.0, 1.0)),
    ((5.0, 70.0, 60.0), 20.0, (0.25986, -0.0171, 0.21613), (0.66446, -0.34202, -0.66446)),
    ((-20.0, 40.0, 80.0), -30.0, (0.08696, 0.025, 0.38135), (0.85287, 0.5, -0.15038)),
]


def close(a, b, tol):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


class Geometry(unittest.TestCase):
    def test_link_lengths_match_the_cad_manifest(self):
        lengths = json.loads((ARM / 'cad/model.json').read_text())['lengths']   # cm
        self.assertEqual([K.L1, K.L2, K.HEAD_PIVOT], [v / 100 for v in lengths])

    def test_fk_matches_mujoco(self):
        for q, roll, pos, d in MUJOCO_POSES:
            h = K.fk(q, roll)
            self.assertTrue(close(h.light, pos, 1e-4), (q, roll, h.light))
            self.assertTrue(close(h.axis, d, 1e-4), (q, roll, h.axis))

    def test_home_looks_forward_and_up(self):
        h = K.fk(HOME)
        self.assertGreater(h.axis[0], 0.8)
        self.assertGreater(h.axis[2], 0.5)          # ~33 deg above level: at the user, not the desk
        self.assertIsNone(K.beam_hit(HOME))

    def test_plus_roll_swings_the_beam_to_the_lamps_right(self):
        self.assertLess(K.fk((0, 60, 80), 20).axis[1], 0)

    def test_shade_frame_is_orthonormal(self):
        for phi, r in ((30, 0), (120, 25), (170, -40)):
            d, v, w = K.shade_frame(phi, r)
            for a in (d, v, w):
                self.assertAlmostEqual(K._dot(a, a), 1.0)
            self.assertAlmostEqual(K._dot(d, v), 0.0)
            self.assertAlmostEqual(K._dot(d, w), 0.0)

    def test_yaw_seam_rotates_about_vertical(self):
        a, b = K.fk(HOME), K.fk(HOME, yaw=90)
        self.assertTrue(close(b.light, (-a.light[1], a.light[0], a.light[2]), 1e-12))


class InverseKinematics(unittest.TestCase):
    def test_pose_for_round_trips_fk(self):
        rng = random.Random(3)
        n = 0
        while n < 200:
            q = tuple(rng.uniform(lo + 2, hi - 2) for lo, hi in SAFE_RANGES)
            h = K.fk(q)
            back = K.pose_for(h.pivot[0], h.pivot[2], h.phi)
            if back is None:          # the other elbow branch: not a lamp-like bend
                self.assertLess(q[1], 0)
                continue
            n += 1
            self.assertTrue(close(K.fk(back).pivot, h.pivot, 1e-9))
            self.assertAlmostEqual(K.fk(back).phi, h.phi)

    def test_aim_hits_the_target_inside_the_limits(self):
        for target in [(0.30, 0.0, 0.0), (0.40, 0.15, 0.0), (0.50, -0.25, 0.0), (0.55, 0.0, 0.02)]:
            a = K.aim(target, ref=HOME)
            self.assertIsNotNone(a, target)
            for v, (lo, hi) in zip(a.q, SAFE_RANGES):
                self.assertTrue(lo <= v <= hi, (target, a.q))
            self.assertTrue(ROLL_RANGE[0] <= a.roll <= ROLL_RANGE[1])
            hit = K.beam_hit(a.q, a.roll, target[2])
            self.assertTrue(close(hit, target, 1e-6), (target, hit))
            self.assertAlmostEqual(K.standoff_of(a.q, a.roll, target), a.standoff, places=6)
            self.assertGreaterEqual(a.standoff, K.SHADE_DEPTH - K.LIGHT_OFFSET + K.SHADE_CLEARANCE - 1e-9)

    def test_unreachable_points_return_none(self):
        for target in [(0.05, 0.0, 0.0),       # under the lamp: the elbow can't fold that far
                       (0.90, 0.0, 0.0),       # beyond reach
                       (0.35, 0.60, 0.0)]:     # too far sideways for the roll range
            self.assertIsNone(K.aim(target, ref=HOME), target)

    def test_move_along_beam_keeps_the_aim_and_clamps(self):
        target = (0.35, 0.05, 0.0)
        a = K.aim(target, ref=HOME)
        b, moved = K.move_along_beam(a.q, a.roll, target, 0.02)
        self.assertAlmostEqual(moved, 0.02)
        self.assertAlmostEqual(b.standoff, a.standoff - 0.02, places=6)
        self.assertTrue(close(K.beam_hit(b.q, b.roll), target, 1e-6))
        # with no roll the pivot moves exactly along the light axis
        c = K.aim((0.40, 0.0, 0.0), ref=HOME)
        d, moved = K.move_along_beam(c.q, c.roll, c.target, 0.02)
        axis = K.fk(c.q).axis
        step = tuple(x - y for x, y in zip(d.pivot, c.pivot))
        self.assertTrue(close(step, tuple(0.02 * v for v in axis), 1e-6), step)
        # keep pushing closer: it stops at the limits and never brings the shade onto the book
        q, r, total = a.q, a.roll, 0.0
        for _ in range(30):
            s, moved = K.move_along_beam(q, r, target, 0.03)
            q, r, total = s.q, s.roll, total + moved
        self.assertEqual(moved, 0.0)
        self.assertGreaterEqual(s.standoff, K.SHADE_DEPTH - K.LIGHT_OFFSET + K.SHADE_CLEARANCE - 1e-9)
        for v, (lo, hi) in zip(q, SAFE_RANGES):
            self.assertTrue(lo <= v <= hi)


if __name__ == '__main__':
    unittest.main()


# These tests exercise the general machinery with a driven shoulder (arm_config.SHOULDER_LOCK is
# set on the real arm while its motor is out); the locked behavior has its own tests.
_lock = None


def setUpModule():
    global _lock
    import arm_config
    _lock = arm_config.SHOULDER_LOCK
    arm_config.unlock_shoulder()


def tearDownModule():
    import arm_config
    if _lock is not None:
        arm_config.lock_shoulder(_lock)
