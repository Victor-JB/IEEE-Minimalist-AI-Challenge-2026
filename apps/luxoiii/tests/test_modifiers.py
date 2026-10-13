"""Ported modifier rules: each makes a plan the worker can play, inside the arm's caps."""
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from luxoiii import config as C                 # noqa: E402
from luxoiii.behavior import modifiers as M     # noqa: E402

AIM = [C.HOME[0] + 5, C.HOME[1] - 5, C.HOME[2] + 25]


def peak_speeds(plan, start):
    """Per-joint peak min-jerk speed of every goto in the plan."""
    out, at = [], list(start)
    for step in plan:
        if step[0] == "goto":
            out.append([1.875 * abs(b - a) / step[2] for a, b in zip(at, step[1])])
            at = step[1]
    return out


class Modifiers(unittest.TestCase):
    def setUp(self):
        self.rng = random.Random(3)

    def assertWithinCaps(self, plan, start):
        for speeds in peak_speeds(plan, start):
            for j, v in enumerate(speeds):
                self.assertLessEqual(v, C.SPEED_CAPS[j] + 1e-6, (j, plan))

    def test_aim_glances_with_the_head_then_winds_up_then_moves(self):
        plan = M.expressive_move(list(C.HOME), AIM, 1.5, self.rng, 0.25, "aim")
        kinds = [s[0] for s in plan]
        self.assertEqual(kinds, ["goto", "wait", "goto", "goto"])
        glance = plan[0][1]
        self.assertEqual(glance[:2], list(C.HOME[:2]))            # head only
        self.assertGreater(glance[2], C.HOME[2])                  # toward the book (head down)
        windup = plan[2][1]
        self.assertLess(windup[2], glance[2])                     # counter-motion: back up first
        self.assertEqual(plan[-1][1], AIM)
        self.assertWithinCaps(plan, C.HOME)

    def test_adjust_windup_is_small_and_opposite(self):
        start = AIM
        end = [a + d for a, d in zip(AIM, (3.0, -4.0, 1.0))]
        plan = M.expressive_move(start, end, 0.8, self.rng, 0.25, "adjust")
        self.assertEqual(len(plan), 2)
        wind = [w - a for w, a in zip(plan[0][1], start)]
        self.assertLess(wind[0], 0)                               # shoulder: opposite to +3
        self.assertGreater(wind[1], 0)                            # elbow: opposite to -4
        self.assertLessEqual(max(abs(x) for x in wind), 3.0)
        self.assertWithinCaps(plan, start)

    def test_respect_caps_slows_fast_moves_and_elbow_lift(self):
        plan = M.respect_caps([("goto", [C.HOME[0], C.HOME[1] - 30, C.HOME[2]], 0.2)], C.HOME)
        self.assertLessEqual(1.875 * 30 / plan[0][2], C.ELBOW_LIFT_CAP + 1e-6)
        slow = M.respect_caps([("goto", list(C.HOME), 5.0)], C.HOME)
        self.assertEqual(slow[0][2], 5.0)                         # never speeds up

    def test_plans_stay_in_safe_ranges(self):
        far = [C.SAFE_RANGES[0][0] - 20, 0, 0]
        for style in ("aim", "adjust", "uncertain", None):
            for step in M.expressive_move(list(C.HOME), far, 1.0, self.rng, 0.9, style):
                if step[0] == "goto":
                    for v, (lo, hi) in zip(step[1], C.SAFE_RANGES):
                        self.assertTrue(lo <= v <= hi, (style, step))

    def test_hesitation_retreats_then_commits_slower(self):
        plan = M.hesitation(list(C.HOME), AIM, 1.0, self.rng, 0.5)
        self.assertEqual([s[0] for s in plan], ["wait", "goto", "wait", "goto"])
        self.assertLess(plan[1][1][2], C.HOME[2])                 # away from the target
        self.assertGreater(plan[3][2], 1.0)

    def test_idle_breathe_fills_long_holds_only(self):
        self.assertEqual(M.idle_breathe(1.5, self.rng, 0.3), [("wait", 1.5)])
        plan = M.idle_breathe(10.0, self.rng, 0.3)
        self.assertTrue(all(s[0] == "play" and s[1] == M.BREATHE for s in plan))
        ts = plan[0][3]
        self.assertTrue(M.BREATHE_TS[0] <= ts <= M.BREATHE_TS[1])
        self.assertAlmostEqual(len(plan) * M.BREATHE_S * ts, 10.0)
        sleepy = M.idle_breathe(16.0, self.rng, 0.3, amplitude=0.3, time_scale=2.0)
        self.assertEqual(sleepy, [("play", M.BREATHE, 0.3, 2.0)] * 2)

    def test_small_moves_are_left_alone(self):
        end = [C.HOME[0] + 1, C.HOME[1], C.HOME[2] + 2]
        self.assertEqual(M.anticipation_windup(list(C.HOME), end, 1.0, self.rng, 0.5), [("goto", end, 1.0)])
        self.assertEqual(M.glance_before_move(list(C.HOME), end, 1.0, self.rng, 0.5), [("goto", end, 1.0)])



class RollPoses(unittest.TestCase):
    def test_move_time_ignores_a_roll_value(self):
        """Poses from luxoiii-geometry carry J5 roll as a 4th value; planning must not index caps with it."""
        from luxoiii.behavior import modifiers as M
        a, b = [0.0, 60.0, 5.0], [5.0, 55.0, 15.0]
        self.assertEqual(M.move_time(a + [10.0], b + [-10.0]), M.move_time(a, b))

if __name__ == "__main__":
    unittest.main()
