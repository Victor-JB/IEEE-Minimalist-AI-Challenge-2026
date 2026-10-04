"""python -m unittest discover motion_lib/tests"""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from checks import check_trajectory  # noqa: E402
from functional import SCENARIOS, run_scenario  # noqa: E402
from modifiers import Context, Segment, apply_rules, render  # noqa: E402
from modifiers.rules import AnticipationWindup, IdleBreathe, OvershootSettle  # noqa: E402
from robot import Robot  # noqa: E402

ROBOT = Robot()
FAR_LEFT = {"base_yaw": 60, "shoulder_pitch": 20, "elbow_pitch": 40}
FAR_RIGHT = {"base_yaw": -60, "shoulder_pitch": 20, "elbow_pitch": 40}


def run(segments, gamma=1.0, seed=0, rules=None):
    return apply_rules(segments, ROBOT, Context(), gamma, np.random.default_rng(seed), rules=rules)


class ModifierTest(unittest.TestCase):
    def test_gamma_zero_changes_nothing(self):
        segs = SCENARIOS["reach_desk_spot"]["segments"]
        out, log = run(segs, gamma=0.0)
        self.assertEqual(out, segs)
        self.assertEqual(log, [])

    def test_same_seed_same_result(self):
        segs = SCENARIOS["uncertain_move"]["segments"]
        self.assertEqual(run(segs, seed=7), run(segs, seed=7))

    def test_every_output_passes_the_checks(self):
        for name, scenario in SCENARIOS.items():
            for gamma in (0.3, 1.0):
                for seed in range(3):
                    with self.subTest(scenario=name, gamma=gamma, seed=seed):
                        traj, _ = run_scenario(scenario, ROBOT, gamma, seed)
                        self.assertTrue(check_trajectory(traj, ROBOT).ok)

    def test_steady_hold_is_never_touched(self):
        steady = Segment("move", 1.5, FAR_LEFT, steady_hold=True)
        out, _ = run([steady, Segment("wait", 4.0, steady_hold=True)])
        self.assertEqual(out, [steady, Segment("wait", 4.0, steady_hold=True)])

    def test_no_trigger_no_gesture(self):
        small = [Segment("move", 1.5, {"wrist_pitch": -55}), Segment("wait", 1.0)]  # 5 deg, short wait
        out, log = run(small)
        self.assertEqual(out, small)
        self.assertFalse([entry for entry in log if entry.startswith("+")])

    def test_cooldown_blocks_back_to_back_firing(self):
        rule = AnticipationWindup()
        rule.probability = 1.0
        quick = [Segment("move", 1.2, FAR_LEFT), Segment("move", 2.2, FAR_RIGHT)]  # second starts < 3 s later
        _, log = run(quick, rules=[rule])
        self.assertEqual(sum(entry.startswith("+") for entry in log), 1)

    def test_rules_scale_with_gamma(self):
        rule = OvershootSettle()
        rule.probability = 1.0
        seg = [Segment("move", 1.5, FAR_LEFT)]
        peaks = []
        for gamma in (0.2, 1.0):
            out, _ = run(seg, gamma=gamma, rules=[rule])
            yaw = render(out, ROBOT, ROBOT.home).q[:, ROBOT.index("base_yaw")]
            peaks.append(yaw.max() - FAR_LEFT["base_yaw"])
        self.assertLess(peaks[0], peaks[1])

    def test_idle_breathe_fills_the_wait_exactly(self):
        out, _ = run([Segment("wait", 2.5)], rules=[IdleBreathe()])
        self.assertEqual(out[0].kind, "clip")
        self.assertAlmostEqual(render(out, ROBOT, ROBOT.home).duration, 2.5, delta=0.03)


if __name__ == "__main__":
    unittest.main()
