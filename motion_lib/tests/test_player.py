"""python -m unittest discover motion_lib/tests"""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from checks import check_clip_schema  # noqa: E402
from player import list_clips, load_clip, move, play  # noqa: E402
from robot import Robot  # noqa: E402

ROBOT = Robot()


class PlayerTest(unittest.TestCase):
    def test_every_clip_file_is_well_formed(self):
        for name in list_clips():
            with self.subTest(clip=name):
                self.assertEqual(check_clip_schema(load_clip(name), ROBOT), [])

    def test_additive_clip_returns_to_any_start(self):
        clip = load_clip("nod_yes/medium")
        start = ROBOT.pose(base_yaw=30, shoulder_pitch=10)
        traj = play(clip, ROBOT, start=start)
        np.testing.assert_allclose(traj.q[0], start)
        np.testing.assert_allclose(traj.q[-1], start, atol=1e-6)

    def test_amplitude_scales_offsets_and_keeps_entry_exit(self):
        clip = load_clip("nod_yes/medium")
        p = ROBOT.index("wrist_pitch")
        base = play(clip, ROBOT, amplitude=1.0)
        half = play(clip, ROBOT, amplitude=0.5)
        np.testing.assert_allclose(half.q[:, p] - ROBOT.home[p], 0.5 * (base.q[:, p] - ROBOT.home[p]), atol=1e-9)

        sit = load_clip("relaxed_sit_down/calm")
        exit_pose = [sit["exit_pose"][n] for n in ROBOT.joint_names]
        for amplitude in (0.5, 1.4):
            traj = play(sit, ROBOT, amplitude=amplitude)
            np.testing.assert_allclose(traj.q[0], ROBOT.home, atol=1e-9)
            np.testing.assert_allclose(traj.q[-1], exit_pose, atol=1e-9)

    def test_time_scale_and_hold_change_duration(self):
        clip = load_clip("curious_head_tilt/single")
        base = play(clip, ROBOT).duration
        self.assertAlmostEqual(play(clip, ROBOT, time_scale=2.0).duration, 2 * base, delta=0.03)
        self.assertAlmostEqual(play(clip, ROBOT, hold=0.5).duration, base + 0.5, delta=0.03)

    def test_out_of_range_params_are_refused(self):
        clip = load_clip("nod_yes/medium")
        with self.assertRaises(ValueError):
            play(clip, ROBOT, amplitude=clip["params"]["amplitude"]["range"][1] + 0.5)

    def test_absolute_clip_refuses_far_start(self):
        with self.assertRaises(ValueError):
            play(load_clip("wake_up/slow_groggy"), ROBOT, start=ROBOT.home)

    def test_loop_repeats_seamlessly(self):
        clip = load_clip("dance_110/groove")
        one, three = play(clip, ROBOT), play(clip, ROBOT, repeat=3)
        self.assertAlmostEqual(three.duration, 3 * one.duration, delta=3 * one.dt)  # grid rounding per loop
        step = np.abs(np.diff(three.q, axis=0)).max()
        self.assertLess(step, ROBOT.max_vel / ROBOT.rate)  # no jump at the loop seams

    def test_move_hits_target_at_control_rate(self):
        target = ROBOT.pose(base_yaw=40)
        traj = move(ROBOT, ROBOT.home, target, 1.0)
        self.assertAlmostEqual(traj.dt, 1 / ROBOT.rate)
        np.testing.assert_allclose(traj.q[-1], target)


if __name__ == "__main__":
    unittest.main()
