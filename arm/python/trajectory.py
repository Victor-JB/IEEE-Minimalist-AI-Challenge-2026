"""Clip -> real-arm trajectory (shoulder, elbow, head), slowed until it respects the arm's caps.

Used by both luxoiii and emote.py. Clips are authored for the 6-joint concept robot, so their
shoulder/elbow/wrist-pitch offsets are applied around `home` on the real 3-joint arm; the clip is
slowed until every joint is under its speed cap and the acceleration cap, and the result is
clamped to arm_config.SAFE_RANGES.
"""
import numpy as np

import arm_config as C
from player import load_clip, play


class ConceptRobot:
    """The 6-joint robot the clips were authored on; just enough for play()."""
    joint_names = ["base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_yaw", "head_roll"]
    home = np.array([0, -10, 70, -60, 0, 0.0])
    rate = 50


def trajectory(ref, home, amplitude=1.0, speeds=C.SPEED_CAPS, accel=C.ACCEL_CAP,
               gains=(1, 1, 1), elbow_lift=C.ELBOW_LIFT_CAP, safe=C.SAFE_RANGES):
    """(t s, q deg (N,3), velocity deg/s (N,3), time_scale) for one clip ref like "nod_yes/medium"."""
    return _trajectory(ref, home, amplitude, speeds, accel, gains, elbow_lift, safe)[:4]


def trajectory_with_roll(ref, home, amplitude=1.0, roll_home=0.0, roll_range=C.ROLL_RANGE, **caps):
    """trajectory() plus the clip's head_roll track for the head-tilt servo: (t, q, vel,
    time_scale, roll deg (N,)), on the same time base. Roll doesn't slow the clip down: the
    servo is much faster than the steppers. caps: trajectory()'s speeds/accel/gains/... keywords."""
    t, q, vel, scale, tr = _trajectory(ref, home, amplitude, **caps)
    roll = np.clip(roll_home + tr.q[:, 5] - ConceptRobot.home[5], *roll_range)
    return t, q, vel, scale, roll


def _trajectory(ref, home, amplitude=1.0, speeds=C.SPEED_CAPS, accel=C.ACCEL_CAP,
                gains=(1, 1, 1), elbow_lift=C.ELBOW_LIFT_CAP, safe=C.SAFE_RANGES):
    clip, sim = load_clip(ref), ConceptRobot()

    def render(scale):
        # start=None: additive clips start at the concept home, absolute ones (home, wake_up,
        # go_to_sleep, ...) at their own entry pose. Either way the offsets from the concept home
        # are mapped onto the real arm around `home` below.
        return play(clip, sim, start=None, amplitude=amplitude, time_scale=scale, enforce_ranges=False)

    tr = render(1.0)
    dq = np.gradient(tr.q[:, 1:4], axis=0) * np.array(gains)
    speed = (np.abs(dq).max(0) * sim.rate / np.array(speeds)).max()
    if elbow_lift:   # straightening the elbow lifts forearm and head: it stalls above ~15 deg/s
        speed = max(speed, -dq[:, 1].min() * sim.rate / elbow_lift)
    acc = np.abs(np.gradient(dq, axis=0)).max() * sim.rate ** 2
    scale = max(1.0, speed, (acc / accel) ** 0.5)
    if scale > 1.0:
        tr = render(scale)
    q = np.array(home) + (tr.q[:, 1:4] - sim.home[1:4]) * np.array(gains)
    for j, (lo, hi) in enumerate(safe):
        q[:, j] = np.clip(q[:, j], lo, hi)
    return tr.t, q, np.gradient(q, axis=0) * sim.rate, scale, tr
