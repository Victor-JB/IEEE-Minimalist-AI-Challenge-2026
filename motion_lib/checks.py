"""Physical checks for a dense trajectory (sprint brief Part C).

check_trajectory flags problems; it never clamps or fixes anything.
"""

from dataclasses import dataclass, field

import mujoco
import numpy as np

from player import VALID_EASES

PENETRATION_M = 0.001  # contacts shallower than this are grazing, not collisions


@dataclass
class Report:
    problems: list = field(default_factory=list)
    stats: dict = field(default_factory=dict)

    @property
    def ok(self):
        return not self.problems


def derivatives(traj):
    dt = traj.dt
    vel = np.diff(traj.q, axis=0) / dt
    acc = np.diff(vel, axis=0) / dt
    jerk = np.diff(acc, axis=0) / dt
    return vel, acc, jerk


def first_collision(traj, robot, every=2):
    """(time, 'body_a/body_b') of the first penetrating contact, or None."""
    model = robot.model(collisions=True)
    data = mujoco.MjData(model)
    for i in range(0, len(traj.t), every):
        data.qpos[:] = np.radians(traj.q[i])
        mujoco.mj_forward(model, data)
        for c in data.contact[: data.ncon]:
            if c.dist < -PENETRATION_M:
                b1 = model.body(model.geom_bodyid[c.geom1]).name or "world"
                b2 = model.body(model.geom_bodyid[c.geom2]).name or "world"
                return traj.t[i], f"{b1}/{b2}"
    return None


def peak_torques(traj, robot):
    """Max |torque| per joint (N*m) to follow the trajectory, gravity included."""
    model = robot.model()
    data = mujoco.MjData(model)
    q = np.radians(traj.q)
    vel = np.gradient(q, traj.dt, axis=0)
    acc = np.gradient(vel, traj.dt, axis=0)
    peak = np.zeros(robot.n)
    for i in range(len(q)):
        data.qpos[:], data.qvel[:], data.qacc[:] = q[i], vel[i], acc[i]
        mujoco.mj_inverse(model, data)
        peak = np.maximum(peak, np.abs(data.qfrc_inverse))
    return peak


def check_trajectory(traj, robot, expect_end=None, end_tolerance=2.0, collisions=True,
                     torque=False, torque_headroom=0.3):
    r = Report()
    names, q = traj.joint_names, traj.q

    low, high = q.min(axis=0), q.max(axis=0)
    for j, name in enumerate(names):
        if low[j] < robot.lo[j] + robot.margin:
            r.problems.append(f"{name} reaches {low[j]:.1f} (limit {robot.lo[j]:g} + {robot.margin:g} margin)")
        if high[j] > robot.hi[j] - robot.margin:
            r.problems.append(f"{name} reaches {high[j]:.1f} (limit {robot.hi[j]:g} - {robot.margin:g} margin)")

    if len(q) >= 4:
        vel, acc, jerk = derivatives(traj)
        for label, values, limit, unit in (
            ("velocity", vel, robot.max_vel, "deg/s"),
            ("acceleration", acc, robot.max_acc, "deg/s^2"),
            ("jerk", jerk, robot.max_jerk, "deg/s^3"),
        ):
            peak = np.abs(values).max(axis=0)
            r.stats[f"peak_{label}"] = float(peak.max())
            j = int(peak.argmax())
            if peak[j] > limit:
                r.problems.append(f"{label} {peak[j]:.0f} {unit} on {names[j]} > {limit:g}")

    if collisions:
        hit = first_collision(traj, robot)
        if hit:
            r.problems.append(f"collision {hit[1]} at t={hit[0]:.2f}s")

    if expect_end is not None:
        err = np.abs(q[-1] - np.asarray(expect_end)).max()
        if err > end_tolerance:
            r.problems.append(f"ends {err:.1f} deg from the expected pose")

    if torque:
        peak = peak_torques(traj, robot)
        r.stats["peak_torque"] = {n: round(float(t), 3) for n, t in zip(names, peak)}
        for j, limit in enumerate(robot.max_torque):
            if limit and peak[j] > (1 - torque_headroom) * limit:
                r.problems.append(f"torque {peak[j]:.2f} N*m on {names[j]} leaves <{torque_headroom:.0%} headroom")
    return r


def check_clip_schema(clip, robot):
    """Problems with a clip file's structure (before any playback)."""
    p = []
    for key in ("name", "variant", "tags", "mode", "joint_mask", "keyframes", "params",
                "loopable", "duration_nominal"):
        if key not in clip:
            p.append(f"missing field '{key}'")
    if p:
        return p
    if clip["mode"] not in ("additive", "absolute"):
        p.append(f"mode must be additive or absolute, not {clip['mode']!r}")
    for name in clip["joint_mask"]:
        if name not in robot.joint_names:
            p.append(f"unknown joint {name!r}")
    kfs = clip["keyframes"]
    if not kfs or kfs[0]["t"] != 0:
        p.append("first keyframe must be at t=0")
    for k in kfs:
        if k.get("ease", "ease_in_out") not in VALID_EASES:
            p.append(f"bad ease {k['ease']!r} at t={k['t']}")
        for name in k["joints"]:
            if name not in clip["joint_mask"]:
                p.append(f"keyframe t={k['t']} moves {name}, which is not in joint_mask")
    for name in clip["joint_mask"]:
        track = [k for k in kfs if name in k["joints"]]
        times = [k["t"] for k in track]
        if not track or times[0] != 0:
            p.append(f"{name} has no keyframe at t=0")
            continue
        if any(b <= a for a, b in zip(times, times[1:])):
            p.append(f"{name} keyframe times are not increasing")
        first, last = track[0]["joints"][name], track[-1]["joints"][name]
        if clip["mode"] == "additive" and first != 0:
            p.append(f"additive clip: {name} must start at offset 0")
        if clip.get("returns_to_start") and last != 0:
            p.append(f"returns_to_start: {name} ends at offset {last}")
        if clip.get("loopable") and first != last:
            p.append(f"loopable: {name} starts at {first} but ends at {last}")
        if clip["mode"] == "absolute" and clip.get("exit_pose") and abs(last - clip["exit_pose"][name]) > 1e-6:
            p.append(f"{name} ends at {last}, exit_pose says {clip['exit_pose'][name]}")
    for key, spec in clip["params"].items():
        lo, hi = spec["range"]
        if not lo <= spec["default"] <= hi:
            p.append(f"param {key}: default {spec['default']} outside range {spec['range']}")
    return p
