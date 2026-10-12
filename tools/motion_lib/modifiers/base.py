"""Functional trajectories and the engine that layers expressive rules onto them.

A functional trajectory is a list of Segments (moves, waits, clips) from a start pose.
Rules look at trajectory features only (move length, flags, task category), never at
perception. Each rule has a trigger, an intensity scaled by gamma, a fire probability,
a cooldown and bounded jitter; every change must pass the Part C checks or it is dropped.
"""

import dataclasses
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np

from checks import check_trajectory
from player import hold_pose, load_clip, move, play

GAMMA_PRESETS = {"function": 0.25, "social": 0.85}  # ELEGNT: expressiveness helps social tasks most


@dataclass
class Segment:
    kind: str  # "move" | "wait" | "clip"
    duration: float = 1.0  # seconds, for move and wait (a clip uses its own length)
    target: dict = field(default_factory=dict)  # move: {joint: deg}; joints not listed keep their value
    ease: str = "ease_in_out"
    clip: str = ""  # kind == "clip": clip reference, e.g. "idle_breathe/calm"
    params: dict = field(default_factory=dict)  # kind == "clip": amplitude, time_scale, hold, repeat
    steady_hold: bool = False  # e.g. holding a lighting angle: no rule may touch this segment
    uncertain: bool = False  # the planner is unsure about this move
    out_of_reach: bool = False  # the target is outside the joint ranges (the move stops at the limit)
    task_complete: bool = False  # the task is done when this segment ends
    instruction: bool = False  # an instruction was received just before this segment
    looks_at: str = ""  # "user" | "task": what the head faces when this segment ends
    light: float | None = None  # light brightness to set when this segment starts
    origin: str = "function"  # "function", or the name of the rule that inserted it
    label: str = ""

    def replace(self, **changes):
        return dataclasses.replace(self, **changes)


@dataclass
class Context:
    category: str = "function"  # "function" | "social"
    user_yaw: float = 0.0  # world yaw (deg) toward the user's face
    user_pitch: float = -15.0  # world head pitch (deg, + = down) that looks at the user's face

    @property
    def gamma(self):
        return GAMMA_PRESETS[self.category]


@lru_cache(maxsize=None)
def _clip(ref):
    return load_clip(ref)


def target_pose(seg, pose, robot):
    """Absolute pose a move ends at; out-of-range targets stop at the safe limit."""
    q = np.array(pose, float)
    for name, value in seg.target.items():
        q[robot.index(name)] = value
    return robot.safe_clip(q)


def render_segment(seg, pose, robot):
    label = seg.label or (f"function:{seg.kind}" if seg.origin == "function" else seg.origin)
    if seg.kind == "move":
        part = move(robot, pose, target_pose(seg, pose, robot), seg.duration, seg.ease, label)
    elif seg.kind == "wait":
        part = hold_pose(robot, pose, seg.duration, label)
    elif seg.kind == "clip":
        part = play(_clip(seg.clip), robot, start=pose, **seg.params)
        part.marks = [(0.0, part.duration, label)]
    else:
        raise ValueError(f"unknown segment kind {seg.kind!r}")
    if seg.light is not None:
        part.events.insert(0, {"t": 0.0, "light": seg.light})
    return part


def render(segments, robot, start):
    traj, pose = None, np.array(start, float)
    for seg in segments:
        part = render_segment(seg, pose, robot)
        traj = part if traj is None else traj.then(part)
        pose = part.q[-1]
    return traj


def pose_after(seg, pose, robot):
    return render_segment(seg, pose, robot).q[-1] if seg.kind == "clip" else (
        target_pose(seg, pose, robot) if seg.kind == "move" else np.array(pose, float))


def seg_duration(seg):
    if seg.kind != "clip":
        return seg.duration
    clip = _clip(seg.clip)
    p = seg.params
    return clip["duration_nominal"] * p.get("time_scale", 1.0) * p.get("repeat", 1) + p.get("hold", 0.0)


def time_for(robot, delta, minimum):
    """Seconds an eased move over `delta` needs to stay under 90% of the speed limit."""
    return max(minimum, 1.875 * float(np.abs(delta).max()) / (0.9 * robot.max_vel))


def jitter(rng, value, frac=0.15):
    """value +/- frac, uniformly: the only randomness rules may add to a parameter."""
    return value * (1 + rng.uniform(-frac, frac))


class Rule:
    name = "rule"
    probability = 1.0  # chance to fire once triggered (times min(1, 2 * gamma))
    cooldown = 3.0  # seconds of trajectory time between two firings of this rule
    on_inserted = False  # may it trigger on segments that other rules inserted?

    def triggers(self, seg, pose, ctx, robot):
        raise NotImplementedError

    def apply(self, seg, pose, ctx, gamma, rng, robot):
        """Replacement segments for `seg` (usually including a modified `seg`)."""
        raise NotImplementedError


def apply_rules(segments, robot, ctx, gamma, rng, start=None, rules=None):
    """(modified segments, log). gamma 0 returns the input unchanged."""
    from modifiers.rules import DEFAULT_RULES

    start = np.array(robot.home if start is None else start, float)
    segs, log = list(segments), []
    if gamma <= 0:
        return segs, log
    baseline = check_trajectory(render(segs, robot, start), robot)
    if not baseline.ok:
        return segs, [f"! functional trajectory fails checks, left unmodified: {baseline.problems[0]}"]

    for rule in DEFAULT_RULES if rules is None else rules:
        i, t, pose, last = 0, 0.0, start, -np.inf
        while i < len(segs):
            seg = segs[i]
            eligible = not seg.steady_hold and (seg.origin == "function" or rule.on_inserted)
            if eligible and t - last >= rule.cooldown and rule.triggers(seg, pose, ctx, robot):
                if rng.random() < rule.probability * min(1.0, 2 * gamma):
                    new = rule.apply(seg, pose, ctx, gamma, rng, robot)
                    candidate = segs[:i] + new + segs[i + 1:]
                    report = check_trajectory(render(candidate, robot, start), robot)
                    if report.ok:
                        segs, last = candidate, t
                        log.append(f"+ {rule.name} @ {t:.1f}s")
                        for s in new:
                            t += seg_duration(s)
                            pose = pose_after(s, pose, robot)
                        i += len(new)
                        continue
                    log.append(f"x {rule.name} @ {t:.1f}s rejected: {report.problems[0]}")
                else:
                    log.append(f"- {rule.name} @ {t:.1f}s not fired (chance)")
            t += seg_duration(seg)
            pose = pose_after(seg, pose, robot)
            i += 1
    return segs, log
