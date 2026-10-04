"""The eight expressive modifier rules ("sprinkles") from the sprint brief, Part D.

Intensity grows with gamma; jitter is bounded (+/-15% unless noted). Order matters:
rules that need the functional flags (task_complete, looks_at) run before rules that
split a move into pieces.
"""

import numpy as np

from modifiers.base import Rule, Segment, jitter, pose_after, seg_duration, target_pose, time_for

Y, S, E, P, W, R = "base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_yaw", "head_roll"


def _delta(seg, pose, robot):
    return target_pose(seg, pose, robot) - pose


def _full(robot, q):
    return {n: float(v) for n, v in zip(robot.joint_names, q)}


class Hesitation(Rule):
    """Uncertain move: pause, micro-retreat, pause, then commit (a little slower)."""

    name, probability, cooldown = "hesitation", 1.0, 5.0

    def triggers(self, seg, pose, ctx, robot):
        return seg.kind == "move" and seg.uncertain

    def apply(self, seg, pose, ctx, gamma, rng, robot):
        retreat = robot.safe_clip(pose - jitter(rng, 0.05 + 0.05 * gamma) * _delta(seg, pose, robot))
        return [
            Segment("wait", jitter(rng, 0.3 + 0.3 * gamma), origin=self.name, label="hesitate"),
            Segment("move", time_for(robot, retreat - pose, 0.35), target=_full(robot, retreat),
                    origin=self.name, label="micro-retreat"),
            Segment("wait", jitter(rng, 0.25 + 0.25 * gamma), origin=self.name, label="pause"),
            seg.replace(duration=max(seg.duration * (1 + 0.3 * gamma),
                                     time_for(robot, target_pose(seg, retreat, robot) - retreat, 0))),
        ]


class EffortStretch(Rule):
    """Out-of-reach target: strain at the limit a few times, then droop in defeat."""

    name, probability, cooldown = "effort_stretch", 1.0, 0.0

    def triggers(self, seg, pose, ctx, robot):
        return seg.kind == "move" and seg.out_of_reach

    def apply(self, seg, pose, ctx, gamma, rng, robot):
        reach = target_pose(seg, pose, robot)
        wanted = np.array(pose, float)
        for name, value in seg.target.items():
            wanted[robot.index(name)] = value
        pull = np.sign(wanted - reach) * (np.abs(wanted - reach) > 0.5)  # joints stopped by a limit
        give = jitter(rng, 2.0 + 2.0 * gamma)
        out = [seg]
        for _ in range(2 + int(round(gamma))):
            out += [Segment("move", 0.25, target=_full(robot, reach - pull * give), ease="cubic",
                            origin=self.name, label="strain"),
                    Segment("move", 0.25, target=_full(robot, reach), ease="cubic", origin=self.name, label="strain")]
        droop = robot.pose(reach, **{P: reach[robot.index(P)] + 10 * gamma, R: reach[robot.index(R)] + 6 * gamma})
        out += [Segment("move", 0.7, target=_full(robot, robot.safe_clip(droop)), origin=self.name, label="give up"),
                Segment("wait", jitter(rng, 0.3 + 0.4 * gamma), origin=self.name, label="give up")]
        return out


class LookBackAtUser(Rule):
    """After a task (or on an instruction): look at the user, hold, look back at the task."""

    name, probability, cooldown = "look_back_at_user", 0.9, 6.0

    def triggers(self, seg, pose, ctx, robot):
        return (seg.task_complete or seg.instruction) and seg.looks_at != "user"

    def apply(self, seg, pose, ctx, gamma, rng, robot):
        at = pose if seg.instruction else pose_after(seg, pose, robot)
        share = min(1.0, jitter(rng, 0.6 + 0.4 * gamma))
        yaw = at[robot.index(W)] + share * (ctx.user_yaw - at[robot.index(Y)] - at[robot.index(W)])
        head_pitch = at[robot.index(S)] + at[robot.index(E)] + at[robot.index(P)]
        pitch = at[robot.index(P)] + share * (ctx.user_pitch - head_pitch)
        turn = np.array([yaw - at[robot.index(W)], pitch - at[robot.index(P)]])
        look = Segment("move", time_for(robot, turn, jitter(rng, 0.5)), target={W: yaw, P: pitch}, looks_at="user",
                       origin=self.name, label="look at user")
        hold = Segment("wait", jitter(rng, 0.4 + 0.8 * gamma), origin=self.name, label="look at user")
        back = Segment("move", time_for(robot, turn, jitter(rng, 0.5)),
                       target={W: float(at[robot.index(W)]), P: float(at[robot.index(P)])},
                       looks_at="task", origin=self.name, label="look back")
        return [look, hold, back, seg] if seg.instruction else [seg, look, hold, back]


class IdleBreathe(Rule):
    """Waits of 2 s or more breathe instead of freezing."""

    name, probability, cooldown = "idle_breathe", 1.0, 0.0
    clip = "idle_breathe/calm"

    def _fit(self, duration):
        """(breaths, time_scale) that fill `duration` exactly within the validated range, or None."""
        from modifiers.base import _clip

        clip = _clip(self.clip)
        s_lo, s_hi = clip["params"]["time_scale"]["range"]
        breaths = max(1, round(duration / clip["duration_nominal"]))
        time_scale = duration / (breaths * clip["duration_nominal"])
        return (breaths, time_scale) if s_lo <= time_scale <= s_hi else None

    def triggers(self, seg, pose, ctx, robot):
        return seg.kind == "wait" and seg.duration >= 2.0 and self._fit(seg.duration) is not None

    def apply(self, seg, pose, ctx, gamma, rng, robot):
        from modifiers.base import _clip

        a_lo, a_hi = _clip(self.clip)["params"]["amplitude"]["range"]
        amplitude = float(np.clip(jitter(rng, 0.5 + 0.7 * gamma), a_lo, a_hi))
        breaths, time_scale = self._fit(seg.duration)
        return [seg.replace(kind="clip", clip=self.clip, origin=self.name, label="breathe",
                            params={"amplitude": amplitude, "time_scale": time_scale, "repeat": breaths})]


class GlanceBeforeMove(Rule):
    """Big turns: the head looks toward the target first, holds, then the body follows."""

    name, probability, cooldown = "glance_before_move", 0.9, 3.0

    def triggers(self, seg, pose, ctx, robot):
        return seg.kind == "move" and abs(_delta(seg, pose, robot)[robot.index(Y)]) >= 20

    def apply(self, seg, pose, ctx, gamma, rng, robot):
        w = robot.index(W)
        turn = _delta(seg, pose, robot)[robot.index(Y)]
        lead = np.clip(turn, -40, 40) * jitter(rng, 0.5 + 0.5 * gamma)
        glance_yaw = float(np.clip(pose[w] + lead, robot.lo[w] + robot.margin, robot.hi[w] - robot.margin))
        return [
            Segment("move", time_for(robot, glance_yaw - pose[w], jitter(rng, 0.45)), target={W: glance_yaw},
                    origin=self.name, label="glance"),
            Segment("wait", jitter(rng, 0.15 + 0.25 * gamma), origin=self.name, label="glance hold"),
            # the head unwinds as the body turns
            seg.replace(target={**seg.target, W: seg.target.get(W, float(pose[w]))},
                        duration=max(seg.duration, time_for(robot, np.array([glance_yaw - pose[w], turn]), 0))),
        ]


class AnticipationWindup(Rule):
    """Long moves start with a small counter-motion (10-20% of the move)."""

    name, probability, cooldown = "anticipation_windup", 0.8, 3.0

    def triggers(self, seg, pose, ctx, robot):
        return seg.kind == "move" and seg.duration >= 0.8 and np.abs(_delta(seg, pose, robot)).max() >= 30

    def apply(self, seg, pose, ctx, gamma, rng, robot):
        d = _delta(seg, pose, robot)
        d[np.abs(d) < 5] = 0
        windup = robot.safe_clip(pose - min(0.2, jitter(rng, 0.1 + 0.1 * gamma)) * d)
        main = seg.replace(duration=max(seg.duration, time_for(robot, target_pose(seg, windup, robot) - windup, 0)))
        return [Segment("move", time_for(robot, windup - pose, jitter(rng, 0.15 + 0.1 * gamma)),
                        target=_full(robot, windup), origin=self.name, label="windup"), main]


class OvershootSettle(Rule):
    """Arrive slightly past the target (5-10% of the move) and settle back."""

    name, probability, cooldown = "overshoot_settle", 0.8, 2.0

    def triggers(self, seg, pose, ctx, robot):
        return seg.kind == "move" and np.abs(_delta(seg, pose, robot)).max() >= 15

    def apply(self, seg, pose, ctx, gamma, rng, robot):
        target = target_pose(seg, pose, robot)
        past = robot.safe_clip(target + jitter(rng, 0.05 + 0.05 * gamma) * (target - pose))
        arrive = seg.replace(target=_full(robot, past), task_complete=False, looks_at="", label="function:overshoot",
                             duration=max(seg.duration, time_for(robot, past - pose, 0)))
        settle = Segment("move", time_for(robot, past - target, jitter(rng, 0.25 + 0.15 * gamma)),
                         target=_full(robot, target),
                         task_complete=seg.task_complete, looks_at=seg.looks_at, origin=self.name, label="settle")
        return [arrive, settle]


class LightCoupling(Rule):
    """Dim the light while looking at the user; restore it when looking back at the task."""

    name, probability, cooldown, on_inserted = "light_coupling", 1.0, 0.0, True

    def triggers(self, seg, pose, ctx, robot):
        return seg.looks_at in ("user", "task") and seg.light is None

    def apply(self, seg, pose, ctx, gamma, rng, robot):
        if seg.looks_at == "task":
            return [seg.replace(light=1.0)]
        return [seg.replace(light=float(np.clip(1 - jitter(rng, 0.4 + 0.4 * gamma), 0.1, 0.9)))]


DEFAULT_RULES = [Hesitation(), EffortStretch(), LookBackAtUser(), IdleBreathe(), GlanceBeforeMove(),
                 AnticipationWindup(), OvershootSettle(), LightCoupling()]
