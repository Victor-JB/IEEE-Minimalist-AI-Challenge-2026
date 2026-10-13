"""Expressive modifier rules, ported from tools/motion_lib/modifiers/rules.py (dev branch) to the
real 3-joint arm (shoulder, elbow, head pitch; no yaw yet).

A plan is a list of MotionWorker.seq steps: ("goto", pose, s[, clear_offset]), ("wait", s),
("play", ref, amplitude, time_scale), ("mark", i), ("rebase",). Each rule is a small function
from a move (start, end) or a hold to a plan, and plans compose by concatenation. Every plan
should end in respect_caps(), which slows any goto that would break arm_config's speed and
acceleration caps (min-jerk peaks at 1.875x the mean speed and 5.77 d/T^2).

Ported: GlanceBeforeMove (the head leads, then the body: here pitch, since there is no yaw),
AnticipationWindup (a small counter-motion first), IdleBreathe (holds >= 2 s breathe) and
Hesitation (pause, micro-retreat, pause, commit slower). LookBackAtUser lives in the director's
CHECK-IN (it needs the face); LightCoupling is the light's job.
"""
from .. import config as C

BREATHE = "idle_breathe/calm"
BREATHE_S = 4.0                 # the clip's nominal length
BREATHE_TS = (1.0, 2.0)         # validated range is 0.5-2, but the worker only ever plays slower
SPEED_MARGIN = 0.8              # plan moves at this fraction of the caps (the firmware needs headroom)
GLANCE_MIN_DEG = 8.0            # head change that earns a glance first
WINDUP_MIN_DEG = 15.0           # AIM: move size that earns a windup (dev rule: 30 on the concept robot)
WINDUP_FLOOR_DEG = 2.5          # smaller counter-moves vanish in the gear backlash (~2-2.5 deg)


def jitter(rng, x, spread=0.15):
    """x with bounded noise (+/- spread), as in motion_lib."""
    return x * rng.uniform(1 - spread, 1 + spread)


def gamma_for(energy, social=False):
    """Expressiveness: motion_lib's 0.25 (functional) / 0.85 (social), nudged by the lamp's energy."""
    base = 0.85 if social else 0.25
    return max(0.1, min(0.95, base + 0.5 * (energy - C.ENERGY_REST)))


def clamp_pose(pose):
    """Joints into SAFE_RANGES; a 4th value (the J5 roll some poses carry) passes through."""
    return [max(lo, min(hi, v)) for v, (lo, hi) in zip(pose, C.SAFE_RANGES)] + list(pose[3:])


def move_time(start, end, margin=SPEED_MARGIN):
    """Shortest min-jerk duration from start to end inside the speed, elbow-lift and accel caps."""
    t = 0.0
    for j, (a, b) in enumerate(zip(start, end)):
        d = abs(b - a)
        cap = C.SPEED_CAPS[j]
        if j == 1 and b < a:                      # elbow straightening lifts the forearm + head
            cap = min(cap, C.ELBOW_LIFT_CAP)
        t = max(t, 1.875 * d / (cap * margin), (5.77 * d / (C.ACCEL_CAP * margin)) ** 0.5)
    return t


def respect_caps(plan, start):
    """Slow every goto in the plan that would exceed the caps (never speeds one up)."""
    out, at = [], list(start)
    for step in plan:
        if step[0] == "goto":
            pose = clamp_pose(step[1])
            step = ("goto", pose, max(step[2], move_time(at, pose))) + tuple(step[3:])
            at = pose
        out.append(step)
    return out


def plan_end(plan, start):
    """Pose the arm holds after the plan's last goto."""
    at = list(start)
    for step in plan:
        if step[0] == "goto":
            at = list(step[1])
    return at


# -- rules -------------------------------------------------------------------------
def anticipation_windup(start, end, duration, rng, gamma, min_deg=WINDUP_MIN_DEG):
    """Long moves start with a small counter-motion (10-20% of the move), then the move."""
    d = [b - a for a, b in zip(start, end)]
    big = max(abs(x) for x in d)
    if big < min_deg:
        return [("goto", list(end), duration)]
    d = [x if abs(x) >= 0.2 * big else 0.0 for x in d]           # only the joints that really move
    k = min(0.2, jitter(rng, 0.1 + 0.1 * gamma))
    k = max(k, WINDUP_FLOOR_DEG / big)                           # visible past the backlash
    windup = clamp_pose([a - k * x for a, x in zip(start, d)])
    return [("goto", windup, jitter(rng, 0.15 + 0.1 * gamma)), ("goto", list(end), duration)]


def glance_before_move(start, end, duration, rng, gamma, min_deg=GLANCE_MIN_DEG):
    """Big head changes: the head looks toward the target first, holds, then the body follows."""
    turn = end[2] - start[2]
    if abs(turn) < min_deg:
        return [("goto", list(end), duration)]
    lead = max(-15.0, min(15.0, turn)) * min(1.0, jitter(rng, 0.5 + 0.5 * gamma))
    glance = clamp_pose([start[0], start[1], start[2] + lead])
    return [("goto", glance, jitter(rng, 0.45)), ("wait", jitter(rng, 0.15 + 0.25 * gamma)),
            ("goto", list(end), duration)]


def hesitation(start, end, duration, rng, gamma):
    """Uncertain move: pause, micro-retreat (5-10% back), pause, then commit a little slower."""
    k = jitter(rng, 0.05 + 0.05 * gamma)
    retreat = clamp_pose([a - k * (b - a) for a, b in zip(start, end)])
    return [("wait", jitter(rng, 0.3 + 0.3 * gamma)), ("goto", retreat, 0.35),
            ("wait", jitter(rng, 0.25 + 0.25 * gamma)), ("goto", list(end), duration * (1 + 0.3 * gamma))]


def idle_breathe(duration, rng, gamma, amplitude=None, time_scale=None):
    """A hold of 2 s or more breathes instead of freezing: whole breaths that fill `duration`.

    amplitude / time_scale override the noise (SLEEP breathes at 0.3 and 2.0). Holds shorter than
    2 s, or that no whole number of breaths can fill, stay a plain wait.
    """
    if duration < 2.0:
        return [("wait", duration)]
    if time_scale is None:
        breaths = max(1, round(duration / BREATHE_S))
        time_scale = duration / (breaths * BREATHE_S)
        if not BREATHE_TS[0] <= time_scale <= BREATHE_TS[1]:
            breaths += 1 if time_scale > BREATHE_TS[1] else -1
            if breaths < 1:
                return [("wait", duration)]
            time_scale = max(BREATHE_TS[0], min(BREATHE_TS[1], duration / (breaths * BREATHE_S)))
    else:
        breaths = max(1, round(duration / (BREATHE_S * time_scale)))
    if amplitude is None:
        amplitude = max(0.5, min(1.5, jitter(rng, 0.5 + 0.7 * gamma)))
    return [("play", BREATHE, amplitude, time_scale)] * breaths


def expressive_move(start, end, duration, rng, gamma, style):
    """A functional move dressed by the rules for its kind, inside the caps.

    style "aim": glance with the head toward the book, then the body, with a windup.
    style "adjust": a small windup (a few degrees back) before each CLOSER / FURTHER step.
    style "uncertain": hesitation.  anything else: the plain move.
    """
    if style == "aim":
        plan = glance_before_move(start, end, duration, rng, gamma)
        pre, (_, target, dur) = plan[:-1], plan[-1]
        plan = pre + anticipation_windup(plan_end(pre, start), target, dur, rng, gamma)
    elif style == "adjust":
        plan = anticipation_windup(start, end, duration, rng, gamma, min_deg=2.0)
    elif style == "uncertain":
        plan = hesitation(start, end, duration, rng, gamma)
    else:
        plan = [("goto", list(end), duration)]
    return respect_caps(plan, start)
