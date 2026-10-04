"""Authoring source for the clip library.

    python motion_lib/author.py   # -> motion_lib/clips/*.json and motion_lib/poses.json

Clips are written here as code rather than hand-edited JSON, so variants share structure
and the animation principles live in reusable helpers. Edit here, re-run, then run
validate.py (which writes the validated parameter ranges back into the JSON files).

Angles are joint degrees (0 = the straight pose in sim/robot.toml). Joint aliases:
    Y base_yaw (+ = turn left)      S shoulder_pitch (+ = lean forward)
    E elbow_pitch (+ = bend/lower)  P wrist_pitch (+ = head looks down)
    W wrist_yaw (+ = head turns left)   R head_roll (head tilt)
Head pitch in the world is S + E + P, so changing P by -(dS + dE) keeps the head level.
Velocity budget: a quintic ease covering D degrees in T seconds peaks at 1.875 * D / T.
"""

import json
import shutil

from player import CLIPS_DIR
from robot import Robot

ROBOT = Robot()
JOINTS = ROBOT.joint_names
ALIASES = dict(zip("YSEPWR", JOINTS))

HOME = list(ROBOT.home)  # [0, -10, 70, -60, 0, 0]
POSES = {
    "home": HOME,
    "sit": [0, -30, 105, -75, 0, 0],         # lowered, head level
    "sleep": [0, -30, 110, -35, 0, 8],       # curled, head down and tilted
    "alert": [0, 0, 40, -45, 0, 0],          # tall, looking slightly up
    "reach": [0, 75, 0, -35, 0, 0],          # stretched forward toward the far desk
}

CATEGORIES = ("intention", "attention", "attitude", "emotion")
FAMILIES = ("kinesics-spatial", "kinesics-temporal", "proxemics-static", "proxemics-dynamic")


# --- keyframe helpers ----------------------------------------------------------------


def K(t, ease="ease_in_out", peak=False, **joints):
    """Keyframe at time t. Joints by alias (Y S E P W R)."""
    k = {"t": round(t, 3), "joints": {ALIASES[a]: round(float(v), 2) for a, v in joints.items()}, "ease": ease}
    if peak:
        k["peak"] = True
    return k


def A(t, pose, ease="ease_in_out", peak=False, **offsets):
    """Absolute keyframe: a named pose (or pose list) plus offsets by alias."""
    base = POSES[pose] if isinstance(pose, str) else pose
    values = {a: base[i] + offsets.get(a, 0) for i, a in enumerate("YSEPWR")}
    return K(t, ease, peak, **values)


def level(S=0, E=0):
    """Wrist pitch offset that keeps the head's pitch unchanged when S and E move."""
    return -(S + E)


def zeros(target):
    return {a: 0 for a in target}


def scaled(target, k):
    return {a: v * k for a, v in target.items()}


def accent(t, target, *, rise, anticip=0.0, t_anticip=0.15, overshoot=0.0, settle=0.2,
           hold=0.4, ret=0.6):
    """One expressive beat on additive offsets: anticipation (small counter-move) ->
    move with overshoot -> settle -> hold -> return. Returns (keyframes, end time)."""
    kfs = []
    if anticip:
        t += t_anticip
        kfs.append(K(t, **scaled(target, -anticip)))
    t += rise
    if overshoot:
        kfs.append(K(t, **scaled(target, 1 + overshoot)))
        t += settle
    kfs.append(K(t, peak=True, **target))
    if hold:
        t += hold
        kfs.append(K(t, **target))
    if ret:
        t += ret
        kfs.append(K(t, **zeros(target)))
    return kfs, t


def alternate(t, a, b, n, h, end=0.0):
    """Smooth oscillation a, b, a, b... (n poses, h seconds apart), then back to 0 over
    `end` seconds (default 1.5 h). Velocity peaks near 1.5 * |a - b| / h."""
    kfs = [K(t, **zeros({**a, **b}))] if t > 0 else []  # hold still until the oscillation starts
    for i in range(n):
        t += h
        kfs.append(K(t, "cubic", **(a if i % 2 == 0 else b)))
    t += end or 1.5 * h
    kfs.append(K(t, **zeros({**a, **b})))
    return kfs, t


def decaying(t, amps, n, h, decay=1.0):
    """Oscillation +A, -A*d, +A*d^2 ... (n swings) then back to 0."""
    kfs = [K(t, **zeros(amps))] if t > 0 else []
    for i in range(n):
        t += h
        kfs.append(K(t, "cubic", **{a: v * (-1) ** i * decay ** i for a, v in amps.items()}))
    t += 1.2 * h
    kfs.append(K(t, **zeros(amps)))
    return kfs, t


def lag(kfs, aliases="PWR", dt=0.08):
    """Follow-through: the listed joints arrive dt later than the rest (the head trails
    the arm). The t=0 keyframe stays put."""
    out = []
    names = {ALIASES[a] for a in aliases}
    for k in kfs:
        if k["t"] == 0:
            out.append(k)
            continue
        lead = {n: v for n, v in k["joints"].items() if n not in names}
        trail = {n: v for n, v in k["joints"].items() if n in names}
        if lead:
            out.append({**k, "joints": lead})
        if trail:
            out.append({**k, "t": round(k["t"] + dt, 3), "joints": trail, "peak": False})
    return out


# --- clip assembly ----------------------------------------------------------------------


def clip(name, variant, *, category, family, valence, arousal, kfs, description,
         mode="additive", entry=None, exit=None, end=None, events=(), loopable=False,
         amp=(0.5, 1.5), ts=(0.5, 2.0), hold=(0.0, 1.0), extra=None):
    assert category in CATEGORIES and family in FAMILIES
    kfs = sorted(kfs, key=lambda k: k["t"])
    mask = [n for n in JOINTS if any(n in k["joints"] for k in kfs)]
    entry_pose = dict(zip(JOINTS, POSES[entry] if isinstance(entry, str) else entry)) if entry else None
    exit_pose = dict(zip(JOINTS, POSES[exit] if isinstance(exit, str) else exit)) if exit else None
    end_offset = {ALIASES[a]: v for a, v in end.items()} if end else None

    # Every masked joint starts at t=0 (offset 0, or the entry pose) ...
    if kfs[0]["t"] != 0:
        kfs.insert(0, {"t": 0.0, "joints": {}, "ease": "ease_in_out"})
    for n in mask:
        if n not in kfs[0]["joints"]:
            kfs[0]["joints"][n] = entry_pose[n] if mode == "absolute" and entry_pose else 0.0
    # ... and ends on one final keyframe (merging any that share the last time) at its end value.
    T = kfs[-1]["t"]
    final = {"t": T, "joints": {}, "ease": kfs[-1]["ease"]}
    for k in [k for k in kfs if k["t"] == T]:
        final["joints"].update(k["joints"])
    kfs = [k for k in kfs if k["t"] < T] + [final]
    for n in mask:
        if mode == "absolute":
            final["joints"].setdefault(n, exit_pose[n])
        else:
            final["joints"].setdefault(n, (end_offset or {}).get(n, 0.0))

    return {
        "name": name,
        "variant": variant,
        "description": description,
        "tags": {"category": category, "family": family, "valence": valence, "arousal": arousal},
        "mode": mode,
        "joint_mask": mask,
        "keyframes": kfs,
        "events": list(events),
        "params": {
            "amplitude": {"default": 1.0, "range": list(amp)},
            "time_scale": {"default": 1.0, "range": list(ts)},
            "hold": {"default": 0.0, "range": list(hold)},
        },
        "entry_pose": entry_pose,
        "exit_pose": exit_pose,
        "returns_to_start": mode == "additive" and not end_offset,
        "end_offset": end_offset,
        "pose_tolerance_deg": 2.0,
        "loopable": loopable,
        "duration_nominal": T,
        **(extra or {}),
    }


def light(t, brightness):
    return {"t": round(t, 3), "light": brightness}


# --- the library -------------------------------------------------------------------------
# Each function returns that clip's variants. Valence -1..1, arousal 0..1.


def nod_yes():
    tags = dict(category="attitude", family="kinesics-spatial")
    calm = [K(0.4, P=12, S=1.5, peak=True), K(0.8, P=-2, S=0), K(1.15, P=0)]
    medium = [K(0.15, P=-3), K(0.45, P=15, S=2, peak=True), K(0.75, P=1, S=0), K(1.0, P=10, S=1),
              K(1.3, P=-1, S=0), K(1.55, P=0)]
    eager = [K(0.15, P=-4, S=-1), K(0.5, P=18, S=4, peak=True), K(0.8, P=0, S=0), K(1.05, P=14, S=3),
             K(1.3, P=0, S=0), K(1.53, P=10, S=2), K(1.78, P=-1, S=0), K(2.05, P=0)]
    return [
        clip("nod_yes", "small_calm", **tags, valence=0.5, arousal=0.2, kfs=lag(calm, "S", 0.06),
             description="One slow, gentle nod."),
        clip("nod_yes", "medium", **tags, valence=0.6, arousal=0.4, kfs=lag(medium, "S", 0.06),
             description="Two decaying nods with a small upward anticipation."),
        clip("nod_yes", "big_eager", **tags, valence=0.8, arousal=0.8, kfs=lag(eager, "S", 0.06),
             description="Three quick nods, the whole body joins in."),
    ]


def shake_no():
    tags = dict(category="attitude", family="kinesics-spatial")
    small, _ = decaying(0, {"W": 10}, 2, 0.3)
    medium, _ = decaying(0, {"W": 14}, 4, 0.38, decay=0.8)
    medium = [K(0.3, P=4)] + medium + [K(medium[-1]["t"], P=0)]
    big, _ = decaying(0.1, {"W": 16}, 5, 0.42, decay=0.75)
    big = [K(0.35, P=6, S=3)] + big + [K(big[-1]["t"] + 0.2, P=0, S=0)]
    return [
        clip("shake_no", "small_calm", **tags, valence=-0.3, arousal=0.2, kfs=small,
             description="A single small left-right shake."),
        clip("shake_no", "medium", **tags, valence=-0.5, arousal=0.4, kfs=medium,
             description="Two decaying shakes, head slightly lowered."),
        clip("shake_no", "big_emphatic", **tags, valence=-0.7, arousal=0.7, kfs=big,
             description="Leans in, then an emphatic shake that dies out."),
    ]


def hesitate():
    tags = dict(category="attitude", family="kinesics-temporal")
    subtle = [K(0.45, S=5, P=-5), K(0.95, S=5, P=-5), K(1.35, S=-1, P=-2, peak=True), K(1.85, S=-1, P=-2),
              K(2.45, S=0, P=0)]
    unsure = [K(0.4, S=6, P=-6), K(0.7, S=6, P=-6), K(1.05, S=1, P=-1), K(1.35, S=1, P=-1),
              K(1.75, S=8, P=-8), K(2.05, S=8, P=-8), K(2.5, S=-2, P=-3, R=5, peak=True),
              K(3.1, S=-2, P=-3, R=5), K(3.8, S=0, P=0, R=0)]
    return [
        clip("hesitate", "subtle", **tags, valence=-0.2, arousal=0.3, kfs=subtle,
             description="Starts to lean in, freezes, eases back a little, then returns."),
        clip("hesitate", "unsure", **tags, valence=-0.3, arousal=0.4, kfs=lag(unsure, "PR", 0.06),
             description="Two false starts, then a small retreat with a puzzled tilt."),
    ]


def confident_snap():
    tags = dict(category="attitude", family="kinesics-temporal")
    firm, _ = accent(0, {"P": 10}, rise=0.21, anticip=0.3, t_anticip=0.12, hold=0.4, ret=0.4)
    emphatic, _ = accent(0, {"P": 12, "S": 3}, rise=0.26, anticip=0.3, t_anticip=0.12, hold=0.5, ret=0.45)
    return [
        clip("confident_snap", "firm", **tags, valence=0.5, arousal=0.6, kfs=firm,
             description="A quick decisive downward nod that holds."),
        clip("confident_snap", "emphatic", **tags, valence=0.6, arousal=0.7, kfs=emphatic,
             description="Decisive nod with the body committing forward."),
    ]


def shrug_fail():
    tags = dict(category="attitude", family="kinesics-spatial")
    small = [K(0.35, E=-6, P=4, R=8, peak=True), K(0.85, E=-6, P=4, R=8), K(1.45, E=4, S=2, P=8, R=2),
             K(2.2, E=0, S=0, P=0, R=0)]
    big = [K(0.4, E=-9, S=-2, P=6, R=12, peak=True), K(0.95, E=-9, S=-2, P=6, R=12),
           K(1.7, E=6, S=4, P=12, R=-3), K(2.3, E=6, S=4, P=12, R=-3), K(3.1, E=0, S=0, P=0, R=0)]
    return [
        clip("shrug_fail", "small", **tags, valence=-0.3, arousal=0.4, kfs=lag(small, "PR", 0.06),
             description="Lifts and tilts the head (shrug), then slumps a little."),
        clip("shrug_fail", "big", **tags, valence=-0.4, arousal=0.5, kfs=lag(big, "PR", 0.08),
             description="Bigger shrug, then a defeated slump with the head down."),
    ]


def look_at_user_return():
    tags = dict(category="attention", family="proxemics-static")

    def look(side, hold, tilt):
        w, y = 28 * side, 10 * side
        return [K(0.45, W=w, P=-4, R=tilt), K(0.25, Y=0), K(0.85, Y=y, W=w - y + 4, peak=True),
                K(0.85 + hold, Y=y, W=w - y + 4, P=-4, R=tilt), K(1.35 + hold, W=0, P=0, R=0),
                K(1.6 + hold, Y=0)]

    return [
        clip("look_at_user_return", "quick_left", **tags, valence=0.3, arousal=0.4, kfs=look(1, 0.5, 0),
             description="Head glances left toward the user, body follows a little, then both return."),
        clip("look_at_user_return", "slow_right", **tags, valence=0.3, arousal=0.2, kfs=look(-1, 1.2, -6),
             description="A slower, longer look to the right with a soft tilt."),
    ]


def scan_room():
    tags = dict(category="attention", family="proxemics-dynamic")
    calm = [K(1.3, Y=30), K(1.7, Y=30), K(3.5, Y=-30), K(3.9, Y=-30), K(5.0, Y=0),
            K(0.6, "cubic", W=10), K(1.3, "cubic", W=4), K(2.1, "cubic", W=-10), K(3.5, "cubic", W=-4),
            K(4.2, "cubic", W=8), K(5.0, W=0), K(0.6, P=-4), K(4.4, P=-4), K(5.0, P=0)]
    alert = [K(1.0, Y=40), K(1.6, Y=40), K(3.0, Y=-40), K(3.6, Y=-40), K(4.5, Y=0),
             K(0.5, W=8), K(1.1, W=8), K(1.3, W=18), K(1.6, W=10), K(2.2, W=-8), K(2.9, W=-8),
             K(3.1, W=-18), K(3.6, W=-10), K(4.5, W=0), K(0.5, P=-6), K(4.0, P=-6), K(4.5, P=0)]
    return [
        clip("scan_room", "calm", **tags, valence=0.0, arousal=0.3, kfs=calm,
             description="Slow sweep left then right, head leading the body."),
        clip("scan_room", "alert", **tags, valence=0.0, arousal=0.6, kfs=alert,
             description="Wider sweep that pauses at each side for a quick head check."),
    ]


def double_take():
    tags = dict(category="attention", family="kinesics-temporal")
    subtle = [K(0.5, W=18), K(0.95, W=0), K(1.2, W=0), K(1.55, W=20, P=-4, S=-2, peak=True),
              K(2.15, W=20, P=-4, S=-2), K(2.85, W=0, P=0, S=0)]
    big = [K(0.5, W=20), K(0.95, W=0), K(1.25, W=0), K(1.6, W=22, Y=0, P=-6, S=-4, E=-4, peak=True),
           K(1.85, Y=8, W=16), K(2.6, Y=8, W=16, P=-6, S=-4, E=-4), K(3.4, Y=0, W=0, P=0, S=0, E=0)]
    return [
        clip("double_take", "subtle", **tags, valence=0.1, arousal=0.5, kfs=subtle,
             description="Casual glance away and back, then a quick second look."),
        clip("double_take", "big", **tags, valence=0.1, arousal=0.8, kfs=lag(big, "P", 0.05),
             description="Second look snaps harder: the body rises and turns to follow."),
    ]


def perk_up():
    tags = dict(category="attention", family="kinesics-spatial")
    small, _ = accent(0, {"S": -4, "E": -10, "P": 9}, rise=0.35, anticip=0.15, overshoot=0.08, hold=0.6, ret=0.9)
    big, _ = accent(0, {"S": -8, "E": -16, "P": 14}, rise=0.4, anticip=0.2, overshoot=0.1, hold=0.8, ret=1.1)
    return [
        clip("perk_up", "small", **tags, valence=0.3, arousal=0.6, kfs=lag(small, "P", 0.06),
             description="Rises a little and looks up, alert."),
        clip("perk_up", "big", **tags, valence=0.4, arousal=0.9, kfs=lag(big, "P", 0.08),
             description="Dips, then shoots up tall looking up; slow return."),
    ]


def glance_then_reach():
    tags = dict(category="intention", family="proxemics-dynamic")

    def reach(side):
        y, w = 35 * side, 28 * side
        kfs = [K(0.45, W=w, P=6), K(0.7, W=w, P=6), K(0.7, Y=0, S=0, E=0),
               K(1.6, Y=y, S=20, E=-15), K(1.75, W=0, P=10)]
        return kfs, {"Y": y, "S": 20, "E": -15, "P": 10}

    left, left_end = reach(1)
    right, right_end = reach(-1)
    note = "Template: amplitude scales how far it reaches. Ends at end_offset (does not return)."
    return [
        clip("glance_then_reach", "left", **tags, valence=0.2, arousal=0.4, kfs=left, end=left_end,
             description="Head glances to the target first, then the body turns and reaches. " + note),
        clip("glance_then_reach", "right", **tags, valence=0.2, arousal=0.4, kfs=right, end=right_end,
             description="Mirror of left. " + note),
    ]


def point_with_head():
    tags = dict(category="intention", family="kinesics-spatial")
    small = [K(0.45, W=20, P=4, S=4), K(0.65, W=20, P=8, S=8, peak=True), K(0.85, W=20, P=6, S=6),
             K(1.45, W=20, P=6, S=6), K(2.05, W=0, P=0, S=0)]
    big = [K(0.5, W=-25, Y=-8, P=4, S=5), K(0.7, P=10, S=10, peak=True), K(0.9, P=6, S=6),
           K(1.1, P=10, S=10), K(1.3, P=6, S=6), K(1.9, W=-25, Y=-8, P=6, S=6), K(2.6, W=0, Y=0, P=0, S=0)]
    return [
        clip("point_with_head", "left_small", **tags, valence=0.2, arousal=0.4, kfs=small,
             description="Turns left and jabs the head toward the point once."),
        clip("point_with_head", "right_big", **tags, valence=0.3, arousal=0.6, kfs=big,
             description="Turns right with the body and jabs twice."),
    ]


def lean_in_interest():
    tags = dict(category="attention", family="proxemics-dynamic")
    subtle, _ = accent(0, {"S": 12, "E": -4, "P": -2, "R": 5}, rise=1.0, overshoot=0.04, settle=0.3, hold=1.0, ret=0.9)
    deep, _ = accent(0, {"S": 20, "E": -8, "P": -2, "R": 9}, rise=1.2, overshoot=0.04, settle=0.35, hold=1.5, ret=1.1)
    return [
        clip("lean_in_interest", "subtle", **tags, valence=0.5, arousal=0.4, kfs=lag(subtle, "PR", 0.12),
             description="Leans in slowly with a slight tilt, holds, eases back."),
        clip("lean_in_interest", "deep", **tags, valence=0.6, arousal=0.5, kfs=lag(deep, "PR", 0.15),
             description="A deeper, longer lean in."),
    ]


def excited_bounce():
    tags = dict(category="emotion", family="kinesics-temporal")
    medium, _ = alternate(0, {"S": -2, "E": 8, "P": -6}, {"S": 1, "E": -4, "P": 3}, 6, 0.2)
    big, _ = alternate(0, {"S": -3, "E": 10, "P": -7, "R": 4}, {"S": 2, "E": -5, "P": 3, "R": -4}, 10, 0.2)
    return [
        clip("excited_bounce", "medium", **tags, valence=0.8, arousal=0.8, kfs=lag(medium, "P", 0.04),
             description="Three bounces, the head stays level."),
        clip("excited_bounce", "big_eager", **tags, valence=0.9, arousal=1.0, kfs=lag(big, "PR", 0.04),
             description="Five bounces with a head wiggle.", events=[light(0.0, 1.0)]),
    ]


def tail_wag():
    tags = dict(category="emotion", family="kinesics-spatial")
    small, _ = alternate(0, {"Y": 6, "W": -3.6}, {"Y": -6, "W": 3.6}, 4, 0.28)
    big, _ = alternate(0, {"Y": 10, "W": -6, "S": 3}, {"Y": -10, "W": 6, "S": 3}, 6, 0.3)
    return [
        clip("tail_wag", "small", **tags, valence=0.6, arousal=0.5, kfs=lag(small, "W", 0.06),
             description="Base wags side to side while the head counter-turns to keep looking ahead."),
        clip("tail_wag", "big", **tags, valence=0.8, arousal=0.7, kfs=lag(big, "W", 0.06),
             description="Bigger, longer wag with a slight lean in."),
    ]


def sad_droop():
    tags = dict(category="emotion", family="kinesics-spatial")
    slight, end = accent(0, {"S": 6, "E": 12, "P": 10, "R": 6}, rise=1.6, hold=1.5, ret=1.4)
    deep, end2 = accent(0, {"S": 10, "E": 20, "P": 16, "R": 10}, rise=2.0, hold=2.0, ret=1.8)
    return [
        clip("sad_droop", "slight", **tags, valence=-0.6, arousal=0.2, kfs=lag(slight, "PR", 0.15),
             events=[light(0.3, 0.6), light(end, 1.0)],
             description="Head sinks and tilts slowly, holds, recovers."),
        clip("sad_droop", "deep", **tags, valence=-0.8, arousal=0.1, kfs=lag(deep, "PR", 0.2),
             events=[light(0.3, 0.3), light(end2, 1.0)],
             description="Deep slow droop, light dims, long hold."),
    ]


def startle_recoil():
    tags = dict(category="emotion", family="kinesics-temporal")
    small, _ = accent(0, {"S": -8, "E": -6, "P": -6}, rise=0.17, overshoot=0.1, settle=0.2, hold=0.5, ret=1.2)
    big = [K(0.22, S=-13.2, E=-11, P=-13.2, R=6.6), K(0.47, S=-12, E=-10, P=-12, R=6, peak=True),
           K(1.25, S=-12, E=-10, P=-12, R=6), K(1.95, S=-5, E=-4, P=-5, R=2), K(2.35, S=-5, E=-4, P=-5, R=2),
           K(3.4, S=0, E=0, P=0, R=0)]
    return [
        clip("startle_recoil", "small", **tags, valence=-0.4, arousal=0.9, kfs=small,
             description="Quick pull back with the head up, freeze, slow return. No anticipation (reflex)."),
        clip("startle_recoil", "big", **tags, valence=-0.6, arousal=1.0, kfs=big,
             description="Bigger recoil, long freeze, then a cautious two-step return.",
             events=[light(0.0, 1.0)]),
    ]


def relaxed_sit_down():
    tags = dict(category="emotion", family="proxemics-dynamic")
    calm = [A(0, "home"), A(0.4, "home", E=-3, P=3), A(2.0, "sit", E=4, P=-4), A(2.5, "sit")]
    plop = [A(0, "home"), A(0.15, "home", E=-4, P=4), A(0.9, "sit", S=-2, E=6, P=-6),
            A(1.2, "sit", E=-2, P=2), A(1.5, "sit")]
    common = dict(mode="absolute", entry="home", exit="sit")
    return [
        clip("relaxed_sit_down", "calm", **tags, valence=0.4, arousal=0.1, kfs=lag(calm, "P", 0.12), **common,
             description="Slow settle from home into the sit pose with a little rise first."),
        clip("relaxed_sit_down", "quick_plop", **tags, valence=0.5, arousal=0.3, kfs=lag(plop, "P", 0.08), **common,
             description="Drops into the sit pose, bounces once, settles."),
    ]


def celebrate():
    tags = dict(category="emotion", family="kinesics-temporal")

    def cheer(n, roll, yaw):
        up = {"S": -6, "E": -18, "P": 14}
        kfs = [K(0.25, S=-2, E=8, P=-6), K(0.7, peak=True, **up)]
        wiggle, t = alternate(0.7, {"R": roll, "Y": yaw}, {"R": -roll, "Y": -yaw}, n, 0.3, end=0.4)
        kfs += wiggle + [K(t, **up), K(t + 0.8, **zeros(up))]
        flashes = [light(0.7 + 0.3 * i, 1.0 if i % 2 == 0 else 0.5) for i in range(n)] + [light(t, 1.0)]
        return kfs, flashes

    medium, ev1 = cheer(4, 10, 6)
    big, ev2 = cheer(6, 12, 10)
    return [
        clip("celebrate", "medium", **tags, valence=1.0, arousal=0.9, kfs=lag(medium, "P", 0.06), events=ev1,
             description="Crouch, spring up tall looking up, head and base wiggle, light flashes."),
        clip("celebrate", "big", **tags, valence=1.0, arousal=1.0, kfs=lag(big, "P", 0.06), events=ev2,
             description="Longer, wider celebration wiggle."),
    ]


def approach_object():
    tags = dict(category="intention", family="proxemics-dynamic")
    end = {"S": 18, "E": -8, "P": 10}
    cautious = [K(0.5, P=8), K(1.4, S=9, E=-4, P=6), K(1.9, S=9, E=-4, P=6), K(3.0, **end)]
    eager = [K(0.35, P=8), K(1.1, S=20, E=-9, P=11), K(1.4, **end)]
    return [
        clip("approach_object", "cautious", **tags, valence=0.2, arousal=0.3, kfs=cautious, end=end,
             description="Looks first, creeps halfway, pauses, then closes in. Ends leaned in."),
        clip("approach_object", "eager", **tags, valence=0.5, arousal=0.6, kfs=eager, end=end,
             description="Looks and moves in quickly with a small overshoot. Ends leaned in."),
    ]


def avoid_object():
    tags = dict(category="intention", family="proxemics-dynamic")
    mild_end = {"S": -10, "E": 8, "P": 2, "W": -22, "Y": -6}
    mild = [K(0.6, S=-10, E=8, P=2), K(0.6, W=0, Y=0), K(1.3, W=-22), K(1.5, Y=-6)]
    strong_end = {"S": -12, "E": 10, "P": 2, "W": -30, "Y": -10, "R": 6}
    strong = [K(0.32, S=-13.2, E=11, P=2), K(0.55, S=-12, E=10, P=2), K(0.3, W=0, Y=0, R=0),
              K(0.85, W=-30, R=6), K(1.1, Y=-10)]
    return [
        clip("avoid_object", "mild", **tags, valence=-0.3, arousal=0.4, kfs=mild, end=mild_end,
             description="Pulls back, then turns the head away. Ends turned away."),
        clip("avoid_object", "strong", **tags, valence=-0.5, arousal=0.7, kfs=strong, end=strong_end,
             description="Quick recoil and a firm turn away with a tilt. Ends turned away."),
    ]


def curious_head_tilt():
    tags = dict(category="attention", family="proxemics-static")
    single, _ = accent(0, {"R": 18, "P": -4, "W": 5}, rise=0.5, anticip=0.12, overshoot=0.08, hold=0.9, ret=0.6)
    double = [K(0.5, R=16, P=-4, W=4, peak=True), K(1.1, R=16, P=-4, W=4), K(1.85, R=-14, P=-5, W=-4),
              K(2.45, R=-14, P=-5, W=-4), K(3.1, R=0, P=0, W=0)]
    return [
        clip("curious_head_tilt", "single", **tags, valence=0.4, arousal=0.4, kfs=single,
             description="Tilts the head, slight overshoot, holds, returns."),
        clip("curious_head_tilt", "double", **tags, valence=0.5, arousal=0.5, kfs=double,
             description="Tilts one way, then the other, as if puzzling."),
    ]


def nuzzle_touch():
    tags = dict(category="emotion", family="proxemics-static")

    def nuzzle(depth, rubs, h):
        lean = {"S": depth, "E": -2, "P": 10}
        kfs = [K(1.0, **lean)]
        rub, t = alternate(1.0, {"R": 6, "W": 4}, {"R": -6, "W": -4}, rubs, h, end=0.5)
        kfs += rub + [K(t + 0.4, **lean), K(t + 1.4, **zeros(lean))]
        return kfs, t

    gentle, t1 = nuzzle(14, 2, 0.5)
    fond, t2 = nuzzle(16, 4, 0.45)
    return [
        clip("nuzzle_touch", "gentle", **tags, valence=0.8, arousal=0.3, kfs=gentle,
             events=[light(0.8, 0.6), light(t1 + 1.4, 1.0)],
             description="Leans in and gently rubs the head side to side."),
        clip("nuzzle_touch", "affectionate", **tags, valence=0.9, arousal=0.4, kfs=fond,
             events=[light(0.8, 0.5), light(t2 + 1.4, 1.0)],
             description="Deeper lean, more rubs, dimmer light."),
    ]


def stretch_reach_limit():
    tags = dict(category="intention", family="proxemics-dynamic")
    droop = [0, 55, 20, 20, 0, 8]

    def strain(n):
        kfs = [A(0, "home"), A(0.2, "home", S=-3, P=3), A(1.6, "reach")]
        t = 1.6
        for _ in range(n):
            kfs += [A(t + 0.3, "reach", "cubic", S=4, P=-4), A(t + 0.6, "reach", "cubic", S=1, P=-1)]
            t += 0.6
        kfs += [A(t + 0.3, "reach"), A(t + 1.3, droop), A(t + 1.8, droop), A(t + 3.4, "home")]
        return kfs

    common = dict(mode="absolute", entry="home", exit="home")
    return [
        clip("stretch_reach_limit", "short_try", **tags, valence=-0.2, arousal=0.6, kfs=strain(2), **common,
             description="Reaches as far as it can, strains twice, gives up with a droop, returns home."),
        clip("stretch_reach_limit", "persistent", **tags, valence=-0.4, arousal=0.7, kfs=strain(4), **common,
             description="Strains four times before giving up."),
    ]


def wake_up():
    tags = dict(category="emotion", family="kinesics-spatial")
    stretch = [0, -5, 50, -60, 0, -6]
    groggy = [A(0, "sleep"), A(0.8, "sleep", P=-6), A(1.2, "sleep", P=-2), A(2.8, stretch),
              A(3.4, stretch, R=14), A(4.6, "home")]
    bright = [A(0, "sleep"), A(0.25, "sleep", E=3, P=-2), A(1.15, "home", S=2, E=-8, P=6), A(1.45, "home"),
              A(1.65, "home", "cubic", W=6), A(1.85, "home", "cubic", W=-6), A(2.15, "home")]
    common = dict(mode="absolute", entry="sleep", exit="home")
    return [
        clip("wake_up", "slow_groggy", **tags, valence=0.2, arousal=0.3, kfs=groggy, **common,
             events=[light(0.0, 0.0), light(1.2, 0.4), light(2.8, 1.0)],
             description="Stirs, rises into a tall stretch with a head roll, settles to home."),
        clip("wake_up", "quick_bright", **tags, valence=0.6, arousal=0.7, kfs=bright, **common,
             events=[light(0.0, 0.0), light(0.8, 0.6), light(1.15, 1.0)],
             description="Springs up past home, settles, quick head shake."),
    ]


def go_to_sleep():
    tags = dict(category="emotion", family="kinesics-spatial")
    drowsy = [A(0, "home"), A(1.0, "home", P=10, S=2), A(1.4, "home", P=4), A(2.4, "home", P=14, S=3, E=6),
              A(4.2, "sleep")]
    quick = [A(0, "home"), A(1.6, "sleep", E=3, P=-3), A(2.0, "sleep")]
    common = dict(mode="absolute", entry="home", exit="sleep")
    return [
        clip("go_to_sleep", "drowsy", **tags, valence=0.1, arousal=0.0, kfs=lag(drowsy, "PR", 0.1), **common,
             events=[light(2.4, 0.5), light(4.2, 0.0)],
             description="Head nods off, catches itself, then sinks into sleep as the light fades."),
        clip("go_to_sleep", "quick", **tags, valence=0.1, arousal=0.1, kfs=quick, **common,
             events=[light(0.5, 0.5), light(2.0, 0.0)],
             description="Straight down into the sleep pose."),
    ]


def idle_breathe():
    tags = dict(category="emotion", family="kinesics-temporal")
    calm = [K(1.8, E=2.5, P=-2.5, R=0.8), K(4.0, E=0, P=0, R=0)]
    alert = [K(1.4, E=2, P=-2), K(3.0, E=0, P=0), K(4.4, E=2, P=-2), K(6.0, E=0, P=0),
             K(2.0, "cubic", W=3), K(4.5, "cubic", W=-2), K(6.0, W=0)]
    return [
        clip("idle_breathe", "calm", **tags, valence=0.1, arousal=0.1, kfs=calm, loopable=True,
             description="Subtle 4 s breathing loop; the head stays level."),
        clip("idle_breathe", "alert", **tags, valence=0.2, arousal=0.3, kfs=alert, loopable=True,
             description="Quicker breathing with a slow gaze drift (6 s loop)."),
    ]


def home():
    tags = dict(category="intention", family="proxemics-dynamic")
    common = dict(mode="absolute", entry=None, exit="home", amp=(1.0, 1.0))
    note = " Plays from any pose; far starts may need a larger time_scale."
    return [
        clip("home", "gentle", **tags, valence=0.0, arousal=0.1, kfs=[A(0, "home"), A(2.5, "home")], **common,
             description="Smooth return to home." + note),
        clip("home", "brisk", **tags, valence=0.0, arousal=0.4, kfs=[A(0, "home"), A(1.5, "home")], **common,
             description="Quicker return to home." + note),
    ]


def dance(bpm):
    tags = dict(category="emotion", family="kinesics-temporal")
    beat = 60 / bpm

    def loop(bob, sway, roll, head):
        kfs = []
        for b in range(8):  # bob down on the off-beat, back up on the beat
            kfs += [K((b + 0.5) * beat, "cubic", E=bob, P=-bob), K((b + 1) * beat, "cubic", E=0, P=0)]
        kfs += [K(2 * beat, "cubic", Y=sway), K(4 * beat, "cubic", Y=0), K(6 * beat, "cubic", Y=-sway),
                K(8 * beat, "cubic", Y=0)]
        if roll:  # head tilts against the sway
            kfs += [K(2 * beat, "cubic", R=-roll), K(4 * beat, "cubic", R=0), K(6 * beat, "cubic", R=roll),
                    K(8 * beat, "cubic", R=0)]
        if head:
            kfs += [K(2.5 * beat, "cubic", W=head), K(3.5 * beat, "cubic", W=0),
                    K(6.5 * beat, "cubic", W=-head), K(7.5 * beat, "cubic", W=0), K(8 * beat, "cubic", W=0)]
        return kfs

    extra = {"bpm": bpm, "beats": 8}
    common = dict(loopable=True, ts=(1.0, 1.0), extra=extra)
    return [
        clip(f"dance_{bpm}", "groove", **tags, valence=0.7, arousal=0.6, kfs=loop(5, 10, 5, 0), **common,
             description=f"{bpm} BPM, 2-bar loop: off-beat bob, sway each bar, head tilts."),
        clip(f"dance_{bpm}", "party", **tags, valence=0.9, arousal=0.9, kfs=loop(8, 15, 8, 10), **common,
             description=f"{bpm} BPM, 2-bar loop: bigger bob and sway plus head accents."),
    ]


LIBRARY = [nod_yes, shake_no, hesitate, confident_snap, shrug_fail, look_at_user_return, scan_room,
           double_take, perk_up, glance_then_reach, point_with_head, lean_in_interest, excited_bounce,
           tail_wag, sad_droop, startle_recoil, relaxed_sit_down, celebrate, approach_object, avoid_object,
           curious_head_tilt, nuzzle_touch, stretch_reach_limit, wake_up, go_to_sleep, idle_breathe, home,
           lambda: dance(90), lambda: dance(110), lambda: dance(128)]


def build_all():
    clips = [c for make in LIBRARY for c in make()]
    if CLIPS_DIR.exists():
        shutil.rmtree(CLIPS_DIR)
    CLIPS_DIR.mkdir(parents=True)
    for c in clips:
        with open(CLIPS_DIR / f"{c['name']}__{c['variant']}.json", "w", encoding="utf-8") as f:
            json.dump(c, f, indent=2)
    with open(CLIPS_DIR.parent / "poses.json", "w", encoding="utf-8") as f:
        json.dump({name: dict(zip(JOINTS, map(float, pose))) for name, pose in POSES.items()}, f, indent=2)
    return clips


if __name__ == "__main__":
    clips = build_all()
    print(f"wrote {len(clips)} clips ({len({c['name'] for c in clips})} clips x variants) to {CLIPS_DIR}")
