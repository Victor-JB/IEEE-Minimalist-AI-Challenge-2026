"""Lamp-arm geometry: forward kinematics to the shade, and inverse kinematics that points the
light at a desk point. Pure Python (math only), so it runs on the board and in plain python3.

Frame (metres): origin on the desk directly below the shoulder axis, +x forward (the CAD front,
where the lamp faces at HOME), +y the lamp's left, +z up. The desk top is z = 0.

Joints, as everywhere in arm/ (degrees): shoulder, elbow, head are pitch joints about the lateral
axis, measured from straight up, each relative to the previous link, positive leaning toward the
front (arm/calibration "zero_pose"/"positive_direction"; apps/stepper_desk ik_line.py; the desk
page's side view). The pitch of the head link in the world is phi = shoulder + elbow + head
(0 = up, 90 = level forward, 180 = straight down).
J5, the head tilt ("roll", arm_config.ROLL_*), turns the shade about an axis across the head link
that lies in the pitch plane. Because the shade's axis runs along the head link, this swings the
beam sideways: +roll moves it to the lamp's right (-y). Looking down at the desk it acts like a
small pan, and in the camera image it turns about the image's vertical axis.

Shade frame at pitch phi and roll r:
    axis  d = cos r (sin phi, 0, cos phi) - sin r (0, 1, 0)   light (and camera) optical axis
    up    v = (-cos phi, 0, sin phi)                           image up; also the J5 axis
    right w = d x v                                            image right
At HOME the beam points forward and ~33 deg above level (it looks at the user).

Where each number comes from (CAD = arm/cad/model.json, ARM v29 from Fusion; MJCF =
tools/arm_mujoco/robot_arm.xml on origin/dev, merge c088a9d, the CAD-exact MuJoCo model):
    L1, L2       0.185  model.json "lengths" 18.5 / 18.5 cm; MJCF first_link -> second_link -> head
                        body offsets (0.185 each); also ik_line.py L1 = L2 = 0.185.
    HEAD_PIVOT   0.045  model.json "lengths"[2] 4.5 cm; MJCF head -> head_tilt body offset; the desk
                        page's tilt pivot (0, -4.5, 0) in the head frame.
    SHOULDER_Z   0.049  MJCF base body z (shoulder axis above the table) with the full yaw base.
                        The real arm stands on whatever base it has: MEASURE (desk to shoulder axis).
    LIGHT_OFFSET 0.050  MJCF scene in elegnt_demo.py puts lamp_light 5 cm past the J5 axis along the
                        shade axis; the real LED position is not in the CAD: MEASURE.
    SHADE_DEPTH  0.140  the shade (2nd_segments_body8/9) spans 0.030..0.140 m past the J5 axis,
    SHADE_RADIUS 0.105  0.105 m radius (mesh extents in the MJCF at the zero pose).
    CAMERA_MOUNT        camera centre in the shade frame (along d, along v, along w). Not in the CAD.
                        Default: on the shade axis next to the LED, looking along the beam: MEASURE.
Cross-checked against MuJoCo 3.15 on the MJCF (arm/tests/test_kinematics.py MUJOCO_POSES).
"""
import math
from collections import namedtuple

import arm_config as A
from arm_config import ROLL_RANGE, SAFE_RANGES

L1 = 0.185
L2 = 0.185
HEAD_PIVOT = 0.045
SHOULDER_Z = 0.049
LIGHT_OFFSET = 0.050
SHADE_DEPTH = 0.140
SHADE_RADIUS = 0.105
CAMERA_MOUNT = (0.050, 0.0, 0.0)

JOINT_MARGIN = 2.0       # deg kept inside SAFE_RANGES / ROLL_RANGE by IK (room for hold clips)
SHADE_CLEARANCE = 0.05   # m: the shade rim stays at least this far from the aimed point
# "Comfortable" lighting pose: the J5 pivot about this high, the LED about this far from the book.
COMFORT_Z = 0.28
COMFORT_STANDOFF = 0.27
PHI_RANGE = (95.0, 175.0)  # beam pitches the aim search considers (level-ish .. straight down)

Head = namedtuple("Head", "elbow wrist pivot axis up right light camera phi roll")
Aim = namedtuple("Aim", "q roll target standoff pivot")


# -- small vector helpers (tuples) ---------------------------------------------
def _add(a, b, k=1.0):
    return tuple(x + k * y for x, y in zip(a, b))


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a):
    n = math.sqrt(_dot(a, a))
    return tuple(x / n for x in a)


def _yaw(p, yaw):
    """Rotate a point/vector about +z by yaw (deg). 0 today: the seam for a future yaw joint."""
    if not yaw:
        return p
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return (c * p[0] - s * p[1], s * p[0] + c * p[1], p[2])


# -- forward kinematics ----------------------------------------------------------
def shade_frame(phi, roll):
    """(axis, up, right) unit vectors of the shade for beam pitch phi and roll (deg)."""
    p, r = math.radians(phi), math.radians(roll)
    d = (math.cos(r) * math.sin(p), -math.sin(r), math.cos(r) * math.cos(p))
    v = (-math.cos(p), 0.0, math.sin(p))
    return d, v, _cross(d, v)


def fk(q, roll=0.0, yaw=0.0):
    """Joint angles (shoulder, elbow, head) deg + J5 roll deg -> Head (positions in m)."""
    a0 = math.radians(q[0])
    a1 = a0 + math.radians(q[1])
    phi = q[0] + q[1] + q[2]
    elbow = (L1 * math.sin(a0), 0.0, SHOULDER_Z + L1 * math.cos(a0))
    wrist = (elbow[0] + L2 * math.sin(a1), 0.0, elbow[2] + L2 * math.cos(a1))
    p = math.radians(phi)
    pivot = (wrist[0] + HEAD_PIVOT * math.sin(p), 0.0, wrist[2] + HEAD_PIVOT * math.cos(p))
    d, v, w = shade_frame(phi, roll)
    light = _add(pivot, d, LIGHT_OFFSET)
    cam = _add(_add(_add(pivot, d, CAMERA_MOUNT[0]), v, CAMERA_MOUNT[1]), w, CAMERA_MOUNT[2])
    pts = [_yaw(x, yaw) for x in (elbow, wrist, pivot, d, v, w, light, cam)]
    return Head(*pts, phi=phi, roll=roll)


def beam_hit(q, roll=0.0, plane_z=0.0):
    """Where the light axis meets the plane z = plane_z, or None if it points above it."""
    h = fk(q, roll)
    return ray_plane(h.light, h.axis, plane_z)


def ray_plane(origin, direction, plane_z=0.0):
    if direction[2] >= -1e-6:
        return None
    t = (plane_z - origin[2]) / direction[2]
    return _add(origin, direction, t) if t > 0 else None


# -- inverse kinematics ----------------------------------------------------------
def _inside(q, margin=JOINT_MARGIN):
    if A.SHOULDER_LOCK is not None:               # locked: exactly there, no margin to keep
        return abs(q[0] - A.SHOULDER_LOCK) < 1e-6 and all(
            lo + margin <= a <= hi - margin for a, (lo, hi) in zip(q[1:], SAFE_RANGES[1:]))
    return all(lo + margin <= a <= hi - margin for a, (lo, hi) in zip(q, SAFE_RANGES))


def roll_ok(roll, margin=JOINT_MARGIN):
    return ROLL_RANGE[0] + margin <= roll <= ROLL_RANGE[1] - margin


def wrist_ik(x, z):
    """Head-joint (wrist) position in the pitch plane -> [(shoulder, elbow)] deg, elbow-forward
    (the lamp-like bend, elbow > 0) first. [] if out of reach."""
    dx, dz = x, z - SHOULDER_Z
    c = (dx * dx + dz * dz - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c) > 1.0:
        return []
    out = []
    for el in (math.acos(c), -math.acos(c)):
        sh = math.atan2(dx, dz) - math.atan2(L2 * math.sin(el), L1 + L2 * math.cos(el))
        out.append((math.degrees(sh), math.degrees(el)))
    return out


def pose_for(pivot_x, pivot_z, phi, margin=JOINT_MARGIN):
    """Joint angles putting the J5 pivot at (x, z) with the head link at pitch phi, inside
    SAFE_RANGES less margin; None if no such pose."""
    p = math.radians(phi)
    for sh, el in wrist_ik(pivot_x - HEAD_PIVOT * math.sin(p), pivot_z - HEAD_PIVOT * math.cos(p)):
        q = (sh, el, phi - sh - el)
        if _inside(q, margin):
            return q
    return None


def _candidate(target, phi, standoff, margin):
    """The pose whose light axis passes through target with the LED standoff m from it, at beam
    pitch phi; roll takes the sideways part. (q, roll, pivot) or None."""
    s = standoff + LIGHT_OFFSET                      # pivot -> target along the beam
    sr = -target[1] / s
    if abs(sr) > 1.0:
        return None
    roll = math.degrees(math.asin(sr))
    if not roll_ok(roll, margin):
        return None
    d, _, _ = shade_frame(phi, roll)
    pivot = _add(target, d, -s)
    q = pose_for(pivot[0], pivot[2], phi, margin)
    return None if q is None else (q, roll, pivot)


def _locked(target, phi, margin, s_lo=0.0, s_hi=1e9):
    """Locked shoulder: the poses at beam pitch phi whose light axis passes through target.

    The elbow joint is fixed, so the wrist sits on a circle of radius L2 around it. The wrist is
    target - k * (sin phi, cos phi) in the pitch plane, k = standoff_to_pivot * cos(roll) +
    HEAD_PIVOT: one quadratic in k, then the standoff and roll in closed form. [(q, roll, pivot,
    standoff)], up to two."""
    s0 = A.SHOULDER_LOCK
    a0, p = math.radians(s0), math.radians(phi)
    ex, ez = L1 * math.sin(a0), SHOULDER_Z + L1 * math.cos(a0)
    ux, uz = math.sin(p), math.cos(p)
    tx, tz = target[0] - ex, target[2] - ez
    b = ux * tx + uz * tz
    disc = b * b - (tx * tx + tz * tz - L2 * L2)
    if disc < 0:
        return []
    out = []
    for k in (b - math.sqrt(disc), b + math.sqrt(disc)):
        inplane = k - HEAD_PIVOT                     # pivot -> target, in the pitch plane
        if inplane <= 0:
            continue
        s = math.hypot(inplane, target[1])          # pivot -> target along the beam
        standoff = s - LIGHT_OFFSET
        if not s_lo <= standoff <= s_hi:
            continue
        roll = math.degrees(math.asin(-target[1] / s))
        if not roll_ok(roll, margin):
            continue
        wx, wz = target[0] - k * ux, target[2] - k * uz
        elbow = math.degrees(math.atan2(wx - ex, wz - ez)) - s0
        q = (s0, elbow, phi - s0 - elbow)
        if _inside(q, margin):
            pivot = (wx + HEAD_PIVOT * ux, 0.0, wz + HEAD_PIVOT * uz)
            out.append((q, roll, pivot, standoff))
    return out


def _cost(q, roll, pivot, standoff, ref, z_pref, s_pref):
    c = ((pivot[2] - z_pref) / 0.05) ** 2 + ((standoff - s_pref) / 0.05) ** 2 + (roll / 30.0) ** 2
    if ref is not None:
        c += sum(((a - b) / 40.0) ** 2 for a, b in zip(q, ref))
    return c


def aim(target, ref=None, z_pref=COMFORT_Z, standoff=COMFORT_STANDOFF, standoff_range=None,
        phi_range=PHI_RANGE, margin=JOINT_MARGIN):
    """Pose that points the light axis exactly at target (x, y, z) from a comfortable height.

    Searches beam pitch and LED standoff; every candidate's beam passes through the target, and
    the cost only picks how: J5 pivot near z_pref, LED near `standoff` m from the target, little
    roll, close to `ref` (the current pose). Returns Aim(q, roll, target, standoff, pivot) or None
    if the point can't be lit (out of reach, joint or roll limits).

    Yaw seam: with a yaw joint, turn to atan2(y, x) first and solve this with target rotated into
    the arm plane (roll then stays free for expression); fk() already takes yaw.
    """
    target = tuple(target)
    s_lo = SHADE_DEPTH - LIGHT_OFFSET + SHADE_CLEARANCE
    lo, hi = standoff_range or (s_lo, 0.45)
    lo = max(lo, s_lo)
    best = None
    if A.SHOULDER_LOCK is not None:                 # elbow + head only: the standoff is not free
        phi = phi_range[0]
        while phi <= phi_range[1] + 1e-9:
            for q, roll, pivot, s in _locked(target, phi, margin, lo, hi):
                cost = _cost(q, roll, pivot, s, ref, z_pref, standoff)
                if best is None or cost < best[0]:
                    best = (cost, Aim(q, roll, target, s, pivot))
            phi += 0.5
        return None if best is None else best[1]
    phi = phi_range[0]
    while phi <= phi_range[1] + 1e-9:
        s = lo
        while s <= hi + 1e-9:
            c = _candidate(target, phi, s, margin)
            if c is not None:
                cost = _cost(c[0], c[1], c[2], s, ref, z_pref, standoff)
                if best is None or cost < best[0]:
                    best = (cost, Aim(c[0], c[1], target, s, c[2]))
            s += 0.01
        phi += 2.0
    return None if best is None else best[1]


def standoff_of(q, roll, target):
    """LED -> target distance for a pose (along the line from the LED)."""
    h = fk(q, roll)
    return math.dist(h.light, target)


def move_along_beam(q, roll, target, delta, margin=JOINT_MARGIN, tries=8):
    """Move the light delta m along its axis toward target (delta > 0 = closer), still aimed at it.

    Keeps the beam pitch when it can (head-only re-aim within +-20 deg otherwise). With roll 0 the
    pivot moves exactly along the light axis; with roll it moves along the axis' in-plane part
    (there is no yaw to leave the pitch plane) and roll re-solves the sideways aim. The step is
    shortened (bisection) to what is reachable inside the joint limits and the shade clearance.
    Returns (Aim, delta actually moved); delta 0.0 means it is already at the limit.
    """
    phi = q[0] + q[1] + q[2]
    s0 = standoff_of(q, roll, target)
    s_min = SHADE_DEPTH - LIGHT_OFFSET + SHADE_CLEARANCE
    step = min(delta, s0 - s_min)                    # the shade never touches the book
    if A.SHOULDER_LOCK is not None:
        return _move_along_beam_locked(q, roll, target, phi, s0, step, margin, tries)
    for _ in range(tries):
        if abs(step) < 0.002:
            break
        s = s0 - step
        for dphi in (0, 2, -2, 4, -4, 6, -6, 8, -8, 10, -10, 14, -14, 20, -20):
            c = _candidate(target, phi + dphi, s, margin)
            if c is not None:
                return Aim(c[0], c[1], tuple(target), s, c[2]), step
        step /= 2.0
    return Aim(tuple(q), roll, tuple(target), s0, fk(q, roll).pivot), 0.0


def _move_along_beam_locked(q, roll, target, phi, s0, step, margin, tries):
    """move_along_beam with the shoulder locked: the standoff follows from the beam pitch, so
    search the pitch (near the current one) for the pose closest to the wanted standoff."""
    for _ in range(tries):
        if abs(step) < 0.002:
            break
        want, best = s0 - step, None
        for i in range(-60, 61):                     # +-30 deg around the current pitch, 0.5 deg
            for c in _locked(target, phi + 0.5 * i, margin):
                err = abs(c[3] - want)
                if best is None or err < best[0]:
                    best = (err, c)
        if best is not None and best[0] < 0.005:
            qq, rr, pivot, s = best[1]
            return Aim(qq, rr, tuple(target), s, pivot), s0 - s
        step /= 2.0
    return Aim(tuple(q), roll, tuple(target), s0, fk(q, roll).pivot), 0.0
