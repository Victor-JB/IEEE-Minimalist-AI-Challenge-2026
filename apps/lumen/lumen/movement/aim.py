"""Camera geometry for AIM, SEARCH and ADJUST: pixels -> desk points -> poses that light them.

The arm's kinematics (link lengths, the shade, joint limits) live in the shared arm module
(arm/python/kinematics.py); this file adds the camera: a pinhole model of the Arducam USB camera
in the lamp head, with the field of view from config.CAMERA_HFOV_DEG (not measured yet, see there).

Image convention (an upright camera looking along the beam): +u is image right, +v image down.
At pitch phi the image "up" is kinematics' shade-frame up v and "right" is w (both from fk()).
Looking down at the desk, image up is the far side of the desk. Roll (J5) turns the view about the
image's vertical axis, so it pans the image left/right.
"""
import math

from . import config as move_cfg
from .. import paths  # noqa: F401  (puts arm/python on sys.path)
import kinematics as K                     # arm/python/kinematics.py


def focal_px(width):
    """Focal length in pixels for a frame `width` px wide (square pixels)."""
    return (width / 2.0) / math.tan(math.radians(move_cfg.CAMERA_HFOV_DEG) / 2.0)


def pixel_ray(head, u, v, width, height):
    """World direction (unit) of the camera ray through pixel (u, v), for head = kinematics.fk()."""
    f = focal_px(width)
    x, y = (u - width / 2.0) / f, (v - height / 2.0) / f
    ray = tuple(a + x * b - y * c for a, b, c in zip(head.axis, head.right, head.up))
    return K._norm(ray)


def desk_point(q, roll, u, v, width, height, plane_z=None):
    """Where pixel (u, v) of the camera at pose (q, roll) looks on the desk (z = plane_z), or None
    when that ray doesn't come down onto the desk (above the horizon)."""
    head = K.fk(q, roll)
    z = move_cfg.BOOK_TOP_Z if plane_z is None else plane_z
    p = K.ray_plane(head.camera, pixel_ray(head, u, v, width, height), z)
    return p


def project(q, roll, point, width, height):
    """Desk/world point -> (u, v) pixel for the camera at (q, roll), or None if behind it."""
    head = K.fk(q, roll)
    c = tuple(a - b for a, b in zip(point, head.camera))
    zc = K._dot(c, head.axis)
    if zc <= 1e-6:
        return None
    f = focal_px(width)
    return (width / 2.0 + f * K._dot(c, head.right) / zc, height / 2.0 - f * K._dot(c, head.up) / zc)


def box_center(box):
    x, y, w, h = box[:4]
    return x + w / 2.0, y + h / 2.0


def book_target(view_q, view_roll, box, frame_wh):
    """Book box (x, y, w, h, score) seen from pose (view_q, view_roll) -> desk point of its centre."""
    u, v = box_center(box)
    return desk_point(view_q, view_roll, u, v, *frame_wh)


def aim_at_book(view_q, view_roll, box, frame_wh, ref=None):
    """kinematics.Aim pose that puts the light on the book's centre, or None (off the desk or out
    of reach). ref: the current pose, to prefer a short move."""
    p = book_target(view_q, view_roll, box, frame_wh)
    if p is None:
        return None
    return K.aim(p, ref=ref)


def move_time(a, b, minimum=0.6):
    """Seconds for a min-jerk move a -> b (3 joints) that stays under arm_config SPEED_CAPS
    (and ELBOW_LIFT_CAP while the elbow straightens). Min-jerk peaks at 1.875x the mean speed."""
    t = minimum
    for j, (x, y) in enumerate(zip(a[:3], b[:3])):
        cap = move_cfg.ELBOW_LIFT_CAP if j == 1 and y < x else move_cfg.SPEED_CAPS[j]
        t = max(t, 1.875 * abs(y - x) / cap)
    return t


# -- SEARCH: where to pause ------------------------------------------------------
def coverage(views, area=None, frame_wh=(640, 480), inset=None, step=0.02):
    """Fraction of the desk area seen (inside the frame less an inset) from at least one view.
    views: [(q, roll)]. area: ((x0, x1), (y0, y1)) m on the desk."""
    (x0, x1), (y0, y1) = area or move_cfg.DESK_AREA
    w, h = frame_wh
    m = move_cfg.SWEEP_FRAME_INSET if inset is None else inset
    seen = total = 0
    x = x0
    while x <= x1 + 1e-9:
        y = y0
        while y <= y1 + 1e-9:
            total += 1
            for q, r in views:
                p = project(q, r, (x, y, 0.0), w, h)
                if p and m * w <= p[0] <= (1 - m) * w and m * h <= p[1] <= (1 - m) * h:
                    seen += 1
                    break
            y += step
        x += step
    return seen / total


def sweep_poses(area=None, n=None, rolls=None):
    """3-5 pause poses, near to far: pause i aims the beam (and the coaxial camera) at the desk
    centre line, x spread evenly over the area, from high and far back (a big footprint).
    rolls (config.SWEEP_ROLLS, cycled) adds a J5 left/right look per pause: the aim point moves
    sideways by the amount that needs that roll, which widens what the pitch-only sweep sees.

    Yaw seam: with a yaw joint, aim each pause's column with yaw instead and keep roll at 0.
    Returns [(q, roll)]."""
    (x0, x1), _ = area or move_cfg.DESK_AREA
    n = n or move_cfg.SWEEP_PAUSES
    rolls = move_cfg.SWEEP_ROLLS if rolls is None else rolls
    out, ref = [], tuple(move_cfg.HOME)
    for i in range(n):
        x = x0 + (x1 - x0) * (i + 0.5) / n
        r = math.radians(rolls[i % len(rolls)] if rolls else 0.0)
        centre = K.aim((x, 0.0, 0.0), ref=ref, z_pref=move_cfg.SWEEP_Z, standoff=move_cfg.SWEEP_STANDOFF)
        if centre is None:
            continue
        reach = math.dist(centre.pivot, (x, 0.0, 0.0))
        sol = K.aim((x, -math.tan(r) * reach, 0.0), ref=ref, z_pref=move_cfg.SWEEP_Z,
                    standoff=move_cfg.SWEEP_STANDOFF) or centre
        out.append((sol.q, sol.roll))
        ref = sol.q
    return out
