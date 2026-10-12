#!/usr/bin/env python3
"""Labelled renders of the arm: arm_parts.png (the whole arm) and drive_parts.png (inside the
First Link: the shoulder and elbow cycloidal drives).

  python3 label_parts.py          (needs mujoco, numpy, pillow)
"""
import math
import os

import sys

import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from lamp_demo import set_arm_pose  # noqa: E402
W, H = 1800, 1100
POSE = dict(yaw=0.0, shoulder=15.0, elbow=45.0, wrist=55.0, head_tilt=0.0)   # deg, lamp "look" pose

# (label, description, body, mesh name or None for the body/joint origin, side)
ARM = [
    ('Base plate', 'navy; table mount, holds the yaw bearings', 'base', 'base_base_plate_underside_motor_bracket', 'L'),
    ('Base (yaw) stepper', '17HS4401, under the plate', 'base', 'stepper_body2', 'L'),
    ('20T pinion', 'on the yaw motor shaft', 'base_pinion', 'base_drive_20t_pinion_20t_pinion_with_d_shaft_hub', 'L'),
    ('Yaw spindle + 100T wheel', 'yellow turntable; 20T:100T = 5:1', 'yaw_spindle', 'yaw_spindle_body205', 'L'),
    ('J1 yaw', 'base rotation, vertical axis', 'yaw_spindle', '@yaw', 'L'),
    ('J2 shoulder', 'axle = yaw-spindle fork, 16:1 cycloid', 'first_link', '@shoulder', 'L'),
    ('First Link (upper arm)', 'teal: EnclosureTop + EnclosureBottom', 'first_link', 'enclosuretop_body1', 'L'),
    ('Shoulder stepper', '17HS4401; drives J2 through its cycloid', 'first_link', ('stepper_body2', 0), 'R'),
    ('Elbow stepper', '17HS4401; drives J3 through its cycloid', 'first_link', ('stepper_body2', 1), 'R'),
    ('J3 elbow', 'axle on the Second Link, 16:1 cycloid', 'second_link', '@elbow', 'R'),
    ('Second Link (forearm)', 'brown: ShellTop shell', 'second_link', 'shelltop_body9', 'R'),
    ('Wrist stepper', '17HS4023 pancake; drives J4', 'second_link', '17hs4023_stepper_42x23_stator', 'R'),
    ('J4 wrist', 'axle on the Head, 16:1 cycloid', 'head', '@wrist', 'R'),
    ('Head (1st segment)', 'blue yoke carrying the tilt servo', 'head', '1stsegment_body2', 'R'),
    ('Head-tilt servo', 'HPS-2027, own position loop', 'head', '01_housing_center_housing_red', 'R'),
    ('J5 head tilt', 'swings the shade sideways, +/-135 deg', 'head_tilt', '@head_tilt', 'R'),
    ('Shade (2nd segments)', 'lamp head; the light sits inside', 'head_tilt', '2nd_segments_body4', 'R'),
]

DRIVE = [
    ('Shoulder stepper (input)', 'turns the eccentric shaft; 16 turns = 1 joint turn', 'first_link', ('stepper_body2', 0), 'R'),
    ('Eccentric shaft (cams)', 'cam a-b + a-c on the motor shaft, 0.8 mm offset', 'shoulder_cam', 'cam_a_b_body1', 'R'),
    ('Cam bearings x3', 'green; 120 deg apart, one per plate', 'shoulder_cam', ('cam_bearing_body1', 1), 'R'),
    ('Cycloid plates a / b / c', 'pink; 17 lobes, orbit 0.8 mm without spinning', 'shoulder_plate_a', 'ring_a_body1', 'L'),
    ('Output pins x16', 'steel dowels on the output (yaw-spindle) axle', 'yaw_spindle', ('pin_body1', 0), 'L'),
    ('Joint bearings', 'First Link turns on the axle', 'first_link', ('bearing_body1', 1), 'L'),
    ('Output axle / fork', 'part of the yaw spindle: the J2 output', 'yaw_spindle', 'yaw_spindle_base_spindle_100t_drive_wheel_1', 'L'),
    ('Elbow stepper + drive', 'the same drive, mirrored, for J3', 'first_link', ('stepper_body2', 1), 'R'),
    ('Elbow output pins', '16 pins inside the plates, on the Second Link', 'second_link', ('pin_mirror_body1', 0), 'R'),
    ('Wrist drive (Second Link)', 'same design; cam 132.5 mm from the wrist', 'wrist_cam', 'cam_bearing_body1_2', 'R'),
]


def font(size):
    try:
        return ImageFont.load_default(size=size)
    except Exception:
        return ImageFont.load_default()


def set_pose(m, d):
    set_arm_pose(m, d, [math.radians(POSE[j]) for j in ('yaw', 'shoulder', 'elbow', 'wrist', 'head_tilt')])
    mujoco.mj_forward(m, d)


def anchor(m, d, body, what):
    """World point for a label: a joint anchor (@name), the n-th geom with a mesh, or the body origin."""
    b = m.body(body).id
    if what is None:
        return d.xpos[b].copy()
    if isinstance(what, str) and what.startswith('@'):
        return d.xanchor[m.joint(what[1:]).id].copy()
    name, n = (what, 0) if isinstance(what, str) else what
    hits = [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
            and m.mesh(m.geom_dataid[g]).name == name]
    if not hits:
        raise KeyError(f'{body}: no mesh {name}')
    return d.geom_xpos[hits[n]].copy()


def project(cam, p):
    """World point -> pixel, using the renderer's OpenGL camera."""
    fwd, up = np.array(cam.forward), np.array(cam.up)
    right = np.cross(fwd, up)
    v = p - np.array(cam.pos)
    z = v @ fwd
    x, y = cam.frustum_near * (v @ right) / z, cam.frustum_near * (v @ up) / z
    fw = cam.frustum_width or 0.5 * (cam.frustum_top - cam.frustum_bottom) * W / H   # 0 = from the aspect ratio
    u = (x - (cam.frustum_center - fw)) / (2 * fw) * W
    w = (cam.frustum_top - y) / (cam.frustum_top - cam.frustum_bottom) * H
    return u, w


def spread(ys, gap, lo, hi):
    """Push label y positions apart (keeping order) inside [lo, hi]."""
    ys = list(ys)
    for i in range(1, len(ys)):
        ys[i] = max(ys[i], ys[i - 1] + gap)
    if ys and ys[-1] > hi:
        ys[-1] = hi
        for i in range(len(ys) - 2, -1, -1):
            ys[i] = min(ys[i], ys[i + 1] - gap)
    return [max(lo, y) for y in ys]


def render(xml, labels, cam_cfg, out, title, hide_shells=False):
    m = mujoco.MjModel.from_xml_path(os.path.join(HERE, xml))
    d = mujoco.MjData(m)
    m.geom_rgba[m.geom('table').id] = [0, 0, 0, 0]              # show the parts under the base plate
    set_pose(m, d)
    m.vis.global_.offwidth, m.vis.global_.offheight = W, H
    r = mujoco.Renderer(m, H, W)
    opt = mujoco.MjvOption()
    if hide_shells:
        opt.geomgroup[1] = 0
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    look, cam.distance, cam.azimuth, cam.elevation = cam_cfg
    if look == 'arm':            # centre of the robot's parts
        ids = [g for g in range(m.ngeom) if m.geom_bodyid[g] != 0]
        lo, hi = d.geom_xpos[ids].min(0), d.geom_xpos[ids].max(0)
        look = (lo + hi) / 2
    elif look == 'first_link':   # between the shoulder and elbow axes
        look = (d.xanchor[m.joint('shoulder').id] + d.xanchor[m.joint('elbow').id]) / 2
    cam.lookat[:] = look
    r.update_scene(d, cam, opt)
    img = Image.fromarray(r.render()).convert('RGB')
    glcam = mujoco.mjv_averageCamera(r.scene.camera[0], r.scene.camera[1])   # the two eyes -> the mono camera
    dr = ImageDraw.Draw(img, 'RGBA')
    f_name, f_desc, f_title = font(26), font(19), font(34)
    pts = [(lab, desc, project(glcam, anchor(m, d, body, what))) for lab, desc, body, what, _ in labels]
    cx = np.median([p[2][0] for p in pts])
    pts = [(lab, desc, uv, 'L' if uv[0] < cx else 'R') for lab, desc, uv in pts]      # label on the near side
    colw, gap, top = 420, 62, 90
    for side in 'LR':
        items = sorted([p for p in pts if p[3] == side], key=lambda p: p[2][1])
        ys = spread([p[2][1] for p in items], gap, top, H - 70)
        for (lab, desc, (ax, ay), _), y in zip(items, ys):
            tx = 24 if side == 'L' else W - colw - 24
            ex = tx + colw if side == 'L' else tx - 8                 # where the leader line meets the label
            dr.line([(ax, ay), (ex + (0 if side == 'L' else 0), y + 14)], fill=(30, 30, 40, 230), width=2)
            dr.ellipse([ax - 6, ay - 6, ax + 6, ay + 6], fill=(255, 140, 30, 255), outline=(30, 30, 40, 255), width=2)
            box = [tx - 10, y - 6, tx + colw, y + 56]
            dr.rounded_rectangle(box, radius=10, fill=(255, 255, 255, 225), outline=(60, 60, 70, 160))
            dr.text((tx, y), lab, fill=(20, 20, 28), font=f_name)
            dr.text((tx, y + 31), desc, fill=(80, 80, 92), font=f_desc)
    tw = dr.textlength(title, font=f_title)
    dr.rounded_rectangle([14, 16, 34 + tw, 66], radius=10, fill=(255, 255, 255, 215))
    dr.text((24, 24), title, fill=(20, 20, 28), font=f_title)
    img.save(os.path.join(HERE, out))
    print('wrote', out)


if __name__ == '__main__':
    render('robot_arm.xml', ARM, ('arm', 0.95, 128, -12), 'arm_parts.png',
           'ARM v29 - parts and joints (lamp "look" pose; table hidden)')
    render('robot_arm.xml', DRIVE, ('first_link', 0.48, -112, -10), 'drive_parts.png',
           'Inside the First Link - shoulder & elbow cycloidal drives (shells hidden)', hide_shells=True)
