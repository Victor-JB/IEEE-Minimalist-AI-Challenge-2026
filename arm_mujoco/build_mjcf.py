#!/usr/bin/env python3
"""Build robot_arm.xml (MuJoCo MJCF) from the Fusion export in fusion_export.json.

fusion_export.json and meshes/ were exported from the Fusion design "ARM v29" at its
home pose (all joints at 0). Every mesh is in its own component frame (mm); each
instance carries the exact Fusion transform that places it in the assembly.

Edit the CONFIG section and re-run:  python3 build_mjcf.py
"""
import json
import math
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))

# ----------------------------------------------------------------------------- CONFIG
PLA_SOLID = 1240.0             # kg/m^3, solid PLA
PLA_FILL_FRACTION = 0.55       # 15-20% infill + walls ~= 55% of solid
STEEL = 7850.0                 # bearings, cam bearings, cycloid pins (assumed steel dowels)

# Bought parts: total mass is spread over the part's bodies by volume.
MOTOR40_MASS = 0.28            # 17HS4401 (42x40), base + First Link steppers
MOTOR23_MASS = 0.14            # 17HS4023 (42x23), Second Link (wrist) stepper
SERVO_MASS = 0.066             # Hiwonder HPS-2027

# Motor torques (N*m at the motor shaft) and rotor inertias (kg*m^2)
MOTOR40_TORQUE = 0.40          # 17HS4401 holding torque
MOTOR23_TORQUE = 0.155         # 17HS4023 holding torque
SERVO_TORQUE = 1.96            # HPS-2027 stall torque, 20 kgf*cm at 7.4 V
MOTOR40_ROTOR = 54e-7          # 54 g*cm^2 (typical 42x40)
MOTOR23_ROTOR = 20e-7          # ~20 g*cm^2 (typical 42x23)

# Transmissions: motor_shaft = RATIO * arm_joint (signs come from the mechanism)
BASE_GEAR = -5.0               # 20T pinion -> 100T wheel (external mesh reverses)
SHOULDER_GEAR = 16.0           # cycloid: 16 pins / 17-lobe plates, output reversed vs. input
ELBOW_GEAR = -16.0             # (sign differs because the elbow/wrist housings are the parent link)
WRIST_GEAR = -16.0

# Stepper position stiffness: a hybrid stepper (50 rotor teeth) reaches holding torque at
# 1/4 tooth pitch, so its small-angle stiffness is ~50 * holding torque [N*m/rad] at the
# shaft. Seen at the arm joint that is multiplied by gear^2. Beyond holding torque the
# actuator saturates (forcerange) - i.e. the motor would skip steps.
STEPPER_TEETH = 50
SERVO_KP = 20.0                # HPS-2027: full stall torque at ~5.6 deg error (estimate)

# Damping of the stepper position loops, computed from the worst-case effective inertia of
# each arm joint (measured in this model over the workspace, incl. reflected rotor inertia).
ARM_INERTIA = dict(yaw=0.107, shoulder=0.108, elbow=0.030, wrist=0.0049)   # kg*m^2
ZETA = 1.0                     # damping ratio of the linear region
BRAKE_LOOKAHEAD = 0.03         # s; kv >= kp * lookahead so a saturated motor starts braking in time
# Stepper torque falls with speed (back-EMF). Modelled as a linear drop to zero at this
# motor speed, i.e. joint damping = holding torque / no-load speed on each motor shaft.
MOTOR_NOLOAD_RPM = 1500.0
# A stepper driver ramps the step rate; here the motor's target eases toward the command
# with this time constant, so big slider jumps don't slam the arm. 0 disables it.
SETPOINT_TIMECONST = 0.15      # s

TIMESTEP = 0.0005
GEAR_SOLREF = "0.001 1"        # stiffness of the gear/eccentric couplings
GEAR_SOLIMP = "0.95 0.99 0.001"

# Link shells that hide the cycloidal drives. They are in geom group 1, so they can be
# hidden in the viewer; robot_arm_xray.xml draws them at XRAY_ALPHA opacity.
SHELL_PARTS = ('EnclosureTop', 'EnclosureBottom', 'ShellTop')
XRAY_ALPHA = 0.25
# ------------------------------------------------------------------------------------

PLA = PLA_SOLID * PLA_FILL_FRACTION

COLORS = {  # PLA colour per link (roughly matching the Fusion component colours)
    'base': '0.25 0.29 0.62 1', 'base_pinion': '0.95 0.65 0.25 1', 'yaw_spindle': '0.86 0.80 0.25 1',
    'first_link': '0.40 0.75 0.85 1', 'second_link': '0.93 0.45 0.30 1', 'head': '0.30 0.45 0.80 1',
    'head_tilt': '0.96 0.65 0.30 1', 'cam': '0.55 0.85 0.55 1', 'plate': '0.85 0.55 0.75 1',
}


def load():
    with open(os.path.join(HERE, 'fusion_export.json')) as fh:
        return json.load(fh)


def stl_triangles(path):
    with open(path, 'rb') as fh:
        data = fh.read()
    n = struct.unpack('<I', data[80:84])[0]
    tris = []
    for i in range(n):
        off = 84 + 50 * i
        v = struct.unpack('<12f', data[off:off + 48])
        tris.append((v[3:6], v[6:9], v[9:12]))
    return tris


def mat_apply(M, p):
    return [M[0] * p[0] + M[1] * p[1] + M[2] * p[2] + M[3],
            M[4] * p[0] + M[5] * p[1] + M[6] * p[2] + M[7],
            M[8] * p[0] + M[9] * p[1] + M[10] * p[2] + M[11]]


def quat_from_R(R):
    (m00, m01, m02), (m10, m11, m12), (m20, m21, m22) = R
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        w, x, y, z = (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        w, x, y, z = (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2
        w, x, y, z = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s
    return w, x, y, z


def det3(R):
    return (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
            - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
            + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))


def f(v, nd=6):
    return ' '.join(f'{x:.{nd}f}'.rstrip('0').rstrip('.') if abs(x) > 1e-12 else '0' for x in v)


def main(shell_alpha=1.0, out_name='robot_arm.xml'):
    d = load()
    piv = d['pivots']
    meshes = {m['file']: m for m in d['meshes']}

    # ---- body tree: name -> (parent, pivot point in Fusion mm, joint spec or None)
    def pt(key, zero_z=True):
        p = list(piv[key][0])
        return p
    tree = {}
    order = []

    def add(name, parent, pos, joint=None):
        tree[name] = dict(parent=parent, pos=pos, joint=joint, children=[])
        if parent:
            tree[parent]['children'].append(name)
        order.append(name)

    Z = [0, 0, 1]
    UP = [0, -1, 0]            # Fusion -Y is up; yaw positive = CCW seen from above
    add('base', None, [0, 0, 0])
    add('base_pinion', 'base', [piv['base_motor'][0][0], 0, piv['base_motor'][0][2]],
        dict(name='base_motor_shaft', axis=UP))
    add('yaw_spindle', 'base', [0, 0, 0], dict(name='yaw', axis=UP, rotor=MOTOR40_ROTOR, motor_torque=MOTOR40_TORQUE, gear=BASE_GEAR))
    add('first_link', 'yaw_spindle', [0, 0, 0], dict(name='shoulder', axis=Z, rotor=MOTOR40_ROTOR, motor_torque=MOTOR40_TORQUE, gear=SHOULDER_GEAR))
    add('shoulder_cam', 'first_link', [piv['shoulder_cam'][0][0], piv['shoulder_cam'][0][1], 0],
        dict(name='shoulder_motor_shaft', axis=Z))
    add('elbow_cam', 'first_link', [piv['elbow_cam'][0][0], piv['elbow_cam'][0][1], 0],
        dict(name='elbow_motor_shaft', axis=Z))
    add('second_link', 'first_link', [piv['elbow'][0][0], piv['elbow'][0][1], 0], dict(name='elbow', axis=Z, rotor=MOTOR40_ROTOR, motor_torque=MOTOR40_TORQUE, gear=ELBOW_GEAR))
    add('wrist_cam', 'second_link', [piv['wrist_cam'][0][0], piv['wrist_cam'][0][1], 0],
        dict(name='wrist_motor_shaft', axis=Z))
    add('head', 'second_link', [piv['wrist'][0][0], piv['wrist'][0][1], 0], dict(name='wrist', axis=Z, rotor=MOTOR23_ROTOR, motor_torque=MOTOR23_TORQUE, gear=WRIST_GEAR))
    ht = piv['head_tilt']
    add('head_tilt', 'head', [0, ht[0][1], ht[0][2]],
        dict(name='head_tilt', axis=[1, 0, 0], range=(-math.radians(135), math.radians(135))))
    for drive in ('shoulder', 'elbow', 'wrist'):
        for x in 'abc':
            c = piv[f'{drive}_plate_{x}'][0]
            add(f'{drive}_plate_{x}', f'{drive}_cam', [c[0], c[1], 0], dict(name=f'{drive}_plate_{x}', axis=Z))
    # depth-first order (MuJoCo qpos order follows XML nesting)
    dfs = []

    def walk(n):
        dfs.append(n)
        for ch in tree[n]['children']:
            walk(ch)
    walk('base')

    # ---- mass split for bought parts (by volume within each instance's part)
    inst = d['instances']
    part_vol = {}
    for it in inst:
        part_vol[it['occurrence']] = part_vol.get(it['occurrence'], 0) + meshes[it['mesh']]['volume_mm3']

    def geom_mass_attr(it):
        cls = it['cls']
        v = meshes[it['mesh']]['volume_mm3']
        if v < 1.0:          # Fusion surface body (open sheet, no volume): visual only, no mass
            return 'density="0"', 0.0
        if cls == 'pla':
            return f'density="{PLA:.1f}"', PLA * v * 1e-9
        if cls == 'steel':
            return f'density="{STEEL:.0f}"', STEEL * v * 1e-9
        total = dict(motor40=MOTOR40_MASS, motor23=MOTOR23_MASS, servo=SERVO_MASS)[cls]
        if cls == 'servo':   # servo is split across two moving bodies: share by volume over the whole servo
            sv = sum(meshes[i['mesh']]['volume_mm3'] for i in inst if i['cls'] == 'servo')
            m = total * v / sv
        else:
            m = total * v / part_vol[it['occurrence']]
        return f'mass="{m:.6f}"', m

    # which instances also collide (link shells, motors, servo housing); the rest are visual only
    COLLIDE = ('Base plate', 'EnclosureTop', 'EnclosureBottom', 'ShellTop', '1stsegment', '2nd segments',
               'Stepper:', '17HS4023', '01 Housing', 'Base spindle')

    def collides(it):
        if meshes[it['mesh']]['volume_mm3'] < 1.0:
            return False
        return any(k in it['occurrence'] or k in it.get('body', '') for k in COLLIDE)

    def is_shell(it):
        return any(k in it['occurrence'] for k in SHELL_PARTS)

    def material_for(it, body):
        cls = it['cls']
        if is_shell(it):
            return 'shell_' + body
        if cls == 'pla':
            if body.endswith('_cam'):
                return 'pla_cam'
            if '_plate_' in body:
                return 'pla_plate'
            return 'pla_' + body
        return {'steel': 'steel', 'motor40': 'motor', 'motor23': 'motor', 'servo': 'servo'}[cls]

    # ---- floor height: the base plate's largest downward face (Fusion +Y normal)
    bp = [it for it in inst if it['mjbody'] == 'base' and it['cls'] == 'pla'][0]
    hist = {}
    for a, b, c in stl_triangles(os.path.join(HERE, 'meshes', bp['mesh'])):
        A, B, C = (mat_apply(bp['world_from_mesh_mm'], p) for p in (a, b, c))
        u = [B[i] - A[i] for i in range(3)]
        w = [C[i] - A[i] for i in range(3)]
        n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
        area = 0.5 * math.sqrt(sum(x * x for x in n))
        if area > 0 and n[1] / (2 * area) > 0.99:     # faces pointing +Y (= down)
            y = round((A[1] + B[1] + C[1]) / 3, 1)
            hist[y] = hist.get(y, 0) + area
    plate_bottom_y = max(hist, key=hist.get)

    # ---- world pivot of each body (Fusion mm)
    wpos = {n: tree[n]['pos'] for n in tree}

    expected_mass = {}
    lines = []
    W = lines.append
    W('<!-- Robot arm (ARM v29) for MuJoCo, generated by build_mjcf.py from the Fusion design. -->')
    W('<!-- Units: m, kg, s, rad. Fusion -Y (up) is mapped to MuJoCo +Z. Home pose = all joints 0 = arm straight up. -->')
    W('<mujoco model="robot_arm">')
    W('  <compiler angle="radian" meshdir="meshes" autolimits="true"/>')
    W(f'  <option timestep="{TIMESTEP}" integrator="implicitfast"/>')
    W('  <visual><global offwidth="1600" offheight="1200"/><quality shadowsize="4096"/></visual>')
    W('  <default>')
    W('    <mesh scale="0.001 0.001 0.001" inertia="exact"/>')
    W('    <geom type="mesh"/>')
    W('    <default class="visual"><geom contype="0" conaffinity="0" group="2"/></default>')
    W('    <default class="solid"><geom contype="1" conaffinity="1" group="2" condim="3" friction="0.6 0.01 0.001" solref="0.004 1" solimp="0.95 0.99 0.001"/></default>')
    W('    <default class="shell"><geom contype="1" conaffinity="1" group="1" condim="3" friction="0.6 0.01 0.001" solref="0.004 1" solimp="0.95 0.99 0.001"/></default>')
    W(f'    <default class="gear"><equality solref="{GEAR_SOLREF}" solimp="{GEAR_SOLIMP}"/></default>')
    W('  </default>')
    W('  <asset>')
    W('    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.82 0.80" rgb2="0.70 0.70 0.68" width="512" height="512"/>')
    W('    <material name="floor" texture="grid" texrepeat="8 8" reflectance="0.05"/>')
    W('    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.6 0.75 0.9" rgb2="0.15 0.2 0.3" width="256" height="256"/>')
    for k, c in COLORS.items():
        W(f'    <material name="pla_{k}" rgba="{c}" specular="0.2" shininess="0.2"/>')
    for k in ('first_link', 'second_link'):    # shell materials: last rgba value = opacity
        rgb = ' '.join(COLORS[k].split()[:3])
        W(f'    <material name="shell_{k}" rgba="{rgb} {shell_alpha:g}" specular="0.2" shininess="0.2"/>')
    W('    <material name="steel" rgba="0.72 0.73 0.76 1" specular="0.8" shininess="0.8"/>')
    W('    <material name="motor" rgba="0.12 0.12 0.13 1" specular="0.5" shininess="0.4"/>')
    W('    <material name="servo" rgba="0.75 0.12 0.12 1" specular="0.3" shininess="0.3"/>')
    for fn in sorted(meshes):
        W(f'    <mesh name="{fn[:-4]}" file="{fn}"/>')
    W('  </asset>')
    W('  <worldbody>')
    W('    <light pos="0.6 -0.6 1.4" dir="-0.4 0.4 -1" diffuse="0.8 0.8 0.8" castshadow="true"/>')
    W('    <light pos="-0.6 0.4 1.0" dir="0.4 -0.3 -1" diffuse="0.35 0.35 0.35" castshadow="false"/>')
    W('    <geom name="table" type="plane" size="1 1 0.02" material="floor" solref="0.004 1" solimp="0.95 0.99 0.001"/>')
    W('    <camera name="overview" pos="0.75 -0.75 0.55" xyaxes="0.707 0.707 0 -0.25 0.25 0.935"/>')

    def emit(name, depth):
        node = tree[name]
        ind = '  ' * depth
        if name == 'base':
            z0 = plate_bottom_y / 1000.0       # put the plate's underside on the table (z = 0)
            W(f'{ind}<body name="base" pos="0 0 {z0:.4f}" quat="0.7071068 -0.7071068 0 0">')
        else:
            par = node['parent']
            rel = [(wpos[name][i] - wpos[par][i]) / 1000.0 for i in range(3)]
            W(f'{ind}<body name="{name}" pos="{f(rel)}">')
        j = node['joint']
        if j:
            extra = ''
            if 'armature' in j:
                extra += f' armature="{j["armature"]:.3g}"'
            if 'motor_torque' in j:   # stepper rotor inertia and back-EMF torque drop, reflected through the gearing (x gear^2)
                w_nl = MOTOR_NOLOAD_RPM * 2 * math.pi / 60
                g2 = j['gear'] ** 2
                extra += f' armature="{j["rotor"] * g2:.4g}" damping="{j["motor_torque"] / w_nl * g2:.4g}"'
            if 'range' in j:
                extra += f' range="{j["range"][0]:.4f} {j["range"][1]:.4f}"'
            W(f'{ind}  <joint name="{j["name"]}" type="hinge" axis="{f(j["axis"])}"{extra}/>')
        mtot = 0.0
        for it in inst:
            if it['mjbody'] != name:
                continue
            M = it['world_from_mesh_mm']
            R = [[M[0], M[1], M[2]], [M[4], M[5], M[6]], [M[8], M[9], M[10]]]
            if det3(R) < 0:
                raise SystemExit(f'mirrored transform not supported: {it["occurrence"]}')
            t = [(M[3] - wpos[name][0]) / 1000.0, (M[7] - wpos[name][1]) / 1000.0, (M[11] - wpos[name][2]) / 1000.0]
            q = quat_from_R(R)
            massattr, m = geom_mass_attr(it)
            mtot += m
            cls = 'shell' if is_shell(it) else ('solid' if collides(it) else 'visual')
            W(f'{ind}  <geom class="{cls}" mesh="{it["mesh"][:-4]}" pos="{f(t)}" quat="{f(q, 7)}" material="{material_for(it, name)}" {massattr}/>')
        expected_mass[name] = mtot
        for ch in node['children']:
            emit(ch, depth + 1)
        W(f'{ind}</body>')

    emit('base', 2)
    W('  </worldbody>')

    # ---- contact exclusions for bodies that physically nest inside each other
    W('  <contact>')
    W('    <exclude body1="world" body2="yaw_spindle"/>  <!-- spindle + 100T wheel sit below the base plate, through the table -->')
    W('    <exclude body1="base" body2="yaw_spindle"/>')
    W('    <exclude body1="yaw_spindle" body2="first_link"/>')
    W('  </contact>')

    # ---- exact kinematic couplings
    W('  <equality>')
    W(f'    <joint class="gear" name="base_gear_20T_100T" joint1="base_motor_shaft" joint2="yaw" polycoef="0 {BASE_GEAR:g} 0 0 0"/>')
    W(f'    <joint class="gear" name="shoulder_cycloid_16to1" joint1="shoulder_motor_shaft" joint2="shoulder" polycoef="0 {SHOULDER_GEAR:g} 0 0 0"/>')
    W(f'    <joint class="gear" name="elbow_cycloid_16to1" joint1="elbow_motor_shaft" joint2="elbow" polycoef="0 {ELBOW_GEAR:g} 0 0 0"/>')
    W(f'    <joint class="gear" name="wrist_cycloid_16to1" joint1="wrist_motor_shaft" joint2="wrist" polycoef="0 {WRIST_GEAR:g} 0 0 0"/>')
    for drive in ('shoulder', 'elbow', 'wrist'):
        for x in 'abc':
            W(f'    <joint class="gear" name="{drive}_plate_{x}_no_spin" joint1="{drive}_plate_{x}" joint2="{drive}_motor_shaft" polycoef="0 -1 0 0 0"/>')
    W('  </equality>')

    # ---- actuators: one per motor; the command is the ARM joint angle, limited to motor torque x gear
    W('  <actuator>')
    acts = [('base_motor', 'base_motor_shaft', BASE_GEAR, MOTOR40_TORQUE, math.pi, 'yaw'),
            ('shoulder_motor', 'shoulder_motor_shaft', SHOULDER_GEAR, MOTOR40_TORQUE, math.pi, 'shoulder'),
            ('elbow_motor', 'elbow_motor_shaft', ELBOW_GEAR, MOTOR40_TORQUE, math.pi, 'elbow'),
            ('wrist_motor', 'wrist_motor_shaft', WRIST_GEAR, MOTOR23_TORQUE, math.pi, 'wrist')]
    for name, joint, g, tq, cr, armj in acts:
        fr = abs(g) * tq                         # max torque at the arm joint
        kp = STEPPER_TEETH * tq * g * g          # stepper stiffness seen at the arm joint
        kv = max(2 * ZETA * math.sqrt(kp * ARM_INERTIA[armj]), kp * BRAKE_LOOKAHEAD)
        tc = f' timeconst="{SETPOINT_TIMECONST:g}"' if SETPOINT_TIMECONST > 0 else ''
        # applied at the arm joint: motor torque x gear (equivalent to driving the shaft through the rigid coupling)
        W(f'    <position name="{name}" joint="{armj}" kp="{kp:.4g}" kv="{kv:.4g}"{tc} ctrlrange="{-cr:.4f} {cr:.4f}" forcerange="{-fr:.4g} {fr:.4g}"/>')
    W(f'    <position name="head_servo" joint="head_tilt" kp="{SERVO_KP:g}" dampratio="1" ctrlrange="{-math.radians(135):.4f} {math.radians(135):.4f}" forcerange="{-SERVO_TORQUE:g} {SERVO_TORQUE:g}"/>')
    W('  </actuator>')

    # ---- sensors
    W('  <sensor>')
    for j in ('yaw', 'shoulder', 'elbow', 'wrist', 'head_tilt'):
        W(f'    <jointpos name="{j}_pos" joint="{j}"/>')
    for a in ('base_motor', 'shoulder_motor', 'elbow_motor', 'wrist_motor', 'head_servo'):
        W(f'    <actuatorfrc name="{a}_force" actuator="{a}"/>')
    W('  </sensor>')

    # ---- keyframes (qpos follows depth-first joint order)
    joint_order = [tree[n]['joint']['name'] for n in dfs if tree[n]['joint']]

    def qpos_for(yaw, sh, el, wr, tilt):
        v = {'yaw': yaw, 'shoulder': sh, 'elbow': el, 'wrist': wr, 'head_tilt': tilt,
             'base_motor_shaft': BASE_GEAR * yaw, 'shoulder_motor_shaft': SHOULDER_GEAR * sh,
             'elbow_motor_shaft': ELBOW_GEAR * el, 'wrist_motor_shaft': WRIST_GEAR * wr}
        for drive in ('shoulder', 'elbow', 'wrist'):
            for x in 'abc':
                v[f'{drive}_plate_{x}'] = -v[f'{drive}_motor_shaft']
        return ' '.join(f'{v[j]:.6g}' for j in joint_order)

    poses = {'home': (0, 0, 0, 0, 0), 'reach': (0.6, -0.5, 1.1, 0.6, -0.4)}
    W('  <keyframe>')
    for k, p in poses.items():
        act = f' act="{" ".join(f"{x:.4g}" for x in p[:4])}"' if SETPOINT_TIMECONST > 0 else ''
        W(f'    <key name="{k}" qpos="{qpos_for(*p)}" ctrl="{" ".join(f"{x:.4g}" for x in p)}"{act}/>')
    W('  </keyframe>')
    W('</mujoco>')

    out = os.path.join(HERE, out_name)
    with open(out, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print(f'wrote {out}')
    if shell_alpha < 1.0:
        return
    with open(os.path.join(HERE, 'expected_masses.json'), 'w') as fh:
        json.dump(dict(expected_mass_kg=expected_mass, joint_order=joint_order,
                       plate_bottom_fusion_y_mm=plate_bottom_y), fh, indent=1)
    print(f'base plate underside at Fusion y = {plate_bottom_y} mm (placed on the table, z = 0)')
    print('expected body masses (kg): ' + ', '.join(f'{k}={v:.3f}' for k, v in expected_mass.items()))
    print(f'total moving mass: {sum(v for k, v in expected_mass.items() if k != "base"):.3f} kg')


if __name__ == '__main__':
    main()                                          # opaque shells
    main(XRAY_ALPHA, 'robot_arm_xray.xml')          # see-through shells
