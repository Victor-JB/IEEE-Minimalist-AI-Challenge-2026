"""Validate robot_arm.xml: masses vs the Fusion volumes, holding poses under gravity, gear and
cam-plate couplings, plate orbit, contacts, a home->reach move, and the shoulder gravity torque.

    python3 check_model.py            (needs: pip install mujoco)
"""
import json, math, os, sys
import numpy as np
import mujoco

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
m = mujoco.MjModel.from_xml_path(os.path.join(ROOT, 'robot_arm.xml'))
d = mujoco.MjData(m)
exp = json.load(open(os.path.join(ROOT, 'expected_masses.json')))
print(f'loaded: nbody={m.nbody} njnt={m.njnt} neq={m.neq} nu={m.nu} ngeom={m.ngeom} nmesh={m.nmesh} timestep={m.opt.timestep}')

# masses: MuJoCo (from mesh volumes) vs expected (Fusion volumes x density)
worst = 0
for name, e in exp['expected_mass_kg'].items():
    b = m.body(name)
    mm = m.body_mass[b.id]
    worst = max(worst, abs(mm - e) / max(e, 1e-9))
    print(f'  {name:18s} mujoco {mm*1000:8.1f} g   expected {e*1000:8.1f} g')
print(f'worst mass mismatch {worst*100:.2f}%  | total moving mass {sum(m.body_mass[m.body(n).id] for n in exp["expected_mass_kg"] if n != "base"):.3f} kg')

jid = {m.joint(i).name: i for i in range(m.njnt)}
def q(n): return d.qpos[m.jnt_qposadr[jid[n]]]

def body_xmat(name): return d.xmat[m.body(name).id].reshape(3, 3)
def rel_angle(a, b):
    R = body_xmat(b).T @ body_xmat(a)
    return math.degrees(math.atan2(R[1, 0], R[0, 0])), R

RATIO = {m.joint(m.eq_obj1id[i]).name: float(m.eq_data[i, 1]) for i in range(m.neq)}   # motor = RATIO * joint

def couplings():
    out = {}
    out['base'] = q('base_motor_shaft') - RATIO['base_motor_shaft'] * q('yaw')
    out['sh'] = q('shoulder_motor_shaft') - RATIO['shoulder_motor_shaft'] * q('shoulder')
    out['el'] = q('elbow_motor_shaft') - RATIO['elbow_motor_shaft'] * q('elbow')
    out['wr'] = q('wrist_motor_shaft') - RATIO['wrist_motor_shaft'] * q('wrist')
    spin = 0
    for drive, housing in [('shoulder', 'first_link'), ('elbow', 'first_link'), ('wrist', 'second_link')]:
        for x in 'abc':
            R = body_xmat(housing).T @ body_xmat(f'{drive}_plate_{x}')
            spin = max(spin, abs(math.degrees(math.acos(max(-1, min(1, (np.trace(R) - 1) / 2))))))
    out['plate_spin_deg'] = spin
    return out

def orbit_radius(drive, housing, ref_body):
    # plate frame origin = eccentric centre; distance from the cam (input) axis, measured in the housing frame
    p = d.xpos[m.body(f'{drive}_plate_a').id] - d.xpos[m.body(f'{drive}_cam').id]
    ph = body_xmat(housing).T @ p
    return math.hypot(ph[0], ph[1]) * 1000  # mm (axis is housing Z in Fusion frame)

def contacts():
    res = []
    for i in range(d.ncon):
        c = d.contact[i]
        b1 = m.body(m.geom_bodyid[c.geom1]).name; b2 = m.body(m.geom_bodyid[c.geom2]).name
        res.append((b1, b2, c.dist * 1000))
    return res

def run(key, seconds, label):
    mujoco.mj_resetDataKeyframe(m, d, m.key(key).id)
    mujoco.mj_forward(m, d)
    c0 = contacts()
    n = int(seconds / m.opt.timestep)
    maxv = 0
    for _ in range(n):
        mujoco.mj_step(m, d)
        maxv = max(maxv, np.abs(d.qvel).max())
        if not np.isfinite(d.qpos).all():
            print(label, 'DIVERGED'); return
    ctrl = d.ctrl.copy()
    arm = [q(j) for j in ('yaw', 'shoulder', 'elbow', 'wrist', 'head_tilt')]
    err = [round(math.degrees(a - c), 3) for a, c in zip(arm, ctrl)]
    cp = couplings()
    print(f'[{label}] after {seconds}s: arm tracking error (deg) {err}; max |qvel| {maxv:.3f}')
    print(f'    coupling residuals (rad): base {cp["base"]:.2e}, shoulder {cp["sh"]:.2e}, elbow {cp["el"]:.2e}, wrist {cp["wr"]:.2e}; max plate spin vs housing {cp["plate_spin_deg"]:.4f} deg')
    print(f'    plate orbit radius (mm): shoulder {orbit_radius("shoulder","first_link",None):.3f}, elbow {orbit_radius("elbow","first_link",None):.3f}, wrist {orbit_radius("wrist","second_link",None):.3f}')
    print(f'    actuator forces: {np.round(d.actuator_force, 3).tolist()}')
    print(f'    contacts at start: {c0[:6]} ... ({len(c0)}) ; at end: {contacts()[:6]} ({d.ncon})')

run('home', 2.0, 'hold home under gravity')
run('reach', 2.0, 'hold reach pose')

# drive from home to reach with smooth commands and check the drives turn as specified
mujoco.mj_resetDataKeyframe(m, d, m.key('home').id)
target = np.array(m.key('reach').ctrl)
T = 3.0; n = int(T / m.opt.timestep)
for i in range(n):
    s = min(1.0, (i * m.opt.timestep) / 2.0)
    s = 0.5 - 0.5 * math.cos(math.pi * s)
    d.ctrl[:] = s * target
    mujoco.mj_step(m, d)
arm = np.array([q(j) for j in ('yaw', 'shoulder', 'elbow', 'wrist', 'head_tilt')])
print(f'[move home->reach] final arm (deg) {np.round(np.degrees(arm), 2).tolist()} target {np.round(np.degrees(target), 2).tolist()}')
print(f'    motor shafts turned (deg): base {math.degrees(q("base_motor_shaft")):.1f}, shoulder {math.degrees(q("shoulder_motor_shaft")):.1f}, elbow {math.degrees(q("elbow_motor_shaft")):.1f}, wrist {math.degrees(q("wrist_motor_shaft")):.1f}')
cp = couplings(); print(f'    coupling residuals: {[f"{k}={v:.2e}" for k, v in cp.items()]}')

# gravity torque needed at the shoulder with the arm horizontal (worst case) vs. what the motor can give
mujoco.mj_resetData(m, d)
for jn, v in [('shoulder', math.pi / 2), ('shoulder_motor_shaft', 16 * math.pi / 2)]:
    d.qpos[m.jnt_qposadr[jid[jn]]] = v
for x in 'abc':
    d.qpos[m.jnt_qposadr[jid['shoulder_plate_' + x]]] = -16 * math.pi / 2
mujoco.mj_forward(m, d)
print(f'gravity torque at shoulder, arm horizontal: {abs(d.qfrc_bias[m.jnt_dofadr[jid["shoulder"]]] + 16 * d.qfrc_bias[m.jnt_dofadr[jid["shoulder_motor_shaft"]]] - 16*sum(d.qfrc_bias[m.jnt_dofadr[jid["shoulder_plate_"+x]]] for x in "abc")):.3f} N*m'
      f'  (motor limit at the joint: {16*0.40:.2f} N*m)')
