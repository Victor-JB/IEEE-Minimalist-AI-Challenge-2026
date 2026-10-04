"""Big step commands on every joint (as when you drag a slider): rise, overshoot, settling, peak torque.

    python3 step_response.py [robot_arm.xml] [--inertia]   (--inertia also prints each joint's effective inertia)
"""
import mujoco, numpy as np, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
args = [a for a in sys.argv[1:] if not a.startswith('--')]
path = args[0] if args else os.path.join(HERE, 'robot_arm.xml')
m = mujoco.MjModel.from_xml_path(path); d = mujoco.MjData(m)
names = ['yaw', 'shoulder', 'elbow', 'wrist', 'head_tilt']
jid = {m.joint(i).name: i for i in range(m.njnt)}
def dof(n): return m.jnt_dofadr[jid[n]]
gears = {'yaw': ('base_motor_shaft', -5, None), 'shoulder': ('shoulder_motor_shaft', 16, 'shoulder'), 'elbow': ('elbow_motor_shaft', -16, 'elbow'), 'wrist': ('wrist_motor_shaft', -16, 'wrist'), 'head_tilt': (None, 0, None)}
def eff_inertia(key='home'):
    mujoco.mj_resetDataKeyframe(m, d, m.key(key).id); mujoco.mj_forward(m, d)
    M = np.zeros((m.nv, m.nv)); mujoco.mj_fullM(m, d, M)
    out = {}
    for n in names:
        v = np.zeros(m.nv); v[dof(n)] = 1
        sh, g, drive = gears[n]
        if sh: v[dof(sh)] = g
        if drive:
            for x in 'abc': v[dof(f'{drive}_plate_{x}')] = -g
        out[n] = v @ M @ v
    return out
if '--inertia' in sys.argv:
    for k in ('home', 'reach'):
        print(k, {n: round(v, 5) for n, v in eff_inertia(k).items()})
def step(joint, target, T=3.0):
    mujoco.mj_resetDataKeyframe(m, d, m.key('home').id)
    a = names.index(joint)
    d.ctrl[:] = 0; d.ctrl[a] = target
    if m.na: d.act[:] = 0
    qs = []; frc = []
    for i in range(int(T / m.opt.timestep)):
        mujoco.mj_step(m, d)
        qs.append(d.qpos[m.jnt_qposadr[jid[joint]]]); frc.append(abs(d.actuator_force[a]))
    qs = np.array(qs); t = np.arange(len(qs)) * m.opt.timestep
    over = max(0.0, (qs - target).max() / target * 100 if target > 0 else (qs - target).min() / target * 100)
    band = 0.02 * abs(target)
    outside = np.where(np.abs(qs - target) > band)[0]
    settle = t[outside[-1]] if len(outside) else 0
    zc = int(np.sum(np.diff(np.sign(qs - target)) != 0))
    t90 = t[np.argmax(np.abs(qs) >= 0.9 * abs(target))]
    print(f'  {joint:10s} 0 -> {target:+.2f} rad: rise(90%) {t90:4.2f}s  overshoot {over:5.1f}%  settle(2%) {settle:4.2f}s  zero-crossings {zc:3d}  final err {math.degrees(qs[-1]-target):+.3f} deg  peak|force| {max(frc):.2f}/{m.actuator_forcerange[a,1]:.2f}')
for j, tgt in [('yaw', 1.5), ('shoulder', -1.0), ('elbow', 1.2), ('wrist', 1.0), ('head_tilt', 1.0)]:
    step(j, tgt)
