#!/usr/bin/env python3
"""Expressive-lamp demo for robot_arm.xml.

The arm is driven the way real stepper firmware would drive it: every move is a smooth
(minimum-jerk) profile whose duration respects per-joint speed and acceleration limits.

  mjpython lamp_demo.py               live, in the MuJoCo viewer (macOS needs mjpython)
  python3  lamp_demo.py --report      run the show headless and print how well it tracked
  python3  lamp_demo.py --video lamp.mp4 [--xray]   render a video (needs ffmpeg)

  --drive pid | pid_tuned | smooth    play it through the simulated stepper drivers and STM32
                                      loop of stepper_pid_sim.py instead of ideal motors
  --sensor joint | motor | none       where the AS5600s are (with --drive)
  --current 1.41 / --vbus 24          driver current (x rated) and supply voltage (with --drive)

Requires:  pip install mujoco   (plus ffmpeg for --video)
"""
import argparse
import math
import os
import subprocess
import sys
import time

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
JOINTS = ['yaw', 'shoulder', 'elbow', 'wrist', 'head_tilt']
ACTUATORS = ['base_motor', 'shoulder_motor', 'elbow_motor', 'wrist_motor', 'head_servo']

# Motion limits at the arm joints (rad/s, rad/s^2). Stepper speeds are kept well below the
# point where their torque fades (see MOTOR_NOLOAD_RPM in build_mjcf.py); the head is the
# HPS-2027's rated 0.20 s/60 deg.
VMAX = np.array([3.0, 2.0, 2.5, 3.0, 4.5])
AMAX = np.array([12.0, 8.0, 12.0, 20.0, 30.0])

D = math.radians
# Poses: (yaw, shoulder, elbow, wrist, head_tilt) in degrees. Positive shoulder/elbow/wrist
# lean toward +X (the lamp's "front"); head_tilt cocks the shade sideways.
POSE = {
    'sleep':      (0, 10, 90, 80, 0),
    'tall':       (0, 0, 0, 0, 0),
    'yawn':       (0, -8, -10, -25, 0),
    'look':       (0, 15, 45, 55, 0),
    'look_left':  (55, 15, 40, 60, 18),
    'look_right': (-55, 15, 40, 60, -18),
    'curious':    (10, 20, 50, 45, 32),
    'crouch':     (0, -15, 80, 40, 0),
    'peek':       (0, 50, 15, 60, 0),
    'nod_down':   (0, 15, 45, 72, 0),
    'shake_l':    (16, 15, 45, 55, 0),
    'shake_r':    (-16, 15, 45, 55, 0),
    'bounce_dn':  (0, 8, 62, 50, 0),
}
# The show: (pose, extra hold after arriving [s], speed factor <= 1, joint stagger [s])
# stagger delays each successive joint a little (follow-through / overlapping action).
SHOW = [
    ('sleep', 1.0, 1.0, 0.0),
    ('crouch', 0.15, 0.6, 0.05),     # anticipation before waking
    ('tall', 0.3, 1.0, 0.06),        # wake up
    ('yawn', 0.6, 0.5, 0.08),        # stretch
    ('look', 0.5, 1.0, 0.05),
    ('look_left', 0.5, 1.0, 0.04),
    ('look_right', 0.6, 0.8, 0.04),
    ('look', 0.3, 1.0, 0.0),
    ('curious', 0.8, 0.7, 0.08),     # head cock
    ('look', 0.2, 1.0, 0.0),
    ('crouch', 0.1, 1.0, 0.0),       # anticipation
    ('peek', 0.7, 1.0, 0.05),        # lean in
    ('look', 0.2, 1.0, 0.0),
    ('nod_down', 0.0, 1.0, 0.0), ('look', 0.0, 1.0, 0.0), ('nod_down', 0.0, 1.0, 0.0), ('look', 0.4, 1.0, 0.0),  # yes
    ('shake_l', 0.0, 1.0, 0.0), ('shake_r', 0.0, 1.0, 0.0), ('shake_l', 0.0, 1.0, 0.0), ('look', 0.4, 1.0, 0.0),  # no
    ('bounce_dn', 0.0, 1.0, 0.0), ('look', 0.0, 1.0, 0.0), ('bounce_dn', 0.0, 1.0, 0.0), ('look', 0.5, 1.0, 0.0),  # happy
    ('tall', 0.3, 0.7, 0.05),
    ('sleep', 1.5, 0.45, 0.10),      # slowly back to sleep
]


def couplings(m):
    """{dependent joint: (source joint, ratio)} from the model's joint equalities: motor shafts
    geared to the arm joints, cam plates to their motor shafts."""
    return {m.joint(m.eq_obj1id[i]).name: (m.joint(m.eq_obj2id[i]).name, float(m.eq_data[i, 1]))
            for i in range(m.neq) if m.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT}


def set_arm_pose(m, d, q):
    """Put the 5 arm joints at q (rad) with every motor shaft and cam plate on its coupling."""
    val = dict(zip(JOINTS, q))
    pend = couplings(m)
    for _ in range(len(pend)):
        for jn, (src, k) in list(pend.items()):
            if src in val:
                val[jn] = k * val[src]
                del pend[jn]
    for jn, v in val.items():
        d.qpos[m.jnt_qposadr[m.joint(jn).id]] = v


def quintic(s):
    """Minimum-jerk time scaling: position, velocity, acceleration factors for s in [0, 1]."""
    s = min(max(s, 0.0), 1.0)
    return (10 * s**3 - 15 * s**4 + 6 * s**5, (30 * s**2 - 60 * s**3 + 30 * s**4), (60 * s - 180 * s**2 + 120 * s**3))


def build_timeline():
    """Return a list of segments (t0, T, q_from, q_to, stagger, hold)."""
    segs, t = [], SHOW[0][1]          # rest in the first pose before the show starts
    q = np.radians(POSE[SHOW[0][0]])
    for name, hold, speed, stagger in SHOW[1:]:
        qn = np.radians(POSE[name])
        dq = np.abs(qn - q)
        # quintic peak velocity = 1.875*dq/T, peak acceleration = 5.774*dq/T^2
        T = max(0.25, float(np.max(np.maximum(1.875 * dq / (VMAX * speed), np.sqrt(5.774 * dq / (AMAX * speed * speed))))))
        segs.append((t, T, q.copy(), qn.copy(), stagger, name))
        t += T + stagger * 4 + hold
        q = qn
    return segs, t + 0.5


SEGS, DURATION = build_timeline()
START = np.radians(POSE[SHOW[0][0]])


def reference(t):
    """Desired arm joint position, velocity and acceleration at time t."""
    q = START.copy(); qd = np.zeros(5); qdd = np.zeros(5)
    for t0, T, qa, qb, stagger, _ in SEGS:
        if t < t0:
            break
        for j in range(5):
            tj = t - t0 - stagger * j
            if tj <= 0:
                continue
            p, v, a = quintic(tj / T)
            q[j] = qa[j] + (qb[j] - qa[j]) * p
            qd[j] = (qb[j] - qa[j]) * v / T if tj < T else 0.0
            qdd[j] = (qb[j] - qa[j]) * a / T / T if tj < T else 0.0
    return q, qd, qdd


class Arm:
    def __init__(self, xml, drive='ideal', sensor='joint'):
        self.m = mujoco.MjModel.from_xml_path(xml)
        self.d = mujoco.MjData(self.m)
        self.sim = None
        m = self.m
        self.qadr = [m.jnt_qposadr[m.joint(j).id] for j in JOINTS]
        self.act = [m.actuator(a).id for a in ACTUATORS]
        # first-order setpoint smoothing on the stepper actuators (timeconst) -> feed it forward
        self.tc = np.array([m.actuator_dynprm[a, 0] if m.actuator_dyntype[a] != mujoco.mjtDyn.mjDYN_NONE else 0.0 for a in self.act])
        self.limit = np.array([m.actuator_forcerange[a, 1] for a in self.act])
        # velocity feed-forward: the position loop's damping (kv) and the motors' speed-dependent
        # torque drop (joint damping) both pull back in proportion to speed; lead the command by that
        kp = np.array([m.actuator_gainprm[a, 0] for a in self.act])
        kv = np.array([-m.actuator_biasprm[a, 2] for a in self.act])
        b = np.array([m.dof_damping[m.jnt_dofadr[m.joint(j).id]] for j in JOINTS])
        L = (kv + b) / kp
        self.lead = self.tc + L           # x velocity
        self.lead2 = self.tc * L          # x acceleration (the damping lead also passes through the setpoint filter)
        if drive != 'ideal':
            # Linux streams the show to the MCU, which drives the steppers (stepper_pid_sim.py)
            import stepper_pid_sim
            self.sim = stepper_pid_sim.drive(self.m, lambda t: np.degrees(reference(max(t, 0.0))[0]), DURATION, drive, sensor)
            self.d = self.sim.d
            self.title = f'drive: {drive}'
        self.reset()

    def reset(self):
        m, d = self.m, self.d
        if self.sim:
            self.sim.reset()
            return
        mujoco.mj_resetData(m, d)
        set_arm_pose(m, d, START)          # incl. motor shafts and cam plates, on their couplings
        d.ctrl[self.act] = START
        k = 0
        for i, a in enumerate(self.act):
            if self.tc[i] > 0:
                d.act[m.actuator_actadr[a]] = START[i]
        mujoco.mj_forward(m, d)

    def control(self, t):
        q, qd, qdd = reference(t)
        self.d.ctrl[self.act] = q + self.lead * qd + self.lead2 * qdd   # exact inverse of filter + damping
        return q

    def step(self):
        """Advance one timestep: ideal motors, or the simulated stepper drives."""
        if self.sim:
            self.sim.step()
        else:
            self.control(self.d.time)
            mujoco.mj_step(self.m, self.d)

    def arm_q(self):
        return self.d.qpos[self.qadr].copy()


def run_report(xml, drive='ideal', sensor='joint'):
    arm = Arm(xml, drive, sensor)
    m, d = arm.m, arm.d
    if arm.sim:
        import stepper_pid_sim as S
        r = S.metrics(arm.sim.run())
        print(f'Show length {DURATION:.1f} s through the simulated drives ({drive}, sensor on {sensor}), '
              f'streamed from Linux at {S.SEND_HZ} Hz. Error vs the intended motion, constant delay '
              f'({1000 * r["delay"]:.0f} ms) removed:')
        for j in range(4):
            print(f'{JOINTS[j]:10s} rms {r["rms"][j]:5.2f} deg  max {r["max"][j]:6.2f} deg  peak motor torque '
                  f'{100 * r["tq_peak"][j]:4.0f}% of pull-out  lost steps {r["slips"][j]}')
        print('contacts during the show: ' + (', '.join(f'{a}-{b}' for a, b in r['contacts']) if r['contacts'] else 'none'))
        return
    n = int(DURATION / m.opt.timestep)
    err_max = np.zeros(5); err_sq = np.zeros(5); frc_max = np.zeros(5); sat_time = np.zeros(5)
    contacts = set()
    for i in range(n):
        t = i * m.opt.timestep
        q = arm.control(t)
        mujoco.mj_step(m, d)
        e = np.degrees(arm.arm_q() - q)
        err_max = np.maximum(err_max, np.abs(e)); err_sq += e * e
        f = np.abs(d.actuator_force[arm.act])
        frc_max = np.maximum(frc_max, f)
        sat_time += (f >= 0.99 * arm.limit) * m.opt.timestep
        for k in range(d.ncon):
            c = d.contact[k]
            contacts.add(tuple(sorted((m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name))))
    rms = np.sqrt(err_sq / n)
    print(f'Show length {DURATION:.1f} s, {len(SEGS)} moves.')
    print(f'{"joint":10s} {"max err":>9s} {"rms err":>9s} {"peak torque":>13s} {"of limit":>9s} {"time at limit":>14s}')
    for j in range(5):
        print(f'{JOINTS[j]:10s} {err_max[j]:7.2f} deg {rms[j]:7.3f} deg {frc_max[j]:8.2f} N*m {100 * frc_max[j] / arm.limit[j]:7.0f}% {sat_time[j]:11.2f} s')
    print('contacts during the show: ' + (', '.join(f'{a}-{b}' for a, b in sorted(contacts)) if contacts else 'none'))


def run_video(xml, out, fps=30, w=1280, h=720, drive='ideal', sensor='joint'):
    arm = Arm(xml, drive, sensor)
    m, d = arm.m, arm.d
    r = mujoco.Renderer(m, h, w)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.10, 0.0, 0.23]; cam.distance = 0.82; cam.azimuth = 150; cam.elevation = -10
    ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{w}x{h}', '-r', str(fps), '-i', '-',
                           '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', out], stdin=subprocess.PIPE)
    try:
        from PIL import Image, ImageDraw, ImageFont
        font = ImageFont.load_default(size=30)
    except Exception:
        Image = None
    steps_per_frame = int(round(1.0 / fps / m.opt.timestep))
    nframes = int(DURATION * fps)
    for f in range(nframes):
        for _ in range(steps_per_frame):
            arm.step()
        r.update_scene(d, cam)
        frame = r.render()
        if Image is not None:
            label = SHOW[0][0]
            for seg in SEGS:
                if seg[0] <= d.time: label = seg[5]
            img = Image.fromarray(frame); dr = ImageDraw.Draw(img)
            dr.text((28, 22), f'{label.replace("_", " ")}', fill=(255, 255, 255), font=font, stroke_width=3, stroke_fill=(30, 30, 30))
            dr.text((28, h - 52), f't = {d.time:5.1f} s', fill=(255, 255, 255), font=font, stroke_width=3, stroke_fill=(30, 30, 30))
            frame = np.asarray(img)
        ff.stdin.write(frame.tobytes())
    ff.stdin.close(); ff.wait()
    print(f'wrote {out} ({DURATION:.1f} s)')


def run_viewer(xml, drive='ideal', sensor='joint'):
    import mujoco.viewer
    arm = Arm(xml, drive, sensor)
    m, d = arm.m, arm.d
    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.lookat[:] = [0.10, 0.0, 0.23]; v.cam.distance = 0.82; v.cam.azimuth = 150; v.cam.elevation = -10
        while v.is_running():
            arm.reset()
            t_wall = time.time()
            while v.is_running() and d.time < DURATION:
                arm.step()
                if d.time > time.time() - t_wall:      # keep real time
                    if arm.sim:
                        import stepper_pid_sim
                        v.set_texts(stepper_pid_sim.drive_overlay(arm.sim, arm.title))
                    v.sync()
                    time.sleep(max(0.0, d.time - (time.time() - t_wall)))
            time.sleep(0.5)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--report', action='store_true')
    ap.add_argument('--video')
    ap.add_argument('--xray', action='store_true', help='use robot_arm_xray.xml (see-through shells)')
    ap.add_argument('--drive', choices=['ideal', 'pid', 'pid_tuned', 'smooth'], default='ideal',
                    help='ideal motors, or the simulated stepper drivers + STM32 loop with this controller')
    ap.add_argument('--sensor', choices=['joint', 'motor', 'none'], default='joint', help='AS5600 placement (with --drive)')
    ap.add_argument('--current', type=float, help='with --drive: driver peak current / rated (1.41 = TMC2209 at rated RMS)')
    ap.add_argument('--vbus', type=float, help='with --drive: motor supply voltage')
    a = ap.parse_args()
    if a.current or a.vbus:
        import stepper_pid_sim
        stepper_pid_sim.CURRENT = a.current or stepper_pid_sim.CURRENT
        stepper_pid_sim.VBUS = a.vbus or stepper_pid_sim.VBUS
    xml = os.path.join(HERE, 'robot_arm_xray.xml' if a.xray else 'robot_arm.xml')
    os.chdir(HERE)
    if a.report:
        run_report(xml, a.drive, a.sensor)
    elif a.video:
        run_video(xml, a.video, drive=a.drive, sensor=a.sensor)
    else:
        run_viewer(xml, a.drive, a.sensor)
