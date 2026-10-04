#!/usr/bin/env python3
"""Stepper drivers + the STM32 control loop (Arduino UNO Q), simulated on robot_arm.xml.

robot_arm.xml drives the joints with idealised position actuators. This script replaces them
with the real chain, per stepper joint (yaw, shoulder, elbow, wrist):

  Linux side   streams joint targets over the Bridge (SEND_HZ, integer degrees, like
               apps/expressive_robot) or sends single goal poses
  STM32 loop   every 1/LOOP_HZ: reads the AS5600s one after another over I2C, runs the
               controller, sets each motor's step rate
  step timer   an interrupt at STEP_TICK_HZ emits STEP pulses, at most one per tick per motor
  driver       microstepping current chopper: the phase currents follow the step count
               until the supply voltage runs out (winding inductance + back-EMF at speed)
  motor        hybrid stepper, 50 rotor teeth: torque = Kt * i * sin(load angle) + detent,
               rotor inertia. Pushed past pull-out it really slips, 4 full steps at a time
  gearbox      cycloid (or the 5:1 yaw spur gear) with backlash, torsional stiffness and
               dry friction, between the rotor and the arm joint
  AS5600       12 bit, slow-filter lag and noise from the datasheet, magnet-misalignment
               nonlinearity, mounted on the joint output or on the motor shaft
The head servo keeps its own internal position loop (the model's position actuator).

Controllers (--controller):
  pid      PID on (target - measured angle) -> joint speed -> step rate, with a speed limit.
           The usual first version; set your own gains with --kp --ki --kd.
  pid_tuned  the same PID plus what a stepper needs: a speed ramp, braking in time, an I-zone,
           a deadband and stall recovery, with gains from tune_stepper.py (PID_TUNED)
  smooth   recommended: the MCU plans a smooth trajectory to each target, issues exactly its
           steps (feed-forward), and the sensor only trims the slow error (sag, backlash)
           through a deadband. A sudden error = lost steps: re-sync and glide back.

  python3  stepper_pid_sim.py --report                    compare controllers x sensor placements
  python3  stepper_pid_sim.py --report --kp 20 --ki 2      ... with your own PID gains
  python3  stepper_pid_sim.py --plot step.png             big-step traces (needs matplotlib)
  mjpython stepper_pid_sim.py --controller pid --sensor joint --scenario step   live viewer

Requires:  pip install mujoco numpy   (matplotlib for --plot)
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
D = math.radians
TWO_PI = 2 * math.pi

# ----------------------------------------------------------------------------- CONFIG
STEPPERS = ['yaw', 'shoulder', 'elbow', 'wrist']
ACTUATOR = dict(yaw='base_motor', shoulder='shoulder_motor', elbow='elbow_motor', wrist='wrist_motor')
GEAR = dict(yaw=-5.0, shoulder=16.0, elbow=-16.0, wrist=-16.0)   # motor angle = GEAR * joint angle
MOTOR_OF = dict(yaw='17HS4401', shoulder='17HS4401', elbow='17HS4401', wrist='17HS4023')

# Motors. hold: holding torque with both phases at rated current [N*m]; i_rated [A/phase];
# R [ohm], L [H] per phase; J rotor inertia [kg*m^2]; detent [N*m]; b rotor losses [N*m*s/rad].
MOTORS = {
    '17HS4401': dict(hold=0.40, i_rated=1.7, R=1.5, L=2.8e-3, J=54e-7, detent=0.022, b=1e-4),
    # 17HS4023: torque and current from the listing; R, L, J and detent are typical values (assumed)
    '17HS4023': dict(hold=0.155, i_rated=1.3, R=2.0, L=2.0e-3, J=20e-7, detent=0.012, b=5e-5),
}
STEPS_PER_REV = 200            # 1.8 deg motors (50 rotor teeth)

# Driver
MICROSTEPS = 16
VBUS = 12.0                    # motor supply [V]
CURRENT = 1.0                  # driver peak phase current / rated current.
#   A4988 / DRV8825 with Vref set to the rated current -> 1.0 (gives 71% of holding torque)
#   TMC2209 with its RMS current set to the rated current -> 1.41 (full holding torque)

# Gearboxes, at the arm joint. Printed parts: GUESSES - measure yours (lock the motor, hang a
# known weight on the link, read the joint AS5600) and put the numbers here.
# The links themselves are rigid here; their flex would add to STIFFNESS.
BACKLASH_DEG = dict(yaw=0.2, shoulder=0.3, elbow=0.3, wrist=0.3)         # total play
STIFFNESS = dict(yaw=2000.0, shoulder=1000.0, elbow=1000.0, wrist=500.0)  # N*m/rad
FRICTION = dict(yaw=0.10, shoulder=0.15, elbow=0.15, wrist=0.08)      # N*m dry friction
GEAR_ZETA = 0.3                # contact damping in the gearbox (printed parts are lossy: a clunk
                               # through the backlash rebounds ~40%)

# AS5600 (datasheet: slow-filter setting -> step-response delay, RMS noise)
AS5600_SF = 0                  # 0 = 16x (power-up default), 1 = 8x, 2 = 4x, 3 = 2x
SF_DELAY = (2.2e-3, 1.1e-3, 0.55e-3, 0.286e-3)
SF_NOISE_DEG = (0.015, 0.021, 0.030, 0.043)
INL_DEG = (0.25, 0.15)         # 1st / 2nd harmonic error from an off-centre, tilted magnet
I2C_READ_US = 200              # one sensor read incl. switching a TCA9548A channel, 400 kHz

# Firmware
LOOP_HZ = 1000                 # control loop
STEP_TICK_HZ = 40000           # step-pulse timer interrupt (max step rate per motor)
SEND_HZ = 20                   # Linux -> MCU target stream (apps/expressive_robot/python/main.py)
SEND_INT_DEG = True            # Bridge.notify("set_joints", [round(a) ...]) sends whole degrees
STREAM_FILTER_HZ = 6.0         # smooth: 2nd-order filter on the stream (removes the 20 Hz corners)
PID = dict(kp=15.0,            # (deg/s) per deg of error
           ki=5.0,             # (deg/s) per (deg*s)
           kd=0.0,             # (deg/s) per (deg/s)
           i_limit=20.0,       # deg*s, integrator clamp
           max_speed=120.0,    # deg/s at the joint (MAX_SPEED_DEG_S in robot_config.h); number or per joint
           max_accel=0.0,      # deg/s^2 ramp on the speed command, 0 = none; number or per joint
           brake=False,        # also cap speed at sqrt(2 * max_accel * |error|), so it can stop in time
           i_zone=0.0,         # only integrate when |error| < i_zone deg (0 = always)
           deadband=0.0,       # deg, treat smaller errors as zero (sensor noise, backlash)
           ff=False,           # add the target stream's own speed (feed-forward)
           resync=False)       # stall recovery: if the joint falls RESYNC_DEG behind what was
                               # commanded, the motor has stalled - drop the speed to 0 and ramp again
# pid_tuned: the PID above with the fixes a stepper needs, tuned by tune_stepper.py on this model
# (12 V, driver current = rated). Speed / acceleration limits per joint: yaw, shoulder, elbow, wrist.
PID_TUNED = dict(kp=8.0, ki=2.0, kd=0.0, deadband=0.1, i_zone=3.0, i_limit=2.0, brake=True, resync=True,
                 ff=False, max_speed=[86, 58, 72, 86], max_accel=[345, 230, 345, 575])
# smooth: per-joint limits for the planner (deg/s, deg/s^2), same as lamp_demo.py
VMAX_DEG = dict(yaw=172.0, shoulder=115.0, elbow=143.0, wrist=172.0, head_tilt=258.0)
AMAX_DEG = dict(yaw=690.0, shoulder=460.0, elbow=690.0, wrist=1150.0, head_tilt=1720.0)
TRIM = dict(ki=4.0,            # 1/s: how fast the sensor trims the slow error (sag, backlash)
            deadband=0.1,      # deg, ignore errors inside this (sensor noise, quantisation)
            rate=5.0)          # deg/s, fastest slow-trim correction
RESYNC_DEG = 2.0               # an error this big means lost steps (or a push): restart from where
                               # the joint really is and glide back to the plan (min-jerk)
RECOVER = (0.5, 0.25)          # glide-back speed and acceleration, as fractions of VMAX / AMAX

SUBSTEP_HZ = 40000             # physics of the drives (electrical + rotor)
WARMUP = 0.5                   # s of holding before the scenario starts (gravity settles)
# ------------------------------------------------------------------------------------

POSES = {                      # (yaw, shoulder, elbow, wrist, head_tilt) deg, from lamp_demo.py
    'look': (0, 15, 45, 55, 0),
    'peek_left': (55, 50, 15, 60, 0),
    'sleep': (0, 10, 90, 80, 0),
}


def scenario(name):
    """Return (stream, duration, events | stream function, push)."""
    if name == 'step':
        # big jumps, like dragging a slider: look -> lean in and turn -> back
        ev = [(0.0, POSES['look']), (0.5, POSES['peek_left']), (4.0, POSES['look'])]
        return dict(stream=False, duration=7.5, events=ev, push=None)
    if name == 'small':
        # small corrections: 3 deg, then 10 deg the other way, then back (checks for hunting)
        a = POSES['look']
        ev = [(0.0, a), (0.5, tuple(x + d for x, d in zip(a, (3, 3, -3, 3, 0)))),
              (2.5, tuple(x + d for x, d in zip(a, (-10, -10, 10, -10, 0)))), (4.5, a)]
        return dict(stream=False, duration=6.5, events=ev, push=None)
    if name == 'push':
        # someone presses down on the lamp head for half a second
        return dict(stream=False, duration=4.0, events=[(0.0, POSES['look'])],
                    push=(1.0, 1.5, 'head_tilt', np.array([0.0, 0.0, -20.0])))
    if name == 'boot':
        # power-up after someone moved the arm by hand while it was off. The firmware last
        # parked it in 'sleep'; it is really at `actual`. Then Linux asks for 'look'.
        return dict(stream=False, duration=4.0, events=[(0.0, POSES['sleep']), (0.5, POSES['look'])],
                    actual=(-30, 20, 75, 70, 0), push=None)
    if name == 'show':
        sys.path.insert(0, HERE)
        import lamp_demo
        ref = lambda t: np.degrees(lamp_demo.reference(t)[0])
        return dict(stream=True, duration=lamp_demo.DURATION, ref=ref, push=None)
    raise ValueError(name)


def deadzone(x, b):
    return x - b if x > b else (x + b if x < -b else 0.0)


def quintic_coeffs(p0, v0, a0, p1, T):
    """Quintic from (p0, v0, a0) to (p1, 0, 0) in time T."""
    dp = p1 - p0
    return (p0, v0, a0 / 2,
            (20 * dp - 12 * v0 * T - 3 * a0 * T * T) / (2 * T ** 3),
            (-30 * dp + 16 * v0 * T + 3 * a0 * T * T) / (2 * T ** 4),
            (12 * dp - 6 * v0 * T - a0 * T * T) / (2 * T ** 5))


def quintic_eval(c, t):
    c0, c1, c2, c3, c4, c5 = c
    p = c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * c5))))
    v = c1 + t * (2 * c2 + t * (3 * c3 + t * (4 * c4 + t * 5 * c5)))
    a = 2 * c2 + t * (6 * c3 + t * (12 * c4 + t * 20 * c5))
    return p, v, a


class Firmware:
    """What runs on the STM32. It only sees sensor counts and the targets from Linux; it
    outputs step rates. Angles in degrees at the joint, like the sketch."""

    def __init__(self, controller, sensor, start_deg, sc, pid=None):
        self.ctl, self.sensor, self.sc = controller, sensor, sc
        self.pid = dict(PID, **(pid or {}))
        self.names = STEPPERS + ['head_tilt']
        self.spd = [abs(GEAR[j]) * STEPS_PER_REV * MICROSTEPS / 360.0 for j in STEPPERS]  # microsteps per joint deg
        self.sign = [math.copysign(1, GEAR[j]) for j in STEPPERS]
        self.target = list(start_deg)          # last target received from Linux
        self.prev_target = list(start_deg)
        self.t_recv = -1.0
        self.integ = [0.0] * 4; self.e_prev = [0.0] * 4; self.u_prev = [0.0] * 4
        self.cmd = list(start_deg[:4])         # pid: where the issued steps should have put each joint
        # commanded joint angle = plan + trim. trim holds the step-count offset and the slow
        # correction (trim_goal); after lost steps it glides back to trim_goal along rec
        self.trim = [0.0] * 4; self.trim_goal = [0.0] * 4; self.rec = [None] * 4
        self.fp = list(start_deg); self.fv = [0.0] * 5   # stream filter state
        # planner state (smooth, goal mode)
        self.seg_t0 = 0.0; self.seg_T = 1e-9
        self.seg = [quintic_coeffs(p, 0, 0, p, 1.0) for p in start_deg]
        self.ref_hist = []                     # (t, ref) for latency-matched trim
        # sensor bookkeeping
        self.counts_prev = [None] * 4; self.turns = [0] * 4
        self.meas = list(start_deg[:4])
        self.start = list(start_deg)

    def adopt_measured(self):
        """Joint sensors are absolute: start from where the arm really is."""
        self.start[:4] = self.meas[:4]
        for lst in (self.target, self.prev_target, self.fp, self.cmd):
            lst[:4] = self.meas[:4]
        self.seg = [quintic_coeffs(p, 0, 0, p, 1.0) for p in self.start]

    def boot(self, n_issued):
        # the step counter starts wherever the motors are: fold that offset into the trim so the
        # first command doesn't jump (the gearbox wind-up stays, as on the real arm)
        for k in range(4):
            self.trim[k] = self.trim_goal[k] = n_issued[k] / self.spd[k] * self.sign[k] - self.start[k]

    # -- Linux -> MCU
    def receive(self, t, targets):
        self.prev_target, self.target = self.target, list(targets)
        self.t_recv = t
        if self.ctl == 'smooth' and not self.sc['stream']:
            self.replan(t)

    def replan(self, t):
        cur = [quintic_eval(c, min(t - self.seg_t0, self.seg_T)) for c in self.seg]
        T = 0.2
        for k, j in enumerate(self.names):
            dq = abs(self.target[k] - cur[k][0])
            T = max(T, 1.875 * dq / VMAX_DEG[j], math.sqrt(5.774 * dq / AMAX_DEG[j]))
        self.seg = [quintic_coeffs(cur[k][0], cur[k][1] if t - self.seg_t0 < self.seg_T else 0.0,
                                   cur[k][2] if t - self.seg_t0 < self.seg_T else 0.0, self.target[k], T)
                    for k in range(5)]
        self.seg_t0, self.seg_T = t, T

    def advance(self, t, dt):
        """smooth: the planned joint angles (deg) now and at the next tick."""
        if not self.sc['stream']:
            plan = lambda tt: [quintic_eval(c, min(max(tt - self.seg_t0, 0.0), self.seg_T))[0] for c in self.seg]
            return plan(t), plan(t + dt)
        # stream: interpolate between the last two samples (one send period late), then a
        # critically damped 2nd-order filter, so the corners every 1/SEND_HZ don't shake the arm.
        # Its speed and acceleration are capped at the joint limits: a safety net for any clip.
        s = min(max((t + dt - self.t_recv) * SEND_HZ, 0.0), 1.0)
        x = [a + (b - a) * s for a, b in zip(self.prev_target, self.target)]
        w = TWO_PI * STREAM_FILTER_HZ
        now = list(self.fp)
        for k, j in enumerate(self.names):
            acc = w * w * (x[k] - self.fp[k]) - 2 * w * self.fv[k]
            acc = min(max(acc, -AMAX_DEG[j]), AMAX_DEG[j])
            self.fv[k] = min(max(self.fv[k] + acc * dt, -VMAX_DEG[j]), VMAX_DEG[j])
            self.fp[k] += self.fv[k] * dt
        return now, list(self.fp)

    # -- AS5600 counts -> joint angle
    def read(self, k, counts):
        if self.counts_prev[k] is None:
            # boot: joint sensor is absolute within a turn; a motor sensor only knows the
            # rotor angle, so assume the arm was homed (start pose) to pick the motor turn
            if self.sensor == 'joint':
                self.turns[k] = 0 if counts < 2048 else -1
            else:
                exp_turn = GEAR[STEPPERS[k]] * self.start[k] / 360.0
                self.turns[k] = round(exp_turn - (counts + 0.5) / 4096)
        else:
            dc = counts - self.counts_prev[k]
            if dc > 2048: self.turns[k] -= 1
            elif dc < -2048: self.turns[k] += 1
        self.counts_prev[k] = counts
        ang = (self.turns[k] * 4096 + counts + 0.5) * 360.0 / 4096
        self.meas[k] = ang if self.sensor == 'joint' else ang / GEAR[STEPPERS[k]]

    # -- control loop
    def control(self, t, dt, n_issued, n_acc):
        """Return (step rates [microsteps/s] for the 4 motors, head target deg, ref or None)."""
        rates = [0.0] * 4
        if self.ctl == 'pid':
            p = self.pid
            per = lambda v, k: v[k] if isinstance(v, (list, tuple)) else v
            for k in range(4):
                e = self.target[k] - self.meas[k] if self.sensor != 'none' else 0.0
                e = deadzone(e, p['deadband'])
                if not p['i_zone'] or abs(e) < p['i_zone']:
                    self.integ[k] = min(max(self.integ[k] + e * dt, -p['i_limit']), p['i_limit'])
                else:
                    self.integ[k] = 0.0
                u = p['kp'] * e + p['ki'] * self.integ[k] + p['kd'] * (e - self.e_prev[k]) / dt
                self.e_prev[k] = e
                if p['ff'] and self.sc['stream']:
                    u += (self.target[k] - self.prev_target[k]) * SEND_HZ
                vmax, amax = per(p['max_speed'], k), per(p['max_accel'], k)
                if p['brake'] and amax > 0:
                    vmax = min(vmax, math.sqrt(2 * amax * abs(e)))
                u = min(max(u, -vmax), vmax)
                if amax > 0:
                    u = min(max(u, self.u_prev[k] - amax * dt), self.u_prev[k] + amax * dt)
                if p['resync'] and self.sensor != 'none' and abs(self.cmd[k] - self.meas[k]) > RESYNC_DEG:
                    u = 0.0                            # stalled: stop, re-sync, ramp up again
                    self.cmd[k] = self.meas[k]
                self.cmd[k] += u * dt
                self.u_prev[k] = u
                rates[k] = u * self.spd[k] * self.sign[k]
            return rates, self.target[4], None
        # smooth
        ref_now, ref_next = self.advance(t, dt)
        self.ref_hist.append((t, ref_now))
        if len(self.ref_hist) > 64:
            del self.ref_hist[0]
        if self.sensor != 'none':
            # compare with the reference from when the sensor actually sampled (filter lag)
            lag = SF_DELAY[AS5600_SF] / 2.2 + 2 * I2C_READ_US * 1e-6
            back = max(0, len(self.ref_hist) - 1 - int(round(lag / dt)))
            ref_seen = self.ref_hist[back][1]
            for k in range(4):
                tv = ta = 0.0                              # glide velocity / acceleration
                if self.rec[k] is not None:
                    c, t0, T = self.rec[k]
                    self.trim[k], tv, ta = quintic_eval(c, min(t - t0, T))
                    if t - t0 >= T:
                        self.rec[k] = None; tv = ta = 0.0
                # where the joint should be now, including any glide-back in progress
                e = ref_seen[k] + (self.trim[k] - self.trim_goal[k]) - self.meas[k]
                if abs(e) > RESYNC_DEG:
                    # lost steps: the step count is now off by e. Re-sync it, and glide from where
                    # the motor is to the plan instead of jumping (a jump would skip again)
                    j = STEPPERS[k]
                    self.trim_goal[k] += e
                    dist = abs(self.trim_goal[k] - self.trim[k])
                    T = max(0.3, 1.875 * dist / (RECOVER[0] * VMAX_DEG[j]),
                            math.sqrt(5.774 * dist / (RECOVER[1] * AMAX_DEG[j])))
                    self.rec[k] = (quintic_coeffs(self.trim[k], tv, ta, self.trim_goal[k], T), t, T)
                elif self.rec[k] is None:
                    step = TRIM['ki'] * deadzone(e, TRIM['deadband']) * dt
                    self.trim_goal[k] += min(max(step, -TRIM['rate'] * dt), TRIM['rate'] * dt)
                    self.trim[k] = self.trim_goal[k]
        for k in range(4):
            n_goal = (ref_next[k] + self.trim[k]) * self.spd[k] * self.sign[k]
            rates[k] = (n_goal - n_issued[k] - n_acc[k]) / dt
        return rates, ref_next[4], ref_now


class Sim:
    def __init__(self, controller='smooth', sensor='joint', scenario_name='step', xml=None, seed=1, pid=None):
        self.sc = scenario(scenario_name)
        self.controller, self.sensor, self.name = controller, sensor, scenario_name
        self.m = m = mujoco.MjModel.from_xml_path(xml or os.path.join(HERE, 'robot_arm.xml'))
        self.d = mujoco.MjData(m)
        self.rng = np.random.default_rng(seed)
        self.qadr = [m.jnt_qposadr[m.joint(j).id] for j in STEPPERS + ['head_tilt']]
        self.dadr = [m.jnt_dofadr[m.joint(j).id] for j in STEPPERS + ['head_tilt']]
        self.head_act = m.actuator('head_servo').id
        # take the idealised stepper actuators out: torque now comes from the drive model below
        for j in STEPPERS:
            a = m.actuator(ACTUATOR[j]).id
            m.actuator_gainprm[a, :] = 0; m.actuator_biasprm[a, :] = 0
            dof = m.jnt_dofadr[m.joint(j).id]
            m.dof_armature[dof] = 0.0                  # rotor inertia is simulated explicitly now
            m.dof_damping[dof] = 0.0                   # back-EMF too
            m.dof_frictionloss[dof] = FRICTION[j]
        self.dts = 1.0 / SUBSTEP_HZ
        self.nsub = int(round(m.opt.timestep * SUBSTEP_HZ))
        self.tick_every = max(1, int(round(SUBSTEP_HZ / STEP_TICK_HZ)))
        # per-motor constants
        self.P = []
        for j in STEPPERS:
            mo = MOTORS[MOTOR_OF[j]]
            kt = mo['hold'] / (math.sqrt(2) * mo['i_rated'])
            g = GEAR[j]
            k = STIFFNESS[j]
            self.P.append(dict(g=g, kt=kt, ipk=CURRENT * mo['i_rated'], R=mo['R'], L=mo['L'], J=mo['J'],
                               td=mo['detent'], b=mo['b'], k=k, bl=D(BACKLASH_DEG[j]) / 2,
                               c=2 * GEAR_ZETA * math.sqrt(k * mo['J'] * g * g),
                               tpk=kt * CURRENT * mo['i_rated']))
        self.phase = [(math.cos(i * math.pi / 2 / MICROSTEPS), math.sin(i * math.pi / 2 / MICROSTEPS))
                      for i in range(4 * MICROSTEPS)]
        self.inl = [(D(INL_DEG[0]), self.rng.uniform(0, TWO_PI), D(INL_DEG[1]), self.rng.uniform(0, TWO_PI))
                    for _ in STEPPERS]
        self.pid_over = pid
        self.reset()

    # ---------------------------------------------------------------- state
    def set_pose(self, q5):
        m, d = self.m, self.d
        full = dict(zip(STEPPERS + ['head_tilt'], q5))
        for jn, (src, k) in {'base_motor_shaft': ('yaw', -5), 'shoulder_motor_shaft': ('shoulder', 16),
                             'elbow_motor_shaft': ('elbow', -16), 'wrist_motor_shaft': ('wrist', -16)}.items():
            full[jn] = k * full[src]
        for drive in ('shoulder', 'elbow', 'wrist'):
            for x in 'abc':
                full[f'{drive}_plate_{x}'] = -full[f'{drive}_motor_shaft']
        for jn, v in full.items():
            d.qpos[m.jnt_qposadr[m.joint(jn).id]] = v

    def reset(self):
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        sc = self.sc
        start = list(sc['ref'](0.0)) if sc['stream'] else list(sc['events'][0][1])   # firmware's belief
        q0 = [D(a) for a in sc.get('actual', start)]                                 # where the arm is
        self.set_pose(q0)
        d.ctrl[self.head_act] = q0[4]
        mujoco.mj_forward(m, d)
        # drive state: start in static equilibrium - each gearbox already wound up by the gravity
        # torque on its joint, each rotor lagging its step position by the matching load angle
        self.thr, self.n = [], []
        for k in range(4):
            p = self.P[k]
            tau = float(d.qfrc_bias[self.dadr[k]])            # gravity torque the drive must hold
            x = tau / p['k'] + math.copysign(p['bl'], tau)    # gearbox wind-up at the joint
            th = p['g'] * (q0[k] + x)
            delta = math.asin(min(max(tau / p['g'] / p['tpk'], -1.0), 1.0))
            self.thr.append(th)
            self.n.append(int(round((50.0 * th + delta) / (math.pi / 2 / MICROSTEPS))))
        self.w = [0.0] * 4
        self.ia = [self.P[k]['ipk'] * self.phase[self.n[k] % (4 * MICROSTEPS)][0] for k in range(4)]
        self.ib = [self.P[k]['ipk'] * self.phase[self.n[k] % (4 * MICROSTEPS)][1] for k in range(4)]
        self.acc = [0.0] * 4
        self.rate = [0.0] * 4
        self.slip_k = [0] * 4
        self.slips = [0] * 4                       # full steps lost
        self.sfilt = [q0[k] if self.sensor == 'joint' else self.thr[k] for k in range(4)]
        self.fw = Firmware(self.controller, self.sensor, start, sc, self.pid_over)
        self.last_ref = None
        self.ks = 0                                # substep counter
        self.loop_n = int(round(SUBSTEP_HZ / LOOP_HZ))
        self.read_n = max(1, int(round(I2C_READ_US * 1e-6 * SUBSTEP_HZ)))
        if self.sensor != 'none' and 4 * self.read_n >= self.loop_n:
            print(f'warning: 4 I2C reads ({4 * I2C_READ_US} us) do not fit in the {1e6 / LOOP_HZ:.0f} us loop')
        self.t_off = -WARMUP
        self.events = [] if sc['stream'] else list(sc['events'])
        self.next_send = 0.0
        self.log = []
        self.contacts = set()
        # boot: one read of each sensor; joint sensors tell the firmware the real pose
        if self.sensor != 'none':
            for k in range(4):
                self.fw.read(k, self.sample(k))
            if self.sensor == 'joint':
                self.fw.adopt_measured()
        self.fw.boot(self.n)

    # ---------------------------------------------------------------- AS5600
    def sample(self, k):
        y = self.sfilt[k]
        a1, p1, a2, p2 = self.inl[k]
        ang = y + a1 * math.sin(y + p1) + a2 * math.sin(2 * y + p2) + D(SF_NOISE_DEG[AS5600_SF]) * self.rng.standard_normal()
        return int(math.floor((ang % TWO_PI) / TWO_PI * 4096)) & 4095

    # ---------------------------------------------------------------- one MuJoCo step
    def step(self):
        m, d, P = self.m, self.d, self.P
        t = d.time + self.t_off
        # Linux side
        sc, fw = self.sc, self.fw
        if sc['stream']:
            if t >= self.next_send:
                tg = sc['ref'](max(t, 0.0))
                fw.receive(t, [round(a) for a in tg] if SEND_INT_DEG else list(tg))
                self.next_send += 1.0 / SEND_HZ
        else:
            while self.events and t >= self.events[0][0]:
                fw.receive(t, list(self.events.pop(0)[1]))
        # external push
        push = sc['push']
        if push:
            b = m.body(push[2]).id
            d.xfrc_applied[b, :3] = push[3] if push[0] <= t < push[1] else 0.0

        q = [float(d.qpos[a]) for a in self.qadr[:4]]
        qd = [float(d.qvel[a]) for a in self.dadr[:4]]
        dts, nsub, tick_every = self.dts, self.nsub, self.tick_every
        thr, w, ia, ib, n, acc, rate = self.thr, self.w, self.ia, self.ib, self.n, self.acc, self.rate
        phase, nph = self.phase, 4 * MICROSTEPS
        sfilt = self.sfilt
        alpha = dts / (SF_DELAY[AS5600_SF] / 2.2)
        joint_sensor = self.sensor == 'joint'
        tau_sum = [0.0] * 4
        tq_max = [0.0] * 4
        V = VBUS
        for s in range(nsub):
            ks = self.ks
            # firmware: sensor reads one after another, then the control law
            ph = ks % self.loop_n
            if self.sensor != 'none' and ph < 4 * self.read_n and ph % self.read_n == 0:
                k = ph // self.read_n
                fw.read(k, self.sample(k))
            if ph == (4 * self.read_n if self.sensor != 'none' else 0):
                tc = t + s * dts
                dt = 1.0 / LOOP_HZ
                rates, head, ref = fw.control(tc, dt, n, acc)
                for k in range(4):
                    rate[k] = min(max(rates[k], -STEP_TICK_HZ), STEP_TICK_HZ)
                d.ctrl[self.head_act] = D(head)
                self.last_ref = ref
            tick = ks % tick_every == 0
            for k in range(4):
                p = P[k]
                if tick:
                    a = acc[k] + rate[k] * dts * tick_every
                    if a >= 1.0:
                        n[k] += 1; a -= 1.0
                    elif a <= -1.0:
                        n[k] -= 1; a += 1.0
                    acc[k] = a
                cr, sr = phase[n[k] % nph]
                iar = p['ipk'] * cr; ibr = p['ipk'] * sr
                th = thr[k]; om = w[k]
                the = 50.0 * th
                c = math.cos(the); sn = math.sin(the)
                ke = p['kt']
                ea = -ke * om * sn; eb = ke * om * c
                R, L = p['R'], p['L']
                va = R * ia[k] + ea + L * (iar - ia[k]) / dts
                vb = R * ib[k] + eb + L * (ibr - ib[k]) / dts
                va = V if va > V else (-V if va < -V else va)      # out of supply voltage
                vb = V if vb > V else (-V if vb < -V else vb)
                ia[k] += dts * (va - R * ia[k] - ea) / L
                ib[k] += dts * (vb - R * ib[k] - eb) / L
                s2 = 2 * sn * c; c2 = c * c - sn * sn
                tm = ke * (ib[k] * c - ia[k] * sn) - p['td'] * 2 * s2 * c2 - p['b'] * om
                # gearbox: deflection seen at the joint, with backlash
                g = p['g']
                x = th / g - (q[k] + qd[k] * s * dts)
                bl = p['bl']
                if x > bl:
                    tg = p['k'] * (x - bl) + p['c'] * (om / g - qd[k])
                elif x < -bl:
                    tg = p['k'] * (x + bl) + p['c'] * (om / g - qd[k])
                else:
                    tg = 0.0
                om += dts * (tm - tg / g) / p['J']
                thr[k] = th + dts * om
                w[k] = om
                tau_sum[k] += tg
                atm = abs(tm) / p['tpk']
                if atm > tq_max[k]: tq_max[k] = atm
                sfilt[k] += alpha * ((q[k] + qd[k] * s * dts if joint_sensor else thr[k]) - sfilt[k])
            self.ks += 1
        for k in range(4):
            d.qfrc_applied[self.dadr[k]] = tau_sum[k] / nsub
            # lost steps: the rotor fell a whole electrical cycle (4 full steps) behind/ahead
            lag = n[k] * (math.pi / 2 / MICROSTEPS) - 50.0 * thr[k]
            kk = int(round(lag / TWO_PI))
            if kk != self.slip_k[k]:
                self.slips[k] += 4 * abs(kk - self.slip_k[k]); self.slip_k[k] = kk
        mujoco.mj_step(m, d)
        if t >= 0:
            for i in range(d.ncon):
                c = d.contact[i]
                b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
                self.contacts.add(tuple(sorted((m.body(b1).name, m.body(b2).name))))
        return t, tq_max

    def run(self, until=None, record_every=2):
        m, d = self.m, self.d
        until = self.sc['duration'] if until is None else until
        i = 0
        while d.time + self.t_off < until:
            t, tq = self.step()
            if t >= 0 and i % record_every == 0:
                self.record(t, tq)
            i += 1
        return self

    def record(self, t, tq):
        d, fw = self.d, self.fw
        q = [math.degrees(d.qpos[a]) for a in self.qadr]
        cmd = [self.n[k] / fw.spd[k] * fw.sign[k] for k in range(4)]
        ideal = list(self.sc['ref'](t)) if self.sc['stream'] else None
        ref = self.last_ref
        self.log.append(dict(t=t, q=q, target=list(fw.target), ref=list(ref) if ref else None, cmd=cmd,
                             meas=list(fw.meas), slips=list(self.slips), tq=list(tq), ideal=ideal))


# ---------------------------------------------------------------------- analysis
def metrics(sim):
    L = sim.log
    t = np.array([r['t'] for r in L])
    q = np.array([r['q'] for r in L])
    tq = np.array([r['tq'] for r in L])
    free = np.ones(len(L), bool)
    if sim.sc['push']:                          # while being pushed the motors are overloaded on purpose
        free = (t < sim.sc['push'][0]) | (t > sim.sc['push'][1] + 0.3)
    out = {'slips': list(sim.slips), 'contacts': sorted(sim.contacts), 'tq_peak': tq.max(0),
           'tq_time': float((tq[free].max(1) > 0.9).sum() * (t[1] - t[0]))}
    if sim.name == 'show':
        # error against the intended motion, after removing the constant delay of the stream
        ideal = np.array([r['ideal'] for r in L])[:, :4]
        dt = t[1] - t[0]
        best = None
        for n in range(0, int(0.25 / dt), max(1, int(0.005 / dt))):
            e = q[n:, :4] - ideal[:len(ideal) - n]
            rms = float(np.sqrt((e ** 2).mean()))
            if best is None or rms < best[0]:
                best = (rms, n * dt, np.sqrt((e ** 2).mean(0)), np.abs(e).max(0))
        out['delay'], out['rms'], out['max'] = best[1], best[2], best[3]
        return out
    if sim.name in ('push', 'boot'):
        goal = np.array(sim.sc['events'][-1][1], float)
        late = t > (sim.sc['push'][1] + 2.0 if sim.sc['push'] else sim.sc['duration'] - 0.5)
        out['final'] = np.abs(q[late][:, :4] - goal[:4]).mean(0)
        return out
    # step: per jump, overshoot / settle time / final error / residual wobble
    ev = sim.sc['events']
    res = []
    for i in range(1, len(ev)):
        t0, goal = ev[i][0], np.array(ev[i][1], float)
        t1 = ev[i + 1][0] if i + 1 < len(ev) else sim.sc['duration']
        sel = (t >= t0) & (t < t1)
        tt, qq = t[sel], q[sel][:, :4]
        start = np.array(ev[i - 1][1], float)[:4]
        over = np.maximum(0, ((qq - goal[:4]) * np.sign(goal[:4] - start)).max(0))
        err = np.abs(qq - goal[:4])
        settle = np.zeros(4)
        for k in range(4):
            bad = np.where(err[:, k] > 0.5)[0]
            if len(bad):
                settle[k] = np.inf if bad[-1] == len(tt) - 1 else tt[bad[-1] + 1] - t0
        tail = tt > t1 - 0.5
        res.append(dict(over=over, settle=settle, final=err[tail].mean(0), osc=qq[tail].max(0) - qq[tail].min(0)))
    out['jumps'] = res
    return out


CONFIGS = [   # (controller, sensor, PID changes, label, CONFIG changes)
    ('pid', 'joint', {}, 'PID -> step rate, sensor on joint', {}),
    ('pid', 'joint', {'max_accel': 300.0}, 'PID + speed ramp, sensor on joint', {}),
    ('pid', 'joint', PID_TUNED, 'tuned PID, sensor on joint', {}),
    ('pid', 'motor', PID_TUNED, 'tuned PID, sensor on motor', {}),
    ('smooth', 'joint', None, 'planned + feed-forward, sensor on joint', {}),
    ('smooth', 'joint', None, '  same, joint sensor calibrated', {'INL_DEG': (0.0, 0.0)}),
    ('smooth', 'motor', None, 'planned + feed-forward, sensor on motor', {}),
    ('smooth', 'none', None, 'planned, open loop (no sensor)', {}),
]


def run_config(xml, cfg, scn, pid):
    ctl, sen, change, label, over = cfg
    p = dict(change or {}, **(pid or {}))   # your --kp/--ki/... win over the row's changes
    g = globals()
    saved = {k: g[k] for k in over}
    g.update(over)
    try:
        return Sim(ctl, sen, scn, xml, pid=p).run()
    finally:
        g.update(saved)


def run_report(xml, pid, scenarios=('step', 'boot', 'push', 'show')):
    p = dict(PID, **(pid or {}))
    print(f'Supply {VBUS:g} V, {MICROSTEPS} microsteps, driver current {CURRENT:g} x rated, loop {LOOP_HZ} Hz, '
          f'step timer {STEP_TICK_HZ / 1000:g} kHz, AS5600 slow filter {SF_DELAY[AS5600_SF] * 1e3:g} ms')
    print(f'PID: kp {p["kp"]:g}, ki {p["ki"]:g}, kd {p["kd"]:g}, speed limit {p["max_speed"]} deg/s; '
          f'"speed ramp" rows add a 300 deg/s^2 ramp. Tuned PID: {PID_TUNED}')
    print('"calibrated" = AS5600 nonlinearity removed.')
    print('Angles are the TRUE joint angles (what you see), not what the sensor says. Lost steps: yaw/shoulder/elbow/wrist.\n')
    W = 40
    for scn in scenarios:
        if scn == 'step':
            print('STEP  target jumps look -> turn and lean in -> back, like dragging a slider')
            print(f'  {"":{W}s} {"overshoot":>10s} {"settled":>8s} {"end err":>8s} {"wobble":>7s}  {"lost steps":<20s} {"torque":>6s}')
        elif scn == 'push':
            print(f'PUSH  {sim_push_force():g} N pressing down on the head for 0.5 s, then let go')
            print(f'  {"":{W}s} {"error 2 s after letting go (deg)":>34s}  {"lost steps":<20s} {"torque":>6s}')
        elif scn == 'boot':
            b = scenario('boot')
            print(f'BOOT  moved by hand while off (really at {b["actual"][:4]}, firmware thinks {b["events"][0][1][:4]}), '
                  'then asked for "look"')
            print(f'  {"":{W}s} {"final error y/s/e/w (deg)":>34s}  {"lost steps":<20s} {"torque":>6s}')
        else:
            print(f'SHOW  lamp_demo streamed from Linux at {SEND_HZ} Hz{" in whole degrees" if SEND_INT_DEG else ""}; '
                  'error vs the intended motion, constant delay removed')
            print(f'  {"":{W}s} {"rms error y/s/e/w (deg)":>26s} {"max":>6s} {"delay":>6s}  {"lost steps":<20s} {"torque":>6s}')
        for cfg in CONFIGS:
            sim = run_config(xml, cfg, scn, pid)
            r = metrics(sim)
            lost = '/'.join(str(s) for s in r['slips'])
            hits = [f'{a}-{b}' for a, b in r['contacts']]
            tail = f'  {lost:<20s} {100 * r["tq_peak"].max():5.0f}%' + (f'  HIT {", ".join(hits)}' if hits else '')
            if scn == 'step':
                j = r['jumps']
                over = max(x['over'].max() for x in j); settle = max(x['settle'].max() for x in j)
                end = max(x['final'].max() for x in j); osc = max(x['osc'].max() for x in j)
                st = 'never' if not np.isfinite(settle) else f'{settle:.2f} s'
                print(f'  {cfg[3]:{W}s} {over:8.1f} d {st:>8s} {end:6.2f} d {osc:5.2f} d' + tail)
            elif scn in ('push', 'boot'):
                print(f'  {cfg[3]:{W}s} {" ".join(f"{x:6.2f}" for x in r["final"]):>34s}' + tail)
            else:
                print(f'  {cfg[3]:{W}s} {" ".join(f"{x:5.2f}" for x in r["rms"]):>26s} {r["max"].max():6.2f} '
                      f'{1000 * r["delay"]:4.0f}ms' + tail)
        print()


def sim_push_force():
    return float(-scenario('push')['push'][3][2])


def run_plot(xml, out, pid, scn='step'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    cols = (0, 1, 2)
    fig, axes = plt.subplots(len(CONFIGS), len(cols), figsize=(15, 2.4 * len(CONFIGS)), sharex=True)
    for row, cfg in enumerate(CONFIGS):
        sim = run_config(xml, cfg, scn, pid)
        t = np.array([r['t'] for r in sim.log])
        q = np.array([r['q'] for r in sim.log]); tg = np.array([r['target'] for r in sim.log])
        cmd = np.array([r['cmd'] for r in sim.log])
        for col, k in enumerate(cols):
            ax = axes[row, col]
            ax.plot(t, tg[:, k], color='0.55', lw=1, ls='--', label='target from Linux')
            if sim.log[0]['ref'] is not None:
                ax.plot(t, [r['ref'][k] for r in sim.log], color='tab:green', lw=1, label='MCU plan')
            ax.plot(t, cmd[:, k], color='tab:orange', lw=0.8, alpha=0.8, label='steps sent')
            ax.plot(t, q[:, k], color='tab:blue', lw=1.4, label='actual joint')
            lost = sim.slips[k]
            ax.set_title(f'{STEPPERS[k]}: {cfg[3]}' + (f'  ({lost} steps lost)' if lost else ''), fontsize=9,
                         color='tab:red' if lost else 'black')
            lo, hi = min(tg[:, k].min(), q[:, k].min()), max(tg[:, k].max(), q[:, k].max())
            ax.set_ylim(lo - 0.15 * (hi - lo + 1) - 2, hi + 0.15 * (hi - lo + 1) + 2)
            ax.set_ylabel('deg'); ax.grid(alpha=0.3)
    axes[0, 0].legend(fontsize=7, loc='lower right')
    for ax in axes[-1]:
        ax.set_xlabel('time (s)')
    fig.tight_layout(); fig.savefig(out, dpi=100)
    print(f'wrote {out}')


def run_viewer(xml, ctl, sen, scn, pid, title=None):
    import mujoco.viewer
    sim = Sim(ctl, sen, scn, xml, pid=pid)
    m, d = sim.m, sim.d
    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.lookat[:] = [0.10, 0.0, 0.23]; v.cam.distance = 0.9; v.cam.azimuth = 150; v.cam.elevation = -10
        while v.is_running():
            sim.reset()
            wall0 = time.time()
            last_txt = 0
            while v.is_running() and d.time + sim.t_off < sim.sc['duration']:
                t, tq = sim.step()
                if d.time > time.time() - wall0:
                    if time.time() - last_txt > 0.1:
                        last_txt = time.time()
                        lines = [f'{title or ctl} / sensor: {sen} / {scn}   t = {t:5.2f} s']
                        for k, j in enumerate(STEPPERS):
                            q = math.degrees(d.qpos[sim.qadr[k]])
                            lines.append(f'{j:9s} target {sim.fw.target[k]:7.1f}  actual {q:7.1f}  lost steps {sim.slips[k]}')
                        try:
                            v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                         '\n'.join(lines), ''))
                        except Exception:
                            pass
                    v.sync()
                    time.sleep(max(0.0, d.time - (time.time() - wall0)))
            time.sleep(1.0)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--controller', choices=['pid', 'pid_tuned', 'smooth'], default='pid')
    ap.add_argument('--sensor', choices=['joint', 'motor', 'none'], default='joint')
    ap.add_argument('--scenario', choices=['step', 'small', 'boot', 'push', 'show'], default='step')
    ap.add_argument('--report', action='store_true', help='run every controller x sensor combination')
    ap.add_argument('--plot', help='write step-response traces to this PNG')
    ap.add_argument('--kp', type=float); ap.add_argument('--ki', type=float); ap.add_argument('--kd', type=float)
    ap.add_argument('--max-speed', type=float, help='PID speed limit, deg/s')
    ap.add_argument('--max-accel', type=float, help='PID speed ramp, deg/s^2 (0 = none)')
    ap.add_argument('--vbus', type=float, help=f'motor supply voltage (default {VBUS:g})')
    ap.add_argument('--microsteps', type=int, help=f'driver microstepping (default {MICROSTEPS})')
    ap.add_argument('--current', type=float, help=f'driver peak current / rated (default {CURRENT:g})')
    ap.add_argument('--xray', action='store_true', help='see-through shells in the viewer')
    a = ap.parse_args()
    if a.vbus: VBUS = a.vbus
    if a.microsteps: MICROSTEPS = a.microsteps
    if a.current: CURRENT = a.current
    pid = {k: v for k, v in dict(kp=a.kp, ki=a.ki, kd=a.kd, max_speed=a.max_speed, max_accel=a.max_accel).items() if v is not None}
    title = a.controller
    if a.controller == 'pid_tuned':
        a.controller, pid = 'pid', dict(PID_TUNED, **pid)
    xml = os.path.join(HERE, 'robot_arm_xray.xml' if a.xray else 'robot_arm.xml')
    if a.report:
        run_report(xml, pid)
    elif a.plot:
        run_plot(xml, a.plot, pid, a.scenario)
    else:
        run_viewer(xml, a.controller, a.sensor, a.scenario, pid, title)
