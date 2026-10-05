#!/usr/bin/env python3
"""ELEGNT-style expressive + functional movement for the lamp robot.

Based on: Hu, Huang, Sivapurapu, Zhang, "ELEGNT: Expressive and Functional Movement
Design for Non-anthropomorphic Robot", arXiv:2501.12493 (2025).

The paper plans robot motion that maximises  F(tau) + gamma * E(tau):
  F = functional utility (reach the goal pose efficiently),
  E = expressive utility (convey intention, attention, attitude, emotion),
  gamma = how much expression is mixed in.
Here every scenario is a script of functional actions (always performed) plus expressive
actions (glances, gaze, lean, head tilt, nod, shake, hesitation, stretch, droop, bounce,
dance), whose amplitude and timing are scaled by gamma. gamma = 0 is the paper's
"function-driven" robot, gamma = 1 its "expression-driven" robot.

The six scenarios are the paper's user-study tasks: photograph light, project assistance,
failure indication, remind water, social conversation and play music.

  python3  elegnt_demo.py --video elegnt.mp4          side-by-side video, all scenarios
  python3  elegnt_demo.py --report                    tracking/torque report, both gammas
  mjpython elegnt_demo.py --scenario water --gamma 1  live in the MuJoCo viewer

  --drive pid | pid_tuned | smooth    play it through the simulated stepper drivers and STM32
                                      loop of stepper_pid_sim.py instead of ideal motors
  --sensor joint | motor | none       where the AS5600s are (with --drive)
  --current 1.41 / --vbus 24          driver current (x rated) and supply voltage (with --drive)
"""
import argparse
import math
import os
import re
import subprocess
import sys
import time

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from lamp_demo import JOINTS, ACTUATORS, VMAX, AMAX, quintic  # noqa: E402

YAW, SH, EL, WR, TILT = range(5)
WRIST_SAFE = (math.radians(-110), math.radians(140))   # beyond this the shade folds into the arm
TILT_SAFE = math.radians(75)

# ------------------------------------------------------------------ scene (world frame, m)
USER_POS = np.array([0.70, 0.0, 0.0])
USER_HEAD = USER_POS + [0.0, 0.0, 0.47]
HAND_REST = np.array([0.56, -0.17, 0.03])
PLANT = np.array([0.27, 0.15, 0.0])
PLANT_LOOK = PLANT + [0.0, 0.0, 0.09]
CUP = np.array([0.24, -0.13, 0.05])
PAPER = np.array([0.38, 0.03, 0.0])
FAR = np.array([0.95, 0.45, 0.10])          # out of reach (a "bookshelf" off-table)


def scene_xml(xray=False):
    src = open(os.path.join(HERE, 'robot_arm_xray.xml' if xray else 'robot_arm.xml')).read()
    src = src.replace('meshdir="meshes"', 'meshdir="%s"' % os.path.join(HERE, 'meshes'))
    src = re.sub(r'\s*<keyframe>.*?</keyframe>', '', src, flags=re.S)     # nq changes (cup)
    src = src.replace('<visual>', '<visual><headlight ambient="0.22 0.22 0.22" diffuse="0.22 0.22 0.22" specular="0.05 0.05 0.05"/>', 1)
    src = src.replace('diffuse="0.8 0.8 0.8"', 'diffuse="0.5 0.5 0.5"').replace('diffuse="0.35 0.35 0.35"', 'diffuse="0.18 0.18 0.18"')
    # the lamp's light, inside the shade, shining out of the opening (head frame -Y)
    src = re.sub(r'(<joint name="head_tilt"[^>]*/>)', r'\1\n<light name="lamp_light" pos="0 -0.05 0" dir="0 -1 0" cutoff="38" exponent="6" '
                 r'diffuse="1.0 0.86 0.6" specular="0.15 0.15 0.15" castshadow="true" active="false"/>', src)
    skin = '0.92 0.78 0.66 1'
    objects = f'''
    <body name="user" pos="{USER_POS[0]} {USER_POS[1]} 0">
      <geom type="capsule" fromto="0 0 0.06 0 0 0.33" size="0.11" rgba="0.35 0.45 0.62 1" contype="0" conaffinity="0"/>
      <geom type="sphere" pos="0 0 0.47" size="0.085" rgba="{skin}" contype="0" conaffinity="0"/>
      <geom type="sphere" pos="-0.076 0.03 0.49" size="0.011" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0"/>
      <geom type="sphere" pos="-0.076 -0.03 0.49" size="0.011" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0"/>
      <site name="user_shoulder" pos="-0.03 -0.13 0.30" size="0.01" rgba="{skin}"/>
    </body>
    <body name="user_hand" mocap="true" pos="{HAND_REST[0]} {HAND_REST[1]} {HAND_REST[2]}">
      <geom type="sphere" size="0.028" rgba="{skin}" contype="0" conaffinity="0"/>
      <site name="user_hand_site" size="0.005" rgba="{skin}"/>
    </body>
    <body name="plant" pos="{PLANT[0]} {PLANT[1]} 0">
      <geom type="cylinder" pos="0 0 0.03" size="0.035 0.03" rgba="0.75 0.40 0.25 1"/>
      <geom type="sphere" pos="0 0 0.085" size="0.04" rgba="0.30 0.65 0.30 1"/>
      <geom type="ellipsoid" pos="0.02 0.015 0.115" size="0.025 0.02 0.03" rgba="0.36 0.72 0.36 1"/>
    </body>
    <geom name="paper" type="box" pos="{PAPER[0]} {PAPER[1]} 0.0008" size="0.075 0.105 0.0008" rgba="0.97 0.97 0.95 1" contype="0" conaffinity="0"/>
    <body name="cup" pos="{CUP[0]} {CUP[1]} {CUP[2]}">
      <freejoint name="cup_free"/>
      <geom name="cup" type="cylinder" size="0.034 0.05" rgba="0.55 0.75 0.95 1" mass="0.25" condim="4" friction="0.5 0.02 0.001" solref="0.004 1" solimp="0.95 0.99 0.001"/>
    </body>
'''
    tendon = f'''  <tendon>
    <spatial name="user_arm" width="0.021" rgba="{skin}"><site site="user_shoulder"/><site site="user_hand_site"/></spatial>
  </tendon>
'''
    src = src.replace('  </worldbody>', objects + '  </worldbody>\n' + tendon, 1)
    return src


# ------------------------------------------------------------------ lamp kinematics / gaze
class LampKin:
    """Planar 2-link arm + wrist + sideways head joint, measured from the model.

    Shade direction in the arm frame (forward, left, up) after yaw:
        d = (cos(tilt) sin(phi), -sin(tilt), cos(tilt) cos(phi)),  phi = shoulder+elbow+wrist
    """

    def __init__(self, m):
        d = mujoco.MjData(m)
        mujoco.mj_resetData(m, d)
        mujoco.mj_forward(m, d)
        x = lambda n: d.xpos[m.body(n).id].copy()
        self.S = x('first_link')
        self.L1 = x('second_link')[2] - self.S[2]
        self.L2 = x('head')[2] - x('second_link')[2]
        self.h = x('head_tilt')[2] - x('head')[2]

    def wrist(self, q):
        a1, a2 = q[SH], q[SH] + q[EL]
        return self.L1 * math.sin(a1) + self.L2 * math.sin(a2), self.S[2] + self.L1 * math.cos(a1) + self.L2 * math.cos(a2)

    def _aim(self, yaw, Wr, Wz, P):
        f = np.array([math.cos(yaw), math.sin(yaw)]); l = np.array([-math.sin(yaw), math.cos(yaw)])
        rel = np.asarray(P)[:2] - self.S[:2]
        rP, yP, zP = rel @ f, rel @ l, P[2]
        phi = math.atan2(rP - Wr, zP - Wz)
        for _ in range(6):
            Tr, Tz = Wr + self.h * math.sin(phi), Wz + self.h * math.cos(phi)
            phi = math.atan2(rP - Tr, zP - Tz)
        dist = math.sqrt((rP - Tr) ** 2 + yP ** 2 + (zP - Tz) ** 2)
        return phi, math.asin(max(-1.0, min(1.0, -yP / dist)))

    def ik(self, reach, height):
        dr, dz = reach, height - self.S[2]
        D = math.hypot(dr, dz); Dmax = self.L1 + self.L2 - 1e-3
        if D > Dmax:
            dr, dz, D = dr * Dmax / D, dz * Dmax / D, Dmax
        c2 = (D * D - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
        th2 = math.acos(max(-1.0, min(1.0, c2)))           # elbow bends forward (lamp-like)
        th1 = math.atan2(dr, dz) - math.atan2(self.L2 * math.sin(th2), self.L1 + self.L2 * math.cos(th2))
        return th1, th2

    @staticmethod
    def _near(a, ref):
        return ref + (a - ref + math.pi) % (2 * math.pi) - math.pi

    def look(self, P, reach, height, tilt=0.0, yaw=None, ref=None):
        """Whole-body pose: wrist at (reach, height) in the arm plane, shade aimed at P."""
        y = math.atan2(P[1] - self.S[1], P[0] - self.S[0]) if yaw is None else yaw
        if ref is not None:
            y = self._near(y, ref[YAW])
        th1, th2 = self.ik(reach, height)
        Wr, Wz = self.wrist([0, th1, th2])
        phi, dl = self._aim(y, Wr, Wz, P)
        return np.array([y, th1, th2, phi - th1 - th2, max(-2.3, min(2.3, dl + tilt))])

    def glance(self, q, P, tilt=0.0):
        """Head-only gaze shift: keep yaw/shoulder/elbow, re-aim wrist + head."""
        Wr, Wz = self.wrist(q)
        phi, dl = self._aim(q[YAW], Wr, Wz, P)
        out = np.array(q, float)
        out[WR] = min(max(phi - q[SH] - q[EL], WRIST_SAFE[0]), WRIST_SAFE[1])   # head-only: never fold back
        out[TILT] = max(-TILT_SAFE, min(TILT_SAFE, dl + tilt))
        return out

    def place_head(self, T, phi, yaw=None, ref=None):
        """Put the head pivot at world point T with the shade pitched at phi (no side tilt)."""
        y = math.atan2(T[1] - self.S[1], T[0] - self.S[0]) if yaw is None else yaw
        if ref is not None:
            y = self._near(y, ref[YAW])
        r = math.hypot(T[0] - self.S[0], T[1] - self.S[1])
        th1, th2 = self.ik(r - self.h * math.sin(phi), T[2] - self.h * math.cos(phi))
        return np.array([y, th1, th2, phi - th1 - th2, 0.0])

    def reach_of(self, P):
        return math.hypot(P[0] - self.S[0], P[1] - self.S[1])


# ------------------------------------------------------------------ choreography
class Show:
    """A timeline: functional moves (always) + expressive moves (scaled by gamma)."""

    def __init__(self, kin, gamma, q0, title, note):
        self.kin, self.g, self.title, self.note = kin, float(gamma), title, note
        self.q0 = np.array(q0, float); self.q = self.q0.copy(); self.t = 0.0
        self.segs, self.oscs, self.says, self.lights = [], [], [], [(0.0, False)]
        self.hand, self.scribbles, self.beats = [], [], []
        self.marks = []                               # (t, label) for the report

    # -- timing
    def at(self, t):
        self.t = max(self.t, t)

    def hold(self, dt, E=False):
        self.t += dt * (self.g if E else 1.0)

    # -- moves
    def move(self, q, speed=1.0, stagger=0.0, E=False, min_T=0.3, label=None):
        if E and self.g <= 0:
            return
        q = np.array(q, float)
        if E:
            q = self.q + self.g * (q - self.q)          # expression amount ~ gamma
        dq = np.abs(q - self.q)
        T = max(min_T, float(np.max(np.maximum(1.875 * dq / (VMAX * speed), np.sqrt(5.774 * dq / (AMAX * speed * speed))))))
        self.segs.append((self.t, T, self.q.copy(), q.copy(), stagger))
        if label:
            self.marks.append((self.t, label))
        self.t += T + 4 * stagger
        self.q = q

    def osc(self, amp, freq, cycles, phase=None, E=True, label=None):
        """Oscillation overlay (nod, shake, bounce, wag, tremble, dance) while holding the base pose."""
        if E and self.g <= 0:
            return
        amp = np.array(amp, float) * (self.g if E else 1.0)
        w = 2 * math.pi * freq
        amp = np.clip(amp, -0.8 * VMAX / w, 0.8 * VMAX / w)
        amp = np.clip(amp, -0.8 * AMAX / w / w, 0.8 * AMAX / w / w)
        ph = np.zeros(5) if phase is None else np.array(phase, float)
        dur = cycles / freq
        self.oscs.append((self.t, dur, amp, freq, ph))
        if label:
            self.marks.append((self.t, label))
        self.t += dur

    # -- primitives (paper section 3.2: intention, attention, attitude, emotion)
    def glance(self, P, speed=1.0, tilt=0.0, E=True, label=None):
        self.move(self.kin.glance(self.q, P, tilt), speed=speed, E=E, label=label)

    def look(self, P, reach, height, tilt=0.0, speed=1.0, stagger=0.0, E=False, label=None):
        self.move(self.kin.look(P, reach, height, tilt, ref=self.q), speed=speed, stagger=stagger, E=E, label=label)

    def gaze_shift(self, P, reach, height, tilt=0.0, speed=1.0, E=True, label=None):
        """Head leads, body follows (intention): quick head glance, then whole body re-aims."""
        self.glance(P, speed=min(1.0, speed * 1.4), E=E, label=label)
        self.hold(0.12, E=E)
        self.look(P, reach, height, tilt, speed=speed, stagger=0.04, E=E)

    def nod(self, n=2, amp=0.16, freq=2.0, E=True):
        self.osc([0, 0, 0.25 * amp, amp, 0], freq, n, phase=[0, 0, -0.6, 0, 0], E=E, label='nod')

    def shake(self, n=2.5, amp=0.24, freq=1.6, E=True):
        self.osc([0.06, 0, 0, 0, amp], freq, n, phase=[0, 0, 0, 0, 0.4], E=E, label='head shake')

    def bounce(self, n=4, freq=2.0, E=True):          # light, bouncy = happiness / excitement (+ tail wag)
        self.osc([0.06, -0.05, 0.12, 0.10, 0.20], freq, n, phase=[0, 0, 0, 1.2, 0.6], E=E, label='bouncy joy')

    def tremble(self, dur=0.8, E=True):               # strain while stretching
        self.osc([0, 0.01, 0.01, 0.02, 0.02], 5.0, dur * 5.0, phase=[0, 0, 1.0, 2.0, 0.5], E=E, label='strain')

    def hesitate(self, q_goal, E=True):               # pauses + jerky partial starts
        if E and self.g <= 0:
            return
        q_start = self.q.copy()
        self.hold(0.45, E=E)
        self.move(q_start + 0.35 * (q_goal - q_start), speed=1.3, E=E, label='hesitation')
        self.hold(0.40, E=E)
        self.move(q_start + 0.22 * (q_goal - q_start), speed=1.1, E=E)
        self.hold(0.30, E=E)

    def relax(self, E=True):                          # "sitting down" = relaxation: slow, low, breathing
        P = self.kin.S + np.array([0.20, 0.0, -self.kin.S[2]])
        self.look(P, reach=0.07, height=0.25, tilt=0.12, speed=0.35, E=E, label='relaxed: sit down')
        self.osc([0, 0.03, -0.04, 0.03, 0], 0.35, 1.0, phase=[0, 0, 0.3, 0.6, 0], E=E)

    def startle(self, P_threat, E=True):              # fear: sudden, jerky avoidance + tremble
        self.look(P_threat, reach=0.0, height=0.40, speed=1.6, E=E, label='startled: avoid')
        self.tremble(0.6, E=E)
        self.hold(0.4, E=E)

    def look_away(self, P, E=True):                   # disinterest: point the head away from the stimulus
        rel = np.asarray(P)[:2] - self.kin.S[:2]
        a = math.atan2(rel[1], rel[0]) + 2.0
        away = self.kin.S + np.array([0.3 * math.cos(a), 0.3 * math.sin(a), -self.kin.S[2] + 0.05])
        self.look(away, reach=0.12, height=0.31, speed=0.45, stagger=0.06, E=E, label='uninterested: turn away')

    def dance(self, bpm, beats, E=True):
        fb = bpm / 60.0
        self.beats.append((self.t, self.t + beats / fb, bpm))
        self.osc([0.16, 0.05, 0.10, 0.12, 0.22], fb, beats, phase=[0, 0, 0, 1.4, 0.7], E=E, label='dance')

    def sway(self, bpm, beats, E=True):               # slow side sway layered on the dance
        fb = bpm / 60.0
        self.osc([0.20, 0.04, 0, 0, 0.18], fb / 4.0, beats / 4.0, phase=[0, 1.57, 0, 0, 0.8], E=E)

    # -- light, speech, user
    def light(self, on):
        self.lights.append((self.t, bool(on)))

    def say(self, text, dur=2.6, wait=False):
        self.says.append((self.t, self.t + dur, 'Lamp', text))
        if wait:
            self.t += dur

    def user_say(self, t0, text, dur=2.6):
        self.says.append((t0, t0 + dur, 'User', text))

    def user_hand(self, t0, T, P):
        self.hand.append((t0, T, np.array(P, float)))

    def user_scribble(self, t0, t1, radius=0.018, freq=1.3, drift=(0.0, 0.05)):
        self.scribbles.append((t0, t1, radius, freq, np.array([drift[0], drift[1], 0.0])))

    @property
    def end(self):
        e = self.t
        for t0, t1, *_ in self.says:
            e = max(e, t1)
        for t0, T, _ in self.hand:
            e = max(e, t0 + T)
        return e + 0.8

    # -- evaluation
    def _q(self, t):
        q = self.q0.copy()
        for t0, T, qa, qb, st in self.segs:
            if t < t0:
                break
            for j in range(5):
                tj = t - t0 - st * j
                if tj > 0:
                    q[j] = qa[j] + (qb[j] - qa[j]) * quintic(tj / T)[0]
        for t0, dur, amp, f, ph in self.oscs:
            tau = t - t0
            if 0.0 <= tau <= dur:
                ramp = min(1.0, tau / 0.18, (dur - tau) / 0.18)
                env = math.sin(0.5 * math.pi * max(0.0, ramp)) ** 2
                q = q + env * amp * np.sin(2 * math.pi * f * tau + ph)
        return q

    def ref(self, t, h=0.002):
        a, b, c = self._q(t - h), self._q(t), self._q(t + h)
        return b, (c - a) / (2 * h), (c - 2 * b + a) / (h * h)

    def light_at(self, t):
        on = False
        for t0, v in self.lights:
            if t0 <= t:
                on = v
        return on

    def hand_at(self, t):
        p = HAND_REST.copy()
        for t0, T, P in self.hand:
            if t >= t0:
                s = quintic((t - t0) / T)[0]
                p = p + (P - p) * s
        for t0, t1, r, f, drift in self.scribbles:
            if t0 <= t <= t1:
                tau = t - t0
                env = min(1.0, tau / 0.3, (t1 - t) / 0.3)
                p = p + env * np.array([r * math.cos(2 * math.pi * f * tau), r * math.sin(2 * math.pi * f * tau), 0.004 * math.sin(4 * math.pi * f * tau)]) \
                    + drift * (tau / (t1 - t0))
        return p


# ------------------------------------------------------------------ the six scenarios
def idle_pose(kin):
    return kin.look(USER_HEAD, reach=0.13, height=0.36)


def light_pose(kin, P, ref):
    return kin.look(P, reach=max(0.06, kin.reach_of(P) - 0.15), height=0.38, ref=ref)


def sc_photo(kin, g):
    s = Show(kin, g, idle_pose(kin), 'Photograph light', 'Light the plant for a photo. E: curiosity (lean in), attention to the command (look back at the user).')
    s.user_say(0.4, 'Can you light up my plant for a photo?', 2.6)
    s.user_hand(1.0, 0.7, [0.42, 0.11, 0.15]); s.user_hand(3.2, 0.8, HAND_REST)
    s.at(1.7)                                         # gesture detected
    s.glance(USER_HEAD, label='look back at user')    # attention to the instructive gesture
    s.nod(n=1, amp=0.12, freq=2.2)                    # acknowledgement
    s.glance(PLANT_LOOK, speed=1.2, label='glance at plant')   # intention
    s.look(PLANT_LOOK, reach=kin.reach_of(PLANT) - 0.12, height=0.33, tilt=0.30, speed=0.65, E=True, label='curious lean-in + head tilt')
    s.hold(0.7, E=True)
    s.move(light_pose(kin, PLANT_LOOK, s.q), label='light the plant')   # functional goal
    s.light(True)
    s.osc([0, 0, 0.05, 0.06, 0], 1.6, 1, E=True)      # small satisfied settle
    s.hold(1.2)
    return s


def sc_project(kin, g):
    s = Show(kin, g, idle_pose(kin), 'Project assistance', 'Light/project onto the user\'s sketch. E: curiosity toward the activity, joint attention (gaze follows the hand).')
    s.user_say(0.4, 'Let me sketch out my idea...', 2.4)
    hand_on_paper = PAPER + [0.02, -0.02, 0.012]
    s.user_hand(0.8, 0.8, hand_on_paper); s.user_scribble(1.6, 9.0)
    s.at(1.8)
    s.glance(s.hand_at(2.0), label='glance at the hand')
    s.look(s.hand_at(2.6), reach=kin.reach_of(PAPER) - 0.13, height=0.31, tilt=-0.30, speed=0.7, E=True, label='curious lean-in')
    t = s.t
    for k in range(5):                                # joint attention: gaze tracks the hand
        s.glance(s.hand_at(t + 0.55 * (k + 1)), speed=0.8, E=True, label='joint attention' if k == 0 else None)
    s.move(light_pose(kin, PAPER, s.q), label='move to projection pose')
    s.light(True)
    s.nod(n=1, amp=0.10, freq=2.0)
    s.hold(1.0)
    return s


def sc_failure(kin, g):
    s = Show(kin, g, idle_pose(kin), 'Failure indication', 'Asked to light something out of reach. E: hesitation, stretching with effort, look back + head shake.')
    s.user_say(0.4, 'Can you light up the bookshelf over there?', 2.6)
    s.user_hand(0.9, 0.8, [0.58, 0.04, 0.24]); s.user_hand(3.4, 0.8, HAND_REST)
    s.at(1.8)
    q_reach = kin.look(FAR, reach=0.34, height=0.20, ref=s.q)      # fully extended toward the goal
    s.glance(FAR, speed=1.0, label='glance at target')
    s.hesitate(q_reach)
    s.move(q_reach, label='reach toward goal (limit)')
    q_stretch = q_reach + np.array([0.0, 0.06, -0.04, 0.10, 0.0])
    s.move(q_stretch, speed=0.5, E=True, label='stretch (effort)')
    s.tremble(0.9)
    s.move(q_reach, speed=0.6, E=True)
    s.gaze_shift(USER_HEAD, reach=0.24, height=0.28, label='look back at user')
    s.shake()
    s.say('Sorry - that is out of my reach.', 2.8, wait=True)
    return s


def sc_water(kin, g):
    s = Show(kin, g, kin.look(PAPER, reach=0.15, height=0.36), 'Remind water', 'Remind the user to drink. E: push the cup toward the user, gaze at them before speaking.')
    s.user_hand(0.0, 0.6, PAPER + [0.02, -0.02, 0.012]); s.user_scribble(0.6, 12.0)
    s.at(1.2)
    cup_look = CUP + [0, 0, 0.01]
    s.move(kin.look(cup_look, reach=kin.reach_of(CUP) - 0.15, height=0.38, ref=s.q), label='point at the cup')
    s.light(True)
    if g > 0:
        u = USER_HEAD[:2] - CUP[:2]; u = u / np.linalg.norm(u)
        back = CUP[:2] - (0.105 + 0.034 + 0.012) * u          # shade rim just behind the cup
        T_pre = np.array([back[0], back[1], 0.05 + 0.14])      # rim at cup mid-height
        s.move(kin.place_head(T_pre + [0, 0, 0.07], math.pi, ref=s.q), speed=0.8, E=True, label='push the cup')
        s.move(kin.place_head(T_pre, math.pi, ref=s.q), speed=0.5, E=True)
        T_push = T_pre.copy(); T_push[:2] += 0.085 * u
        s.move(kin.place_head(T_push, math.pi, ref=s.q), speed=0.35, E=True)
        s.move(kin.place_head(T_push + [0, 0, 0.09], math.pi, ref=s.q), speed=0.6, E=True)
        s.gaze_shift(USER_HEAD, reach=0.16, height=0.34, label='gaze at user')
        s.hold(0.3, E=True)
    s.say('Hey - time to drink some water!', 2.8, wait=True)
    s.hold(0.6)
    return s


def sc_social(kin, g):
    s = Show(kin, g, idle_pose(kin), 'Social conversation', 'F: verbal replies only. E: gaze, excitement (bouncy dance), sadness (lowered head), pointing at the object.')
    s.user_say(0.3, 'Guess what - I got the job!', 2.2)
    s.at(2.3)
    s.look(USER_HEAD, reach=0.10, height=0.40, speed=1.2, E=True, label='perk up')
    s.say('No way - that is amazing!', 2.2)
    s.bounce(n=5, freq=2.0)
    s.look(USER_HEAD, reach=0.13, height=0.36, E=True)
    s.at(5.6)
    s.user_say(5.6, '...but I will have to move away.', 2.4)
    s.at(8.0)
    s.look(np.array([0.22, -0.02, 0.0]), reach=0.12, height=0.27, tilt=0.15, speed=0.35, E=True, label='sad: lower the head')
    s.say('Oh... I will miss you.', 2.4)
    s.hold(1.6, E=True)
    s.at(10.6)
    s.look(USER_HEAD, reach=0.13, height=0.35, speed=0.5, E=True)
    s.user_say(11.0, 'Could you pass me that plant?', 2.2)
    s.at(13.2)
    s.gaze_shift(PLANT_LOOK, reach=0.20, height=0.35, label='point at the plant')   # >= 25 mm clear of the plant
    s.say('This one?', 1.8)
    s.nod(n=2, amp=0.10, freq=2.4)
    s.gaze_shift(USER_HEAD, reach=0.13, height=0.36, label='gaze back at user')
    s.at(15.4)
    return s


def sc_music(kin, g):
    s = Show(kin, g, idle_pose(kin), 'Play music', 'F: plays music, no movement. E: dances in time with the 100 BPM beat.')
    s.user_say(0.3, 'Play some music!', 1.8)
    s.at(1.6)
    s.say('(music playing - 100 BPM)', 11.0)
    s.at(2.0)
    groove = kin.look(USER_HEAD + [0, 0, -0.10], reach=0.15, height=0.33, ref=s.q)
    s.move(groove, speed=0.8, E=True, label='dance')
    if g > 0:
        t_dance = s.t
        s.dance(100, 16)                              # bounce + nod on every beat
        t_end = s.t
        s.t = t_dance
        s.sway(100, 16)                               # slower side-to-side sway, same beat grid
        s.t = max(t_end, s.t)
    s.at(12.8)
    s.move(idle_pose(kin), speed=0.7, E=True)
    s.at(13.2)
    return s


def sc_gallery(kin, g):
    s = Show(kin, g, idle_pose(kin), 'Gesture gallery', 'The rest of the paper\'s design space: sit/relax, approach (curiosity), avoid (fear), point away (disinterest).')
    T_THREAT, T_TALK = 10.5, 14.5                     # user actions at fixed times (same on both sides)
    threat = np.array([0.36, -0.02, 0.24])
    s.user_hand(T_THREAT, 0.35, threat); s.user_hand(T_THREAT + 1.6, 0.8, HAND_REST)
    s.user_say(T_TALK, 'Want to see my tax spreadsheet?', 2.4)
    mood = (lambda text, dur: s.say(text, dur)) if g > 0 else (lambda text, dur: None)
    s.at(0.5)
    mood('(relaxed)', 4.5)
    s.relax()
    s.look(USER_HEAD, reach=0.13, height=0.36, speed=0.6, E=True)
    mood('(curious)', 3.0)
    s.gaze_shift(PLANT_LOOK, reach=kin.reach_of(PLANT) - 0.12, height=0.33, tilt=0.35, speed=0.6, label='curious: approach')
    s.hold(0.5, E=True)
    s.at(T_THREAT + 0.15)
    mood('(startled!)', 1.8)
    s.startle(threat)
    s.look(USER_HEAD, reach=0.10, height=0.37, speed=0.4, E=True, label='cautious peek')
    s.at(T_TALK + 1.6)
    mood('(not really...)', 2.0)
    s.look_away(USER_HEAD)
    s.hold(1.2, E=True)
    s.look(USER_HEAD, reach=0.13, height=0.36, speed=0.6, E=True)
    s.hold(0.5)
    return s


SCENARIOS = {'photo': sc_photo, 'project': sc_project, 'failure': sc_failure,
             'water': sc_water, 'social': sc_social, 'music': sc_music, 'gallery': sc_gallery}


# ------------------------------------------------------------------ simulation
class Rig:
    def __init__(self, xml, show, drive='ideal', sensor='joint'):
        self.m = mujoco.MjModel.from_xml_string(xml)
        self.d = mujoco.MjData(self.m)
        self.show = show
        self.sim = None
        m = self.m
        self.qadr = [m.jnt_qposadr[m.joint(j).id] for j in JOINTS]
        self.act = [m.actuator(a).id for a in ACTUATORS]
        tc = np.array([m.actuator_dynprm[a, 0] if m.actuator_dyntype[a] != mujoco.mjtDyn.mjDYN_NONE else 0.0 for a in self.act])
        kp = np.array([m.actuator_gainprm[a, 0] for a in self.act])
        kv = np.array([-m.actuator_biasprm[a, 2] for a in self.act])
        b = np.array([m.dof_damping[m.jnt_dofadr[m.joint(j).id]] for j in JOINTS])
        self.tc, self.lead, self.lead2 = tc, tc + (kv + b) / kp, tc * (kv + b) / kp
        self.limit = np.array([m.actuator_forcerange[a, 1] for a in self.act])
        self.light = m.light('lamp_light').id
        self.hand = m.body('user_hand').mocapid[0]
        if drive != 'ideal':
            # Linux streams the planned motion to the MCU, which drives the steppers (stepper_pid_sim.py)
            import stepper_pid_sim
            self.sim = stepper_pid_sim.drive(m, lambda t: np.degrees(show._q(max(t, 0.0))), show.end, drive, sensor)
            self.d = self.sim.d
            self.title = f'drive: {drive}'
        self.reset()

    def reset(self):
        m, d, q0 = self.m, self.d, self.show.q0
        if self.sim:
            self.sim.reset()
            d.mocap_pos[self.hand] = self.show.hand_at(0.0)
            m.light_active[self.light] = 1 if self.show.light_at(0.0) else 0
            mujoco.mj_forward(m, d)
            return
        mujoco.mj_resetData(m, d)
        full = dict(zip(JOINTS, q0))
        for jn, (src, k) in {'base_motor_shaft': ('yaw', -5), 'shoulder_motor_shaft': ('shoulder', 16),
                             'elbow_motor_shaft': ('elbow', -16), 'wrist_motor_shaft': ('wrist', -16)}.items():
            full[jn] = k * full[src]
        for drive in ('shoulder', 'elbow', 'wrist'):
            for x in 'abc':
                full[f'{drive}_plate_{x}'] = -full[f'{drive}_motor_shaft']
        for jn, v in full.items():
            d.qpos[m.jnt_qposadr[m.joint(jn).id]] = v
        d.ctrl[self.act] = q0
        for i, a in enumerate(self.act):
            if self.tc[i] > 0:
                d.act[m.actuator_actadr[a]] = q0[i]
        d.mocap_pos[self.hand] = self.show.hand_at(0.0)
        mujoco.mj_forward(m, d)

    def control(self, t):
        q, qd, qdd = self.show.ref(t)
        self.d.ctrl[self.act] = q + self.lead * qd + self.lead2 * qdd
        self.m.light_active[self.light] = 1 if self.show.light_at(t) else 0
        self.d.mocap_pos[self.hand] = self.show.hand_at(t)
        return q

    def step(self):
        """Advance one timestep (ideal motors or the simulated stepper drives); returns the plan."""
        t = self.d.time
        if not self.sim:
            q = self.control(t)
            mujoco.mj_step(self.m, self.d)
            return q
        self.m.light_active[self.light] = 1 if self.show.light_at(t) else 0
        self.d.mocap_pos[self.hand] = self.show.hand_at(t)
        _, self.tq = self.sim.step()
        return self.show._q(t)

    def arm_q(self):
        return self.d.qpos[self.qadr].copy()


def robot_contacts(m, d):
    out = set()
    robot = {m.body(n).id for n in ('yaw_spindle', 'first_link', 'second_link', 'head', 'head_tilt')}
    for c in d.contact[:d.ncon]:
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if b1 in robot or b2 in robot:
            n = lambda g: m.geom(g).name or m.body(m.geom_bodyid[g]).name
            out.add(tuple(sorted((n(c.geom1), n(c.geom2)))))
    return out


# ------------------------------------------------------------------ outputs
def run_report(names, gammas, xray=False, drive='ideal', sensor='joint'):
    xml = scene_xml(xray)
    kin = LampKin(mujoco.MjModel.from_xml_string(xml))
    if drive != 'ideal':
        print(f'Through the simulated stepper drives ({drive}, sensor on {sensor}). Error vs the plan with the '
              'stream delay removed; torque = motor torque / pull-out torque (steppers 0-3, head servo not included).')
    for name in names:
        for g in gammas:
            show = SCENARIOS[name](kin, g)
            rig = Rig(xml, show, drive, sensor); m, d = rig.m, rig.d
            n = int(show.end / m.opt.timestep)
            emax = np.zeros(5); fmax = np.zeros(5); contacts = set(); plan, real = [], []
            cup0 = d.xpos[m.body('cup').id].copy()
            for i in range(n):
                q = rig.step()
                if rig.sim:
                    plan.append(q); real.append(rig.arm_q())
                    fmax[:4] = np.maximum(fmax[:4], rig.tq)
                else:
                    emax = np.maximum(emax, np.abs(np.degrees(rig.arm_q() - q)))
                    fmax = np.maximum(fmax, np.abs(d.actuator_force[rig.act]) / rig.limit)
                contacts |= robot_contacts(m, d)
            extra = ''
            if rig.sim:
                plan, real = np.degrees(plan), np.degrees(real)
                best = min((np.abs(real[k:] - plan[:len(plan) - k]).max(), k) for k in range(0, 500, 10))
                emax[:] = best[0]
                extra = f'  delay {1000 * best[1] * m.opt.timestep:3.0f} ms  lost steps {rig.sim.slips}'
            cup = d.xpos[m.body('cup').id] - cup0
            print(f'{name:8s} gamma={g:.1f}  {show.end:5.1f}s  max err {emax.max():5.2f} deg  peak torque {100 * fmax.max():3.0f}% '
                  f'(per joint {np.round(100 * fmax).astype(int).tolist()})  cup moved {1000 * np.linalg.norm(cup[:2]):5.1f} mm{extra}  '
                  f'contacts: {sorted(contacts) if contacts else "none"}')


def _font(size):
    from PIL import ImageFont
    try:
        return ImageFont.load_default(size=size)
    except Exception:
        return ImageFont.load_default()


def _wrap(dr, text, font, width):
    words, lines, cur = text.split(), [], ''
    for w in words:
        t = (cur + ' ' + w).strip()
        if dr.textlength(t, font=font) > width and cur:
            lines.append(cur); cur = w
        else:
            cur = t
    return lines + ([cur] if cur else [])


def _panel(img, show, t, label, W, H):
    from PIL import ImageDraw
    dr = ImageDraw.Draw(img, 'RGBA')
    big, mid, small = _font(30), _font(24), _font(19)
    dr.rectangle([0, 0, W, 50], fill=(15, 18, 28, 170))
    dr.text((16, 9), show.title, fill=(255, 255, 255), font=big)
    col = (120, 200, 255) if show.g <= 0 else (255, 190, 90)
    tw = dr.textlength(label, font=mid)
    dr.text((W - tw - 16, 13), label, fill=col, font=mid)
    y = H - 22
    for t0, t1, who, text in sorted(show.says, key=lambda s: s[0], reverse=True):
        if t0 <= t <= t1:
            lines = _wrap(dr, f'{who}: {text}', mid, W - 60)
            hgt = 32 * len(lines) + 14
            y -= hgt
            fill = (255, 255, 255, 215) if who == 'User' else (255, 236, 200, 225)
            dr.rounded_rectangle([18, y, W - 18, y + hgt - 6], radius=12, fill=fill)
            for k, ln in enumerate(lines):
                dr.text((32, y + 6 + 32 * k), ln, fill=(25, 25, 30), font=mid)
            y -= 8
    for t0, t1, bpm in show.beats:
        if t0 <= t <= t1:
            ph = ((t - t0) * bpm / 60.0) % 1.0
            r = 16 if ph < 0.18 else 9
            dr.ellipse([W - 46 - r, 70 - r, W - 46 + r, 70 + r], fill=(255, 190, 90, 230))
            dr.text((W - 170, 60), f'{bpm} BPM', fill=(255, 255, 255), font=small)
    cur = None
    for tm, lab in show.marks:
        if tm <= t:
            cur = (tm, lab)
    if cur and show.g > 0 and t - cur[0] < 2.0:
        dr.text((16, 60), cur[1], fill=(255, 220, 160), font=small)
    return img


def run_video(names, out, xray=False, fps=30, W=960, H=600, drive='ideal', sensor='joint'):
    from PIL import Image, ImageDraw
    xml = scene_xml(xray)
    kin = LampKin(mujoco.MjModel.from_xml_string(xml))
    ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{2 * W}x{H}', '-r', str(fps), '-i', '-',
                           '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '21', out], stdin=subprocess.PIPE)

    def card(lines, secs=2.2):
        img = Image.new('RGB', (2 * W, H), (14, 16, 24)); dr = ImageDraw.Draw(img)
        y = H // 2 - 30 * len(lines)
        for k, (txt, sz, col) in enumerate(lines):
            f = _font(sz); tw = dr.textlength(txt, font=f)
            dr.text(((2 * W - tw) / 2, y), txt, fill=col, font=f); y += sz + 22
        b = np.asarray(img).tobytes()
        for _ in range(int(secs * fps)):
            ff.stdin.write(b)

    card([('ELEGNT-style expressive movement', 52, (255, 255, 255)),
          ('after Hu et al., "ELEGNT: Expressive and Functional Movement Design for Non-anthropomorphic Robot" (arXiv 2501.12493)', 22, (190, 200, 215)),
          ('left: function-driven (gamma = 0)        right: expression-driven (gamma = 1)', 28, (255, 210, 140))], 3.5)
    for name in names:
        F, E = SCENARIOS[name](kin, 0.0), SCENARIOS[name](kin, 1.0)
        card([(F.title, 48, (255, 255, 255)), (F.note, 24, (200, 210, 225))], 2.2)
        rigs = [Rig(xml, F, drive, sensor), Rig(xml, E, drive, sensor)]
        rends = [mujoco.Renderer(r.m, H, W) for r in rigs]
        cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [0.33, 0.0, 0.21]; cam.distance = 1.22; cam.azimuth = 112; cam.elevation = -13
        T = max(F.end, E.end)
        spf = int(round(1.0 / fps / rigs[0].m.opt.timestep))
        for f in range(int(T * fps)):
            frames = []
            for rig, rend, show, lab in zip(rigs, rends, (F, E), ('FUNCTION-DRIVEN  (gamma = 0)', 'EXPRESSION-DRIVEN  (gamma = 1)')):
                for _ in range(spf):
                    rig.step()
                rend.update_scene(rig.d, cam)
                img = Image.fromarray(rend.render())
                frames.append(np.asarray(_panel(img, show, rig.d.time, lab, W, H)))
            frame = np.concatenate(frames, axis=1)
            frame[:, W - 1:W + 1] = 30
            ff.stdin.write(frame.tobytes())
        for r in rends:
            r.close()
    ff.stdin.close(); ff.wait()
    print(f'wrote {out}')


def run_viewer(name, gamma, xray=False, drive='ideal', sensor='joint'):
    import mujoco.viewer
    xml = scene_xml(xray)
    kin = LampKin(mujoco.MjModel.from_xml_string(xml))
    show = SCENARIOS[name](kin, gamma)
    rig = Rig(xml, show, drive, sensor); m, d = rig.m, rig.d
    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.lookat[:] = [0.33, 0.0, 0.21]; v.cam.distance = 1.22; v.cam.azimuth = 112; v.cam.elevation = -13
        while v.is_running():
            rig.reset(); t0 = time.time()
            while v.is_running() and d.time < show.end:
                rig.step()
                if d.time > time.time() - t0:
                    if rig.sim:
                        import stepper_pid_sim
                        v.set_texts(stepper_pid_sim.drive_overlay(rig.sim, rig.title))
                    v.sync(); time.sleep(max(0.0, d.time - (time.time() - t0)))
            time.sleep(0.8)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scenario', choices=list(SCENARIOS) + ['all'], default='all')
    ap.add_argument('--gamma', type=float, default=1.0, help='expressiveness, 0 = function-driven, 1 = expression-driven')
    ap.add_argument('--report', action='store_true')
    ap.add_argument('--video')
    ap.add_argument('--xray', action='store_true')
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
    names = list(SCENARIOS) if a.scenario == 'all' else [a.scenario]
    if a.report:
        run_report(names, [0.0, a.gamma] if a.gamma > 0 else [0.0], a.xray, a.drive, a.sensor)
    elif a.video:
        run_video(names, a.video, a.xray, drive=a.drive, sensor=a.sensor)
    else:
        run_viewer(names[0] if a.scenario != 'all' else 'social', a.gamma, a.xray, a.drive, a.sensor)
