"""The desk's arm thread: one ArmMotion session, driven by commands from the page.

Only this thread talks to the arm (the transport carries one request at a time); the HTTP
threads queue commands with send() and read snapshot(). Motion streams setpoints at STREAM_HZ
like Lumen's worker. Unlike Lumen, a stall is not pushed through here: the arm halts, holds
where it stopped and the page says so; manual control leaves the next move to the operator.

Power is automatic: everything sleeps while there is no input. The stepper drivers wake only
for a move (jog, pose, clip) and sleep again SETTLE_S after it ends, stops or stalls; the
head-tilt servo goes limp SETTLE_S after its tilt finishes. Motion needs healthy sensors and
movement enabled (Enable movement / Sleep & lock). A jog keeps going only while the page keeps
renewing it (hold-to-run): JOG_DEADMAN_S without a renewal and it stops.
"""
import math
import queue
import threading
import time
from collections import deque

from .. import config as C
from .. import paths

import arm_config as A                        # arm/python (paths puts it on sys.path)
from arm_client import JOINTS, LIMITS, MOTOR, angles, healthy
from arm_motion import STATES

DT = 1.0 / C.STREAM_HZ
JOG_DEADMAN_S = 0.3
SETTLE_S = 0.4                # hold powered this long after a move ends, then let the drivers sleep
STATUS_EVERY_S = 0.1          # telemetry + keepalive when nothing streams (firmware drops an owner after 2 s)
SERVO_EVERY_S = 0.5
ROLL_EVERY_S = 0.5
HISTORY_S = 30
ROLL_WIDE = (-135.0, 135.0)   # the servo's whole travel; the page's "wide" switch unlocks it
ROLL_DEADBAND = 0.3
GOTO_SPEED = (2.0, 30.0)      # deg/s the page may ask for
JOG_SPEED = (1.0, 20.0)


def magnet_text(bits):
    if not bits & 0x20:
        return "no magnet"
    if bits & 0x08:
        return "field too strong"
    if bits & 0x10:
        return "field weak"
    return "ok"


def _minjerk(s):
    return s * s * s * (10 - 15 * s + 6 * s * s)


def path_to(start, goal, speed):
    """Min-jerk setpoints (one per DT) from start to goal, peak speed about `speed` deg/s."""
    far = max(abs(g - s) for s, g in zip(start, goal))
    n = max(2, math.ceil(max(0.4, 1.875 * far / speed) / DT))
    return [[s + (g - s) * _minjerk(k / n) for s, g in zip(start, goal)] for k in range(1, n + 1)]


def clamp_pose(pose):
    return [min(hi, max(lo, v)) for v, (lo, hi) in zip(pose, C.SAFE_RANGES)]


class DeskArm(threading.Thread):
    def __init__(self, factory=None, clock=time.monotonic):
        super().__init__(daemon=True, name="desk-arm")
        self.factory = factory or self._open_real
        self.clock = clock
        self.cmds = queue.Queue()
        self.lock = threading.Lock()              # guards what snapshot() reads
        self.arm = None
        self.notice = "Disconnected. Connect to read the joints."
        self.recording = None
        self.release_at = None                    # when to let the drivers sleep after a move
        self.roll_release_at = None               # when to let the head-tilt servo go limp
        self.sample = self.servo = self.rollst = None
        self.jog = None                           # dict(joint, dir, speed, until)
        self.motion = None                        # dict(kind, label, points=[(pose, vel, roll)], i)
        self.setpoint = None                      # last streamed pose (joint order), None when idle
        self.roll_sent, self.roll_wide = None, False
        self.history = deque(maxlen=int(HISTORY_S * C.STREAM_HZ))
        self._next = dict(status=0.0, servo=0.0, roll=0.0)
        self._stop = threading.Event()

    # -- any thread ---------------------------------------------------------------------------
    def send(self, *cmd):
        self.cmds.put(cmd)

    def stop(self):
        self._stop.set()

    def snapshot(self):
        with self.lock:
            s, servo, roll = self.sample, self.servo, self.rollst
            joints = []
            for j, name in enumerate(JOINTS):
                axis = s["axes"][MOTOR[j]] if s else None
                ok = axis is not None and axis["angle"] != -2147483648
                joints.append(dict(
                    name=name, angle=axis["angle"] / 1000 if ok else None,
                    setpoint=self.setpoint[j] if self.setpoint else None,
                    magnet=axis["magnet"] if axis else None,
                    magnet_text=magnet_text(axis["magnet"]) if axis else None,
                    age=axis["age"] if axis else None, sensor_error=axis["sensor_error"] if axis else None,
                    limits=LIMITS[j], safe=C.SAFE_RANGES[j], home=C.HOME[j], motor=MOTOR[j]))
            return dict(
                connected=self.arm is not None, notice=self.notice,
                busy=("jog" if self.jog else self.motion["label"] if self.motion else None),
                recording=str(self.recording) if self.recording else None,
                healthy=bool(s and healthy(s)), state=STATES.get(s["state"], s["state"]) if s else None,
                awake=bool(s and s["awake"]), enabled=bool(s and s["enabled"]),
                joints=joints,
                servo=None if not servo else {k: servo[k] for k in ("state", "reason", "stalls", "stalled_joint", "stall_angle")},
                roll=roll, roll_range=A.ROLL_RANGE, roll_wide=self.roll_wide, roll_home=A.ROLL_HOME,
                roll_supported=bool(self.arm and self.arm.roll_supported))

    def history_since(self, t):
        with self.lock:
            return [h for h in self.history if h[0] > t]

    # -- the arm thread -------------------------------------------------------------------------
    def run(self):
        while not self._stop.is_set():
            t0 = self.clock()
            try:
                self._drain()
                if self.arm:
                    self._step(t0)
            except Exception as e:                # ArmError, link loss, firmware rejection
                self._lost(e)
            time.sleep(max(0.0, DT - (self.clock() - t0)))
        self._close()

    def _open_real(self, output):
        from arm_motion import ArmMotion
        return ArmMotion(output=output, max_speed=C.FIRMWARE_MAX_SPEED, max_accel=C.FIRMWARE_MAX_ACCEL,
                         tuck=False, roll=dict(A.ROLL, limits=ROLL_WIDE) if A.ROLL_ENABLED else None).__enter__()

    def _say(self, text):
        with self.lock:
            self.notice = text

    def _drain(self):
        while True:
            try:
                cmd = self.cmds.get_nowait()
            except queue.Empty:
                return
            getattr(self, "_cmd_" + cmd[0])(*cmd[1:])

    def _cmd_connect(self):
        if self.arm:
            return
        paths.LOGS.mkdir(exist_ok=True)
        out = paths.LOGS / time.strftime("desk-%Y%m%d-%H%M%S.jsonl")
        self._say("Connecting...")
        try:
            arm = self.factory(out)
        except Exception as e:
            busy = any(w in str(e).lower() for w in ("busy", "disconnect manual"))
            self._say("Arm busy: stop Lumen (systemctl --user stop lumen) or other arm tools first."
                      if busy else f"Connect failed: {e}")
            return
        with self.lock:
            self.arm, self.recording, self.release_at, self.roll_release_at = arm, out, None, None
            self.jog = self.motion = self.setpoint = self.roll_sent = None
            self.notice = "Connected. Drivers asleep; they wake for each move."
        self._poll(force=True)

    def _cmd_disconnect(self):
        self._close()
        self._say("Disconnected. The arm is released and the drivers sleep.")

    def _cmd_power(self, enabled):
        if not self.arm:
            return
        self._cancel()
        self.release_at = None
        self.arm.power(bool(enabled))
        self._say("Movement enabled." if enabled else "Asleep and locked: movement is disabled.")

    def _cmd_relax(self):
        if not self.arm:
            return
        self._cancel()
        self.release_at = None
        self.arm.relax()
        self._say("Relaxed: drivers asleep, holding torque removed.")

    def _cmd_stop(self):
        if self._cancel(halt=True):
            self._release_soon()
        self._say("Stopped.")

    def _cmd_jog(self, joint, direction, speed):
        if direction == 0:                          # the page's button was released
            if self.jog:
                with self.lock:
                    self.jog = None
                self._release_soon()
            return
        if not self._can_move():
            return
        self.release_at = None
        speed = min(JOG_SPEED[1], max(JOG_SPEED[0], float(speed)))
        with self.lock:
            self.motion = None
            self.jog = dict(joint=int(joint), dir=1 if direction > 0 else -1, speed=speed,
                            until=self.clock() + JOG_DEADMAN_S)

    def _cmd_goto(self, pose, speed, roll=None, label="move"):
        if not self._can_move():
            return
        speed = min(GOTO_SPEED[1], max(GOTO_SPEED[0], float(speed)))
        points = [(p, None, None) for p in path_to(self._here(), clamp_pose(pose), speed)]
        if roll is not None and points:
            points[0] = (points[0][0], None, float(roll))
        self._start(label, points)

    def _cmd_clip(self, ref, amplitude=1.0):
        if not self._can_move():
            return
        import numpy as np
        from trajectory import trajectory_with_roll
        here = self._here()
        t, q, vel, _, roll = trajectory_with_roll(ref, here, float(amplitude), roll_home=A.ROLL_HOME)
        points = [(p, None, None) for p in path_to(here, q[0].tolist(), 20.0)]
        for tk in np.arange(0.0, t[-1] + 1e-9, DT):
            k = min(len(t) - 1, int(np.searchsorted(t, tk)))
            points.append((q[k].tolist(), np.clip(vel[k], -59, 59).tolist(), float(roll[k])))
        points += [(points[-1][0], None, A.ROLL_HOME)]
        self._start(ref, points)

    def _cmd_roll(self, angle, speed=None, wide=False):
        if not self._can_move():
            return
        if not self.arm.roll_supported:
            self._say("This firmware has no head-tilt servo: flash arm/sketch.")
            return
        with self.lock:
            self.roll_wide = bool(wide)
        lo, hi = ROLL_WIDE if wide else A.ROLL_RANGE
        self._roll(min(hi, max(lo, float(angle))), speed, force=True)

    def _cmd_roll_off(self):
        if self.arm and self.arm.roll_supported:
            self.arm.roll_off()
            self.roll_sent = self.roll_release_at = None
            self._say("Head tilt released (servo limp).")

    # -- helpers ---------------------------------------------------------------------------------
    def _can_move(self):
        s = self.sample
        why = ("Connect first." if not self.arm else
               "Sensor feedback unhealthy: check the magnets (joint cards)." if not (s and healthy(s)) else
               "Movement is locked: press Enable movement." if not s["enabled"] else None)
        if why:
            self._say(why)
            return False
        return True

    def _here(self):
        return list(self.setpoint) if self.setpoint else angles(self.sample)

    def _start(self, label, points):
        self.release_at = None
        with self.lock:
            self.jog, self.motion = None, dict(label=label, points=points, i=0)
        self._say(f"Playing {label}..." if "/" in label else f"Moving ({label})...")

    def _cancel(self, halt=False):
        """End any jog or move; True if something was moving."""
        streaming = bool(self.jog or self.motion or self.setpoint)
        with self.lock:
            self.jog = self.motion = self.setpoint = None
        if halt and streaming and self.arm and self.sample and self.sample["state"] in (13, 15, 17):
            self.arm.halt()
        return streaming

    def _release_soon(self):
        self.release_at = self.clock() + SETTLE_S

    def _roll(self, angle, speed=None, force=False):
        if not force and self.roll_sent is not None and abs(angle - self.roll_sent) < ROLL_DEADBAND:
            return
        travel = abs(angle - (self.roll_sent if self.roll_sent is not None else A.ROLL_HOME))
        self.roll_sent = angle
        self.arm.roll(angle, speed)
        eta = travel / min(speed or A.ROLL_SPEED_CAP, A.ROLL_SPEED_CAP)
        self.roll_release_at = max(self.roll_release_at or 0.0, self.clock() + eta + SETTLE_S)

    def _stream(self, pose, vel=None):
        pose = clamp_pose(pose)
        r = self.arm.stream_point(pose, vel, horizon=0.12)
        with self.lock:
            self.setpoint, self.sample = pose, self.arm.sample
        if r["state"] == "stalled":
            servo = self.arm.servo_status()         # before HALT clears the stall record
            self.arm.halt()
            with self.lock:
                self.jog = self.motion = self.setpoint = None
            self._release_soon()
            self._say(f"Stalled: {servo['stalled_joint']} at {servo['stall_angle']} deg; stopped there. "
                      "Jog or move to continue.")

    def _step(self, now):
        jog, motion = self.jog, self.motion
        if jog and now > jog["until"]:
            with self.lock:
                self.jog = jog = None
            self._release_soon()
        if jog:
            p = self._here()
            p[jog["joint"]] += jog["dir"] * jog["speed"] * DT
            self._stream(p)
        elif motion:
            pose, vel, roll = motion["points"][motion["i"]]
            if roll is not None and self.arm.roll_supported:
                self._roll(roll, force=motion["i"] == len(motion["points"]) - 1)   # always land exactly
            self._stream(pose, vel)
            if self.motion:                           # a stall may have cancelled it
                motion["i"] += 1
                if motion["i"] >= len(motion["points"]):
                    with self.lock:
                        self.motion = None
                    self._release_soon()
                    self._say(f"{motion['label']} done.")
        else:
            with self.lock:
                self.setpoint = None
            if self.release_at is None and self.sample and self.sample["awake"]:
                self._release_soon()                  # safety net: awake with nothing to do
            if self.release_at is not None and now >= self.release_at:
                self.release_at = None
                if self.sample and self.sample["awake"]:
                    self.arm.relax()                  # drivers sleep until the next move
            if self.roll_release_at is not None and now >= self.roll_release_at:
                self.roll_release_at = None
                if self.arm.roll_supported:
                    self.arm.roll_off()               # servo limp until the next tilt
        self._poll(now)
        s = self.sample
        if s:
            with self.lock:
                self.history.append((round(now, 3), [round(a, 2) for a in angles(s)] if healthy(s) else None,
                                     [round(a, 2) for a in self.setpoint] if self.setpoint else None))

    def _poll(self, now=None, force=False):
        now = self.clock() if now is None else now
        streaming = self.jog or self.motion
        if force or (not streaming and now >= self._next["status"]):
            self.arm.status()
            self._next["status"] = now + STATUS_EVERY_S
            with self.lock:
                self.sample = self.arm.sample
        if force or now >= self._next["servo"]:
            servo = self.arm.servo_status()
            self._next["servo"] = now + SERVO_EVERY_S
            with self.lock:
                self.servo = servo
        if self.arm.roll_supported and (force or now >= self._next["roll"]):
            r = self.arm.roll_status()
            self._next["roll"] = now + ROLL_EVERY_S
            with self.lock:
                self.rollst = {k: r.get(k) for k in ("enabled", "target", "current", "pulse_us", "fault")}

    def _lost(self, e):
        self._close()
        self._say(f"Lost the arm: {e}. Connect again.")

    def _close(self):
        arm = self.arm
        self.release_at = self.roll_release_at = None
        with self.lock:
            self.arm = None
            self.jog = self.motion = self.setpoint = self.sample = self.servo = self.rollst = None
        if arm:
            try:
                arm.close()                           # firmware releases: drivers sleep, servo limp
            except Exception:
                pass
