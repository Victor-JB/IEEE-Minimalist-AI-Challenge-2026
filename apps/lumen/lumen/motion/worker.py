"""Motion worker: one thread owns the arm and streams setpoints at 25 Hz.

Why a thread: inference takes ~70 ms per frame, and ArmMotion.stream_point must be called at
least every 0.25 s or the firmware halts and holds. So perception never talks to the arm
directly; it queues commands here.

Real mode uses ArmMotion from the shared arm module (arm/python; the closed-loop firmware must
already be flashed by arm/scripts/deploy.sh). Fake mode (--fake-arm) plays the same timing with no
board, so the whole app runs on a laptop with a webcam.

Head tilt (the J5 roll servo): clips play their head_roll track on it, around the base roll; holds
and moves bring it back to the base roll (arm_config.ROLL_HOME unless a pose sets it: a pose may
carry a 4th value, the roll, e.g. AIM's sideways aim). It is skipped (no-op) when the firmware has
no roll servo.

Sweeps settle before they report: AT_POSE (which lets the book model run) is only sent once the
measured angles are within SETTLE_TOL_DEG of the pause pose and at least SETTLE_MIN_S have passed
(or SETTLE_MAX_S, if backlash keeps a joint from closing in), so inference never sees motion blur.
The pause holds exactly there: no hold drift, no look-at offset.

Lamp light (FirmwareBackend in io/light.py): set_light() just stores a per-mille target; this thread
sends it as `LIGHT n fade` between setpoints, at most every LIGHT_FIRMWARE_EVERY_S, faded on the MCU
over that interval so the 25 Hz stream never waits on more than one extra RPC. A reconnect resends it.

Every command carries a token; finished-events come back as (event, token) so the director can
ignore a CLIP_DONE from a clip that a state change already interrupted.

Stalls are handled gracefully, inside _send: the firmware stops a joint that isn't following (gear
backlash, a heavy pose) and holds it powered; we acknowledge with HALT, pause, then push again to
the same setpoint more slowly (a stepper has more torque at low speed), up to STALL_RETRIES times.
A clip's clock waits during the recovery and the rest of it plays slower. If every retry fails the
command is abandoned (the arm keeps holding, the director moves on); only GIVE_UP_LIMIT give-ups
within GIVE_UP_WINDOW_S (a jammed arm) become a FAULT. Any other error closes the session and emits
FAULT, and the thread waits for connect() (sent by BOOT). The thread never exits on an error.
"""
import queue
import threading
import time
import traceback

from .. import config as C
from .. import events as E
from .. import paths  # noqa: F401  (puts arm/python on sys.path)
import arm_config as A                     # arm/python/arm_config.py (roll servo settings)
from trajectory import trajectory_with_roll  # arm/python/trajectory.py (shared with emote.py)

DT = 1.0 / C.STREAM_HZ
STALL_PAUSE_S = 0.25                      # hold powered this long after a stall before pushing again
STALL_RETRIES = 3                         # slower pushes per setpoint before abandoning the command
RETRY_SPEED = (10.0, 6.0, 4.0)            # deg/s of each successive push
STALL_SLOWDOWN = 1.5                      # the rest of a clip plays this much slower after a stall
GIVE_UP_LIMIT, GIVE_UP_WINDOW_S = 6, 60.0 # abandoned commands this close together -> FAULT (jammed)
EASE_SPEED = 20.0                         # deg/s when easing onto a clip's first pose
OFFSET_SLEW = 25.0                        # deg/s: look-at offset changes are smoothed to this
ROLL_RETURN_SPEED = 60.0                  # deg/s the head tilt eases back to ROLL_HOME after a clip
ROLL_DEADBAND = 0.3                       # deg: smaller roll changes aren't sent (saves an RPC per tick)
SWEEP_MIN_MOVE_S = 1.0                    # sweep moves take at least this long (longer if the caps need it)


class Stalled(Exception):
    """Every retry of a stalled setpoint failed; the arm holds where it stopped."""


def _minjerk(s):
    return 10 * s**3 - 15 * s**4 + 6 * s**5


class MotionWorker(threading.Thread):
    def __init__(self, expressive, fake=False):
        super().__init__(daemon=True, name="motion")
        self.x, self.fake = expressive, fake
        self.cmds, self.events = queue.Queue(), queue.Queue()
        self.base = list(C.HOME)            # pose that clips and holds are relative to
        self.base_roll = A.ROLL_HOME        # head tilt that holds and moves return to
        self.measured = None                # last measured angles (from stream_point), None = unknown
        self.offset = [0.0, 0.0, 0.0]       # look-at offset, set by the director every frame
        self._off = [0.0, 0.0, 0.0]         # the offset actually applied, slewed toward self.offset
        self.stall_time = 0.0               # seconds spent recovering from stalls (clips pause their clock)
        self.stall_count = 0
        self.current = list(C.HOME)
        self.roll = None                    # last head-tilt angle sent (None: send the next one)
        self.relaxed = False
        self.arm = None
        self.give_ups = []                  # times commands were abandoned after failed retries
        self.light_cmd = None               # lamp per mille wanted (main thread writes, we read)
        self.light_sent, self.light_at = None, -1e9
        self._stop = threading.Event()

    # -- commands (call from the main thread) ------------------------------------
    def play(self, ref, amplitude, time_scale, token):
        self.cmds.put(("play", ref, amplitude, time_scale, token))

    def goto(self, pose, duration, token, done=E.MOVE_DONE):
        """pose: (shoulder, elbow, head) or (shoulder, elbow, head, roll)."""
        self.cmds.put(("goto", list(pose), duration, token, done))

    def path(self, segments, token, done=E.MOVE_DONE):
        """Several gotos as one command [(pose, duration), ...] (e.g. anticipation, then the move);
        one `done` event at the end. Separate goto()s would interrupt each other."""
        self.cmds.put(("path", [(list(p), d) for p, d in segments], token, done))

    def sweep(self, poses, dwell, token):
        """Visit each pose, settle, pause `dwell` s; emits AT_POSE (info: index) per pose once
        settled, then SWEEP_DONE."""
        self.cmds.put(("sweep", [list(p) for p in poses], dwell, token))

    def relax(self):
        self.cmds.put(("relax",))

    def connect(self):
        """(Re)open the arm session if a fault closed it; no-op when connected or fake."""
        self.cmds.put(("connect",))

    def set_base(self, pose):
        """Pose holds and clips are relative to; a 4th value sets the base roll (else ROLL_HOME)."""
        self.base = list(pose[:3])
        self.base_roll = pose[3] if len(pose) > 3 else A.ROLL_HOME

    def set_light(self, permille):
        """Lamp duty 0-1000 (gamma already applied). Thread-safe: one attribute store."""
        self.light_cmd = max(0, min(1000, int(permille)))

    def set_offset(self, offset):
        self.offset = list(offset)

    def poll_events(self):
        out = []
        while True:
            try:
                out.append(self.events.get_nowait())
            except queue.Empty:
                return out

    def stop(self):
        self._stop.set()

    # -- thread ------------------------------------------------------------------
    def run(self):
        connected = self._connect()
        while not self._stop.is_set():
            if not connected:
                connected = self._wait_for_connect()
                continue
            try:
                self._loop()
            except Exception as e:                    # ArmError, link loss, closed-loop fault
                traceback.print_exc()
                self._disconnect()
                self.events.put((E.FAULT, None, repr(e)))
                connected = False
        self._disconnect()

    def _connect(self):
        if self.fake:
            return True
        from arm_motion import ArmMotion      # arm/python
        paths.LOGS.mkdir(exist_ok=True)
        last = None
        for _ in range(3):                    # the old session's ownership may take ~2 s to clear
            log = paths.LOGS / time.strftime("arm-%Y%m%d-%H%M%S.jsonl")
            try:
                self.arm = ArmMotion(output=log, max_speed=C.FIRMWARE_MAX_SPEED,
                                     max_accel=C.FIRMWARE_MAX_ACCEL, tuck=False,
                                     roll=A.ROLL if A.ROLL_ENABLED else None).__enter__()
                self.current = self.arm.angles()       # start every move from where the arm really is
                self.give_ups, self.roll = [], None
                self.light_sent = None              # resend the lamp level on the new session
                tilt = "head tilt on" if self.arm.roll_supported else "no head tilt"
                print(f"Arm connected at {[round(a, 1) for a in self.current]} ({tilt})")
                return True
            except Exception as e:
                last = e
                self.arm = None
                time.sleep(1.5)
        traceback.print_exception(last)
        self.events.put((E.FAULT, None, f"connect failed: {last!r}"))
        return False

    def _disconnect(self):
        if self.arm:
            try:
                self.arm.relax()
            except Exception:
                pass
            try:
                self.arm.close()
            except Exception:
                pass
        self.arm = None

    def _wait_for_connect(self):
        """Disconnected: drop every command until connect() asks for a new session."""
        try:
            cmd = self.cmds.get(timeout=0.5)
        except queue.Empty:
            return False
        return cmd[0] == "connect" and self._connect()

    def _loop(self):
        while not self._stop.is_set():
            try:
                cmd = self.cmds.get(timeout=DT)
            except queue.Empty:
                self._guard(self._hold_tick)
                continue
            kind = cmd[0]
            if kind == "connect":
                continue                              # already connected
            if kind == "relax":
                if self.arm:
                    self.arm.relax()
                self.relaxed = True
                continue
            if self.relaxed and self.arm:             # drivers were asleep: the arm may have sagged
                self.current = self.arm.angles()
            self.relaxed = False
            if kind == "play":                        # _guard: True done, None stalled, False interrupted
                _, ref, amp, ts, token = cmd
                if self._guard(self._play, ref, amp, ts) is not False:
                    self.events.put((E.CLIP_DONE, token, ref))
            elif kind == "goto":
                _, pose, dur, token, done = cmd
                if self._guard(self._goto, pose, dur) is not False:
                    self.set_base(pose)
                    self.events.put((done, token, None))
            elif kind == "path":
                _, segments, token, done = cmd
                if self._guard(self._path, segments) is not False:
                    self.set_base(segments[-1][0])
                    self.events.put((done, token, None))
            elif kind == "sweep":
                _, poses, dwell, token = cmd
                if self._guard(self._sweep, poses, dwell, token) is None:
                    self.events.put((E.SWEEP_DONE, token, None))

    def _guard(self, fn, *args):
        """Run a motion primitive. If a stall can't be pushed through it ends early (returns None)."""
        try:
            return fn(*args)
        except Stalled as e:
            now = time.monotonic()
            self.give_ups = [t for t in self.give_ups if now - t < GIVE_UP_WINDOW_S] + [now]
            print(f"Arm: gave up after {STALL_RETRIES} retries ({e}); holding. "
                  f"{len(self.give_ups)} in the last {GIVE_UP_WINDOW_S:.0f} s")
            if len(self.give_ups) >= GIVE_UP_LIMIT:
                raise RuntimeError(f"arm jammed: {len(self.give_ups)} stalls could not be pushed "
                                   f"through in {GIVE_UP_WINDOW_S:.0f} s; last: {e}")
            return None

    def _path(self, segments):
        return all(self._goto(pose, dur) for pose, dur in segments)

    def _sweep(self, poses, dwell, token):
        base, base_roll = self.base, self.base_roll
        try:
            for i, pose in enumerate(poses):
                if not self._goto(pose, self._capped_duration(pose, SWEEP_MIN_MOVE_S)):
                    return False
                self.base, self.base_roll = list(pose[:3]), (pose[3] if len(pose) > 3 else A.ROLL_HOME)
                t0 = time.monotonic()
                if not self._settle(pose[:3]):
                    return False
                self.events.put((E.AT_POSE, token, i))
                if not self._wait(max(0.0, dwell - (time.monotonic() - t0)), still=True):
                    return False
            self.events.put((E.SWEEP_DONE, token, None))
            return True
        finally:
            self.base, self.base_roll = base, base_roll   # clips after the sweep play around home again

    def settled(self, target, since):
        """Arrived and still: the measured angles are close to target (real arm), after SETTLE_MIN_S;
        or SETTLE_MAX_S have passed anyway. `since`: seconds since the setpoint arrived."""
        if since < C.SETTLE_MIN_S:
            return False
        if self.measured is None or since >= C.SETTLE_MAX_S:
            return True
        return all(abs(m - t) <= C.SETTLE_TOL_DEG for m, t in zip(self.measured, target))

    def _settle(self, target):
        """Hold target (no drift, no offset) until settled(); False if interrupted."""
        t0 = time.monotonic()
        while not self.settled(target, time.monotonic() - t0):
            if self._interrupted():
                return False
            self._hold_tick(still=True)
        return True

    def _capped_duration(self, pose, minimum):
        """Long enough that a min-jerk move from here to pose stays under the speed caps."""
        t = minimum
        for j, (a, b) in enumerate(zip(self.current, pose[:3])):
            cap = C.ELBOW_LIFT_CAP if j == 1 and b < a else C.SPEED_CAPS[j]
            t = max(t, 1.875 * abs(b - a) / cap)
        return t

    # -- motion primitives (return False if interrupted by a new command) ----------
    def _send(self, pose, vel=None):
        pose = [max(lo, min(hi, v)) for v, (lo, hi) in zip(pose, C.SAFE_RANGES)]
        if self.arm:
            s = self.arm.stream_point(pose, vel, horizon=0.12)
            self.measured = s.get("angles")
            if s["state"] == "stalled":
                self._push_through(pose)
        self.current = pose
        self._light_tick()

    def _push_through(self, pose):
        """A stall: acknowledge, pause powered, then push to pose again, slower each time."""
        t0 = time.monotonic()
        try:
            for speed in RETRY_SPEED:
                servo = self._halt()
                self.stall_count += 1
                print(f"Arm stall: {servo['stalled_joint']} at {servo['stall_angle']}, "
                      f"pushing again at {speed:.0f} deg/s")
                self._sleep_holding(STALL_PAUSE_S)
                if self._slow_move(pose, speed):
                    return
            servo = self._halt()
            raise Stalled(f"{servo['stalled_joint']} at {servo['stall_angle']}")
        finally:
            self.stall_time += time.monotonic() - t0

    def _halt(self):
        servo = self.arm.servo_status()        # read before HALT clears the stall record
        self.arm.halt()                        # holds powered where it stopped
        self.current = self.arm.angles()
        return servo

    def _sleep_holding(self, seconds):
        time.sleep(seconds)                    # HALT holds without a lease; nothing to send

    def _slow_move(self, pose, speed):
        """Stream a min-jerk path from the measured pose; False if it stalls again."""
        start = list(self.current)
        far = max(abs(a - b) for a, b in zip(start, pose))
        n = max(2, int(max(0.3, far / speed) / DT))
        for k in range(1, n + 1):
            s = _minjerk(k / n)
            point = [a + (b - a) * s for a, b in zip(start, pose)]
            if self.arm.stream_point(point, None, horizon=0.12)["state"] == "stalled":
                return False
            self.current = point
            time.sleep(DT)
        return True

    def _light_tick(self):
        """Send the lamp target if it changed, rate limited. Arms without light() (old ArmMotion,
        test fakes) and the fake arm are skipped."""
        want = self.light_cmd
        if want is None or want == self.light_sent or self.arm is None:
            return
        now = time.monotonic()
        if now - self.light_at < C.LIGHT_FIRMWARE_EVERY_S:
            return
        send = getattr(self.arm, "light", None)
        self.light_sent, self.light_at = want, now
        if send:
            send(want, fade_ms=round(C.LIGHT_FIRMWARE_EVERY_S * 1000))

    def _roll(self, angle, speed=None):
        """Head tilt to angle; the firmware slews it (speed None = its cap) and the servo holds it."""
        if self.roll is not None and abs(angle - self.roll) < ROLL_DEADBAND:
            return
        self.roll = angle
        if self.arm:
            self.arm.roll(angle, speed)

    def _slew_offset(self):
        """Move the applied look-at offset toward the requested one, at most OFFSET_SLEW deg/s."""
        step = OFFSET_SLEW * DT
        self._off = [o + max(-step, min(step, t - o)) for o, t in zip(self._off, self.offset)]
        return self._off

    def _interrupted(self):
        return not self.cmds.empty() or self._stop.is_set()

    def _wait(self, seconds, still=False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self._interrupted():
                return False
            self._hold_tick(still)
        return True

    def _hold_tick(self, still=False):
        if self.relaxed:
            self._light_tick()                 # the lamp still works while the drivers sleep
            time.sleep(DT)
            return
        self._roll(self.base_roll, ROLL_RETURN_SPEED)
        if still:                                 # a sweep pause: exactly the pose (the camera is looking)
            self._send(list(self.base))
        else:
            drift = self.x.drift_step(DT)
            off = self._slew_offset()
            self._send([b + o + d for b, o, d in zip(self.base, off, drift)])
        time.sleep(DT)

    def _goto(self, pose, duration):
        """Min-jerk to pose[:3]; pose[3], if given, is the roll (else the base roll)."""
        roll = pose[3] if len(pose) > 3 else self.base_roll
        pose = pose[:3]
        start, n = list(self.current), max(1, int(duration / DT))
        self._roll(roll, ROLL_RETURN_SPEED)
        for k in range(1, n + 1):
            if self._interrupted():
                return False
            s = _minjerk(k / n)
            self._send([a + (b - a) * s for a, b in zip(start, pose)])
            time.sleep(DT)
        return True

    def _ease_to(self, pose):
        """Glide to pose first if the arm is not already there (no snap at a clip's start)."""
        far = max(abs(a - b) for a, b in zip(self.current, pose))
        return far < 1.0 or self._goto(pose, max(0.3, far / EASE_SPEED))

    def _play(self, ref, amplitude, time_scale):
        import numpy as np
        off0 = list(self._slew_offset())
        home = [b + o for b, o in zip(self.base, off0)]
        t, q, vel, _, roll = trajectory_with_roll(ref, home, amplitude, roll_home=self.base_roll)
        if not self._ease_to(q[0].tolist()):
            return False
        stretch = max(1.0, time_scale)             # only ever slower: trajectory() already sits at the caps
        t, vel = np.asarray(t), np.asarray(vel)
        # Clip time advances at `rate` per real second; stall recoveries pause it and slow the rest.
        clock, rate, last = 0.0, 1.0 / stretch, time.monotonic()
        for k in range(len(t)):
            if self._interrupted():
                return False
            while clock < t[k]:
                time.sleep(min(DT / 4, (t[k] - clock) / rate))
                now = time.monotonic()
                clock, last = clock + (now - last) * rate, now
            off = self._slew_offset()               # keep tracking the face while the clip plays
            point = [v + o - o0 for v, o, o0 in zip(q[k].tolist(), off, off0)]
            stalled_for = self.stall_time
            self._roll(float(roll[k]))
            self._send(point, np.clip(vel[k] * rate, -59, 59).tolist())
            if self.stall_time != stalled_for:      # recovered from a stall: resume from here, slower
                rate /= STALL_SLOWDOWN
                last = time.monotonic()
        return True
