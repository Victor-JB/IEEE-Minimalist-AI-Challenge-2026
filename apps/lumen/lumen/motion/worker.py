"""Motion worker: one thread owns the arm and streams setpoints at 25 Hz.

Why a thread: inference takes ~70 ms per frame, and ArmMotion.stream_point must be called at
least every 0.25 s or the firmware halts and holds. So perception never talks to the arm
directly; it queues commands here.

Real mode uses ArmMotion from the shared arm module (arm/python; the closed-loop firmware must
already be flashed by arm/scripts/deploy.sh). Fake mode (--fake-arm) plays the same timing with no
board, so the whole app runs on a laptop with a webcam.

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
from trajectory import trajectory          # arm/python/trajectory.py (shared with emote.py)

DT = 1.0 / C.STREAM_HZ
STALL_PAUSE_S = 0.25                      # hold powered this long after a stall before pushing again
STALL_RETRIES = 3                         # slower pushes per setpoint before abandoning the command
RETRY_SPEED = (10.0, 6.0, 4.0)            # deg/s of each successive push
STALL_SLOWDOWN = 1.5                      # the rest of a clip plays this much slower after a stall
GIVE_UP_LIMIT, GIVE_UP_WINDOW_S = 6, 60.0 # abandoned commands this close together -> FAULT (jammed)
EASE_SPEED = 20.0                         # deg/s when easing onto a clip's first pose
OFFSET_SLEW = 25.0                        # deg/s: look-at offset changes are smoothed to this


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
        self.offset = [0.0, 0.0, 0.0]       # look-at offset, set by the director every frame
        self._off = [0.0, 0.0, 0.0]         # the offset actually applied, slewed toward self.offset
        self.stall_time = 0.0               # seconds spent recovering from stalls (clips pause their clock)
        self.stall_count = 0
        self.current = list(C.HOME)
        self.relaxed = False
        self.arm = None
        self.give_ups = []                  # times commands were abandoned after failed retries
        self._stop = threading.Event()

    # -- commands (call from the main thread) ------------------------------------
    def play(self, ref, amplitude, time_scale, token):
        self.cmds.put(("play", ref, amplitude, time_scale, token))

    def goto(self, pose, duration, token, done=E.MOVE_DONE):
        self.cmds.put(("goto", list(pose), duration, token, done))

    def sweep(self, poses, dwell, token):
        """Visit each pose, pausing `dwell` s; emits AT_PAUSE per pose, then SWEEP_DONE."""
        self.cmds.put(("sweep", [list(p) for p in poses], dwell, token))

    def relax(self):
        self.cmds.put(("relax",))

    def connect(self):
        """(Re)open the arm session if a fault closed it; no-op when connected or fake."""
        self.cmds.put(("connect",))

    def set_base(self, pose):
        self.base = list(pose)

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
                                     max_accel=C.FIRMWARE_MAX_ACCEL, tuck=False).__enter__()
                self.current = self.arm.angles()       # start every move from where the arm really is
                self.give_ups = []
                print(f"Arm connected at {[round(a, 1) for a in self.current]}")
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
                    self.base = list(pose)
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

    def _sweep(self, poses, dwell, token):
        for i, pose in enumerate(poses):
            if not self._goto(pose, 1.0):
                return False
            self.events.put((E.AT_POSE, token, i))
            if not self._wait(dwell):
                return False
        self.events.put((E.SWEEP_DONE, token, None))
        return True

    # -- motion primitives (return False if interrupted by a new command) ----------
    def _send(self, pose, vel=None):
        pose = [max(lo, min(hi, v)) for v, (lo, hi) in zip(pose, C.SAFE_RANGES)]
        if self.arm and self.arm.stream_point(pose, vel, horizon=0.12)["state"] == "stalled":
            self._push_through(pose)
        self.current = pose

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

    def _slew_offset(self):
        """Move the applied look-at offset toward the requested one, at most OFFSET_SLEW deg/s."""
        step = OFFSET_SLEW * DT
        self._off = [o + max(-step, min(step, t - o)) for o, t in zip(self._off, self.offset)]
        return self._off

    def _interrupted(self):
        return not self.cmds.empty() or self._stop.is_set()

    def _wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self._interrupted():
                return False
            self._hold_tick()
        return True

    def _hold_tick(self):
        if self.relaxed:
            time.sleep(DT)
            return
        drift = self.x.drift_step(DT)
        off = self._slew_offset()
        self._send([b + o + d for b, o, d in zip(self.base, off, drift)])
        time.sleep(DT)

    def _goto(self, pose, duration):
        start, n = list(self.current), max(1, int(duration / DT))
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
        t, q, vel, _ = trajectory(ref, home, amplitude)
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
            self._send(point, np.clip(vel[k] * rate, -59, 59).tolist())
            if self.stall_time != stalled_for:      # recovered from a stall: resume from here, slower
                rate /= STALL_SLOWDOWN
                last = time.monotonic()
        return True
