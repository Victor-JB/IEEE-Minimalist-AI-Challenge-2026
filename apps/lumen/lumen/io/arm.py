"""Motion worker: one thread owns the arm and streams setpoints at 25 Hz.

Why a thread: inference takes ~70 ms per frame, and ArmMotion.stream_point must be called at
least every 0.25 s or the firmware halts and holds. So perception never talks to the arm
directly; it queues commands here.

Real mode uses apps/stepper_desk's ArmMotion (the closed-loop firmware must already be flashed
by apps/stepper_desk/scripts/deploy.sh). Fake mode (--fake-arm) plays the same timing with no
board, so the whole app runs on a laptop with a webcam.

Every command carries a token; finished-events come back as (event, token) so the director can
ignore a CLIP_DONE from a clip that a state change already interrupted.
"""
import math
import queue
import threading
import time

from .. import config as C
from .. import events as E
from .. import paths

DT = 1.0 / C.STREAM_HZ


def _minjerk(s):
    return 10 * s**3 - 15 * s**4 + 6 * s**5


class MotionWorker(threading.Thread):
    def __init__(self, expressive, fake=False):
        super().__init__(daemon=True, name="motion")
        self.x, self.fake = expressive, fake
        self.cmds, self.events = queue.Queue(), queue.Queue()
        self.base = list(C.HOME)            # pose that clips and holds are relative to
        self.offset = [0.0, 0.0, 0.0]       # look-at offset, set by the director every frame
        self.current = list(C.HOME)
        self.relaxed = False
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
        self.arm = None
        try:
            if not self.fake:
                from arm_motion import ArmMotion      # apps/stepper_desk/python
                paths.LOGS.mkdir(exist_ok=True)
                log = paths.LOGS / time.strftime("arm-%Y%m%d-%H%M%S.jsonl")
                self.arm = ArmMotion(output=log, max_speed=60, max_accel=500, tuck=False).__enter__()
            self._loop()
        except Exception as e:                        # ArmError, link loss, stall reports
            self.events.put((E.FAULT, None, repr(e)))
        finally:
            if self.arm:
                try:
                    self.arm.relax()
                    self.arm.close()
                except Exception:
                    pass

    def _loop(self):
        while not self._stop.is_set():
            try:
                cmd = self.cmds.get(timeout=DT)
            except queue.Empty:
                self._hold_tick()
                continue
            kind = cmd[0]
            if kind == "relax":
                if self.arm:
                    self.arm.relax()
                self.relaxed = True
                continue
            self.relaxed = False
            if kind == "play":
                _, ref, amp, ts, token = cmd
                if self._play(ref, amp, ts):
                    self.events.put((E.CLIP_DONE, token, ref))
            elif kind == "goto":
                _, pose, dur, token, done = cmd
                if self._goto(pose, dur):
                    self.base = list(pose)
                    self.events.put((done, token, None))
            elif kind == "sweep":
                _, poses, dwell, token = cmd
                for i, pose in enumerate(poses):
                    if not self._goto(pose, 1.0):
                        break
                    self.events.put((E.AT_POSE, token, i))
                    if not self._wait(dwell):
                        break
                else:
                    self.events.put((E.SWEEP_DONE, token, None))

    # -- motion primitives (return False if interrupted by a new command) ----------
    def _send(self, pose, vel=None):
        pose = [max(lo, min(hi, v)) for v, (lo, hi) in zip(pose, _safe_ranges())]
        self.current = pose
        if self.arm:
            self.arm.stream_point(pose, vel, horizon=0.12)

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
        self._send([b + o + d for b, o, d in zip(self.base, self.offset, drift)])
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

    def _play(self, ref, amplitude, time_scale):
        import numpy as np
        from emote import trajectory               # apps/stepper_desk/scripts/emote.py
        home = [b + o for b, o in zip(self.base, self.offset)]
        t, q, vel, _ = trajectory(ref, home, amplitude, list(C.SPEED_CAPS), C.ACCEL_CAP)
        stretch = max(1.0, time_scale)             # only ever slower: emote already sits at the speed caps
        t, vel = np.asarray(t) * stretch, np.asarray(vel) / stretch
        t0 = time.monotonic()
        for k in range(len(t)):
            if self._interrupted():
                return False
            delay = t0 + t[k] - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            self._send(q[k].tolist(), np.clip(vel[k], -59, 59).tolist())
        return True


def _safe_ranges():
    """Shoulder/elbow/head ranges the real arm can do (emote.py's SAFE, else firmware limits)."""
    try:
        from emote import SAFE
        return SAFE
    except Exception:
        return ((-30, 10), (-70, 77), (-78, 85))
