"""Low-power SLEEP: fewer camera frames and, opt-in, a slower CPU governor. No cv2 here (testable).

Camera fps: OpenCV's V4L2 backend applies CAP_PROP_FPS by restarting the stream (and UVC cameras
only offer a few fixed rates), so instead of changing the driver rate live, FrameThrottle makes
Camera.read() wait for the next slot: the main loop, the JPEG decode and the motion gate all run
at the sleep rate, and the CPU idles in between.

CPU governor: off unless config.SLEEP_CPU_GOVERNOR names one (e.g. "powersave"). It writes
/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor only where this user may already write
(no sudo; give the user access with a udev rule if you want it), and puts the old one back on wake.
Any error just skips the hook.
"""
import glob
import os
import time

from .. import config as C


class FrameThrottle:
    """Paces reads to `fps` on fixed slots (no drift); fps None = as fast as the camera."""

    def __init__(self, clock=time.monotonic, sleep=time.sleep):
        self.clock, self.sleep = clock, sleep
        self.fps, self.next_at = None, None

    def set_fps(self, fps):
        self.fps, self.next_at = (fps or None), None

    def wait(self):
        if not self.fps:
            return
        period, now = 1.0 / self.fps, self.clock()
        if self.next_at is None or now - self.next_at > period:   # first slot, or we fell behind
            self.next_at = now
        elif now < self.next_at:
            self.sleep(self.next_at - now)
        self.next_at += period


class CpuGovernor:
    """Opt-in: set scaling_governor while asleep, restore it on wake. Never raises."""

    def __init__(self, governor=None, root="/sys/devices/system/cpu"):
        self.governor, self.root, self.saved = governor, root, {}

    def lower(self):
        if not self.governor or self.saved:
            return False
        for path in sorted(glob.glob(os.path.join(self.root, "cpu*/cpufreq/scaling_governor"))):
            try:
                if not os.access(path, os.W_OK):
                    continue
                with open(path) as f:
                    old = f.read().strip()
                with open(path, "w") as f:
                    f.write(self.governor)
                self.saved[path] = old
            except OSError:
                continue
        return bool(self.saved)

    def restore(self):
        for path, old in self.saved.items():
            try:
                with open(path, "w") as f:
                    f.write(old)
            except OSError:
                pass
        self.saved = {}


class Power:
    """SLEEP enter/exit hooks for the director. The camera is attached once it is open."""

    def __init__(self, camera=None, governor=None):
        self.camera, self.governor = camera, governor or CpuGovernor(C.SLEEP_CPU_GOVERNOR)
        self.asleep = False

    def attach(self, camera):
        self.camera = camera
        if self.asleep:
            camera.set_fps(C.SLEEP_CAMERA_FPS)

    def sleep(self):
        self.asleep = True
        if self.camera:
            self.camera.set_fps(C.SLEEP_CAMERA_FPS)
        self.governor.lower()

    def wake(self):
        self.asleep = False
        if self.camera:
            self.camera.set_fps(None)
        self.governor.restore()
