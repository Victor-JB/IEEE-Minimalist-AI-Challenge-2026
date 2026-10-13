"""SLEEP power hooks with fakes: frame pacing, the opt-in CPU governor, enter/exit wiring."""
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from luxoiii import config as C                                # noqa: E402
from luxoiii.io.power import CpuGovernor, FrameThrottle, Power  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


class Throttle(unittest.TestCase):
    def test_paces_reads_on_fixed_slots(self):
        clock = Clock()
        th = FrameThrottle(clock, clock.sleep)
        th.set_fps(2.0)
        times = []
        for _ in range(5):
            th.wait()
            times.append(clock.t)
            clock.t += 0.07                      # the loop's own work
        self.assertEqual([round(t, 3) for t in times], [0.0, 0.5, 1.0, 1.5, 2.0])

    def test_full_rate_when_off(self):
        clock = Clock()
        th = FrameThrottle(clock, clock.sleep)
        th.set_fps(2.0)
        th.set_fps(None)
        th.wait(); th.wait()
        self.assertEqual(clock.t, 0.0)


class Governor(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp()
        self.files = []
        for cpu in ("cpu0", "cpu1"):
            d = os.path.join(self.root, cpu, "cpufreq")
            os.makedirs(d)
            f = os.path.join(d, "scaling_governor")
            with open(f, "w") as fh:
                fh.write("schedutil\n")
            self.files.append(f)

    def read(self, i):
        with open(self.files[i]) as fh:
            return fh.read().strip()

    def test_off_by_default(self):
        self.assertFalse(CpuGovernor(None, self.root).lower())
        self.assertEqual(self.read(0), "schedutil")

    def test_lowers_and_restores_where_writable(self):
        os.chmod(self.files[1], 0o444)           # not permitted: skipped, no error
        g = CpuGovernor("powersave", self.root)
        self.assertTrue(g.lower())
        self.assertEqual(self.read(0), "powersave")
        self.assertEqual(self.read(1), "schedutil")
        g.restore()
        self.assertEqual(self.read(0), "schedutil")

    def test_missing_sysfs_is_fine(self):
        self.assertFalse(CpuGovernor("powersave", "/nonexistent").lower())


class FakeCamera:
    def __init__(self):
        self.fps = None

    def set_fps(self, fps):
        self.fps = fps


class PowerHooks(unittest.TestCase):
    def test_sleep_and_wake(self):
        cam, p = FakeCamera(), Power(governor=CpuGovernor(None))
        p.sleep()                                # camera not open yet: applied on attach
        p.attach(cam)
        self.assertEqual(cam.fps, C.SLEEP_CAMERA_FPS)
        p.wake()
        self.assertIsNone(cam.fps)


if __name__ == "__main__":
    unittest.main()
