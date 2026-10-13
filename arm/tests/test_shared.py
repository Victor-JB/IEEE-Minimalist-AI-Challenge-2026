"""The shared pieces luxoiii and the manual tools both rely on: config, clips, trajectories."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
import arm_config as C   # noqa: E402

try:
    import numpy  # noqa: F401
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False


class Config(unittest.TestCase):
    def test_safe_ranges_inside_firmware_limits(self):
        for (lo, hi), (llo, lhi) in zip(C.SAFE_RANGES, C.LIMITS):
            self.assertGreaterEqual(lo, llo + C.MARGIN)
            self.assertLessEqual(hi, lhi - C.MARGIN)
        self.assertGreaterEqual(C.SAFE_RANGES[0][0], -34.0)   # firmware shoulder floor

    def test_home_is_safe(self):
        for h, (lo, hi) in zip(C.HOME, C.SAFE_RANGES):
            self.assertTrue(lo <= h <= hi)


@unittest.skipUnless(HAVE_NUMPY, 'numpy not installed')
class Trajectories(unittest.TestCase):
    def test_every_clip_plays_within_caps_and_ranges(self):
        import numpy as np
        from player import list_clips
        from trajectory import trajectory
        for ref in list_clips():
            t, q, vel, scale = trajectory(ref, C.HOME)
            for j, (lo, hi) in enumerate(C.SAFE_RANGES):
                self.assertTrue(lo - 1e-6 <= q[:, j].min() and q[:, j].max() <= hi + 1e-6, (ref, j))
            # clamping only ever slows motion, so the speed caps hold (10% slack for gradient edges)
            self.assertTrue((np.abs(vel).max(0) <= np.array(C.SPEED_CAPS) * 1.1).all(), ref)


if __name__ == '__main__':
    unittest.main()
