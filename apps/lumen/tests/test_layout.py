"""Guards for the layout: Lumen's code lives in apps/lumen; everything about the arm comes from arm/."""
import re
import sys
import unittest
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
ARM_PARAMS = ("HOME", "SAFE_RANGES", "SPEED_CAPS", "ACCEL_CAP", "ELBOW_LIFT_CAP", "STREAM_HZ")


class Layout(unittest.TestCase):
    def test_only_paths_touches_sys_path(self):
        for f in (APP / "lumen").rglob("*.py"):
            if f.name != "paths.py":
                self.assertIsNone(re.search(r"sys\.path\.(insert|append)", f.read_text()), f"{f}: only paths.py may edit sys.path")

    def test_arm_parameters_come_from_arm_config(self):
        """Lumen must not define its own arm caps/ranges, or it could drift from the manual tools."""
        for f in (APP / "lumen").rglob("*.py"):
            for name in ARM_PARAMS:
                self.assertIsNone(re.search(rf"^{name}\s*=", f.read_text(), re.M), f"{f} defines {name}")
        from lumen.movement import config as move_cfg
        import arm_config
        for name in ARM_PARAMS:
            self.assertIs(getattr(move_cfg, name), getattr(arm_config, name))

    def test_one_config_per_folder_and_no_value_in_two(self):
        """Tunables live in their folder's config.py (README "Layout"); a name in two would drift.
        Each model's config is its own namespace (face_cfg.SCORE_THRESHOLD vs book_cfg....), so
        model configs may share names with each other, but not with the shared configs."""
        shared = ["movement", "ai", "state", "light", "io", "ui"]
        models = ["ai/face", "ai/hands", "ai/book", "ai/motion_gate"]
        self.assertFalse((APP / "lumen/config.py").exists(), "use the folder configs, not one global config")

        def names(folder):
            f = APP / "lumen" / folder / "config.py"
            self.assertTrue(f.exists(), f)
            for m in re.finditer(r"^([A-Z][A-Z0-9_]*(?:\s*,\s*[A-Z][A-Z0-9_]*)*)\s*=", f.read_text(), re.M):
                yield from re.split(r"\s*,\s*", m.group(1))
        seen = {}
        for folder in shared:
            for name in names(folder):
                self.assertNotIn(name, seen, f"{name} is in both {seen.get(name)} and {folder}/config.py")
                seen[name] = f"{folder}/config.py"
        for folder in models:
            for name in names(folder):
                self.assertNotIn(name, seen, f"{name} is in both {seen.get(name)} and {folder}/config.py")

    def test_models_present(self):
        self.assertTrue((APP / "models/face_det_lite-tflite-w8a8/face_det_lite.tflite").exists())
        self.assertTrue((APP / "models/mediapipe_hand_gesture-tflite-w8a8/hand_landmark_detector.tflite").exists())
        self.assertTrue((APP / "models/book_det-tflite-w8a8/yolox.tflite").exists())


try:
    import numpy  # noqa: F401
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class Clips(unittest.TestCase):
    def test_every_pool_clip_plays_on_the_arm(self):
        from lumen.movement import config as move_cfg
        from lumen.movement.pools import POOLS
        from trajectory import trajectory
        for pool, entries in POOLS.items():
            for ref, _ in entries:
                t, q, vel, scale = trajectory(ref, move_cfg.HOME)
                self.assertGreater(len(t), 1, (pool, ref))


if __name__ == "__main__":
    unittest.main()
