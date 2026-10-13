"""Runtime and geometry checks; accuracy is measured separately on public images."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from luxoiii.perception.gesture_features import FEATURE_VERSION, LandmarkClassifier, features

APP = Path(__file__).resolve().parents[1]
MODEL = APP / "models/gesture_landmarks_v1.npz"


class GestureClassifier(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = LandmarkClassifier(MODEL)
        cls.poses = json.loads((Path(__file__).parent / "fixtures/gesture_landmarks.json").read_text())["poses"]

    def test_supported_commands_and_noncommand_poses(self):
        for name, pose in self.poses.items():
            with self.subTest(pose=name):
                self.assertEqual(self.model.classify(pose["points"]), pose["expected"])

    def test_pointing_and_palm_survive_rotation_and_mirroring(self):
        for name in ("one", "point", "palm"):
            p = np.asarray(self.poses[name]["points"])
            for angle in np.linspace(0, 2*np.pi, 16, endpoint=False):
                rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
                for sign in (-1, 1):
                    with self.subTest(pose=name, angle=angle, mirror=sign):
                        moved = p @ rotation.T * [sign, 1]
                        self.assertEqual(self.model.classify(moved), self.poses[name]["expected"])

    def test_image_aspect_and_position_do_not_change_features_or_labels(self):
        for name, pose in self.poses.items():
            p = np.asarray(pose["points"])
            # Pixel coordinates from differently shaped frames must agree with
            # source coordinates normalized independently by width and height.
            for scale in ([360, 480], [1920, 1080], [.7, 1.4]):
                with self.subTest(pose=name, scale=scale):
                    moved = p * scale + [173, -72]
                    np.testing.assert_allclose(features(moved), features(p), atol=.002)
                    self.assertEqual(self.model.classify(moved), pose["expected"])

    def test_batched_probabilities_agree_with_single_hand(self):
        p = np.asarray([pose["points"] for pose in self.poses.values()])
        batch = self.model.probabilities(p)
        self.assertEqual(batch.shape, (len(p), 5))
        np.testing.assert_allclose(batch.sum(axis=1), 1, atol=1e-6)
        for i, hand in enumerate(p):
            np.testing.assert_allclose(batch[i], self.model.probabilities(hand), atol=1e-6)

    def test_invalid_or_degenerate_hands_do_not_generate_commands(self):
        invalid = (None, [], [[0, 0]]*20, [[0, 0]]*21, np.full((21, 3), np.nan),
                   np.full((21, 3), np.inf), [["invalid", 0]]*21, np.zeros((21, 1)))
        for points in invalid:
            with self.subTest(points=repr(points)[:70]):
                self.assertIsNone(self.model.classify(points))

    def test_low_confidence_rejects_command(self):
        self.assertIsNone(self.model.classify(self.poses["point"]["points"], threshold=1.0))

    def test_artifact_rejects_wrong_dimensions_nonfinite_weights_and_version(self):
        with np.load(MODEL, allow_pickle=False) as source:
            valid = {key: source[key] for key in source}
        variants = [{**valid, "w1": np.zeros((105, 48))},
                    {**valid, "b2": np.full(5, np.nan)},
                    {**valid, "feature_version": np.array(FEATURE_VERSION+1)}]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "invalid.npz"
            for weights in variants:
                np.savez(path, **weights)
                with self.assertRaises(ValueError):
                    LandmarkClassifier(path)


if __name__ == "__main__":
    unittest.main()
