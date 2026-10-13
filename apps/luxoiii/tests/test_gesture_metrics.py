"""Gesture score regressions: coverage, misses, commands, and unchanged IoU."""
import copy
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


TOOLS = Path(__file__).resolve().parents[3] / "tools/efficiency"


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, TOOLS / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


with patch.dict(sys.modules, {"metrics": load("gesture_metric_helpers", "metrics.py")}):
    scorer = load("gesture_scorer", "score_gestures.py")


def sample(labels, boxes, predictions, image_id="frame"):
    """Use a 100x100 frame so annotation-to-pixel conversion stays explicit."""
    manifest = {"rows": [{"id": image_id, "labels": labels,
                          "boxes": [[v / 100 for v in box] for box in boxes]}]}
    cache = {"rows": [{"id": image_id, "shape": [100, 100, 3],
                       "predictions": predictions}]}
    return cache, manifest


def detection(box, label="point", score=.9):
    return {"box": box, "label": label, "score": score}


class GestureMetrics(unittest.TestCase):
    def test_every_manifest_image_is_required_exactly_once(self):
        cache, manifest = sample(["one"], [[0, 0, 10, 10]], [])
        invalid = []
        missing = copy.deepcopy(cache)
        missing["rows"] = []
        invalid.append((missing, manifest))
        duplicate_cache = copy.deepcopy(cache)
        duplicate_cache["rows"].append(copy.deepcopy(cache["rows"][0]))
        invalid.append((duplicate_cache, manifest))
        duplicate_manifest = copy.deepcopy(manifest)
        duplicate_manifest["rows"].append(copy.deepcopy(manifest["rows"][0]))
        invalid.append((cache, duplicate_manifest))
        unknown = copy.deepcopy(cache)
        unknown["rows"][0]["id"] = "unknown"
        invalid.append((unknown, manifest))
        for bad_cache, bad_manifest in invalid:
            with self.subTest(cache=bad_cache, manifest=bad_manifest):
                with self.assertRaisesRegex(ValueError, "exactly once"):
                    scorer.score(bad_cache, bad_manifest)

    def test_cache_order_does_not_change_coverage_or_scores(self):
        cache, manifest = sample(["one"], [[0, 0, 10, 10]],
                                 [detection([0, 0, 10, 10])])
        second_cache, second_manifest = sample(["palm"], [[20, 0, 10, 10]],
                                               [detection([20, 0, 10, 10], "open_palm")], "second")
        cache["rows"] += second_cache["rows"]
        manifest["rows"] += second_manifest["rows"]
        cache["rows"].reverse()
        result = scorer.score(cache, manifest)
        self.assertEqual((result["images"], result["gt_hands"], result["correct"]), (2, 2, 2))

    def test_missed_command_stays_in_end_to_end_denominator(self):
        cache, manifest = sample(["point", "no_gesture"],
                                 [[0, 0, 10, 10], [20, 0, 10, 10]],
                                 [detection([20, 0, 10, 10], None)])
        result = scorer.score(cache, manifest)
        self.assertEqual((result["gt_hands"], result["matched_hands"], result["missed_hands"]), (2, 1, 1))
        self.assertEqual(result["matched_accuracy"], 1.)
        self.assertEqual(result["end_to_end_recall"], .5)
        self.assertEqual(result["confusion"], {"none -> none": 1, "point -> miss": 1})

    def test_undetected_noncommand_is_a_miss_not_a_correct_rejection(self):
        cache, manifest = sample(["no_gesture", "fist"],
                                 [[0, 0, 10, 10], [20, 0, 10, 10]], [])
        result = scorer.score(cache, manifest)
        self.assertEqual(result["confusion"], {"none -> miss": 2})
        self.assertEqual((result["correct"], result["missed_hands"]), (0, 2))
        self.assertEqual(result["end_to_end_recall"], 0.)

    def test_wrong_and_unmatched_commands_reduce_command_precision(self):
        cache, manifest = sample(
            ["one", "palm", "no_gesture"],
            [[0, 0, 10, 10], [20, 0, 10, 10], [40, 0, 10, 10]],
            [detection([0, 0, 10, 10], "open_palm"),  # wrong command
             detection([20, 0, 10, 10], "open_palm"),  # correct command
             detection([40, 0, 10, 10], "thumbs_up"),  # command on a negative
             detection([80, 80, 10, 10], "point"),  # unmatched command
             detection([60, 60, 10, 10], None)],  # unmatched non-command
        )
        result = scorer.score(cache, manifest)
        self.assertEqual((result["matched_hands"], result["unmatched_predictions"]), (3, 2))
        self.assertEqual(result["command_predictions"], 4)
        self.assertEqual(result["incorrect_or_unmatched_commands"], 3)
        self.assertEqual(result["command_precision"], .25)
        self.assertEqual(result["correct"], 1)

    def test_duplicate_command_cannot_match_the_same_hand_twice(self):
        cache, manifest = sample(["one"], [[0, 0, 10, 10]],
                                 [detection([0, 0, 10, 10], score=.95),
                                  detection([0, 0, 10, 10], score=.8)])
        result = scorer.score(cache, manifest)
        self.assertEqual((result["correct"], result["unmatched_predictions"]), (1, 1))
        self.assertEqual(result["incorrect_or_unmatched_commands"], 1)
        self.assertEqual(result["command_precision"], .5)

    def test_original_half_iou_boundary_is_preserved_without_padding(self):
        for predicted_width, expected in ((5., 1), (4.999, 0)):
            with self.subTest(width=predicted_width):
                cache, manifest = sample(["one"], [[0, 0, 10, 10]],
                                         [detection([0, 0, predicted_width, 10])])
                before = copy.deepcopy(cache)
                result = scorer.score(cache, manifest)
                self.assertEqual(result["correct"], expected)
                self.assertEqual(result["missed_hands"], 1 - expected)
                self.assertEqual(result["unmatched_predictions"], 1 - expected)
                self.assertEqual(cache, before)
                self.assertEqual(result["rows"][0]["predictions"][0]["box"],
                                 [0, 0, predicted_width, 10])

    def test_landmark_threshold_filters_before_classifier_callback(self):
        cache, manifest = sample(["one", "one"],
                                 [[0, 0, 10, 10], [20, 0, 10, 10]],
                                 [detection([0, 0, 10, 10], None, .65),
                                  detection([20, 0, 10, 10], None, .649)])
        called = []

        def classifier(prediction):
            called.append(prediction)
            return "point"

        result = scorer.score(cache, manifest, classifier, threshold=.65)
        self.assertEqual(len(called), 1)
        self.assertEqual((result["correct"], result["missed_hands"]), (1, 1))
        self.assertEqual(result["command_predictions"], 1)


if __name__ == "__main__":
    unittest.main()
