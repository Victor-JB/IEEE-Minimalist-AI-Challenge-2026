"""Hand pipeline math and control flow with fake interpreters: no OpenCV or TFLite needed.

The last class runs the real models when OpenCV and a TFLite runtime are installed; set
HAND_IMAGE=/path/to/photo.jpg to also check a real hand is found."""
import math
import os
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import numpy as np
    from luxoiii.perception import hands as H          # noqa: E402
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False


class FakeInterp:
    """Just enough of tflite's Interpreter: named float tensors with scale 1, zero point 0."""

    def __init__(self, inputs, outputs):
        self.inputs = [dict(index=i, name=n, shape=s, dtype=np.uint8, quantization=q)
                       for i, (n, s, q) in enumerate(inputs)]
        self.outputs = [dict(index=100 + i, name=n, shape=s, dtype=np.float32, quantization=(1.0, 0))
                        for i, (n, s) in enumerate(outputs)]
        self.tensors = {d["index"]: np.zeros(d["shape"], np.float32) for d in self.outputs}
        self.invokes, self.fed = 0, {}
        self.on_invoke = None

    def get_input_details(self):
        return self.inputs

    def get_output_details(self):
        return self.outputs

    def set_tensor(self, index, value):
        self.fed[index] = value

    def invoke(self):
        self.invokes += 1
        if self.on_invoke:
            self.on_invoke(self)

    def get_tensor(self, index):
        return self.tensors[index]

    def set(self, name, value):
        d = next(d for d in self.outputs if d["name"] == name)
        self.tensors[d["index"]] = np.asarray(value, np.float32).reshape(d["shape"])


def fake_palm():
    return FakeInterp([("image", (1, 256, 256, 3), (1 / 255, 0))],
                      [("box_coords", (1, 2944, 18)), ("box_scores", (1, 2944))])


def fake_landmark():
    return FakeInterp([("image", (1, 224, 224, 3), (1 / 255, 0))],
                      [("landmarks", (1, 63)), ("scores", (1, 1)), ("lr", (1, 1)), ("world_landmarks", (1, 63))])


def upright_hand(cx=112.0, cy=112.0, s=40.0):
    """21 points of an open hand, fingers up, around (cx, cy) in whatever pixels."""
    pts = np.zeros((21, 3), np.float32)
    pts[0] = (cx + 0.1 * s, cy + s, 0)             # under the middle finger
    for f, x in enumerate((-0.8, -0.3, 0.1, 0.5, 0.9)):        # thumb .. pinky columns
        for j in range(4):
            pts[1 + 4 * f + j] = (cx + x * s, cy + s * (0.3 - 0.35 * j), 0)
    return pts


def palm_detection(x1, y1, x2, y2):
    """One decoded detection (model pixels): box + 7 keypoints, wrist below the middle MCP."""
    det = np.zeros(18, np.float32)
    det[:4] = (x1, y1, x2, y2)
    cx, cy, h = (x1 + x2) / 2, (y1 + y2) / 2, y2 - y1
    det[4:6] = (cx, y2)                 # keypoint 0: wrist
    det[8:10] = (cx, y1 + h * 0.2)      # keypoint 2: middle-finger MCP, straight above
    return det


if HAVE_NUMPY:
    class NumpyHands(H.HandLandmarker):
        """Image ops replaced by blanks so the tests never touch OpenCV."""

        @staticmethod
        def _resize(img, size):
            return np.zeros((size[1], size[0]) + img.shape[2:], np.uint8)

        @staticmethod
        def _warp(frame, A, size):
            return np.zeros((size, size, 3), np.uint8)


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class Anchors(unittest.TestCase):
    def test_count_and_layout(self):
        a = H.palm_anchors()
        self.assertEqual(a.shape, (2944, 2))
        np.testing.assert_allclose(a[0], a[1])                      # 2 anchors per 8-px cell
        np.testing.assert_allclose(a[0], (0.5 / 32, 0.5 / 32))
        np.testing.assert_allclose(a[2], (1.5 / 32, 0.5 / 32))      # row-major: x first
        np.testing.assert_allclose(a[2048], (0.5 / 16, 0.5 / 16))   # 16-px grid starts after 32*32*2
        np.testing.assert_allclose(a[2560:2566], [(0.5 / 8, 0.5 / 8)] * 6)   # 8x8 grid, 6 per cell
        np.testing.assert_allclose(a[-1], (7.5 / 8, 7.5 / 8))


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class Decoding(unittest.TestCase):
    def test_decode_raw_adds_anchor_centers_not_to_size(self):
        anchors = H.palm_anchors()
        raw = np.zeros((2944, 18), np.float32)
        raw[2048, :4] = (2, -3, 20, 30)                  # center offset, w, h
        raw[2048, 4:6] = (1, 1)                          # keypoint 0 offset
        logits = np.zeros(2944, np.float32)
        logits[2048] = 100
        coords, scores = H.decode_raw(raw, logits, anchors)
        c = anchors[2048] * 256                          # (8, 8)
        np.testing.assert_allclose(coords[2048, :4], (c[0] + 2 - 10, c[1] - 3 - 15, c[0] + 2 + 10, c[1] - 3 + 15))
        np.testing.assert_allclose(coords[2048, 4:6], c + 1)
        self.assertAlmostEqual(scores[2048], 1.0)
        self.assertAlmostEqual(scores[0], 0.5)

    def test_nms_keeps_best_of_overlapping(self):
        boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11], [50, 50, 60, 60]], np.float32)
        self.assertEqual(H.nms(boxes, np.array([0.8, 0.9, 0.7]), top_k=5), [1, 2])
        self.assertEqual(H.nms(boxes, np.array([0.8, 0.9, 0.7]), top_k=1), [1])

    def test_rotation_turns_fingers_up(self):
        self.assertAlmostEqual(H.rotation((0, 0), (0, -1)), 0)           # already up
        self.assertAlmostEqual(H.rotation((0, 0), (1, 0)), math.pi / 2)  # fingers to the right
        self.assertAlmostEqual(H.rotation((0, 0), (-1, 0)), -math.pi / 2)
        self.assertAlmostEqual(abs(H.rotation((0, 0), (0, 1))), math.pi)

    def test_affine_maps_crop_to_rotated_square(self):
        r = H.Rect(100, 200, 112, math.pi / 2)            # fingers to the right
        A = r.affine()
        apply = lambda p: A[:, :2] @ np.array(p, np.float32) + A[:, 2]
        np.testing.assert_allclose(apply((112, 112)), (100, 200), atol=1e-4)    # center -> center
        np.testing.assert_allclose(apply((112, 0)), (156, 200), atol=1e-4)      # crop top -> right
        np.testing.assert_allclose(apply((224, 112)), (100, 256), atol=1e-4)    # crop right -> down

    def test_palm_rect_shifts_toward_fingers_and_grows(self):
        r = H.rect_from_palm(palm_detection(100, 100, 140, 140))
        self.assertAlmostEqual(r.rotation, 0)
        self.assertAlmostEqual(r.cx, 120)
        self.assertAlmostEqual(r.cy, 120 - 0.5 * 40)
        self.assertAlmostEqual(r.size, 40 * 2.6, places=4)
        # Same palm turned to point right: the shift goes right.
        det = palm_detection(100, 100, 140, 140)
        det[4:6], det[8:10] = (100, 120), (132, 120)
        r = H.rect_from_palm(det)
        self.assertAlmostEqual(r.rotation, math.pi / 2)
        self.assertAlmostEqual(r.cx, 140)
        self.assertAlmostEqual(r.cy, 120)

    def test_landmark_rect_follows_the_hand(self):
        pts = upright_hand(300, 200, 50)
        r = H.rect_from_landmarks(pts)
        self.assertAlmostEqual(r.rotation, 0, places=5)
        self.assertTrue(250 < r.cx < 350 and 120 < r.cy < 220)
        self.assertGreater(r.size, 2 * 50)
        # Rotating the hand 90 degrees rotates the crop with it.
        rot = pts.copy()
        rot[:, 0], rot[:, 1] = 300 + (200 - pts[:, 1]), 200 + (pts[:, 0] - 300)   # fingers now right
        self.assertAlmostEqual(H.rect_from_landmarks(rot).rotation, math.pi / 2, places=5)

    def test_clip_roi(self):
        self.assertEqual(H.clip_roi(None, (480, 640, 3)), (0, 0, 640, 480))
        self.assertEqual(H.clip_roi((-10, 400, 100, 200), (480, 640, 3)), (0, 400, 90, 80))
        self.assertEqual(H.clip_roi((700, 0, 50, 50), (480, 640, 3))[2], 0)


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class Pipeline(unittest.TestCase):
    """Fake models: the palm finds one box; the landmark model returns an upright hand."""

    def setUp(self):
        self.palm, self.lm = fake_palm(), fake_landmark()
        coords, scores = np.zeros((2944, 18), np.float32), np.zeros(2944, np.float32)
        coords[7] = palm_detection(40, 40, 80, 80)
        scores[7] = 0.9
        self.palm.set("box_coords", coords)
        self.palm.set("box_scores", scores)
        self.lm.set("landmarks", upright_hand())         # crop pixels, centered in the 224 crop
        self.lm.set("scores", [0.95])
        self.frame = np.zeros((480, 640, 3), np.uint8)

    def make(self, **kw):
        return NumpyHands(interpreters=(self.palm, self.lm), **kw)

    def test_fake_models_load(self):
        h = self.make()
        self.assertTrue(h.available)
        self.assertEqual(len(h.detect(self.frame)), 1)
        self.assertEqual(len(h.detect(self.frame)[0]), 21)

    def test_roi_offsets_palm_back_to_frame(self):
        h = self.make()
        roi = (100, 50, 320, 240)                          # letterbox scale 256 / 320 = 0.8
        h.detect(self.frame, roi)
        self.assertEqual(self.palm.fed[0].shape, (1, 256, 256, 3))
        det = h.decode_palms(256 / 320, roi[:2])[0]
        np.testing.assert_allclose(det[:4], (100 + 50, 50 + 50, 100 + 100, 50 + 100), atol=1e-4)
        np.testing.assert_allclose(det[4:6], (100 + 75, 50 + 100), atol=1e-4)   # keypoints offset too
        rect = H.rect_from_palm(det)
        self.assertAlmostEqual(h.last[0].rect.cx, rect.cx, places=4)
        # Landmarks: crop center -> rect center, so the hand lands where the palm was.
        pts = np.array(h.last[0].points)
        A = rect.affine()
        np.testing.assert_allclose(pts[:, :2], upright_hand()[:, :2] @ A[:, :2].T + A[:, 2], atol=1e-3)
        np.testing.assert_allclose(pts[:, 2], 0)

    def test_raw_palm_export_decodes_to_the_same_box(self):
        anchors = H.palm_anchors()
        det = palm_detection(40, 40, 80, 80)
        raw = np.zeros((2944, 18), np.float32)
        center = anchors[7] * 256
        raw[7, :4] = (60 - center[0], 60 - center[1], 40, 40)
        raw[7, 4:] = (det[4:].reshape(7, 2) - center).ravel()
        logits = np.full(2944, -10, np.float32)
        logits[7] = 3
        self.palm.set("box_coords", raw)
        self.palm.set("box_scores", logits)
        h = self.make(palm_decoded=False)
        h.detect(self.frame)
        np.testing.assert_allclose(h.decode_palms(1, (0, 0))[0], det, atol=1e-4)

    def test_presence_score_gates_the_hand(self):
        self.lm.set("scores", [0.3])
        self.assertEqual(self.make().detect(self.frame), [])

    def test_no_palm_no_landmark_run(self):
        self.palm.set("box_scores", np.zeros(2944))
        self.assertEqual(self.make().detect(self.frame), [])
        self.assertEqual(self.lm.invokes, 0)

    def test_tracking_skips_the_palm_detector(self):
        h = self.make()
        h.detect(self.frame)
        h.detect(self.frame)
        h.detect(self.frame)
        self.assertEqual((self.palm.invokes, self.lm.invokes), (1, 3))
        self.assertTrue(h.last[0].tracked)

    def test_two_hand_mode_discovers_a_second_hand_then_tracks_both(self):
        h = self.make(max_hands=2)
        self.assertEqual(len(h.detect(self.frame)), 1)
        coords, scores = np.zeros((2944, 18), np.float32), np.zeros(2944, np.float32)
        coords[7] = palm_detection(40, 40, 80, 80)
        coords[17] = palm_detection(140, 40, 180, 80)
        scores[7], scores[17] = 0.9, 0.85
        self.palm.set("box_coords", coords)
        self.palm.set("box_scores", scores)
        self.assertEqual(len(h.detect(self.frame)), 2)
        self.assertEqual(len(h.detect(self.frame)), 2)
        self.assertTrue(all(hand.tracked for hand in h.last))
        self.assertEqual((self.palm.invokes, self.lm.invokes), (2, 5))

    def test_lost_track_falls_back_to_palm_on_the_same_frame(self):
        h = self.make()
        h.detect(self.frame)                               # palm + landmark
        scores = iter([0.2, 0.9])                          # tracked crop fails, the fresh one passes
        self.lm.on_invoke = lambda it: it.set("scores", [next(scores)])
        hands = h.detect(self.frame)
        self.assertEqual(len(hands), 1)
        self.assertEqual((self.palm.invokes, self.lm.invokes), (2, 3))
        self.assertFalse(h.last[0].tracked)

    def test_hand_gone_clears_the_track(self):
        h = self.make()
        h.detect(self.frame)
        self.lm.set("scores", [0.1])
        self.assertEqual(h.detect(self.frame), [])
        self.assertEqual(h._tracks, [])
        self.assertEqual(self.palm.invokes, 2)

    def test_tracking_off_runs_palm_every_call(self):
        h = self.make(tracking=False)
        for _ in range(3):
            h.detect(self.frame)
        self.assertEqual((self.palm.invokes, self.lm.invokes), (3, 3))

    def test_missing_models_degrade_to_no_hands(self):
        h = H.HandLandmarker(model_dir=Path("/nonexistent"))
        self.assertFalse(h.available)
        self.assertEqual(h.detect(self.frame), [])


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class Canned(unittest.TestCase):
    def test_features(self):
        pts = upright_hand(300, 200, 50)
        f = H.canned_features(pts, 1.0)
        self.assertEqual(f.shape, (64,))
        self.assertEqual(f[63], 1.0)
        self.assertAlmostEqual(float(max(np.ptp(f[0:63:3]), np.ptp(f[1:63:3]))), 1.0, places=4)
        np.testing.assert_allclose(H.canned_features(pts * 3, 1.0), f, atol=1e-5)    # scale-free
        m = H.canned_features(pts, 1.0, mirror=True)
        np.testing.assert_allclose(m[0:63:3], -f[0:63:3], atol=1e-5)
        self.assertEqual(m[63], 0.0)

    def test_classifier_quantizes_inputs_and_picks_argmax(self):
        q = (0.013154414482414722, 147)
        it = FakeInterp([("hand", (1, 64), q), ("mirrored_hand", (1, 64), q)], [("Identity", (1, 8))])
        it.set("Identity", [0.1, 0, 0, 0, 0, 0.8, 0.1, 0])
        c = H.CannedGestures(interpreter=it)
        label, prob = c.classify(upright_hand(), 0.7)
        self.assertEqual(label, "Thumb_Up")
        self.assertAlmostEqual(prob, 0.8, places=5)
        fed = it.fed[0][0].astype(np.float32)
        np.testing.assert_allclose((fed - q[1]) * q[0], H.canned_features(upright_hand(), 1.0), atol=q[0])
        self.assertEqual(H.CANNED_TO_RULES["Thumb_Up"], "thumbs_up")


def _runtime():
    try:
        import cv2  # noqa: F401
        return HAVE_NUMPY and H.Interpreter is not None
    except ImportError:
        return False


@unittest.skipUnless(_runtime(), "needs opencv + a TFLite runtime")
class RealModels(unittest.TestCase):
    def test_blank_frame_has_no_hand(self):
        h = H.HandLandmarker()
        self.assertTrue(h.available)
        self.assertEqual(h.detect(np.zeros((480, 640, 3), np.uint8)), [])
        self.assertEqual(h.detect(np.zeros((480, 640, 3), np.uint8), (100, 100, 200, 200)), [])

    @unittest.skipUnless(os.environ.get("HAND_IMAGE"), "set HAND_IMAGE to a hand photo")
    def test_photo(self):
        import cv2
        frame = cv2.imread(os.environ["HAND_IMAGE"])
        h = H.HandLandmarker()
        hands = h.detect(frame)
        self.assertGreaterEqual(len(hands), 1)
        H_, W_ = frame.shape[:2]
        for x, y, _ in hands[0]:
            self.assertTrue(-0.1 * W_ < x < 1.1 * W_ and -0.1 * H_ < y < 1.1 * H_)
        self.assertEqual(len(h.detect(frame)), 1)          # second call: tracked
        self.assertTrue(h.last[0].tracked)
        label, _ = H.CannedGestures().classify(hands[0], h.last[0].right)
        self.assertIn(label, H.CANNED_LABELS)


if __name__ == "__main__":
    unittest.main()
