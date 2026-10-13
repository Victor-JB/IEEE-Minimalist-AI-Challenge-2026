"""BookDetector decode, NMS and crop offsets with a fake TFLite interpreter (no model, no runtime).

Runs on plain python3 with numpy; the letterbox test also needs cv2 and skips without it."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import numpy as np
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False
try:
    import cv2  # noqa: F401
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

if HAVE_NUMPY:
    from lumen.ai.book.detector import BookDetector, clip_roi, nms


class FakeInterpreter:
    """Just enough of tflite Interpreter: fixed outputs, records the input it was given."""

    def __init__(self, outputs, size=320, dtype=None, quantization=(1.0, 0)):
        self.arrays = {}
        self.out_details = []
        for i, (name, array, quant) in enumerate(outputs, 1):
            self.arrays[i] = array
            self.out_details.append({"index": i, "name": name, "shape": np.array(array.shape),
                                     "dtype": array.dtype, "quantization": quant})
        self.in_details = [{"index": 0, "shape": np.array([1, size, size, 3]),
                            "dtype": dtype or np.uint8, "quantization": quantization}]
        self.input, self.invokes = None, 0

    def allocate_tensors(self):
        pass

    def get_input_details(self):
        return self.in_details

    def get_output_details(self):
        return self.out_details

    def set_tensor(self, index, tensor):
        self.input = tensor

    def invoke(self):
        self.invokes += 1

    def get_tensor(self, index):
        return self.arrays[index]


def model_dir(tmp, lumen=None):
    meta = {"model_files": {"model.tflite": {}},
            "lumen": lumen or {"decoder": "boxes_scores", "channels": "bgr", "input_range": [0, 255]}}
    (Path(tmp) / "metadata.json").write_text(json.dumps(meta))
    return Path(tmp)


def outputs(boxes, scores, classes=None, names=True):
    """Float outputs in the boxes_scores layout, batch dim added."""
    out = [("boxes" if names else "PartitionedCall:0", np.array([boxes], np.float32), (0.0, 0)),
           ("scores" if names else "PartitionedCall:1", np.array([scores], np.float32), (0.0, 0))]
    if classes is not None:
        out.append(("class_idx", np.array([classes], np.uint8), (0.0, 0)))
    return out


def fake_letterbox(det):
    """Stand-in for _preprocess without cv2: same scale bookkeeping, blank tensor."""
    def preprocess(image):
        h, w = image.shape[:2]
        det.crop_w, det.crop_h = w, h
        det.scale = min(det.input_w / w, det.input_h / h)
        det.seen = image.shape
        return np.zeros((1, det.input_h, det.input_w, 3), np.uint8)
    det._preprocess = preprocess
    return det


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class Nms(unittest.TestCase):
    def test_keeps_the_best_of_overlapping_boxes_and_separate_ones(self):
        boxes = np.array([[0, 0, 100, 100], [5, 5, 105, 105], [200, 200, 260, 260]], np.float32)
        scores = np.array([0.6, 0.9, 0.7], np.float32)
        self.assertEqual(nms(boxes, scores, 0.45), [1, 2])

    def test_threshold_is_iou(self):
        boxes = np.array([[0, 0, 100, 100], [50, 0, 150, 100]], np.float32)   # IoU = 1/3
        scores = np.array([0.9, 0.8], np.float32)
        self.assertEqual(nms(boxes, scores, 0.3), [0])
        self.assertEqual(nms(boxes, scores, 0.4), [0, 1])


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class Decode(unittest.TestCase):
    def make(self, outs, **kwargs):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.interp = FakeInterpreter(outs, **{k: kwargs.pop(k) for k in ("dtype", "quantization") if k in kwargs})
        det = BookDetector(model_dir(self.tmp.name, kwargs.pop("lumen", None)), interpreter=self.interp, **kwargs)
        self.assertTrue(det.available)
        return fake_letterbox(det)

    def test_boxes_come_back_in_frame_pixels(self):
        # 640x480 frame -> 320x320 input: scale 0.5, so model px x2.
        det = self.make(outputs([[10, 20, 60, 120], [0, 0, 5, 5]], [0.8, 0.1]))
        frame = np.zeros((480, 640, 3), np.uint8)
        self.assertEqual(det.detect(frame), [(20, 40, 100, 200, 0.800000011920929)])
        self.assertEqual(self.interp.invokes, 1)

    def test_only_the_book_class_when_the_model_has_class_idx(self):
        det = self.make(outputs([[10, 10, 50, 50], [100, 100, 150, 150]], [0.9, 0.9], classes=[0, 73]))
        dets = det.detect(np.zeros((320, 320, 3), np.uint8))
        self.assertEqual([d[:4] for d in dets], [(100, 100, 50, 50)])

    def test_score_threshold_and_nms(self):
        det = self.make(outputs([[0, 0, 100, 100], [2, 2, 102, 102], [200, 0, 300, 100]], [0.7, 0.9, 0.4]),
                        score_threshold=0.5)
        dets = det.detect(np.zeros((320, 320, 3), np.uint8))
        self.assertEqual(len(dets), 1)
        self.assertAlmostEqual(dets[0][4], 0.9, places=5)

    def test_quantized_outputs_are_dequantized(self):
        boxes = np.array([[[20, 30, 70, 130]]], np.uint8)     # real = (q - 10) * 2
        scores = np.array([[200]], np.uint8)                  # real = q / 250
        det = self.make([("boxes", boxes, (2.0, 10)), ("scores", scores, (1 / 250, 0))])
        self.assertEqual(det.detect(np.zeros((320, 320, 3), np.uint8)), [(20, 40, 100, 200, 0.800000011920929)])

    def test_unnamed_outputs_and_a_trailing_range_pin(self):
        # Our exports lose tensor names and append a constant 1 to the scores.
        det = self.make(outputs([[10, 10, 60, 60]], [0.3, 1.0], names=False), score_threshold=0.5)
        self.assertEqual(det.detect(np.zeros((320, 320, 3), np.uint8)), [])

    def test_boxes_stop_at_the_image_not_the_padding(self):
        # 640x320 frame -> scale 0.5, image fills input rows 0-159; a box running into the pad is cut.
        det = self.make(outputs([[-10, 100, 50, 300]], [0.9]))
        self.assertEqual(det.detect(np.zeros((320, 640, 3), np.uint8))[0][:4], (0, 200, 100, 120))

    def test_roi_crop_is_searched_and_offset_back(self):
        det = self.make(outputs([[0, 0, 160, 80]], [0.9]))
        frame = np.zeros((480, 640, 3), np.uint8)
        dets = det.detect(frame, (300, 200, 160, 80))        # crop 160x80 -> scale 2
        self.assertEqual(det.seen, (80, 160, 3))
        self.assertEqual(dets[0][:4], (300, 200, 80, 40))

    def test_roi_is_clamped_to_the_frame(self):
        det = self.make(outputs([[0, 0, 320, 320]], [0.9]))
        dets = det.detect(np.zeros((480, 640, 3), np.uint8), (600, 400, 100, 100))
        self.assertEqual(det.seen, (80, 40, 3))
        self.assertEqual(dets[0][:4], (600, 400, 40, 80))

    def test_degenerate_roi_means_full_frame(self):
        self.assertIsNone(clip_roi((700, 10, 50, 50), (480, 640)))
        self.assertIsNone(clip_roi(None, (480, 640)))
        det = self.make(outputs([[0, 0, 10, 10]], [0.9]))
        det.detect(np.zeros((480, 640, 3), np.uint8), (0, 0, 3, 3))
        self.assertEqual(det.seen, (480, 640, 3))

    def test_input_lut_for_float_and_offset_quantized_inputs(self):
        det = self.make(outputs([[0, 0, 1, 1]], [0.0]), dtype=np.float32,
                        lumen={"decoder": "boxes_scores", "input_range": [0, 1]})
        self.assertAlmostEqual(float(det.lut[255]), 1.0)
        det = self.make(outputs([[0, 0, 1, 1]], [0.0]), quantization=(1 / 128, 127),
                        lumen={"decoder": "boxes_scores", "input_range": [-1, 1]})
        # real = (q - 127) / 128 must span [-1, 1]: q = 256 p / 255 - 1, clipped to uint8.
        self.assertEqual([int(det.lut[p]) for p in (0, 1, 128, 255)], [0, 0, 128, 255])
        self.assertIsNone(self.make(outputs([[0, 0, 1, 1]], [0.0])).lut)     # 0-255 uint8: no lookup


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class Degrades(unittest.TestCase):
    def test_missing_model_means_unavailable_and_no_books(self):
        with tempfile.TemporaryDirectory() as tmp:
            det = BookDetector(Path(tmp) / "nope")
        self.assertFalse(det.available)
        self.assertEqual(det.detect(np.zeros((480, 640, 3), np.uint8)), [])

    def test_ambiguous_outputs_are_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            outs = outputs([[0, 0, 1, 1]], [0.5], classes=[73], names=False)
            outs[2] = ("PartitionedCall:2",) + outs[2][1:]
            det = BookDetector(model_dir(tmp), interpreter=FakeInterpreter(outs))
        self.assertFalse(det.available)


@unittest.skipUnless(HAVE_NUMPY and HAVE_CV2, "numpy / cv2 not installed")
class Letterbox(unittest.TestCase):
    def test_fit_pad_bottom_right_and_channel_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            det = BookDetector(model_dir(tmp, {"decoder": "boxes_scores", "channels": "rgb", "pad_value": 114,
                                                    "input_range": [0, 255]}),
                               interpreter=FakeInterpreter(outputs([[0, 0, 1, 1]], [0.0])))
        frame = np.zeros((480, 640, 3), np.uint8)
        frame[..., 0] = 255                                   # pure blue in BGR
        t = det._preprocess(frame)
        self.assertEqual(t.shape, (1, 320, 320, 3))
        self.assertEqual(det.scale, 0.5)
        self.assertEqual(t[0, 0, 0].tolist(), [0, 0, 255])    # RGB: blue is last
        self.assertEqual(t[0, 239, 319].tolist(), [0, 0, 255])
        self.assertEqual(t[0, 240, 0].tolist(), [114, 114, 114])   # padding below the image


if __name__ == "__main__":
    unittest.main()
