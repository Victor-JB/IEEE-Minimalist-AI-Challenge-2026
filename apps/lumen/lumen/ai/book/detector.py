"""Book detection: a small COCO detector (int8 TFLite), keeping only the "book" class.

The model lives in apps/lumen/models/book_det-tflite-w8a8/ (model + metadata.json with source
and license). bench/eval_detector.py measures it (recall, false positives, latency). It only runs in SEARCH
(at each sweep pause), AIM, ILLUMINATE/HOLD (0.5 Hz check) and REACQUIRE, and on a crop
around the last book when Perception has one (ctx["book_roi"]). Settings: ai/book/config.py.

Model I/O (the "boxes_scores" layout, also what Qualcomm AI Hub's YOLO exports use):
    input      [1, S, S, 3]  uint8 (or float32), channel order and range set by the tunables below
    boxes      [1, N, 4]     x1, y1, x2, y2 in input pixels, one box per anchor
    scores     [1, N]        confidence of that box (extra trailing entries are ignored)
    class_idx  [1, N]        optional: the box's COCO class (80-class list, "book" = 73). A
                             model without it was exported to score the book class only.
Quantized outputs are dequantized with real = (q - zero_point) * scale. Outputs are found by
name, or by shape when the converter dropped the names.

If the model or the TFLite runtime is missing, the detector says so once, sets
available = False and returns [] forever, so Lumen still runs (SEARCH just never finds one).
"""

import json

import numpy as np

from ...perfstats import timed  # does nothing unless PERF=1

from .config import (BOOK_CLASS, CHANNELS, INPUT_RANGE, MIN_ROI_PX,  # edit in
                     MODEL_DIR, NMS_THRESHOLD, NUM_THREADS, PAD_VALUE, SCORE_THRESHOLD)  # ai/book/config.py


def load_interpreter(model_path, num_threads):
    """A TFLite interpreter from whichever runtime is installed (ImportError if none)."""
    try:
        from ai_edge_litert.interpreter import Interpreter
    except ImportError:
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            from tensorflow.lite import Interpreter
    return Interpreter(model_path=str(model_path), num_threads=num_threads)


def nms(boxes, scores, iou_threshold):
    """Indices of the boxes to keep, best first. boxes: (N, 4) x1, y1, x2, y2."""
    order = np.argsort(-scores, kind="stable")
    areas = np.clip(boxes[:, 2] - boxes[:, 0], 0, None) * np.clip(boxes[:, 3] - boxes[:, 1], 0, None)
    keep = []
    while order.size:
        i, rest = order[0], order[1:]
        keep.append(int(i))
        w = np.clip(np.minimum(boxes[i, 2], boxes[rest, 2]) - np.maximum(boxes[i, 0], boxes[rest, 0]), 0, None)
        h = np.clip(np.minimum(boxes[i, 3], boxes[rest, 3]) - np.maximum(boxes[i, 1], boxes[rest, 1]), 0, None)
        inter = w * h
        iou = inter / np.maximum(areas[i] + areas[rest] - inter, 1e-9)
        order = rest[iou <= iou_threshold]
    return keep


def clip_roi(roi, shape):
    """(x, y, w, h) roi clamped to the frame, or None for the full frame (or an empty roi)."""
    if roi is None:
        return None
    fh, fw = shape[:2]
    x0, y0 = max(0, int(roi[0])), max(0, int(roi[1]))
    x1, y1 = min(fw, int(roi[0] + roi[2])), min(fh, int(roi[1] + roi[3]))
    return (x0, y0, x1 - x0, y1 - y0) if min(x1 - x0, y1 - y0) >= MIN_ROI_PX else None


def name_outputs(details):
    """{"boxes": d, "scores": d[, "class_idx": d]}. TFLite exports often lose the tensor names
    (PartitionedCall:0 ...), so fall back to shapes: boxes are [1, N, 4], scores the one [1, M]."""
    by_name = {d["name"]: d for d in details}
    if "boxes" in by_name and "scores" in by_name:
        return {k: v for k, v in by_name.items() if k in ("boxes", "scores", "class_idx")}
    boxes = [d for d in details if len(d["shape"]) == 3 and d["shape"][-1] == 4]
    flat = [d for d in details if len(d["shape"]) == 2]
    if len(boxes) != 1 or len(flat) != 1:
        raise ValueError(f"can't tell boxes from scores in outputs {[d['name'] for d in details]}")
    return {"boxes": boxes[0], "scores": flat[0]}


class BookDetector:
    available = False

    def __init__(
        self,
        model_dir=MODEL_DIR,
        score_threshold=SCORE_THRESHOLD,
        nms_threshold=NMS_THRESHOLD,
        num_threads=NUM_THREADS,
        interpreter=None,  # tests pass a fake; normally loaded from model_dir
    ):
        self.score_threshold = score_threshold
        self.nms_threshold = nms_threshold
        self.scale = 1.0  # crop pixels -> model pixels, set per frame by _preprocess
        self.crop_w = self.crop_h = 0
        try:
            self._load(model_dir, num_threads, interpreter)
        except (ImportError, OSError, ValueError, KeyError, RuntimeError) as e:
            print(f"Book detector off ({type(e).__name__}: {e}); SEARCH will never find a book.")
            return
        self.available = True

    def _load(self, model_dir, num_threads, interpreter):
        # Load once; this is the slow part. A "lumen" block in metadata.json may override the tunables.
        meta = json.loads((model_dir / "metadata.json").read_text())
        (name,) = meta["model_files"]
        info = meta.get("lumen", {})
        if interpreter is None:
            interpreter = load_interpreter(model_dir / name, num_threads)
        interpreter.allocate_tensors()
        self.interpreter = interpreter
        self.book_class = info.get("book_class", BOOK_CLASS)
        self.rgb = info.get("channels", CHANNELS) == "rgb"
        self.pad_value = info.get("pad_value", PAD_VALUE)

        inp = interpreter.get_input_details()[0]
        self.input_index = inp["index"]
        self.input_h, self.input_w = int(inp["shape"][1]), int(inp["shape"][2])
        self.lut = self._input_lut(inp, info.get("input_range", INPUT_RANGE))
        self.outputs = name_outputs(interpreter.get_output_details())

    @staticmethod
    def _input_lut(inp, value_range):
        """Pixel 0-255 -> input tensor value, or None when the raw pixel already is the input."""
        lo, hi = value_range
        real = np.arange(256, dtype=np.float32) / 255 * (hi - lo) + lo
        dtype = inp["dtype"]
        if np.issubdtype(dtype, np.integer):
            scale, zero_point = inp["quantization"]
            info = np.iinfo(dtype)
            lut = np.clip(np.round(real / scale + zero_point), info.min, info.max).astype(dtype)
        else:
            lut = real.astype(dtype)
        return None if np.array_equal(lut, np.arange(256)) and dtype == np.uint8 else lut

    @timed("book.total")
    def detect(self, frame, roi=None):
        """List of (x, y, w, h, score) in frame pixels, best first, for a BGR frame.

        roi (x, y, w, h) limits the search to that crop; boxes still come back in frame pixels."""
        if not self.available:
            return []
        roi = clip_roi(roi, frame.shape)
        ox, oy = (roi[0], roi[1]) if roi else (0, 0)
        crop = frame[oy: oy + roi[3], ox: ox + roi[2]] if roi else frame
        tensor = self._preprocess(crop)
        with timed("book.invoke"):
            self.interpreter.set_tensor(self.input_index, tensor)
            self.interpreter.invoke()
        return [(x + ox, y + oy, w, h, s) for x, y, w, h, s in self._postprocess()]

    @timed("book.preprocess")
    def _preprocess(self, image):
        import cv2

        # Letterbox: shrink (or grow, for a small crop) to fit without stretching, pad the
        # bottom/right. Padding only there means there is no offset to undo afterwards.
        h, w = image.shape[:2]
        self.crop_w, self.crop_h = w, h
        self.scale = min(self.input_w / w, self.input_h / h)
        size = (max(1, round(w * self.scale)), max(1, round(h * self.scale)))
        if size != (w, h):
            image = cv2.resize(image, size, interpolation=cv2.INTER_AREA if self.scale < 1 else cv2.INTER_LINEAR)
        if self.rgb:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        canvas = np.full((self.input_h, self.input_w, 3), self.pad_value, np.uint8)
        canvas[: image.shape[0], : image.shape[1]] = image
        if self.lut is not None:
            canvas = self.lut[canvas]
        return canvas[np.newaxis]

    def _output(self, name):
        d = self.outputs[name]
        q = self.interpreter.get_tensor(d["index"])[0]
        scale, zero_point = d["quantization"]
        if scale and name != "class_idx":     # class_idx is a plain integer, not quantized
            return (q.astype(np.float32) - zero_point) * scale
        return q

    @timed("book.postprocess")
    def _postprocess(self):
        boxes = self._output("boxes")
        scores = self._output("scores")[: len(boxes)]   # our exports append a constant (range pin)
        keep = scores >= self.score_threshold
        if "class_idx" in self.outputs:
            keep &= self._output("class_idx") == self.book_class
        if not keep.any():
            return []
        boxes, scores = boxes[keep] / self.scale, scores[keep]   # model px -> crop px
        boxes = np.clip(boxes, 0, [self.crop_w, self.crop_h, self.crop_w, self.crop_h])  # not into the padding
        out = []
        for i in nms(boxes, scores, self.nms_threshold):
            x1, y1, x2, y2 = boxes[i]
            out.append((int(x1), int(y1), int(x2 - x1), int(y2 - y1), float(scores[i])))
        return out
