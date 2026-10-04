"""Face detection with Qualcomm's Lightweight-Face-Detection (face_det_lite, w8a8 TFLite).

Model I/O (see face_det_lite-tflite-w8a8/metadata.json):
    input     [1, 480, 640, 1]  uint8 grayscale, 0-255
    heatmap   [1, 60, 80, 1]    face-center logits, one per 8x8 pixel cell
    bbox      [1, 60, 80, 4]    distances (in cells) from the cell to the box's left, top, right, bottom
    landmark  [1, 60, 80, 10]   5 facial points per cell (decoding is still a TODO)
All outputs are uint8-quantized: real = (q - zero_point) * scale.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from ai_edge_litert.interpreter import Interpreter

MODEL_PATH = Path(__file__).parent / "face_det_lite-tflite-w8a8" / "face_det_lite.tflite"
INPUT_W, INPUT_H = 640, 480
STRIDE = 8  # input pixels per output cell (640 / 80)


@dataclass
class Detection:
    x: int  # top-left corner, in frame pixels
    y: int
    w: int
    h: int
    score: float
    landmarks: np.ndarray | None = None  # TODO: (5, 2) points once decoding is verified

    @property
    def center(self):
        return self.x + self.w / 2, self.y + self.h / 2


class FaceDetector:
    def __init__(self, model_path=MODEL_PATH, score_threshold=0.5, nms_threshold=0.4):
        self.score_threshold = score_threshold
        self.nms_threshold = nms_threshold

        # Load once; this is the slow part.
        self.interpreter = Interpreter(model_path=str(model_path))
        self.interpreter.allocate_tensors()
        self.input_index = self.interpreter.get_input_details()[0]["index"]
        self.outputs = {d["name"]: d for d in self.interpreter.get_output_details()}

    def detect(self, frame):
        """Run the full pipeline on a BGR frame and return a list of Detection."""
        tensor = self._preprocess(frame)
        self.interpreter.set_tensor(self.input_index, tensor)
        self.interpreter.invoke()
        return self._postprocess(frame.shape[1], frame.shape[0])

    def _preprocess(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if gray.shape != (INPUT_H, INPUT_W):
            gray = cv2.resize(gray, (INPUT_W, INPUT_H))
        # The model's input quantization (scale 1/255, zero point 0) means raw 0-255 pixels go in as-is.
        return gray[np.newaxis, :, :, np.newaxis]

    def _output(self, name):
        d = self.outputs[name]
        scale, zero_point = d["quantization"]
        q = self.interpreter.get_tensor(d["index"])[0]
        return (q.astype(np.float32) - zero_point) * scale

    def _postprocess(self, frame_w, frame_h):
        heatmap = 1 / (1 + np.exp(-self._output("heatmap")[..., 0]))  # logits -> probability
        bbox = self._output("bbox")

        # A face center is a cell that beats the threshold and is the max of its 3x3 neighbourhood.
        local_max = heatmap == cv2.dilate(heatmap, np.ones((3, 3), np.uint8))
        ys, xs = np.nonzero(local_max & (heatmap > self.score_threshold))
        if len(xs) == 0:
            return []

        sx, sy = frame_w / INPUT_W, frame_h / INPUT_H
        boxes, scores = [], []
        for cx, cy in zip(xs, ys):
            left, top, right, bottom = bbox[cy, cx]
            x1 = (cx - left) * STRIDE * sx
            y1 = (cy - top) * STRIDE * sy
            x2 = (cx + right) * STRIDE * sx
            y2 = (cy + bottom) * STRIDE * sy
            boxes.append([int(x1), int(y1), int(x2 - x1), int(y2 - y1)])
            scores.append(float(heatmap[cy, cx]))

        keep = cv2.dnn.NMSBoxes(boxes, scores, self.score_threshold, self.nms_threshold)
        return [Detection(*boxes[i], scores[i]) for i in np.array(keep).flatten()]
