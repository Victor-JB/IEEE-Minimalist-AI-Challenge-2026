"""Hand landmarks with MediaPipe Hands (Qualcomm AI Hub mediapipe_hand_gesture, w8a8 TFLite).

Model I/O (see apps/luxoiii/models/mediapipe_hand_gesture-tflite-w8a8/metadata.json), all uint8,
real = (q - zero_point) * scale:
    palm_detector           in  [1, 256, 256, 3]  RGB 0-255, the frame (or roi) letterboxed
                            out box_coords [1, 2944, 18]  box x1 y1 x2 y2 + 7 keypoints (x, y), input pixels
                                box_scores [1, 2944]      probability per anchor
    hand_landmark_detector  in  [1, 224, 224, 3]  RGB 0-255, a rotated square crop with the fingers up
                            out landmarks [1, 63]  21 (x, y, z) in crop pixels; scores [1, 1] hand presence;
                                lr [1, 1] right-hand probability; world_landmarks [1, 63] metres
    canned_gesture_classifier  in hand, mirrored_hand [1, 64]; out [1, 8] softmax (CANNED_LABELS)

Qualcomm exports the palm model with its postprocessing baked in (qai_hub_models
mediapipe_hand_gesture: include_postprocessing=True): anchors are already applied and scores
already sigmoided. palm_anchors() + decode_raw() reproduce that step for a raw BlazePalm export
(palm_decoded=False); the tests check both give the same boxes.

Pipeline: palm detector -> NMS -> rotated square crop from the wrist -> middle-finger keypoints
(MediaPipe: shift up half a box, scale 2.6) -> landmark model -> 21 points back in frame pixels,
kept only if the hand-presence score passes. Tracking (on by default): the next frame's crop comes
from this frame's landmarks (MediaPipe: scale 2.0, shift 0.1), so the palm detector only runs
when the hand is new or the presence score drops.
"""
import math
import os
import time
from dataclasses import dataclass

import numpy as np

# Same runtime fallbacks as face_detector.py, but a missing runtime or OpenCV only turns hands
# off (available = False) instead of stopping luxoiii.
try:
    import cv2
except ImportError:
    cv2 = None
try:
    from ai_edge_litert.interpreter import Interpreter
except ImportError:
    try:
        from tflite_runtime.interpreter import Interpreter
    except ImportError:
        try:
            from tensorflow.lite import Interpreter
        except ImportError:
            Interpreter = None

from ..paths import MODELS
from ..perfstats import count, timed  # does nothing unless PERF=1
from .gestures import OPEN_PALM, POINT, THUMBS_DOWN, THUMBS_UP

MODEL_DIR = MODELS / "mediapipe_hand_gesture-tflite-w8a8"
PALM_SIZE, LM_SIZE = 256, 224
NUM_ANCHORS = 2944
# MediaPipe's palm ROI (hand_detection_to_roi) and tracking ROI (hand_landmark_landmarks_to_roi)
PALM_SCALE, PALM_SHIFT = 2.6, 0.5
TRACK_SCALE, TRACK_SHIFT = 2.0, 0.1
TRACK_POINTS = [0, 1, 2, 3, 5, 6, 9, 10, 13, 14, 17, 18]   # landmarks MediaPipe uses for the next crop
CONNECTIONS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10),
               (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17),
               (17, 18), (18, 19), (19, 20)]
CANNED_LABELS = ["None", "Closed_Fist", "Open_Palm", "Pointing_Up", "Thumb_Down", "Thumb_Up", "Victory", "ILoveYou"]
CANNED_TO_RULES = {"Open_Palm": OPEN_PALM, "Pointing_Up": POINT, "Thumb_Up": THUMBS_UP, "Thumb_Down": THUMBS_DOWN}


# --- pure math (tested without cv2 or a runtime) ----------------------------------------------


def palm_anchors(size=PALM_SIZE):
    """MediaPipe SSD anchors for the 256 palm model: (2944, 2) centers in 0-1.

    Layers with strides 8, 16, 32, 32, 32 and 2 anchors each; layers of equal stride share a grid,
    so 32x32 cells x 2 + 16x16 x 2 + 8x8 x 6. fixed_anchor_size: every anchor is 1x1, only the
    centers matter. Order: grid by grid, row-major, the anchors of a cell together.
    """
    out = []
    for stride, per_cell in ((8, 2), (16, 2), (32, 6)):
        g = math.ceil(size / stride)
        ys, xs = np.mgrid[0:g, 0:g]
        centers = np.stack([(xs + 0.5) / g, (ys + 0.5) / g], -1).reshape(-1, 2)
        out.append(np.repeat(centers, per_cell, axis=0))
    return np.concatenate(out).astype(np.float32)


def decode_raw(raw, logits, anchors, size=PALM_SIZE):
    """Raw BlazePalm output -> the decoded form Qualcomm's export emits.

    raw (N, 18): cx cy w h + 7 keypoints, offsets in input pixels from the anchor center.
    Returns (N, 18) x1 y1 x2 y2 + keypoints in input pixels, and sigmoid scores (N,).
    """
    xy = raw.reshape(-1, 9, 2).copy()
    xy[:, [0] + list(range(2, 9))] += anchors[:, None, :] * size     # centers + keypoints; w, h untouched
    c, wh = xy[:, 0], xy[:, 1]
    boxes = np.concatenate([c - wh / 2, c + wh / 2], 1)
    scores = 1 / (1 + np.exp(-np.clip(logits, -100, 100)))
    return np.concatenate([boxes, xy[:, 2:].reshape(-1, 14)], 1), scores


def nms(boxes, scores, iou=0.3, top_k=2):
    """Greedy non-maximum suppression on (N, 4) x1 y1 x2 y2; returns kept indices, best first."""
    order, keep = np.argsort(-scores), []
    area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    while order.size and len(keep) < top_k:
        i, rest = order[0], order[1:]
        keep.append(int(i))
        w = np.clip(np.minimum(boxes[i, 2], boxes[rest, 2]) - np.maximum(boxes[i, 0], boxes[rest, 0]), 0, None)
        h = np.clip(np.minimum(boxes[i, 3], boxes[rest, 3]) - np.maximum(boxes[i, 1], boxes[rest, 1]), 0, None)
        inter = w * h
        order = rest[inter / (area[i] + area[rest] - inter + 1e-9) <= iou]
    return keep


def rotation(p0, p1):
    """Angle (radians) that turns the p0 -> p1 vector straight up; MediaPipe's convention."""
    r = math.pi / 2 - math.atan2(-(p1[1] - p0[1]), p1[0] - p0[0])
    return (r + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Rect:
    """The landmark model's crop: a square of side `size` frame pixels, turned by `rotation`."""
    cx: float
    cy: float
    size: float
    rotation: float   # 0 = fingers up in the frame

    def axes(self):
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        return np.array([c, s]), np.array([-s, c])      # crop x and y (down) axes, in frame pixels

    def affine(self, out=LM_SIZE):
        """2x3 matrix taking crop pixels (0..out) to frame pixels."""
        ex, ey = (a * self.size / out for a in self.axes())
        origin = np.array([self.cx, self.cy]) - (ex + ey) * out / 2
        return np.array([[ex[0], ey[0], origin[0]], [ex[1], ey[1], origin[1]]], np.float32)


def _shifted(center, w, h, r, shift, scale):
    ex, ey = Rect(0, 0, 1, r).axes()
    c = np.asarray(center, np.float64) - shift * h * ey             # move toward the fingers
    return Rect(float(c[0]), float(c[1]), max(w, h) * scale, r)


def rect_from_palm(det):
    """Palm detection (18 values in frame pixels) -> crop around the whole hand."""
    x1, y1, x2, y2 = det[:4]
    kp = det[4:].reshape(7, 2)
    r = rotation(kp[0], kp[2])                                      # wrist -> middle-finger MCP
    return _shifted(((x1 + x2) / 2, (y1 + y2) / 2), x2 - x1, y2 - y1, r, PALM_SHIFT, PALM_SCALE)


def rect_from_landmarks(points):
    """Last frame's 21 landmarks -> next frame's crop (MediaPipe's tracking ROI)."""
    p = np.asarray(points, np.float64)[:, :2]
    r = rotation(p[0], ((p[5] + p[13]) / 2 + p[9]) / 2)
    ex, ey = Rect(0, 0, 1, r).axes()
    u, v = p[TRACK_POINTS] @ ex, p[TRACK_POINTS] @ ey               # box aligned with the hand
    center = (u.min() + u.max()) / 2 * ex + (v.min() + v.max()) / 2 * ey
    return _shifted(center, np.ptp(u), np.ptp(v), r, TRACK_SHIFT, TRACK_SCALE)


def clip_roi(roi, shape):
    """(x, y, w, h) clipped to the frame, or the whole frame when roi is None."""
    H, W = shape[:2]
    if roi is None:
        return 0, 0, W, H
    x, y, w, h = (int(round(v)) for v in roi[:4])
    x0, y0, x1, y1 = max(x, 0), max(y, 0), min(x + w, W), min(y + h, H)
    return x0, y0, max(x1 - x0, 0), max(y1 - y0, 0)


def canned_features(points, right, mirror=False):
    """Qualcomm's preprocess_hand_x64: center on the palm, divide by the x/y extent, + handedness."""
    p = np.asarray(points, np.float32).reshape(21, 3).copy()
    if mirror:
        p[:, 0] *= -1
        right = 1 - right
    p -= p[[0, 1, 5, 9, 13, 17]].mean(0)
    scale = max(np.ptp(p[:, 0]), np.ptp(p[:, 1])) + 1e-5
    return np.append((p / scale).ravel(), right).astype(np.float32)


def _deq(interp, d):
    q = interp.get_tensor(d["index"])[0]
    scale, zero_point = d["quantization"]
    return (q.astype(np.float32) - zero_point) * scale if scale else q.astype(np.float32)


def _quant(x, d):
    scale, zero_point = d["quantization"]
    if not scale:
        return x.astype(d["dtype"])
    return np.clip(np.round(x / scale + zero_point), 0, 255).astype(d["dtype"])


def _load(path, num_threads):
    if Interpreter is None or cv2 is None:
        raise ImportError("needs opencv and a TFLite runtime (ai-edge-litert)")
    interp = Interpreter(model_path=str(path), num_threads=num_threads)
    interp.allocate_tensors()
    return interp


def _rgb(img):
    return np.repeat(img[..., None], 3, 2) if img.ndim == 2 else img[..., ::-1]   # gray / BGR -> RGB


# --- the detector ---------------------------------------------------------------------------------


@dataclass
class Hand:
    points: list            # 21 (x, y, z) in frame pixels; z is relative depth, same scale as x
    score: float            # hand presence
    right: float            # probability it is a right hand
    world: np.ndarray       # (21, 3) metres, origin near the hand's center
    rect: Rect              # the crop it came from
    tracked: bool = False   # crop came from the last frame's landmarks, not the palm detector


class HandLandmarker:
    available = False   # True once both models load

    def __init__(
        self,
        tracking=True,              # False: palm detector on every call (for benchmarking)
        max_hands=1,
        palm_threshold=0.5,
        landmark_threshold=0.5,
        palm_decoded=True,          # Qualcomm's export already applies the anchors
        model_dir=MODEL_DIR,
        num_threads=os.cpu_count(),
        interpreters=None,          # (palm, landmark) to inject, e.g. fakes in tests
        discovery_interval=1,       # >1 is experimental: can reduce temporal accuracy
        track_timeout=0.5,          # reacquire after a gap rather than reuse an old crop
    ):
        self.tracking, self.max_hands, self.palm_decoded = tracking, max_hands, palm_decoded
        self.palm_threshold, self.landmark_threshold = palm_threshold, landmark_threshold
        self.last, self._tracks, self.anchors = [], [], palm_anchors()
        if discovery_interval < 1 or track_timeout <= 0:
            raise ValueError('Positive discovery interval and track timeout required')
        self.discovery_interval, self.track_timeout = discovery_interval, track_timeout
        self._since_palm, self._last_at = 0, None
        self.palm_calls = self.landmark_calls = 0
        try:
            self.palm, self.lm = interpreters or (
                _load(model_dir / "palm_detector.tflite", num_threads),
                _load(model_dir / "hand_landmark_detector.tflite", num_threads))
        except Exception as e:      # missing runtime, model or a bad file: run without hands
            print(f"HandLandmarker off: {e}")
            return
        self.palm_in = self.palm.get_input_details()[0]
        self.palm_out = {d["name"]: d for d in self.palm.get_output_details()}
        self.lm_in = self.lm.get_input_details()[0]
        self.lm_out = {d["name"]: d for d in self.lm.get_output_details()}
        self.available = True

    def reset(self):
        """Forget the tracked hands; the next call runs the palm detector."""
        self._tracks = []
        self._since_palm, self._last_at = 0, None

    @timed("hands.total")
    def detect(self, frame, roi=None, *, now=None):
        """List of hands; each is a list of 21 (x, y, z) in frame pixels. Details in self.last.

        roi (x, y, w, h) limits where the palm detector looks. A tracked hand is followed from
        its own crop, so it can drift a little outside roi without being dropped.
        """
        if not self.available:
            return []
        now = time.monotonic() if now is None else now
        if self._last_at is not None and now - self._last_at > self.track_timeout:
            self.reset()
        self._last_at = now
        # A single visible hand can use its ROI even in two-hand mode. Bound the
        # second-hand discovery delay; an empty/lost track still reacquires now.
        missing_slot = len(self._tracks) < self.max_hands
        discovery_due = missing_slot and self._since_palm >= self.discovery_interval - 1
        tracked = self.tracking and bool(self._tracks) and not discovery_due
        rects = self._tracks if tracked else self._palms(frame, roi)
        self._since_palm = self._since_palm + 1 if tracked else 0
        hands = [h for h in (self._landmarks(frame, r, tracked) for r in rects) if h]
        if tracked and len(hands) < len(rects):     # lost one: look for palms again on this frame
            count("hands.track_lost")
            hands = [h for h in (self._landmarks(frame, r) for r in self._palms(frame, roi)) if h]
            self._since_palm = 0
        self.last = hands
        self._tracks = [rect_from_landmarks(h.points) for h in hands] if self.tracking else []
        return [h.points for h in hands]

    @timed("hands.palm")
    def _palms(self, frame, roi):
        """Palm detector on the roi -> crops for the landmark model, in frame pixels."""
        x, y, w, h = clip_roi(roi, frame.shape)
        if min(w, h) < 16:
            return []
        scale = PALM_SIZE / max(w, h)
        img = self._resize(frame[y:y + h, x:x + w], (max(1, round(w * scale)), max(1, round(h * scale))))
        canvas = np.zeros((PALM_SIZE, PALM_SIZE, 3), np.uint8)       # letterbox, pad bottom/right
        canvas[:img.shape[0], :img.shape[1]] = _rgb(img)
        self.palm.set_tensor(self.palm_in["index"], canvas[np.newaxis])
        self.palm_calls += 1
        self.palm.invoke()
        return [rect_from_palm(d) for d in self.decode_palms(scale, (x, y))]

    def decode_palms(self, scale, offset):
        """Read the palm outputs -> (K, 18) detections in frame pixels, best first, after NMS."""
        coords, scores = _deq(self.palm, self.palm_out["box_coords"]), _deq(self.palm, self.palm_out["box_scores"])
        if not self.palm_decoded:
            coords, scores = decode_raw(coords, scores, self.anchors)
        good = (scores >= self.palm_threshold) & (coords[:, 2] > coords[:, 0]) & (coords[:, 3] > coords[:, 1])
        coords, scores = coords[good], scores[good]
        keep = nms(coords[:, :4], scores, top_k=self.max_hands)
        # Padding is only on the bottom/right, so model pixels -> roi pixels is just / scale.
        return coords[keep] / scale + np.tile(np.asarray(offset, np.float32), 9)

    @timed("hands.landmark")
    def _landmarks(self, frame, rect, tracked=False):
        """Landmark model on one rotated crop -> Hand in frame pixels, or None if no hand."""
        A = rect.affine()
        crop = _rgb(self._warp(frame, A, LM_SIZE))
        self.lm.set_tensor(self.lm_in["index"], crop[np.newaxis])
        self.landmark_calls += 1
        self.lm.invoke()
        score = float(_deq(self.lm, self.lm_out["scores"]).ravel()[0])
        if score < self.landmark_threshold:
            return None
        lm = _deq(self.lm, self.lm_out["landmarks"]).reshape(21, 3)
        xy = lm[:, :2] @ A[:, :2].T + A[:, 2]
        z = lm[:, 2] * rect.size / LM_SIZE
        points = [(float(px), float(py), float(pz)) for (px, py), pz in zip(xy, z)]
        return Hand(points, score, float(_deq(self.lm, self.lm_out["lr"]).ravel()[0]),
                    _deq(self.lm, self.lm_out["world_landmarks"]).reshape(21, 3), rect, tracked)

    # Image ops are methods so tests can swap in numpy versions and run without OpenCV.
    @staticmethod
    def _resize(img, size):
        return cv2.resize(img, size, interpolation=cv2.INTER_AREA)

    @staticmethod
    def _warp(frame, A, size):
        """Crop pixels <- frame pixels through A (crop -> frame), black outside the frame."""
        return cv2.warpAffine(frame, A, (size, size), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                              borderMode=cv2.BORDER_CONSTANT)


class CannedGestures:
    """MediaPipe's canned gesture classifier on one hand's 21 landmarks (+ handedness).

    Its classes (CANNED_LABELS) are not the rules' set: Pointing_Up needs the finger up, and there
    are extra ones (Closed_Fist, Victory, ILoveYou). CANNED_TO_RULES maps the overlap.
    """

    def __init__(self, model_path=MODEL_DIR / "canned_gesture_classifier.tflite",
                 num_threads=1, interpreter=None):
        self.interp = interpreter or _load(model_path, num_threads)
        self.inputs = {d["name"]: d for d in self.interp.get_input_details()}
        self.output = self.interp.get_output_details()[0]

    def probs(self, points, right=0.0):
        """(8,) class probabilities for one hand (frame-pixel landmarks, right-hand probability)."""
        right = float(round(right))                     # the model was fed 0 / 1
        for name, mirror in (("hand", False), ("mirrored_hand", True)):
            d = self.inputs[name]
            self.interp.set_tensor(d["index"], _quant(canned_features(points, right, mirror), d)[np.newaxis])
        self.interp.invoke()
        return _deq(self.interp, self.output)

    def classify(self, points, right=0.0):
        """-> (label from CANNED_LABELS, probability)."""
        p = self.probs(points, right)
        i = int(np.argmax(p))
        return CANNED_LABELS[i], float(p[i])
