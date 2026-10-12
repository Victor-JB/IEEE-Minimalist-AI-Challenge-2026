"""Hand landmarks: 21 (x, y, z) points per hand.

TODO(victor): the models are in apps/lumen/models/mediapipe_hand_gesture-tflite-w8a8/
(palm_detector, hand_landmark_detector, and canned_gesture_classifier, int8). Load them with
paths.MODELS, benchmark like the face model (bench/bench_detector.py --model ...), then fill in
detect(). The canned classifier may replace the rules in gestures.py: compare both.
Run it on a crop (roi) around/below the face in STANDBY and over the book in ILLUMINATE.
"""


class HandLandmarker:
    available = False   # flip to True once the models load

    def __init__(self):
        pass

    def detect(self, frame, roi=None):
        """List of hands; each is a list of 21 (x, y, z) in frame pixels. [] until implemented."""
        return []
