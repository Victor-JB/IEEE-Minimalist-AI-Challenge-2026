"""Hand landmarks: 21 (x, y, z) points per hand.

TODO(victor): MediaPipe-Hand-Detection from Qualcomm AI Hub (palm detector + landmark model,
int8 TFLite). Put the .tflite files in inference/hand_det-tflite-w8a8/ next to the face model,
benchmark with perf/bench_detector.py, then fill in detect().
Run it on a crop (roi) around/below the face in STANDBY and over the book in ILLUMINATE.
"""


class HandLandmarker:
    available = False   # flip to True once the models load

    def __init__(self):
        pass

    def detect(self, frame, roi=None):
        """List of hands; each is a list of 21 (x, y, z) in frame pixels. [] until implemented."""
        return []
