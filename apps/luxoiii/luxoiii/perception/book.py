"""Book detection.

TODO(victor): a quantized YOLO-family detector from Qualcomm AI Hub (COCO class 73 = "book"),
int8 TFLite in apps/luxoiii/models/book_det-tflite-w8a8/. Only runs in SEARCH (at each sweep pause),
AIM, ILLUMINATE/HOLD (0.5 Hz check) and REACQUIRE. Use a crop along the pointing ray when
you have one. Benchmark it like the face model before wiring it in.
"""


class BookDetector:
    available = False

    def __init__(self):
        pass

    def detect(self, frame, roi=None):
        """List of (x, y, w, h, score) in frame pixels. [] until implemented."""
        return []
