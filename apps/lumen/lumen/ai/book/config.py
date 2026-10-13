"""BOOK model: YOLOX-S w8a8 (COCO "book" class) and when its detections become events. Edit here.

Why this model, and the benchmark of the alternatives: docs/MODEL_SELECTION.md.
"""
import os

from ...paths import MODELS

# --- The model (ai/book/detector.py) -----------------------------------------------------------
# Shipped model: Qualcomm AI Hub Yolo-X (YOLOX-S) w8a8, 640x640 input, Apache-2.0, 9.4 MB.
# A model folder's metadata.json may override BOOK_CLASS / CHANNELS / INPUT_RANGE / PAD_VALUE in
# its "lumen" block (the bench candidates do); the input size is read from the model itself.
MODEL_DIR = MODELS / "book_det-tflite-w8a8"
INPUT_SIZE = 640          # informational: the .tflite has a fixed input; re-export to change it
SCORE_THRESHOLD = 0.3     # COCO stand-in set: recall 0.45, 0.20 FP/frame, 0.7% of book-free images
                          # fire (at 0.5: recall 0.25, 0.03 FP/frame). BOOK_FOUND also needs 2 hits.
NMS_THRESHOLD = 0.45      # IoU above which the weaker of two book boxes is dropped
BOOK_CLASS = 73           # "book" in the contiguous 80-class COCO list (COCO category id 84)
CHANNELS = "bgr"          # what the model wants; frames arrive BGR from OpenCV
INPUT_RANGE = (0.0, 1.0)  # real input value range that pixels 0-255 map to
PAD_VALUE = 114           # letterbox fill (YOLOX's own gray)
MIN_ROI_PX = 8            # a smaller book_roi (after clamping to the frame) means "search the full frame"
NUM_THREADS = os.cpu_count()


# --- When detections become events (ai/__init__.py) ----------------------------------------------
BOOK_FOUND_CONSECUTIVE = 2               # BOOK_FOUND after this many hits in a row
BOOK_LOST_S = 2.0                        # BOOK_LOST needs the book gone this long...
BOOK_LOST_MISSES = 3                     # ...and this many misses in a row
BOOK_CONFIRM_HZ = 3.0                    # after a missed check, re-check this fast
BOOK_PER_POSE = 2                        # SEARCH: book inferences per settled sweep pause
