"""FACE model: Qualcomm Lightweight-Face-Detection (face_det_lite, w8a8 TFLite). Edit here."""
from ...paths import MODELS

MODEL_PATH = MODELS / "face_det_lite-tflite-w8a8" / "face_det_lite.tflite"
SCORE_THRESHOLD = 0.5                    # face-center probability needed to count as a face
NMS_THRESHOLD = 0.4                      # overlap above which two boxes are the same face

# --- Which face to follow (ai/face/tracker.py) --------------------------------------------------
SWITCH_FACE_RATIO = 1.5                  # another face must be this much bigger...
SWITCH_FACE_HOLD_S = 2.0                 # ...for this long before the lamp switches
