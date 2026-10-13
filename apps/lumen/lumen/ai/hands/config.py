"""HANDS model: MediaPipe palm detector + hand landmarks (w8a8 TFLite), and the gestures read
from the 21 landmarks. Edit here."""
from ...paths import MODELS

MODEL_DIR = MODELS / "mediapipe_hand_gesture-tflite-w8a8"
PALM_THRESHOLD = 0.5                     # palm detector score to accept a hand
LANDMARK_THRESHOLD = 0.5                 # hand-presence score to keep the landmarks
TRACKING = True                          # next crop from the last landmarks (skips the palm model)
MAX_HANDS = 1

# --- Gesture rules (ai/hands/gestures.py) -------------------------------------------------------
FINGER_EXTENDED_RATIO = 1.1              # finger out: tip this much farther from the wrist than its middle joint
THUMB_OUT_RATIO = 1.2                    # thumb out: tip this much farther from the index knuckle than its joint
THUMB_UPDOWN = 0.5                       # thumbs up/down: tip this many hand sizes above/below the knuckle

# --- When a held gesture becomes an event (ai/__init__.py) --------------------------------------
GESTURE_HOLD_S = 0.4                     # POINT / CLOSER / FURTHER must be held this long
GESTURE_REPEAT_S = 1.0                   # CLOSER / FURTHER repeat while held
PALM_HOLD_S = 1.0                        # open palm = STOP / EXIT
RESET_HOLD_S = 3.0                       # open palm this long in FAULT clears a latched fault
WAVE_REVERSALS, WAVE_WINDOW_S = 2, 1.2   # wave = this many wrist direction reversals within this long
WAVE_SWING = 0.5                         # ...each swing at least this many hand sizes
