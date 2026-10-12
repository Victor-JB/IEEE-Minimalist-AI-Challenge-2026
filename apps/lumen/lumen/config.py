"""Every tunable behavior number from docs/STATE_MACHINE.md in one place. Tune against the power meter.

Arm parameters (home, caps, safe ranges, stream rate) are NOT defined here: they come from the
shared arm/python/arm_config.py, so Lumen and the manual tools always agree.
"""
from . import paths  # noqa: F401  (puts arm/python on sys.path)
from arm_config import (ACCEL_CAP, ELBOW_LIFT_CAP, FIRMWARE_MAX_ACCEL,  # noqa: F401  re-exported
                        FIRMWARE_MAX_SPEED, HOME, SAFE_RANGES, SPEED_CAPS, STREAM_HZ)

# --- Perception debouncing -------------------------------------------------
FACE_SEEN_K, FACE_SEEN_N = 1, 1          # IDLE -> NOTICE on the first face
FACE_CONFIRM_K, FACE_CONFIRM_N = 3, 5    # NOTICE -> STANDBY
FACE_LOST_S = 4.0                        # STANDBY -> LOST USER
GESTURE_HOLD_S = 0.4                     # POINT / CLOSER / FURTHER must be held this long
GESTURE_REPEAT_S = 1.0                   # CLOSER / FURTHER repeat while held
PALM_HOLD_S = 1.0                        # open palm = STOP / EXIT
BOOK_FOUND_CONSECUTIVE = 2
BOOK_LOST_S = 2.0
MOTION_THRESHOLD = 6.0                   # mean abs diff (0-255) on the 80x60 gray frame
NO_MOTION_SLEEP_S = 300.0                # IDLE -> SLEEP
NOBODY_S = 180.0                         # ILLUMINATE -> GOODBYE: no face, hand or motion

# --- State timers -----------------------------------------------------------
NOTICE_TIMEOUT_S = 1.5                   # false alarm -> IDLE
LOST_TIMEOUT_S = 6.0                     # LOST USER -> IDLE (4 s + 6 s = 10 s total)
CHECKIN_EVERY_S = (8.0, 15.0)            # HOLD -> CHECK-IN, random in this range
REACQUIRE_TIMEOUT_S = 4.0

# --- Target face -------------------------------------------------------------
SWITCH_FACE_RATIO = 1.5                  # another face must be this much bigger...
SWITCH_FACE_HOLD_S = 2.0                 # ...for this long before the lamp switches

# --- Expressive layer --------------------------------------------------------
AMPLITUDE_RANGE = (0.8, 1.2)
TIME_SCALE_RANGE = (0.9, 1.15)
NO_REPEAT_LAST = 2
HOLD_DRIFT_DEG = 1.5                     # OU drift on holds
HOLD_DRIFT_HZ = 0.3
ENERGY_START, ENERGY_REST = 0.3, 0.3
ENERGY_BUMP = 0.15                       # per interaction
ENERGY_DECAY_S = 180.0                   # time constant back to rest
