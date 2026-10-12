"""Every tunable number from docs/STATE_MACHINE.md in one place. Tune against the power meter."""

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

# --- Arm (shoulder, elbow, head); mirrors apps/stepper_desk/CLOSED_LOOP.md ----
HOME = (-12.04, 63.46, 5.27)
SPEED_CAPS = (30.0, 25.0, 22.0)          # deg/s, emote.py defaults after the evening slowdown
ACCEL_CAP = 300.0
STREAM_HZ = 25
