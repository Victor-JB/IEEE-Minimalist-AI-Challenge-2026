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
BOOK_CONFIRM_HZ = 3.0                    # after a missed book check, re-check this fast...
BOOK_LOST_MISSES = 3                     # ...and only call it lost after this many misses in a row
RESET_HOLD_S = 3.0                       # open palm this long in FAULT clears a latched fault
MOTION_THRESHOLD = 6.0                   # mean abs diff (0-255) on the 80x60 gray frame
NO_MOTION_SLEEP_S = 300.0                # IDLE -> SLEEP
NOBODY_S = 180.0                         # ILLUMINATE -> GOODBYE: no face, hand or motion

# --- Camera and desk geometry (behavior/aim.py; the arm's own geometry is arm/python/kinematics.py)
# Arducam USB camera, 640x480. Its field of view is not in the repo or the CAD: 60 deg horizontal
# is an assumption (4:3 -> ~47 deg vertical, consistent with tests/test_look_at.py's 22 deg
# half-height). MEASURE: tape a ruler on the desk at a known distance and count pixels.
CAMERA_HFOV_DEG = 60.0
BOOK_TOP_Z = 0.02                        # m: aim at the top of a ~2 cm book, not the desk under it
# Desk area to search and light, m in the arm frame (x forward of the shoulder axis, y = lamp's
# left). Where the arm can light at all is x ~0.30-0.60 (kinematics.aim inside SAFE_RANGES).
DESK_AREA = ((0.25, 0.60), (-0.25, 0.25))

# --- SEARCH sweep and ADJUST (behavior/aim.py) ------------------------------
SWEEP_PAUSES = 4                         # 3-5 pause poses, near to far
# J5 per pause, cycled: a +-20 deg left/right zig-zag. Pitch alone sees ~65% of DESK_AREA (the
# frame is only ~0.3-0.8 m wide down there); with the zig-zag 4 pauses see ~97% (tests/test_aim.py).
SWEEP_ROLLS = (-20.0, 20.0)
SWEEP_Z = 0.36                           # m: search from high up (bigger footprint)...
SWEEP_STANDOFF = 0.40                    # ...and from far back
SWEEP_FRAME_INSET = 0.1                  # coverage counts only this far inside the frame edges
SETTLE_TOL_DEG = 1.5                     # a pause counts as settled when every joint is this close...
SETTLE_MIN_S = 0.25                      # ...and at least this long after the setpoint arrived
SETTLE_MAX_S = 1.0                       # (or this long: backlash may keep it from closing in)
ADJUST_STEP_M = 0.03                     # CLOSER / FURTHER: light moves this far along its axis
ANTICIPATION = 0.25                      # counter-move first, this fraction of the step...
ANTICIPATION_S = 0.25                    # ...over this long

# --- State timers -----------------------------------------------------------
BOOT_TIMEOUT_S = 20.0                    # not homed by then -> FAULT (connect retries + a 3 s home move)
FAULT_COOLDOWN_S = 10.0                  # FAULT -> BOOT after this long (one auto retry)
FAULT_FORGET_S = 120.0                   # a fault this long after the last one counts as the first again
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
HOLD_DRIFT_DEG = 1.5                     # spread of the random drift targets on holds
HOLD_DRIFT_EVERY_S = (2.0, 5.0)          # a new target this often, per joint
HOLD_DRIFT_SMOOTH_S = 0.6                # glide time constant (two stages: smooth, no overshoot)
# Per joint. OFF for now: on the real arm, small slow hold corrections trip the firmware's stall
# detector (60 s hold at HOME: 0 stalls with no drift, a FAULT every ~30 s with drift; gear backlash
# ~2-2.5 deg). Turn back on once the firmware compensates backlash (servo.hpp slackDeg).
HOLD_DRIFT_GAIN = (0.0, 0.0, 0.0)
ENERGY_START, ENERGY_REST = 0.3, 0.3
ENERGY_BUMP = 0.15                       # per interaction
ENERGY_DECAY_S = 180.0                   # time constant back to rest
