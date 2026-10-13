"""MOVEMENT: every number that decides how the arm moves. Edit here.

The arm's hardware limits (HOME pose, speed/accel caps, joint safe ranges, stream rate) are NOT
set here: they live in arm/python/arm_config.py, shared with the firmware tools and the desk
page, and are re-exported below so movement code reads everything from this one module.
Animation pools (which clips each state plays) are in movement/pools.py.
"""
from .. import paths  # noqa: F401  (puts arm/python on sys.path)
from arm_config import (ACCEL_CAP, ELBOW_LIFT_CAP, FIRMWARE_MAX_ACCEL,  # noqa: F401  re-exported
                        FIRMWARE_MAX_SPEED, HOME, SAFE_RANGES, SPEED_CAPS, STREAM_HZ)

# --- Stall handling and the arm session (movement/worker.py) ------------------------------------
STALL_PAUSE_S = 0.25                      # hold powered this long after a stall before pushing again
STALL_RETRIES = 3                         # slower pushes per setpoint before abandoning the command
RETRY_SPEED = (10.0, 6.0, 4.0)            # deg/s of each successive push
STALL_SLOWDOWN = 1.5                      # the rest of a clip plays this much slower after a stall
GIVE_UP_LIMIT, GIVE_UP_WINDOW_S = 6, 60.0 # abandoned commands this close together -> FAULT (jammed)
KEEPALIVE_S = 0.5                         # relaxed: STATUS this often (the MCU drops a silent owner after 2 s)
EASE_SPEED = 20.0                         # deg/s when easing onto a clip's first pose
OFFSET_SLEW = 25.0                        # deg/s: look-at offset changes are smoothed to this
ROLL_RETURN_SPEED = 60.0                  # deg/s the head tilt eases back to ROLL_HOME after a clip
ROLL_DEADBAND = 0.3                       # deg: smaller roll changes aren't sent (saves an RPC per tick)
SWEEP_MIN_MOVE_S = 1.0                    # sweep moves take at least this long (longer if the caps need it)

# --- Face centering (movement/look_at.py) ------------------------------------------------------
RATE_DEG_S = 15.0                        # head speed for a face at the top/bottom edge of the frame
DEADBAND = 0.08                          # normalized error treated as centered, so the lamp doesn't twitch
MAX_DEG = 25.0                           # most the head may turn away from the pose it is tracking around
CENTER_HOLD_S = 0.4                      # centered this long -> `centered` is True
CENTER_TIMEOUT_S = 4.0                   # NOTICE/STANDBY start their clips anyway after this long

# --- Camera and desk geometry (movement/aim.py; the arm's own geometry is arm/python/kinematics.py)
# Arducam USB camera, 640x480. Its field of view is not in the repo or the CAD: 60 deg horizontal
# is an assumption (4:3 -> ~47 deg vertical, consistent with tests/test_look_at.py's 22 deg
# half-height). MEASURE: tape a ruler on the desk at a known distance and count pixels.
CAMERA_HFOV_DEG = 60.0
CAMERA_HALF_VFOV_DEG = 22.0              # CHECK-IN: a face at the top/bottom edge is this far off-axis; MEASURE
BOOK_TOP_Z = 0.02                        # m: aim at the top of a ~2 cm book, not the desk under it
# Desk area to search and light, m in the arm frame (x forward of the shoulder axis, y = lamp's
# left). Where the arm can light at all is x ~0.30-0.60 (kinematics.aim inside SAFE_RANGES).
DESK_AREA = ((0.25, 0.60), (-0.25, 0.25))

# --- SEARCH sweep and ADJUST (movement/aim.py) ----------------------------------------------------
SWEEP_PAUSES = 4                         # 3-5 pause poses, near to far
# J5 per pause, cycled: a +-20 deg left/right zig-zag. Pitch alone sees ~65% of DESK_AREA (the
# frame is only ~0.3-0.8 m wide down there); with the zig-zag 4 pauses see ~97% (tests/test_aim.py).
SWEEP_ROLLS = (-20.0, 20.0)
SWEEP_Z = 0.36                           # m: search from high up (bigger footprint)...
SWEEP_STANDOFF = 0.40                    # ...and from far back
SWEEP_FRAME_INSET = 0.1                  # coverage counts only this far inside the frame edges
SWEEP_DWELL_S = 0.6                      # hold still this long at each pause (the book model runs BOOK_PER_POSE times)
SETTLE_TOL_DEG = 1.5                     # a pause counts as settled when every joint is this close...
SETTLE_MIN_S = 0.25                      # ...and at least this long after the setpoint arrived
SETTLE_MAX_S = 1.0                       # (or this long: backlash may keep it from closing in)
ADJUST_STEP_M = 0.03                     # CLOSER / FURTHER: light moves this far along its axis
ANTICIPATION = 0.25                      # counter-move first, this fraction of the step...
ANTICIPATION_S = 0.25                    # ...over this long

# --- Per-state moves (state/director.py) ----------------------------------------------------------
BOOT_HOME_S = 3.0                        # BOOT: the move to HOME takes this long
GAP_S = {"bored": (2.0, 6.0), "attentive": (1.5, 4.0), "holding": (4.0, 9.0),
         "looking_around": (0.5, 1.5), "asleep": (0.5, 2.0)}                 # pause between looping clips
SLEEP_ENTRY = "go_to_sleep/drowsy"       # once on entering SLEEP; the lamp stays slumped after it
SLEEP_BREATHE = (0.3, 2.0)               # (amplitude, time_scale) of the slow breathe while asleep
SLEEP_BREATHE_S = 16.0                   # breathing per loop (whole 8 s breaths)
WAKE_CLIP = "wake_up/slow_groggy"        # NOTICE right after SLEEP -> IDLE, instead of the notice pool
WAKE_AMPLITUDE = 0.7                     # ~18 s on the real arm (the elbow-lift cap slows it ~4x)
WAKE_WINDOW_S = 5.0                      # NOTICE this soon after waking counts as "from SLEEP"; also how
                                         # long IDLE stays slumped before waking up with no one there
FALSE_ALARM_CLIP = "hesitate/subtle"     # NOTICE -> IDLE on TIMEOUT: sheepish "oh, nobody"
# CHECK-IN (LookBackAtUser): pitch the head up toward where the face was last seen, track it a
# moment, then glide back to the aim pose. Camera pitch is taken as shoulder + elbow + head (all
# pitch joints about parallel axes, motion_lib's convention); TODO check the signs on the arm.
# A CHECK-IN up to CHECKIN_MAX_UP and back takes ~11 s inside the speed caps: lower it to shorten.
CHECKIN_LOOK_S = 1.5                     # face at 10 Hz this long while looking up
CHECKIN_DEFAULT_UP = -20.0               # head change when no face was ever seen (- = look up)
CHECKIN_MAX_UP = 45.0                    # never pitch the head more than this away from the aim pose
# REACQUIRE: a small local sweep around the aim pose, widening, while the book model runs at 3 Hz
# on a crop that perception widens to the full frame after a miss. (shoulder, elbow, head) deg.
REACQUIRE_PEEKS = ((0.0, 0.0, -3.0), (1.5, 0.0, 4.0), (-2.0, 0.0, -6.0))
REACQUIRE_DWELL_S = 0.2
MIRROR_S = 3.0                           # after a wave in STANDBY, the head follows the hand this long

# --- Modifier rules (movement/modifiers.py) -------------------------------------------------------
BREATHE = "idle_breathe/calm"
BREATHE_S = 4.0                          # the clip's nominal length
BREATHE_TS = (1.0, 2.0)                  # validated range is 0.5-2, but the worker only ever plays slower
SPEED_MARGIN = 0.8                       # plan moves at this fraction of the caps (the firmware needs headroom)
GLANCE_MIN_DEG = 8.0                     # head change that earns a glance first
WINDUP_MIN_DEG = 15.0                    # AIM: move size that earns a windup (dev rule: 30 on the concept robot)
WINDUP_FLOOR_DEG = 2.5                   # smaller counter-moves vanish in the gear backlash (~2-2.5 deg)

# --- Expressive layer: noise and energy (movement/expressive.py) ---------------------------------
AMPLITUDE_RANGE = (0.8, 1.2)
TIME_SCALE_RANGE = (0.9, 1.15)
NO_REPEAT_LAST = 2
HOLD_DRIFT_DEG = 1.5                     # spread of the random drift targets on holds
HOLD_DRIFT_EVERY_S = (2.0, 5.0)          # a new target this often, per joint
HOLD_DRIFT_SMOOTH_S = 0.6                # glide time constant (two stages: smooth, no overshoot)
# Per joint. OFF for now: on the real arm, small slow hold corrections trip the firmware's stall
# detector (60 s hold at HOME: 0 stalls with no drift, a FAULT every ~30 s with drift; gear backlash
# ~2-2.5 deg). The firmware fix is written but NOT flashed: servo.hpp backlashDeg (slack observer
# + gentle gear engagement after a reversal; sim test holdDriftThroughBacklash in
# arm/tests/servo_test.cpp). Keep this off until it is validated on the arm:
#   1. bash arm/scripts/test.sh, then bash arm/scripts/deploy.sh (flash).
#   2. Run Lumen with drift still off: 60 s hold at HOME should show 0 "Arm stall" lines (baseline).
#   3. Set HOLD_DRIFT_GAIN = (1.0, 1.0, 1.0), repeat the 60 s hold (also with the head near +13 deg):
#      0-1 stalls and no FAULT = keep it; stalls again = widen backlashDeg[2] (head) and retest.
HOLD_DRIFT_GAIN = (0.0, 0.0, 0.0)
ENERGY_START, ENERGY_REST = 0.3, 0.3
ENERGY_BUMP = 0.15                       # per interaction
ENERGY_DECAY_S = 180.0                   # time constant back to rest
