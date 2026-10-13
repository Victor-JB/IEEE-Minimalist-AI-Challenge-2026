"""Every tunable behavior number from docs/STATE_MACHINE.md in one place. Tune against the power meter.

Arm parameters (home, caps, safe ranges, stream rate) are NOT defined here: they come from the
shared arm/python/arm_config.py, so Lumen and the manual tools always agree.
"""
import os as _os

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

# --- Lamp light (io/light.py) -------------------------------------------------
# Levels are perceptual (0-1, what the HUD shows); the backend gets duty = level ** LIGHT_GAMMA.
LIGHT_RAMP_S = 0.5                       # default ramp (ILLUMINATE on, GOODBYE off, FAULT dim)
LIGHT_STEP_RAMP_S = 0.3                  # ADJUST brightness steps
LIGHT_GAMMA = 2.2
LIGHT_CHECKIN_SCALE = 0.7                # LightCoupling: dim to 70 % while glancing at the user
LIGHT_COUPLING_RAMP_S = 0.3
# Backend, chosen with env LUMEN_LIGHT: "sim" (default: logs only), "firmware" (LIGHT RPC on the
# STM32 through the motion worker's arm session), "linux-pwm" (sysfs PWM), "gpio" (gpiod on/off).
LIGHT_BACKEND = _os.environ.get("LUMEN_LIGHT", "sim")
LIGHT_FIRMWARE_EVERY_S = 0.08            # firmware backend: at most one LIGHT per this, faded over it
LIGHT_PWM_CHIP = int(_os.environ.get("LUMEN_LIGHT_PWMCHIP", "0"))    # /sys/class/pwm/pwmchipN
LIGHT_PWM_CHANNEL = int(_os.environ.get("LUMEN_LIGHT_PWM_CHANNEL", "0"))
LIGHT_PWM_HZ = int(_os.environ.get("LUMEN_LIGHT_PWM_HZ", "20000"))
LIGHT_GPIO_CHIP = _os.environ.get("LUMEN_LIGHT_GPIOCHIP", "/dev/gpiochip0")
LIGHT_GPIO_LINE = int(_os.environ.get("LUMEN_LIGHT_GPIO_LINE", "0"))
