"""One source of truth for the arm's motion parameters.

Lumen (apps/lumen) and every manual / test tool (emote.py, armctl.py, ik_line.py on the dev branch)
import these, so the app and the bench can never disagree about the arm.
Values measured on the real arm on 2026-10-11 (CLOSED_LOOP.md on the dev branch).
"""
from arm_client import JOINTS, LIMITS, MOTOR   # noqa: F401  protocol-level limits the client validates
from arm_motion import MARGIN                  # noqa: F401  firmware keeps targets this far inside LIMITS

# Joint order everywhere: shoulder, elbow, head (degrees). HOME and SAFE_RANGES are lists so
# lock_shoulder() can update them in place for every module that imported them.
HOME = [-12.04, 63.46, 5.27]

# Where streamed motion (clips, look-at, holds) is clamped. The shoulder stays in -30..+10:
# below about -36 it can't lift itself back, and leaning forward past ~+21 it stalled.
# The firmware's own hard floor for the shoulder is -34 (sketch/limits.hpp).
SAFE_RANGES = [(-30.0, 10.0), (-70.0, 77.0), (-78.0, 85.0)]
SHOULDER_FREE_RANGE = SAFE_RANGES[0]

# --- Locked shoulder (2026-10-13) -----------------------------------------------------------------
# The shoulder motor is gone; the joint is locked mechanically and its encoder still reads. While
# SHOULDER_LOCK is set, the shoulder is never moved: SAFE_RANGES[0] is (lock, lock), so every clamp
# (clips, holds, look-at, desk jog) pins it; clips fold their shoulder motion into the elbow
# (trajectory.py); kinematics aims with elbow + head only. Lumen and the desk re-lock it at the
# MEASURED angle on connect (a setpoint off the real angle would make the firmware push the dead
# motor and FAULT), so this value only matters until then. None = a working shoulder motor.
SHOULDER_LOCK = HOME[0]


def lock_shoulder(angle):
    """Hold the shoulder at angle (deg) from now on, everywhere."""
    global SHOULDER_LOCK
    angle = float(angle)
    SHOULDER_LOCK, HOME[0], SAFE_RANGES[0] = angle, angle, (angle, angle)


def unlock_shoulder():
    """Back to a driven shoulder (tests, or once the motor is replaced)."""
    global SHOULDER_LOCK
    SHOULDER_LOCK, SAFE_RANGES[0] = None, SHOULDER_FREE_RANGE


if SHOULDER_LOCK is not None:
    lock_shoulder(SHOULDER_LOCK)
SHOULDER_STOP = -36.0          # abort streaming if the measured shoulder drops below this

# Speed and acceleration caps for expressive motion (after the evening slowdown; heat-limited).
SPEED_CAPS = (30.0, 25.0, 22.0)   # deg/s per joint
ELBOW_LIFT_CAP = 14.0             # deg/s while the elbow straightens (lifts forearm + head)
ACCEL_CAP = 300.0                 # deg/s^2
FIRMWARE_MAX_SPEED = 60.0         # what ArmMotion.configure() accepts
FIRMWARE_MAX_ACCEL = 500.0

# Streaming: setpoints at 25 Hz, each aimed this far ahead (Tailscale points arrive 100-170 ms apart).
STREAM_HZ = 25
HORIZON = {"local": 0.12, "usb": 0.12, "tailscale": 0.3}

# Head tilt (J5, "head_roll" in the clips): Hiwonder HPS-2027, signal on D2, own 7.4 V supply.
# Checked on the arm 2026-10-12: 270 deg over 500-2500 us, 1500 us = level shade, and the servo is
# mounted mirrored: a larger pulse tilts the shade to the lamp's left, while the clips' +head_roll is
# the lamp's right (rotation about +x = the way it faces, sim/robot.toml), so ROLL_SIGN = -1.
# Still to check: how far ROLL_RANGE can widen before the shade or its cable hits something.
# ROLL is what ArmMotion(roll=...) takes; ROLL_ENABLED=False leaves the servo alone.
ROLL_ENABLED = True
ROLL_CENTER_US = 1500
ROLL_SIGN = -1                  # see above: +head_roll = the lamp's right
ROLL_RANGE = (-45.0, 45.0)
ROLL_SPEED_CAP = 180.0          # deg/s the firmware slews the command at (the servo does ~460)
ROLL_US_PER_DEG = 2000 / 270     # 270 deg servo over 500-2500 us
ROLL = dict(center_us=ROLL_CENTER_US, us_per_deg=ROLL_US_PER_DEG, limits=ROLL_RANGE, sign=ROLL_SIGN,
            max_speed=ROLL_SPEED_CAP)
ROLL_HOME = 0.0                 # head tilt held between clips (deg)
