"""One source of truth for the arm's motion parameters.

Lumen (apps/lumen) and every manual / test tool (emote.py, armctl.py, ik_line.py on the dev branch)
import these, so the app and the bench can never disagree about the arm.
Values measured on the real arm on 2026-10-11 (CLOSED_LOOP.md on the dev branch).
"""
from arm_client import JOINTS, LIMITS, MOTOR   # noqa: F401  protocol-level limits the client validates
from arm_motion import MARGIN                  # noqa: F401  firmware keeps targets this far inside LIMITS

# Joint order everywhere: shoulder, elbow, head (degrees).
HOME = (-12.04, 63.46, 5.27)

# Where streamed motion (clips, look-at, holds) is clamped. The shoulder stays in -30..+10:
# below about -36 it can't lift itself back, and leaning forward past ~+21 it stalled.
# The firmware's own hard floor for the shoulder is -34 (sketch/limits.hpp).
SAFE_RANGES = ((-30.0, 10.0), (-70.0, 77.0), (-78.0, 85.0))
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
