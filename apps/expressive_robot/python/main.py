"""Expressive robot - Linux side.

Each loop: perceive -> decide -> send joint targets to the sketch.
For now it sends a slow test sweep so you can watch the sketch's Monitor output
(and later the servos) respond without any perception or planning in place.
"""

import math
import time

from arduino.app_utils import *

from robot_config import HOME  # generated from sim/robot.toml

SEND_HZ = 20


def test_sweep(t):
    """Swing joint 0 (base yaw) +/-30 degrees around HOME every 5 seconds."""
    targets = list(HOME)
    targets[0] += 30 * math.sin(2 * math.pi * t / 5)
    return targets


def loop():
    t = time.monotonic()

    # TODO: perception - grab a camera frame and run the face detector.
    # TODO: planning - pick a behavior state and turn it plus the face position
    #       into joint targets. test_sweep stands in for that for now.
    targets = test_sweep(t)

    # notify = fire-and-forget, so a slow MCU never stalls this loop.
    Bridge.notify("set_joints", [round(a) for a in targets])
    time.sleep(1 / SEND_HZ)


App.run(user_loop=loop)
