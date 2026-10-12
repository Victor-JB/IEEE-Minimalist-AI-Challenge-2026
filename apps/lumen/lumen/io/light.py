"""Lamp LED. TODO(teammate/victor): wire the LED to a MOSFET and drive it.

Two options: (a) a PWM pin on the STM32, which needs a small RPC added to the stepper_desk
firmware ("light <0-1000>"); (b) a Linux-side GPIO/PWM via gpiod. Until then this just
remembers the level so the HUD and logs show what the lamp would do.
"""
import time


class Light:
    def __init__(self):
        self.level, self.target, self.ramp_until = 0.0, 0.0, 0.0

    def set(self, level, ramp_s=0.5):
        self.target = max(0.0, min(1.0, level))
        self.ramp_until = time.monotonic() + ramp_s
        self.level = self.target          # TODO: ramp in hardware

    def step(self, delta):
        self.set(self.target + delta, ramp_s=0.3)
