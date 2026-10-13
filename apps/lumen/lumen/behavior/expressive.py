"""The cross-cutting expressive layer: noise (no two moves alike) and energy (spunk)."""
import math
import random

from .. import config as C
from .pools import POOLS

_ENERGY_W = {"low": lambda e: 1.5 - e, "mid": lambda e: 1.0, "high": lambda e: 0.5 + e}


class Expressive:
    def __init__(self, rng=None, now=0.0):
        self.rng = rng or random.Random()
        self.energy, self._t = C.ENERGY_START, now
        self.recent = []
        self.drift = [0.0, 0.0, 0.0]
        self._drift_mid, self._drift_goal, self._drift_left = [0.0] * 3, [0.0] * 3, [0.0] * 3

    # -- mood ------------------------------------------------------------------
    def bump(self, now, amount=C.ENERGY_BUMP):
        """Call on every interaction (command, gesture, mirrored wave)."""
        self.update(now)
        self.energy = min(1.0, self.energy + amount)

    def update(self, now):
        dt, self._t = max(0.0, now - self._t), now
        k = math.exp(-dt / C.ENERGY_DECAY_S)
        self.energy = C.ENERGY_REST + (self.energy - C.ENERGY_REST) * k
        return self.energy

    # -- clip choice -----------------------------------------------------------
    def pick(self, pool):
        """(clip ref, amplitude, time_scale) from a pool, or None for an empty pool.

        Weighted by energy, never one of the last NO_REPEAT_LAST clips (when the pool allows),
        with random amplitude and speed. High energy also nudges playback faster.
        """
        entries = POOLS.get(pool, [])
        if not entries:
            return None
        fresh = [e for e in entries if e[0] not in self.recent] or entries
        weights = [_ENERGY_W[level](self.energy) for _, level in fresh]
        ref = self.rng.choices([r for r, _ in fresh], weights)[0]
        self.recent = (self.recent + [ref])[-C.NO_REPEAT_LAST:]
        amplitude = self.rng.uniform(*C.AMPLITUDE_RANGE)
        time_scale = self.rng.uniform(*C.TIME_SCALE_RANGE) * (1.1 - 0.2 * self.energy)
        return ref, amplitude, time_scale

    # -- hold drift ------------------------------------------------------------
    def drift_step(self, dt):
        """Offset per joint (deg) to add to any hold so it is never frozen: each joint glides to a
        new random target every HOLD_DRIFT_EVERY_S, through two first-order stages (smooth, no
        overshoot), scaled by HOLD_DRIFT_GAIN (all zero for now; see config)."""
        k = 1 - math.exp(-dt / C.HOLD_DRIFT_SMOOTH_S)
        for j in range(3):
            self._drift_left[j] -= dt
            if self._drift_left[j] <= 0:
                self._drift_left[j] = self.rng.uniform(*C.HOLD_DRIFT_EVERY_S)
                self._drift_goal[j] = self.rng.gauss(0, C.HOLD_DRIFT_DEG)
            self._drift_mid[j] += k * (self._drift_goal[j] - self._drift_mid[j])
            self.drift[j] += k * (self._drift_mid[j] - self.drift[j])
        return [g * d for g, d in zip(C.HOLD_DRIFT_GAIN, self.drift)]
