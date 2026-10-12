import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen.behavior.expressive import Expressive   # noqa: E402
from lumen.behavior.pools import POOLS             # noqa: E402


class ExpressiveTest(unittest.TestCase):
    def test_no_immediate_repeats(self):
        x = Expressive(random.Random(1))
        picks = [x.pick("bored")[0] for _ in range(200)]
        self.assertTrue(all(a != b for a, b in zip(picks, picks[1:])))

    def test_energy_decays_to_rest(self):
        x = Expressive(random.Random(1))
        x.bump(0.0, 0.7)
        self.assertGreater(x.energy, 0.9)
        self.assertLess(abs(x.update(2000.0) - 0.3), 0.01)

    def test_hold_drift_is_smooth(self):
        x = Expressive(random.Random(1))
        from lumen import config as C
        C.HOLD_DRIFT_GAIN, saved = (1.0, 1.0, 0.5), C.HOLD_DRIFT_GAIN
        self.addCleanup(setattr, C, "HOLD_DRIFT_GAIN", saved)
        steps = []
        for _ in range(25 * 60):
            before = list(x.drift)
            after = x.drift_step(0.04)
            steps.append(max(abs(a - b) for a, b in zip(after, before)))
        self.assertLess(max(steps), 0.15)                      # deg per 25 Hz tick, was ~0.6

    def test_every_pool_entry_has_an_energy_level(self):
        for name, entries in POOLS.items():
            for ref, level in entries:
                self.assertIn(level, ("low", "mid", "high"), (name, ref))
                self.assertIn("/", ref)


if __name__ == "__main__":
    unittest.main()
