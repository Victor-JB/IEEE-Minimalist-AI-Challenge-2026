"""The headless full-loop simulation (tools/simulate.py) runs the whole demo and logs it."""
import csv
import sys
import tempfile
import unittest
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
sys.path.insert(0, str(APP / "tools"))
import simulate as S                                # noqa: E402
from luxoiii import events as E                       # noqa: E402
from luxoiii.perception import gestures as G          # noqa: E402
from luxoiii.translog import HEADER                   # noqa: E402

_cache = {}


def run_demo():
    if "res" not in _cache:
        d = tempfile.mkdtemp()
        csv_path, power = Path(d) / "sim.csv", Path(d) / "power.csv"
        res = S.simulate(csv_path=csv_path, power_csv=power, out=lambda s: None, start_wall=1_760_000_000)
        _cache.update(res=res, csv=csv_path, power=power)
    return _cache["res"]


class FakeHands(unittest.TestCase):
    def test_landmarks_classify_as_scripted(self):
        for g, lm in S.HANDS.items():
            self.assertEqual(G.classify(lm), g)
        self.assertIsNone(G.classify(S._hand(None)))


class FullDemo(unittest.TestCase):
    def setUp(self):
        self.res = run_demo()
        self.path = [p for _, _, _, p in self.res.transitions]
        self.edges = [(s, e, p.split("/")[-1]) for _, s, e, p in self.res.transitions]

    def test_visits_every_demo_state(self):
        for state in ("IDLE", "NOTICE", "STANDBY", "ACK", "SEARCH", "NOT_FOUND", "AIM", "ILLUMINATE/HOLD",
                      "ILLUMINATE/ADJUST", "ILLUMINATE/CHECKIN", "ILLUMINATE/REACQUIRE", "GOODBYE", "LOST",
                      "SLEEP", "FAULT", "BOOT"):
            self.assertIn(state, self.path)

    def test_key_edges_in_order(self):
        want = [("IDLE", E.FACE_SEEN, "NOTICE"), ("STANDBY", E.POINT, "ACK"), ("SEARCH", E.SWEEP_DONE, "NOT_FOUND"),
                ("SEARCH", E.BOOK_FOUND, "AIM"), ("HOLD", E.CLOSER, "ADJUST"), ("HOLD", E.FURTHER, "ADJUST"),
                ("HOLD", E.BOOK_LOST, "REACQUIRE"), ("REACQUIRE", E.BOOK_FOUND, "HOLD"), ("HOLD", E.PALM, "GOODBYE"),
                ("GOODBYE", E.CLIP_DONE, "STANDBY"), ("STANDBY", E.FACE_LOST, "LOST"), ("LOST", E.TIMEOUT, "IDLE"),
                ("IDLE", E.NO_MOTION, "SLEEP"), ("SLEEP", E.MOTION, "IDLE"), ("IDLE", E.FAULT, "FAULT"),
                ("FAULT", E.RECOVERED, "BOOT"), ("BOOT", E.READY, "IDLE")]
        it = iter(self.edges)
        for edge in want:
            self.assertIn(edge, it, f"{edge} missing or out of order")

    def test_second_fault_latches_and_reset_reboots(self):
        faults = [i for i, (_, _, d) in enumerate(self.edges) if d == "FAULT"]
        self.assertEqual(len(faults), 2)
        after = self.edges[faults[1] + 1]
        self.assertEqual((after[0], after[2]), ("FAULT", "BOOT"))
        self.assertIn(after[1], ("RESET", "RESTART"))       # RESTART: no RESET event in the machine
        self.assertEqual(self.edges[-1][2], "IDLE")

    def test_csv_matches_translog_format(self):
        with open(_cache["csv"]) as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[0], HEADER)
        self.assertEqual(len(rows) - 1, len(self.res.transitions))
        last = dict(zip(HEADER, rows[-1]))
        self.assertIn("face=", last["inferences_so_far"])
        ts = [float(r[1]) for r in rows[1:]]
        self.assertEqual(ts, sorted(ts))

    def test_power_trace_written(self):
        with open(_cache["power"]) as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[0], ["timestamp", "watts"])
        self.assertGreater(len(rows), 500)

    def test_deterministic(self):
        again = S.simulate(out=lambda s: None, start_wall=1_760_000_000)
        self.assertEqual(again.transitions, self.res.transitions)


class PerceptionFindings(unittest.TestCase):
    """Found while building the scorecard; fixed by ModelScheduler.cancel() (8e35768)."""

    def test_logged_face_count_matches_face_model_calls(self):
        """ModelScheduler.due() counts 'face' before Perception.step drops it for the motion gate,
        so inferences_so_far in the transitions CSV over-counts face in IDLE: the gating savings
        never show in the scorecard (demo run: 504 logged vs 167 real face inferences)."""
        res = run_demo()
        self.assertEqual(res.counts["face"], res.calls["face"])


if __name__ == "__main__":
    unittest.main()
