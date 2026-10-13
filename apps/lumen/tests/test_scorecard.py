"""tools/scorecard.py on a hand-made log (exact numbers) and on the simulated demo."""
import csv
import sys
import tempfile
import unittest
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
sys.path.insert(0, str(APP / "tools"))
import scorecard as SC                        # noqa: E402
import simulate as S                          # noqa: E402
from lumen.translog import HEADER  # noqa: E402

ROWS = [  # wall_time, t, from, to, event, models, energy, inferences_so_far
    ["10:00:00", "100.000", "BOOT", "IDLE", "READY", "gate@10 face@1", "0.30", ""],
    ["10:01:00", "160.000", "IDLE", "NOTICE", "FACE_SEEN", "face@10", "0.30", "gate=600 face=60"],
    ["10:01:30", "190.000", "NOTICE", "STANDBY", "FACE_CONFIRMED", "face@5 hand@10", "0.45", "gate=600 face=360"],
    ["10:02:00", "220.000", "STANDBY", "LOST", "FACE_LOST", "face@5", "0.40", "gate=600 face=510 hand=300"],
]


def write(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    return path


def table(md, first_cell):
    """Cells of the markdown row that starts with first_cell (first match)."""
    for line in md.splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if cells and cells[0] == first_cell:
            return cells
    raise AssertionError(f"no row {first_cell!r} in:\n{md}")


class HandMadeLog(unittest.TestCase):
    def setUp(self):
        self.d = Path(tempfile.mkdtemp())
        self.log = write(self.d / "t.csv", HEADER, ROWS)
        power = []
        for s in range(0, 120):                    # 10:00:00.5 .. 10:01:59.5, 1 Hz
            w = 2.5 if s < 60 else 3.0 if s < 90 else 3.5
            power.append([f"10:{s // 60:02d}:{s % 60:02d}.5", f"{w}"])
        self.power = write(self.d / "meter.csv", ["Time", "Power (W)"], power)

    def test_time_share_and_rates(self):
        md = SC.scorecard([self.log])
        # state, visits, time, share, book?, face, gate, hand, all ; models sorted: face, gate, hand
        self.assertEqual(table(md, "IDLE")[:4], ["IDLE", "1", "60.0", "50.0"])
        self.assertEqual(table(md, "IDLE")[4:], ["60.0 (60)", "600.0 (600)", "", "660.0"])
        self.assertEqual(table(md, "NOTICE")[4], "600.0 (600)")
        self.assertEqual(table(md, "STANDBY")[4:], ["300.0 (300)", "", "600.0 (600)", "900.0"])
        self.assertIn("LOST from t=220.0 (open at the end of the log)", md)

    def test_power_and_energy_per_inference(self):
        md = SC.scorecard([self.log], power=self.power, idle_watts=2.0)
        idle = table(md.split("Power per state")[1], "IDLE")
        self.assertEqual(idle, ["IDLE", "2.50", "0.50", "150.0", "660", "227.3", "45.5"])
        self.assertEqual(table(md.split("Power per state")[1], "NOTICE")[-1], "100.0")
        self.assertEqual(table(md.split("Power per state")[1], "STANDBY")[-1], "100.0")
        self.assertIn("| 2.88 | 2.00 |", md)          # Full system row: meter average, idle

    def test_volts_amps_and_epoch_meter(self):
        import datetime as dt
        base = dt.datetime.combine(dt.date.today(), dt.time(10, 0, 0, 500000)).timestamp()
        rows = [[f"{base + s:.3f}", "5.0", "0.5"] for s in range(60)]     # 2.5 W during IDLE
        meter = write(self.d / "va.csv", ["timestamp", "voltage", "current"], rows)
        md = SC.scorecard([self.log], power=meter, idle_watts=2.0)
        self.assertEqual(table(md.split("Power per state")[1], "IDLE")[1], "2.50")
        self.assertEqual(table(md.split("Power per state")[1], "NOTICE")[1], "no samples")

    def test_restart_starts_a_new_session(self):
        again = [["10:05:00", "3.000", "BOOT", "IDLE", "READY", "gate@10 face@1", "0.30", ""],
                 ["10:05:30", "33.000", "IDLE", "NOTICE", "FACE_SEEN", "face@10", "0.30", "gate=300 face=30"]]
        log2 = write(self.d / "t2.csv", HEADER, ROWS + again)
        md = SC.scorecard([log2])
        self.assertIn("in 2 session(s)", md)
        self.assertEqual(table(md, "IDLE")[:3], ["IDLE", "2", "90.0"])
        self.assertEqual(table(md, "IDLE")[5], "600.0 (600)")

    def test_end_t_closes_the_last_state_without_diluting_rates(self):
        md = SC.scorecard([self.log], end_t=250)
        self.assertEqual(table(md, "LOST")[2], "30.0")
        self.assertEqual(table(md, "IDLE")[5], "600.0 (600)")

    def test_perf_csv(self):
        rows = [["timing_ms", "camera.read", f"{i / 10:.4f}", "12.0"] for i in range(101)]
        rows += [["timing_ms", "detector.total", f"{i / 5:.4f}", f"{60 + i % 3}"] for i in range(50)]
        rows += [["system", "cpu_pct", "1.0", "40.0"], ["system", "cpu_pct", "2.0", "60.0"]]
        perf = write(self.d / "perf.csv", ["kind", "name", "t_s", "value"], rows)
        md = SC.scorecard([self.log], perf=perf, change="baseline")
        self.assertEqual(table(md, "detector.total")[:3], ["detector.total", "50", "60.98"])
        row = table(md, SC.dt.date.today().isoformat())
        self.assertEqual(row[1:6], ["baseline", "10.0", "61.0", "12.0", "50"])


class SimulatedDemo(unittest.TestCase):
    def test_scorecard_on_the_simulated_demo(self):
        d = Path(tempfile.mkdtemp())
        S.simulate(csv_path=d / "sim.csv", power_csv=d / "p.csv", out=lambda s: None, start_wall=1_760_000_000)
        md = SC.scorecard([d / "sim.csv"], power=d / "p.csv", idle_watts=S.SYN_IDLE_W, end_t=520)
        shares = [float(table(md, label)[3]) for label in
                  ("BOOT", "SLEEP", "IDLE", "NOTICE", "STANDBY", "LOST", "ACK", "SEARCH", "AIM", "NOT_FOUND",
                   "ILLUMINATE/HOLD", "ILLUMINATE/ADJUST", "ILLUMINATE/CHECKIN", "ILLUMINATE/REACQUIRE",
                   "GOODBYE", "FAULT")]
        self.assertAlmostEqual(sum(shares), 100.0, delta=0.5)
        power = md.split("Power per state")[1]
        sleep_w, hold_w = float(table(power, "SLEEP")[1]), float(table(power, "ILLUMINATE/HOLD")[1])
        self.assertLess(sleep_w, hold_w)       # the toy model charges the light and the models
        self.assertIn("Row for RESULTS.md", md)

    def test_cli(self):
        d = Path(tempfile.mkdtemp())
        log = write(d / "t.csv", HEADER, ROWS)
        import contextlib
        import io
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            SC.main([str(log), "--change", "x"])
        self.assertTrue(buf.getvalue().startswith("### Scorecard"))


if __name__ == "__main__":
    unittest.main()
