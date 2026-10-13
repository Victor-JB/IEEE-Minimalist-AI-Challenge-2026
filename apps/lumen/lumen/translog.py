"""Append every transition to logs/transitions-<date>.csv.

This is the scorecard's source for inferences/min per state, and the first thing to read
when the demo misbehaves."""
import csv
import time
from pathlib import Path

from . import paths


HEADER = ["wall_time", "t", "from", "to", "event", "models", "energy", "inferences_so_far"]


class TransitionLog:
    def __init__(self, path=None):
        """path: a CSV to append to (tools/simulate.py); default logs/transitions-<date>.csv."""
        if path is None:
            paths.LOGS.mkdir(exist_ok=True)
            path = paths.LOGS / time.strftime("transitions-%Y%m%d.csv")
        self.path = Path(path)
        new = not self.path.exists()
        self.f = open(self.path, "a", newline="")
        self.w = csv.writer(self.f)
        if new:
            self.w.writerow(HEADER)

    def write(self, tr, models, energy, counts, wall=None):
        """wall: epoch seconds for the wall_time column (default: now; the simulator passes its fake clock)."""
        self.w.writerow([time.strftime("%H:%M:%S", time.localtime(wall)), f"{tr.t:.3f}", tr.src, tr.dst, tr.event,
                         " ".join(f"{k}@{v}" for k, v in models.items()), f"{energy:.2f}",
                         " ".join(f"{k}={v}" for k, v in counts.items())])
        self.f.flush()

    def close(self):
        self.f.close()
