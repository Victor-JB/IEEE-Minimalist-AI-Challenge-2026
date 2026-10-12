"""Append every transition to logs/transitions-<date>.csv.

This is the scorecard's source for inferences/min per state, and the first thing to read
when the demo misbehaves."""
import csv
import time

from . import paths


class TransitionLog:
    def __init__(self):
        paths.LOGS.mkdir(exist_ok=True)
        self.path = paths.LOGS / time.strftime("transitions-%Y%m%d.csv")
        new = not self.path.exists()
        self.f = open(self.path, "a", newline="")
        self.w = csv.writer(self.f)
        if new:
            self.w.writerow(["wall_time", "t", "from", "to", "event", "models", "energy", "inferences_so_far"])

    def write(self, tr, models, energy, counts):
        self.w.writerow([time.strftime("%H:%M:%S"), f"{tr.t:.3f}", tr.src, tr.dst, tr.event,
                         " ".join(f"{k}@{v}" for k, v in models.items()), f"{energy:.2f}",
                         " ".join(f"{k}={v}" for k, v in counts.items())])
        self.f.flush()
