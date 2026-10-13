"""Append every transition to logs/transitions-<date>.csv.

This is the scorecard's source for inferences/min per state, and the first thing to read
when the demo misbehaves. Transitions into FAULT carry the reason ('LINK_LOST: ...')."""
import csv
import time

from . import paths

HEADER = ["wall_time", "t", "from", "to", "event", "models", "energy", "inferences_so_far", "reason"]


class TransitionLog:
    def __init__(self, logs=None):
        logs = logs or paths.LOGS
        logs.mkdir(exist_ok=True)
        self.path = logs / time.strftime("transitions-%Y%m%d.csv")
        if self.path.exists() and self.path.open().readline().strip() != ",".join(HEADER):
            self.path = logs / time.strftime("transitions-%Y%m%d-%H%M%S.csv")   # older columns
        new = not self.path.exists()
        self.f = open(self.path, "a", newline="")
        self.w = csv.writer(self.f)
        if new:
            self.w.writerow(HEADER)

    def write(self, tr, models, energy, counts, reason=None):
        self.w.writerow([time.strftime("%H:%M:%S"), f"{tr.t:.3f}", tr.src, tr.dst, tr.event,
                         " ".join(f"{k}@{v}" for k, v in models.items()), f"{energy:.2f}",
                         " ".join(f"{k}={v}" for k, v in counts.items()), reason or ""])
        self.f.flush()
