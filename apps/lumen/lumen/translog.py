"""Append every transition to logs/transitions-<date>.csv.

This is the scorecard's source for inferences/min per state, and the first thing to read
when the demo misbehaves. Transitions into FAULT carry the reason ('LINK_LOST: ...')."""
import csv
import time
from pathlib import Path

from . import paths

HEADER = ["wall_time", "t", "from", "to", "event", "models", "energy", "inferences_so_far", "reason"]


class TransitionLog:
    def __init__(self, path=None):
        """path: a CSV to append to (tools/simulate.py), or a logs folder (default paths.LOGS)
        that gets transitions-<date>.csv."""
        path = Path(path) if path is not None else paths.LOGS
        if path.suffix != ".csv":
            path.mkdir(exist_ok=True)
            logs, path = path, path / time.strftime("transitions-%Y%m%d.csv")
            if path.exists() and path.open().readline().strip() != ",".join(HEADER):
                path = logs / time.strftime("transitions-%Y%m%d-%H%M%S.csv")   # older columns
        self.path = path
        new = not self.path.exists()
        self.f = open(self.path, "a", newline="")
        self.w = csv.writer(self.f)
        if new:
            self.w.writerow(HEADER)

    def write(self, tr, models, energy, counts, reason=None, wall=None):
        """reason: why a FAULT happened. wall: epoch seconds for wall_time (default now; the
        simulator passes its fake clock)."""
        self.w.writerow([time.strftime("%H:%M:%S", time.localtime(wall)), f"{tr.t:.3f}", tr.src, tr.dst, tr.event,
                         " ".join(f"{k}@{v}" for k, v in models.items()), f"{energy:.2f}",
                         " ".join(f"{k}={v}" for k, v in counts.items()), reason or ""])
        self.f.flush()

    def close(self):
        self.f.close()
