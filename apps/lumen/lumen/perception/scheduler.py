"""Run each model only as often as the current state asks (the compute budget).

rates: {"face": 5, "hand": 10, "book": "per_pose", ...} straight from machine.STATES.
"""


class ModelScheduler:
    def __init__(self):
        self.last = {}
        self.counts = {}

    def due(self, rates, now, at_pause_pose=False):
        """Names of models to run on this frame."""
        run = []
        for name, hz in rates.items():
            if hz == "per_pose":
                ok = at_pause_pose
            else:
                ok = now - self.last.get(name, -1e9) >= 1.0 / hz
            if ok:
                run.append(name)
                self.last[name] = now
                self.counts[name] = self.counts.get(name, 0) + 1
        return run

    def reset(self):
        self.last.clear()
