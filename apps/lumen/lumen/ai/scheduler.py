"""Run each model only as often as the current state asks (the compute budget).

rates: {"face": 5, "hand": 10, "book": "per_pose", ...} straight from machine.STATES.
"""

from .config import SLACK_S   # edit in ai/config.py
                 # with a few ms of jitter, and a 2 Hz model must not skip every other one


class ModelScheduler:
    def __init__(self):
        self.last = {}
        self.counts = {}
        self._before = {}                    # name -> its `last` before this frame's due()

    def due(self, rates, now, at_pause_pose=False):
        """Names of models to run on this frame."""
        run = []
        for name, hz in rates.items():
            if hz == "per_pose":
                ok = at_pause_pose
            else:
                ok = now - self.last.get(name, -1e9) >= 1.0 / hz - SLACK_S
            if ok:
                run.append(name)
                self._before[name] = self.last.get(name)
                self.last[name] = now
                self.counts[name] = self.counts.get(name, 0) + 1
        return run

    def cancel(self, name):
        """A model due this frame was skipped after all (e.g. motion-gated): don't count it, and
        let it run again as soon as it's allowed instead of waiting a full period."""
        self.counts[name] -= 1
        before = self._before.pop(name, None)
        if before is None:
            self.last.pop(name, None)
        else:
            self.last[name] = before

    def reset(self):
        self.last.clear()
