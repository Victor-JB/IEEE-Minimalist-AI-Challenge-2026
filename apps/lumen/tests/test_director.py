"""The director with a fake worker and light: which commands each state sends. No arm, no camera."""
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen import config as C                      # noqa: E402
from lumen import events as E                      # noqa: E402
from lumen.behavior.director import Director       # noqa: E402
from lumen.behavior.expressive import Expressive   # noqa: E402
from lumen.machine import Machine                  # noqa: E402


class FakeWorker:
    """Records commands; finished-events are pushed by the test with finish()."""
    def __init__(self):
        self.cmds, self.events = [], []
        self.base, self.current, self.offset = list(C.HOME), list(C.HOME), [0.0, 0.0, 0.0]

    def __getattr__(self, name):                 # play, goto, sweep, seq, relax, connect
        if name.startswith("_"):
            raise AttributeError(name)
        return lambda *a, **k: self.cmds.append((name, a, k))

    def set_base(self, pose):
        self.base = list(pose)

    def set_offset(self, offset):
        self.offset = list(offset)

    def poll_events(self):
        out, self.events = self.events, []
        return out

    def last(self, name=None):
        return next(c for c in reversed(self.cmds) if name is None or c[0] == name)


class FakeLight:
    def __init__(self):
        self.level = 0.0

    def set(self, level, ramp_s=0.5):
        self.level = level

    def step(self, delta):
        self.level += delta


class Det:
    def __init__(self, cx, cy, w=60, h=60):
        self.center, self.w, self.h = (cx, cy), w, h


class FakePerception:
    def __init__(self):
        self.target, self.hand_lm, self.wave_t = None, [], None


class Rig:
    """Machine + director wired the way app.py wires them, on a fake clock."""
    def __init__(self, seed=1):
        self.w, self.light = FakeWorker(), FakeLight()
        self.x = Expressive(random.Random(seed))
        self.d = Director(self.w, self.light, self.x, rng=random.Random(seed))
        self.p = FakePerception()
        self.t = 0.0
        self.m = Machine(on_enter=self.d.on_enter, on_exit=self.d.on_exit, rng=random.Random(seed))

    def ev(self, event, dt=0.1):
        self.t += dt
        self.d.note_events([event], self.t)
        return self.m.handle(event, self.t)

    def finish(self, event=None, info=None):
        """The worker finished the director's current command."""
        self.w.events.append((event or E.CLIP_DONE, self.d.token, info))
        return self.pump()

    def pump(self, dt=0.05):
        self.t += dt
        out = self.d.poll_events(self.t, self.m, self.p)
        for e in out:
            self.m.handle(e, self.t)
        return out

    def tick(self, dt=0.05, shape=(480, 640, 3)):
        self.t += dt
        self.d.tick(self.t, self.m, self.p, shape)
        self.m.tick(self.t)

    def to(self, *events):
        for e in events:
            self.ev(e)


TO_HOLD = (E.READY, E.FACE_SEEN, E.FACE_CONFIRMED, E.POINT, E.CLIP_DONE, E.BOOK_FOUND)


def goto_steps(plan):
    return [s for s in plan if s[0] == "goto"]


class ModifierWiring(unittest.TestCase):
    def test_aim_plan_glances_and_stays_in_caps(self):
        r = Rig()
        r.to(*TO_HOLD)
        self.assertEqual(r.m.state, "AIM")
        name, (plan, tok), kw = r.w.last("seq")
        self.assertEqual(kw["done"], E.AT_POSE)
        self.assertGreaterEqual(len(goto_steps(plan)), 2)              # glance and/or windup first
        self.assertEqual(goto_steps(plan)[-1][1], r.d.aim_pose)
        r.finish(E.AT_POSE)
        self.assertEqual(r.m.state, "HOLD")

    def test_adjust_winds_up_then_steps(self):
        r = Rig()
        r.to(*TO_HOLD)
        r.finish(E.AT_POSE)
        before = list(r.d.aim_pose)
        r.w.current = list(before)
        r.ev(E.CLOSER)
        name, (plan, tok), kw = r.w.last("seq")
        self.assertEqual(kw["done"], E.MOVE_DONE)
        self.assertEqual(len(goto_steps(plan)), 2)
        self.assertEqual(plan[-1][1], r.d.aim_pose)
        r.finish(E.MOVE_DONE)
        self.assertEqual(r.m.state, "HOLD")


if __name__ == "__main__":
    unittest.main()
