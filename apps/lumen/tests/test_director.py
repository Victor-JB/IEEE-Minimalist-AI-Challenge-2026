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


class FakePower:
    def __init__(self):
        self.calls = []

    def sleep(self):
        self.calls.append("sleep")

    def wake(self):
        self.calls.append("wake")


class Rig:
    """Machine + director wired the way app.py wires them, on a fake clock."""
    def __init__(self, seed=1):
        self.w, self.light, self.power = FakeWorker(), FakeLight(), FakePower()
        self.x = Expressive(random.Random(seed))
        self.d = Director(self.w, self.light, self.x, rng=random.Random(seed), power=self.power)
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


class Sleep(unittest.TestCase):
    def asleep(self):
        r = Rig()
        r.to(E.READY, E.NO_MOTION)
        self.assertEqual(r.m.state, "SLEEP")
        return r

    def test_saves_power_and_slumps_once(self):
        r = self.asleep()
        self.assertEqual(r.power.calls, ["sleep"])
        name, (plan, tok), kw = r.w.last()
        self.assertEqual(name, "seq")
        self.assertEqual(plan, [("play", "go_to_sleep/drowsy", 1.0, 1.0), ("rebase",)])

    def test_then_breathes_very_slowly(self):
        r = self.asleep()
        r.finish()                                         # go_to_sleep done
        self.assertEqual(r.m.state, "SLEEP")
        for _ in range(60):                                # gap is at most 2 s
            r.tick()
        name, (plan, tok), kw = r.w.last()
        self.assertEqual(name, "seq")
        self.assertTrue(plan)
        for step in plan:
            self.assertEqual(step, ("play", "idle_breathe/calm", 0.3, 2.0))
        n = len(r.w.cmds)
        r.finish()
        for _ in range(60):
            r.tick()
        self.assertGreater(len(r.w.cmds), n)               # and keeps breathing
        self.assertNotIn("go_to_sleep/drowsy", str(r.w.cmds[n:]))

    def test_motion_wakes_and_restores_power(self):
        r = self.asleep()
        r.ev(E.MOTION)
        self.assertEqual(r.m.state, "IDLE")
        self.assertEqual(r.power.calls, ["sleep", "wake"])


class WakeUp(unittest.TestCase):
    def slept(self):
        r = Rig()
        r.to(E.READY, E.NO_MOTION)
        r.finish()                                         # slumped
        return r

    def test_face_soon_after_waking_gets_the_groggy_wake_up(self):
        r = self.slept()
        r.ev(E.MOTION)
        n = len(r.w.cmds)
        self.assertEqual(r.m.state, "IDLE")
        self.assertEqual(r.w.cmds[n:], [])                 # stays slumped, no snap back to home
        r.ev(E.FACE_SEEN, dt=1.0)
        self.assertEqual(r.m.state, "NOTICE")
        name, (plan, tok), kw = r.w.last()
        self.assertEqual(plan, [("play", "wake_up/slow_groggy", 0.7, 1.0), ("rebase",)])
        self.assertNotIn("double_take", str(r.w.cmds))

    def test_the_wake_up_outlives_notice_and_standby_waits_for_it(self):
        r = self.slept()
        r.ev(E.MOTION)
        r.ev(E.FACE_SEEN, dt=1.0)
        n = len(r.w.cmds)
        r.ev(E.FACE_CONFIRMED)
        self.assertEqual(r.m.state, "STANDBY")
        self.assertTrue(r.d.busy)
        r.p.target = Det(320, 400)
        for _ in range(100):                               # 5 s: past the center timeout
            r.tick()
        self.assertEqual([c for c in r.w.cmds[n:] if c[0] in ("play", "seq")], [])
        self.assertEqual(r.w.offset, [0.0, 0.0, 0.0])      # no look-at while the clip moves the head
        r.w.events.append((E.CLIP_DONE, r.d.wake_tok, None))
        r.pump()
        self.assertFalse(r.d.busy)
        self.assertEqual(r.m.state, "STANDBY")

    def test_late_face_is_the_normal_notice(self):
        r = self.slept()
        r.ev(E.MOTION)
        for _ in range(140):                               # 7 s, nobody: the IDLE loop gets up
            r.tick()
        name, (plan, tok), kw = r.w.last()
        self.assertEqual(plan[0][1], "wake_up/slow_groggy")
        r.finish()
        self.assertFalse(r.d.slumped)
        n = len(r.w.cmds)
        r.ev(E.FACE_SEEN, dt=10)
        self.assertEqual(r.m.state, "NOTICE")
        self.assertFalse(r.d.busy)                         # centering first, as usual
        self.assertNotIn("wake_up", str(r.w.cmds[n:]))


class FalseAlarm(unittest.TestCase):
    def test_notice_timeout_hesitates(self):
        r = Rig()
        r.to(E.READY, E.FACE_SEEN)
        r.tick(dt=2.0)                                     # NOTICE timer (1.5 s)
        self.assertEqual(r.m.state, "IDLE")
        name, (ref, amp, ts, tok), kw = r.w.last()
        self.assertEqual((name, ref), ("play", "hesitate/subtle"))
        self.assertEqual(tok, r.d.token)
        r.finish()
        self.assertEqual(r.m.state, "IDLE")                # then the bored loop resumes

    def test_ordinary_idle_is_bored(self):
        r = Rig()
        r.ev(E.READY)
        name, (ref, *_), kw = r.w.last("play")
        self.assertNotEqual(ref, "hesitate/subtle")


class CheckIn(unittest.TestCase):
    def lit(self, face_y=None):
        r = Rig()
        r.to(E.READY, E.FACE_SEEN, E.FACE_CONFIRMED)
        if face_y is not None:                             # STANDBY sees the face, above center
            r.p.target = Det(320, face_y)
            r.tick()
        r.p.target = None
        r.to(E.POINT, E.CLIP_DONE, E.BOOK_FOUND)
        r.finish(E.AT_POSE)
        self.assertEqual(r.m.state, "HOLD")
        r.w.current = list(r.d.aim_pose)
        r.tick(dt=16.0)                                    # check-in timer (8-15 s)
        self.assertEqual(r.m.state, "CHECKIN")
        return r

    def test_looks_up_toward_the_face_then_back_to_the_aim_pose(self):
        r = self.lit(face_y=120)
        name, (plan, tok), kw = r.w.last()
        self.assertEqual(name, "seq")
        self.assertEqual([s[0] for s in plan], ["goto", "mark", "wait", "mark", "goto"])
        up, back = plan[0][1], plan[-1]
        aim = r.d.aim_pose
        self.assertEqual(up[:2], aim[:2])
        self.assertLess(up[2], aim[2] - 10)                # head up, most of the way to the face
        want = r.d.user_pitch - sum(aim)
        self.assertLessEqual(abs(up[2] - aim[2]), abs(want) + 1e-6)
        self.assertEqual(back[1], aim)
        self.assertTrue(back[3])                           # the look-at offset is cleared on the way back
        for speeds in [[1.875 * abs(b - a) / plan[0][2] for a, b in zip(aim, up)]]:
            for j, v in enumerate(speeds):
                self.assertLessEqual(v, C.SPEED_CAPS[j])

    def test_tracks_the_face_only_while_looking(self):
        r = self.lit(face_y=120)
        r.p.target = Det(320, 60)                          # face high in the image
        r.tick()
        self.assertEqual(r.w.offset, [0.0, 0.0, 0.0])      # still moving up: no steering
        r.finish(E.STEP, 0)
        for _ in range(5):
            r.tick()
        self.assertLess(r.w.offset[2], 0)                  # head goes up toward the face
        r.finish(E.STEP, 1)
        self.assertEqual(r.w.offset, [0.0, 0.0, 0.0])
        r.tick()
        self.assertEqual(r.w.offset, [0.0, 0.0, 0.0])      # and stays cleared on the way back
        r.finish()
        self.assertEqual(r.m.state, "HOLD")
        self.assertEqual(r.w.base, r.d.aim_pose)

    def test_no_face_ever_seen_still_looks_up(self):
        r = self.lit()
        name, (plan, tok), kw = r.w.last()
        self.assertLess(plan[0][1][2], r.d.aim_pose[2])


def plan_seconds(plan):
    """Plan length, counting a play step as 0 (none in these plans)."""
    return sum(s[2] if s[0] == "goto" else s[1] if s[0] == "wait" else 0 for s in plan)


class Reacquire(unittest.TestCase):
    def lost(self):
        r = Rig()
        r.to(*TO_HOLD)
        r.finish(E.AT_POSE)
        r.w.current = list(r.d.aim_pose)
        r.ev(E.BOOK_LOST)
        self.assertEqual(r.m.state, "REACQUIRE")
        return r

    def test_small_widening_sweep_around_the_aim_pose(self):
        r = self.lost()
        name, (plan, tok), kw = r.w.last()
        self.assertEqual(name, "seq")
        aim = r.d.aim_pose
        peeks = [s[1] for s in plan if s[0] == "goto"]
        self.assertEqual(peeks[-1], aim)                   # ends back on the aim pose
        spread = [max(abs(a - b) for a, b in zip(p, aim)) for p in peeks[:-1]]
        self.assertLessEqual(max(spread), 10.0)            # local
        far = [x for x in spread if x > 1.0]               # the peeks (not the micro-retreat)
        self.assertEqual(far, sorted(far))                 # widening
        self.assertLessEqual(plan_seconds(plan[:-1]), C.REACQUIRE_TIMEOUT_S)   # every peek before the timeout
        for step in plan:
            if step[0] == "goto":
                for v, (lo, hi) in zip(step[1], C.SAFE_RANGES):
                    self.assertTrue(lo <= v <= hi)

    def test_found_returns_to_aim_and_holds(self):
        r = self.lost()
        r.w.current = [a + 5 for a in r.d.aim_pose]        # mid-peek
        r.ev(E.BOOK_FOUND)
        self.assertEqual(r.m.state, "HOLD")
        name, (plan, tok), kw = r.w.last("seq")
        self.assertEqual(plan[-1][1], r.d.aim_pose)
        self.assertNotEqual(tok, r.d.token)                # its done is not an event for HOLD
        self.assertEqual(r.w.base, r.d.aim_pose)

    def test_not_found_in_4s(self):
        r = self.lost()
        r.tick(dt=C.REACQUIRE_TIMEOUT_S + 0.1)
        self.assertEqual(r.m.state, "NOT_FOUND")


class ReacquirePerception(unittest.TestCase):
    def test_book_model_runs_at_3hz_on_a_widened_crop(self):
        import numpy as np
        from lumen.machine import STATES
        from lumen.perception import Perception

        class Book:
            def __init__(self):
                self.present, self.rois, self.times = True, [], []

            def detect(self, frame, roi=None):
                self.rois.append(roi)
                return [(300, 200, 100, 80, 0.9)] if self.present else []

        book, ctx = Book(), {}
        p = Perception(book=book)
        frame = np.zeros((480, 640, 3), np.uint8)
        p.step(frame, 0.0, {"book": 3}, ctx)                # seen: crop around it
        self.assertIsNotNone(ctx["book_roi"])
        book.present = False
        n, t = len(book.rois), 0.0
        while t < 2.0:
            t += 1 / 15
            p.step(frame, t, STATES["REACQUIRE"].models, ctx)
        self.assertIsNone(ctx["book_roi"])                 # widened to the full frame
        self.assertIn(None, book.rois[n:])
        self.assertTrue(5 <= len(book.rois) - n <= 7)      # 3 Hz


if __name__ == "__main__":
    unittest.main()
