"""Every edge of the v1 diagram (docs/STATE_MACHINE.md), one test each, with a fake clock.

test_transition_coverage.py checks that these (plus test_machine.py and the simulator test)
exercise every (state, event) key in machine.TRANSITIONS.
"""
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen import config as C          # noqa: E402
from lumen import events as E          # noqa: E402
from lumen.machine import INITIAL_CHILD, Machine, STATES, TRANSITIONS   # noqa: E402

# Shortest event path from BOOT to each state (the machine's own vocabulary).
PATH = {
    "BOOT": [],
    "IDLE": [E.READY],
    "SLEEP": [E.READY, E.NO_MOTION],
    "NOTICE": [E.READY, E.FACE_SEEN],
    "STANDBY": [E.READY, E.FACE_SEEN, E.FACE_CONFIRMED],
}
PATH["LOST"] = PATH["STANDBY"] + [E.FACE_LOST]
PATH["ACK"] = PATH["STANDBY"] + [E.POINT]
PATH["SEARCH"] = PATH["ACK"] + [E.CLIP_DONE]
PATH["AIM"] = PATH["SEARCH"] + [E.BOOK_FOUND]
PATH["NOT_FOUND"] = PATH["SEARCH"] + [E.SWEEP_DONE]
PATH["HOLD"] = PATH["AIM"] + [E.AT_POSE]
PATH["ADJUST"] = PATH["HOLD"] + [E.CLOSER]
PATH["CHECKIN"] = PATH["HOLD"] + [E.CHECKIN_DUE]
PATH["REACQUIRE"] = PATH["HOLD"] + [E.BOOK_LOST]
PATH["GOODBYE"] = PATH["HOLD"] + [E.PALM]
PATH["FAULT"] = [E.FAULT]
# RESET -> BOOT landed on state-machine-fixes (e989877); expected failure on older bases.
_xfail_without_reset = (lambda f: f) if hasattr(E, "RESET") else unittest.expectedFailure
LEAF_STATES = [s for s in STATES if s not in INITIAL_CHILD]


class Rig:
    """A machine with a fake clock and an enter/exit callback log."""

    def __init__(self, seed=0):
        self.t = 0.0
        self.calls = []
        self.m = Machine(on_enter=lambda s, m: self.calls.append(("enter", s)),
                         on_exit=lambda s, m: self.calls.append(("exit", s)),
                         now=self.t, rng=random.Random(seed))

    def ev(self, event, dt=0.1):
        self.t += dt
        return self.m.handle(event, self.t)

    def goto(self, state):
        for event in PATH[state]:
            self.ev(event)
        assert self.m.state == state, (state, self.m.state)
        self.calls.clear()
        return self

    def wait(self, seconds):
        """Advance the fake clock in 50 ms ticks; return the transitions the timers fired."""
        fired, end = [], self.t + seconds
        while self.t < end - 1e-9:
            self.t = min(end, self.t + 0.05)
            tr = self.m.tick(self.t)
            if tr:
                fired.append(tr)
        return fired


class PathsAreRight(unittest.TestCase):
    def test_every_leaf_state_is_reachable(self):
        for state in LEAF_STATES:
            self.assertEqual(Rig().goto(state).m.state, state)


class DiagramEdges(unittest.TestCase):
    """One test per v1 edge that test_machine.py did not already pin down."""

    def check(self, start, event, expect, **ctx):
        r = Rig().goto(start)
        r.m.ctx.update(ctx)
        tr = r.ev(event)
        self.assertIsNotNone(tr, f"{start} ignores {event}")
        self.assertEqual((tr.src, tr.dst, tr.event), (start, expect, event))
        self.assertEqual(r.m.state, expect)
        return r

    # outer states
    def test_idle_to_sleep_on_no_motion(self):
        self.check("IDLE", E.NO_MOTION, "SLEEP")

    def test_sleep_to_idle_on_motion(self):
        self.check("SLEEP", E.MOTION, "IDLE")

    def test_sleep_ignores_faces_and_gestures(self):
        r = Rig().goto("SLEEP")
        for event in (E.FACE_SEEN, E.FACE_CONFIRMED, E.POINT, E.PALM, E.NO_MOTION):
            self.assertIsNone(r.ev(event), event)
        self.assertEqual(r.m.state, "SLEEP")

    def test_notice_to_standby_on_confirm(self):
        self.check("NOTICE", E.FACE_CONFIRMED, "STANDBY")

    def test_standby_to_lost_on_face_lost(self):
        self.check("STANDBY", E.FACE_LOST, "LOST")

    def test_lost_to_standby_on_face_back(self):
        self.check("LOST", E.FACE_SEEN, "STANDBY")

    def test_lost_to_idle_on_timeout_event(self):
        self.check("LOST", E.TIMEOUT, "IDLE")

    def test_standby_to_ack_on_point(self):
        self.check("STANDBY", E.POINT, "ACK")

    def test_ack_to_search_on_clip_done(self):
        self.check("ACK", E.CLIP_DONE, "SEARCH")

    def test_search_to_standby_on_palm(self):
        self.check("SEARCH", E.PALM, "STANDBY")

    def test_search_to_aim_on_book(self):
        self.check("SEARCH", E.BOOK_FOUND, "AIM")

    def test_search_to_not_found_on_sweep_done(self):
        self.check("SEARCH", E.SWEEP_DONE, "NOT_FOUND")

    def test_not_found_to_standby_on_clip_done(self):
        self.check("NOT_FOUND", E.CLIP_DONE, "STANDBY")

    def test_aim_to_illuminate_lands_in_hold(self):
        r = self.check("AIM", E.AT_POSE, "HOLD")
        self.assertEqual(r.m.path, "ILLUMINATE/HOLD")
        self.assertEqual(r.calls, [("exit", "AIM"), ("enter", "ILLUMINATE"), ("enter", "HOLD")])

    def test_goodbye_to_standby_when_face_visible(self):
        self.check("GOODBYE", E.CLIP_DONE, "STANDBY", face_visible=True)

    def test_goodbye_to_idle_when_no_face(self):
        self.check("GOODBYE", E.CLIP_DONE, "IDLE", face_visible=False)

    # inside ILLUMINATE
    def test_hold_to_adjust_closer_and_further(self):
        for event in (E.CLOSER, E.FURTHER):
            r = self.check("HOLD", event, "ADJUST")
            self.assertEqual(r.m.ctx["adjust"], event)
            self.assertEqual(r.calls, [("exit", "HOLD"), ("enter", "ADJUST")])   # parent not re-entered

    def test_adjust_to_hold_on_move_done(self):
        self.check("ADJUST", E.MOVE_DONE, "HOLD")

    def test_hold_to_checkin_and_back(self):
        self.check("HOLD", E.CHECKIN_DUE, "CHECKIN")
        self.check("CHECKIN", E.CLIP_DONE, "HOLD")

    def test_hold_to_reacquire_on_book_lost(self):
        self.check("HOLD", E.BOOK_LOST, "REACQUIRE")

    def test_reacquire_to_hold_on_book_found(self):
        self.check("REACQUIRE", E.BOOK_FOUND, "HOLD")

    def test_reacquire_to_not_found_on_timeout_event(self):
        r = self.check("REACQUIRE", E.TIMEOUT, "NOT_FOUND")
        self.assertEqual(r.calls, [("exit", "REACQUIRE"), ("exit", "ILLUMINATE"), ("enter", "NOT_FOUND")])

    def test_palm_and_nobody_end_illuminate_from_every_substate(self):
        for sub in ("HOLD", "ADJUST", "CHECKIN", "REACQUIRE"):
            for event in (E.PALM, E.NOBODY):
                r = self.check(sub, event, "GOODBYE")
                self.assertIn(("exit", "ILLUMINATE"), r.calls)

    def test_illuminate_substates_ignore_outer_only_events(self):
        for sub in ("HOLD", "ADJUST", "CHECKIN", "REACQUIRE"):
            r = Rig().goto(sub)
            for event in (E.FACE_LOST, E.POINT, E.SWEEP_DONE, E.NO_MOTION):
                self.assertIsNone(r.ev(event), (sub, event))


class Faults(unittest.TestCase):
    def test_fault_from_every_state(self):
        for state in LEAF_STATES:
            r = Rig().goto(state)
            tr = r.ev(E.FAULT)
            self.assertEqual((tr.src, tr.dst), (state, "FAULT"), state)
            self.assertIsNone(r.m.timer_at, state)            # no stale timer survives into FAULT

    def test_fault_from_inside_illuminate_exits_the_parent(self):
        for sub in ("HOLD", "ADJUST", "CHECKIN", "REACQUIRE"):
            r = Rig().goto(sub)
            r.ev(E.FAULT)
            self.assertEqual(r.calls, [("exit", sub), ("exit", "ILLUMINATE"), ("enter", "FAULT")], sub)
            self.assertEqual(r.m.path, "FAULT")
            # back through BOOT, and a later ILLUMINATE enters the parent again
            r.ev(E.RECOVERED)
            self.assertEqual(r.m.state, "BOOT")
            r.calls.clear()
            for event in PATH["HOLD"]:
                r.ev(event)
            self.assertEqual(r.calls.count(("enter", "ILLUMINATE")), 1, sub)

    def test_fault_to_boot_on_recovered(self):
        r = Rig().goto("FAULT")
        tr = r.ev(E.RECOVERED)
        self.assertEqual((tr.src, tr.dst), ("FAULT", "BOOT"))
        self.assertEqual(r.m.timer_at, r.t + C.BOOT_TIMEOUT_S)   # BOOT's homing watchdog is re-armed

    def test_second_fault_stays_in_fault(self):
        r = Rig().goto("FAULT")
        tr = r.ev(E.FAULT)
        self.assertEqual((tr.src, tr.dst), ("FAULT", "FAULT"))
        self.assertEqual(r.calls, [("exit", "FAULT"), ("enter", "FAULT")])   # Director counts this as fault #2

    def test_fault_ignores_everything_but_recovered(self):
        r = Rig().goto("FAULT")
        for event in (E.READY, E.FACE_SEEN, E.MOTION, E.PALM, E.CLIP_DONE, E.TIMEOUT):
            self.assertIsNone(r.ev(event), event)

    @_xfail_without_reset
    def test_reset_clears_a_latched_fault(self):
        """Spec: 'second fault, stays latched until reset'. Before e989877 (state-machine-fixes)
        there was no RESET event and no (FAULT, RESET) transition: expected failure there."""
        reset = getattr(E, "RESET", "RESET")
        r = Rig().goto("FAULT")
        r.ev(E.FAULT)                                  # latched
        tr = r.ev(reset)
        self.assertIsNotNone(tr, "FAULT ignores RESET")
        self.assertEqual(r.m.state, "BOOT")


class Timers(unittest.TestCase):
    """State timers driven by Machine.tick with a fake clock."""

    def test_every_timer_event_has_an_edge(self):
        for name, spec in STATES.items():
            if spec.timer:
                event = spec.timer[1]
                keys = [(name, event), (spec.parent, event), ("*", event)]
                self.assertTrue(any(k in TRANSITIONS for k in keys), f"{name} timer {event} goes nowhere")

    def test_boot_timer(self):
        r = Rig()
        self.assertEqual(r.wait(C.BOOT_TIMEOUT_S - 0.1), [])
        fired = r.wait(0.2)
        self.assertEqual([(t.src, t.dst, t.event) for t in fired], [("BOOT", "FAULT", E.TIMEOUT)])

    def test_boot_timer_cancelled_by_ready(self):
        r = Rig().goto("IDLE")
        self.assertEqual(r.wait(C.BOOT_TIMEOUT_S * 2), [])

    def test_notice_timer(self):
        r = Rig().goto("NOTICE")
        self.assertEqual(r.wait(C.NOTICE_TIMEOUT_S - 0.1), [])
        self.assertEqual([t.dst for t in r.wait(0.2)], ["IDLE"])

    def test_lost_timer_and_cancel(self):
        r = Rig().goto("LOST")
        self.assertEqual(r.wait(C.LOST_TIMEOUT_S - 0.1), [])
        self.assertEqual([(t.dst, t.event) for t in r.wait(0.2)], [("IDLE", E.TIMEOUT)])
        r = Rig().goto("LOST")
        r.wait(C.LOST_TIMEOUT_S / 2)
        r.ev(E.FACE_SEEN)
        self.assertEqual(r.wait(C.LOST_TIMEOUT_S), [])     # STANDBY has no timer
        self.assertEqual(r.m.state, "STANDBY")

    def test_lost_timer_restarts_on_each_visit(self):
        r = Rig().goto("LOST")
        r.wait(C.LOST_TIMEOUT_S - 1)
        r.ev(E.FACE_SEEN)
        r.ev(E.FACE_LOST)
        self.assertEqual(r.wait(C.LOST_TIMEOUT_S - 0.2), [])
        self.assertEqual(r.m.state, "LOST")

    def test_checkin_timer_in_range(self):
        lo, hi = C.CHECKIN_EVERY_S
        for seed in range(20):
            r = Rig(seed).goto("HOLD")
            fired = r.wait(hi + 0.1)
            self.assertEqual(len(fired), 1, seed)
            self.assertEqual(fired[0].dst, "CHECKIN")
            self.assertTrue(lo - 0.06 <= fired[0].t - r.m.history[-2].t <= hi + 0.06, seed)

    def test_checkin_timer_rearms_after_adjust(self):
        lo, _ = C.CHECKIN_EVERY_S
        r = Rig().goto("HOLD")
        r.wait(lo - 1)
        r.ev(E.CLOSER)
        r.ev(E.MOVE_DONE)
        self.assertEqual(r.wait(lo - 0.5), [])      # the clock started again on re-entering HOLD

    def test_checkin_has_no_timer_and_waits_for_clip(self):
        r = Rig().goto("CHECKIN")
        self.assertEqual(r.wait(60), [])

    def test_reacquire_timer(self):
        r = Rig().goto("REACQUIRE")
        self.assertEqual(r.wait(C.REACQUIRE_TIMEOUT_S - 0.1), [])
        fired = r.wait(0.2)
        self.assertEqual([(t.src, t.dst) for t in fired], [("REACQUIRE", "NOT_FOUND")])
        self.assertEqual(r.m.path, "NOT_FOUND")

    def test_reacquire_timer_cancelled_by_book(self):
        r = Rig().goto("REACQUIRE")
        r.wait(C.REACQUIRE_TIMEOUT_S / 2)
        r.ev(E.BOOK_FOUND)
        lo, _ = C.CHECKIN_EVERY_S
        self.assertEqual(r.wait(lo - 0.5), [])
        self.assertEqual(r.m.state, "HOLD")

    def test_fault_timer_none_in_machine(self):
        """FAULT's cool-down lives in the Director (it emits RECOVERED), not in a machine timer."""
        r = Rig().goto("FAULT")
        self.assertEqual(r.wait(C.FAULT_COOLDOWN_S * 3), [])


if __name__ == "__main__":
    unittest.main()
