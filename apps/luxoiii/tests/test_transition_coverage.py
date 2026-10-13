"""Every (state, event) key in machine.TRANSITIONS is exercised by at least one test.

Runs the machine test modules with Machine._lookup / Machine._go instrumented, then checks
that each key fired (and each possible target of a function-valued key was reached).
A new transition without a test fails here with a readable list.
"""
import importlib
import io
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from luxoiii import machine as M        # noqa: E402

MODULES = ["test_machine", "test_machine_edges", "test_simulate"]   # missing ones are skipped
_hits = None


def record_coverage():
    """{(key, event): set(targets reached)} from running MODULES once (cached)."""
    global _hits
    if _hits is not None:
        return _hits
    hits, last_key = {}, {}
    orig_lookup, orig_go = M.Machine._lookup, M.Machine._go

    def lookup(self, event):
        last_key[id(self)] = None
        for key in (self.state, self.spec.parent, "*"):
            if key and (key, event) in M.TRANSITIONS:
                last_key[id(self)] = (key, event)
                break
        return orig_lookup(self, event)

    def go(self, target, event, now):
        key = last_key.pop(id(self), None)
        if key:
            hits.setdefault(key, set()).add(target)
        return orig_go(self, target, event, now)

    suite = unittest.TestSuite()
    for name in MODULES:
        try:
            suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(importlib.import_module(name)))
        except ImportError:
            pass
    with mock.patch.object(M.Machine, "_lookup", lookup), mock.patch.object(M.Machine, "_go", go):
        result = unittest.TextTestRunner(stream=io.StringIO(), verbosity=0).run(suite)
    if result.failures or result.errors:
        raise AssertionError(f"machine tests failed while measuring coverage: "
                             f"{[str(t) for t, _ in result.failures + result.errors]}")
    _hits = hits
    return hits


def possible_targets(dst):
    if not callable(dst):
        return {dst}
    out = set()
    for face in (True, False):
        m = M.Machine.__new__(M.Machine)
        m.ctx = {"face_visible": face}
        out.add(dst(m))
    return out


class TransitionCoverage(unittest.TestCase):
    def test_every_transition_key_is_exercised(self):
        hits = record_coverage()
        missing = [f"{s} --{e}-->" for (s, e) in M.TRANSITIONS if (s, e) not in hits]
        self.assertEqual(missing, [], "no test drives: " + ", ".join(missing))

    def test_every_target_of_a_function_transition_is_reached(self):
        hits = record_coverage()
        missing = []
        for key, dst in M.TRANSITIONS.items():
            for target in possible_targets(dst) - hits.get(key, set()):
                missing.append(f"{key[0]} --{key[1]}--> {target}")
        self.assertEqual(missing, [], "never reached: " + ", ".join(missing))

    def test_wildcard_fault_is_driven_from_every_leaf_state(self):
        """("*", FAULT) is one key, but each state's exit path differs (ILLUMINATE's parent exit)."""
        from test_machine_edges import LEAF_STATES, Rig
        from luxoiii import events as E
        for state in LEAF_STATES:
            r = Rig().goto(state)
            self.assertEqual(r.ev(E.FAULT).dst, "FAULT", state)


if __name__ == "__main__":
    unittest.main()
