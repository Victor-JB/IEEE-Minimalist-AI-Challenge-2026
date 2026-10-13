"""The v1 mermaid diagrams in docs/STATE_MACHINE.md and machine.TRANSITIONS must agree.

Strict on state names and edges (src -> dst) in both directions. Edge labels are prose
("no face 4 s"), so they are mapped to events loosely and mismatches are only reported:
run this file directly to print the report, or see the skip reason with -v.

    python3 apps/luxoiii/tests/test_state_diagram.py
"""
import re
import sys
import unittest
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
from luxoiii import events as E                                         # noqa: E402
from luxoiii.machine import INITIAL_CHILD, Machine, STATES, TRANSITIONS  # noqa: E402

DOC = APP.parents[1] / "docs" / "STATE_MACHINE.md"
ALIAS = {"NOTFOUND": "NOT_FOUND", "REACQ": "REACQUIRE", "ILLUM": "ILLUMINATE", "ANY": "*"}
EDGE = re.compile(r"^\s*(\[\*\]|\w+)\s*-->\s*(\w+)\s*(?::\s*(.*?))?\s*$")
COMPOSITE = re.compile(r"^\s*state\s+(\w+)\s*\{\s*$")

# Loose label -> events. First match wins, so the specific phrases come first.
LABEL_RULES = [
    (r"stall|sensor_stale|overtemp|link_lost|second fault", {E.FAULT}),
    (r"cooled|retry", {E.RECOVERED}),
    (r"^reset", {getattr(E, "RESET", "RESET")}),
    (r"homed|sensors ok", {E.READY}),
    (r"no motion", {E.NO_MOTION}),
    (r"^motion$", {E.MOTION}),
    (r"nobody", {E.NOBODY}),
    (r"every .*s|random", {E.CHECKIN_DUE}),
    (r"not found in|gone within", {E.TIMEOUT}),
    (r"no face", {E.FACE_LOST, E.TIMEOUT}),
    (r"face confirmed", {E.FACE_CONFIRMED}),
    (r"face 3 of 5|face back|face seen|face visible", {E.FACE_SEEN}),
    (r"point", {E.POINT}),
    (r"closer|further", {E.CLOSER, E.FURTHER}),
    (r"stop|exit|open palm", {E.PALM}),
    (r"book lost", {E.BOOK_LOST}),
    (r"book found|book \d+ frames", {E.BOOK_FOUND}),
    (r"sweep done", {E.SWEEP_DONE}),
    (r"move done", {E.MOVE_DONE}),
    (r"clip done|nod done|glance done", {E.CLIP_DONE}),
    (r"over book|at pose", {E.AT_POSE}),
]


def label_events(label):
    for pattern, events in LABEL_RULES:
        if re.search(pattern, label or "", re.I):
            return events
    return set()


def v1_edges(text=None):
    """[(src, dst, label, parent)] from every mermaid block after the '## v1' heading."""
    text = DOC.read_text() if text is None else text
    v1 = text[text.index("## v1"):]
    edges = []
    for block in re.findall(r"```mermaid\n(.*?)```", v1, re.S):
        parents = []
        for line in block.splitlines():
            if COMPOSITE.match(line):
                parents.append(COMPOSITE.match(line).group(1))
            elif line.strip() == "}":
                parents.pop()
            elif (m := EDGE.match(line)):
                src, dst, label = m.groups()
                edges.append((ALIAS.get(src, src), ALIAS.get(dst, dst), label or "",
                              parents[-1] if parents else None))
    return edges


def _targets(dst):
    """A transition's possible targets (a function target is tried with and without a face)."""
    if not callable(dst):
        return {dst}
    out = set()
    for face in (True, False):
        m = Machine.__new__(Machine)
        m.ctx = {"face_visible": face}
        out.add(dst(m))
    return out


def machine_edges():
    """{(src, dst): {events}} from TRANSITIONS."""
    out = {}
    for (src, event), dst in TRANSITIONS.items():
        for target in _targets(dst):
            out.setdefault((src, target), set()).add(event)
    return out


def resolved_events(src, dst):
    """Events that take `src` to `dst`, with the machine's lookup order (state, parent, "*")."""
    parent = STATES[src].parent if src in STATES else None
    out = set()
    for event in {e for _, e in TRANSITIONS}:
        for key in (src, parent, "*"):
            if key and (key, event) in TRANSITIONS:
                if dst in _targets(TRANSITIONS[(key, event)]):
                    out.add(event)
                break
    return out


def compare(edges=None):
    """(strict problems, label notes): lists of readable strings."""
    edges = v1_edges() if edges is None else edges
    code = machine_edges()
    problems, notes = [], []
    seen = set()
    for src, dst, label, parent in edges:
        for name in (src, dst):
            if name not in STATES and name not in ("[*]", "*"):
                problems.append(f"diagram state {name!r} is not in machine.STATES")
        if src == "[*]":
            want = INITIAL_CHILD.get(parent) if parent else Machine.__init__.__defaults__[2]
            if want != dst:
                problems.append(f"diagram: initial state of {parent or 'the machine'} is {dst}, code says {want}")
            continue
        seen.add((src, dst))
        events = code.get((src, dst)) or resolved_events(src, dst)
        if not events:
            problems.append(f"diagram edge {src} --> {dst} ({label!r}) has no transition in TRANSITIONS")
            continue
        guessed = label_events(label)
        if not guessed:
            notes.append(f"{src} --> {dst}: label {label!r} maps to no event (code: {', '.join(sorted(events))})")
        elif not guessed & events:
            notes.append(f"{src} --> {dst}: label {label!r} reads as {', '.join(sorted(guessed))}, "
                         f"code fires on {', '.join(sorted(events))}")
    any_dst = {dst for src, dst in seen if src == "*"}
    for (src, dst), events in sorted(code.items()):
        if (src, dst) not in seen and dst not in any_dst:
            problems.append(f"TRANSITIONS {src} --{'/'.join(sorted(events))}--> {dst} is not drawn in the v1 diagram")
    return problems, notes


class DiagramMatchesCode(unittest.TestCase):
    def test_parser_found_the_v1_diagrams(self):
        edges = v1_edges()
        self.assertGreater(len(edges), 30)
        self.assertIn(("REACQUIRE", "NOT_FOUND", "not found in 4 s", None), edges)
        self.assertIn(("HOLD", "ADJUST", "CLOSER / FURTHER", "ILLUMINATE"), edges)

    def test_every_diagram_state_exists(self):
        names = {n for s, d, _, _ in v1_edges() for n in (s, d)} - {"[*]", "*"}
        self.assertEqual(sorted(names - set(STATES)), [])
        # every state is drawn (ILLUMINATE's substates included)
        self.assertEqual(sorted(set(STATES) - names), [])

    def test_edges_agree(self):
        problems, _ = compare()
        self.assertEqual(problems, [], "\n" + "\n".join(problems))

    def test_compare_catches_a_missing_edge(self):
        edges = v1_edges() + [("SLEEP", "STANDBY", "magic", None)]
        problems, _ = compare(edges)
        self.assertEqual(len(problems), 1)
        self.assertIn("SLEEP --> STANDBY", problems[0])

    def test_labels_loosely_match(self):
        _, notes = compare()
        if notes:
            self.skipTest("label notes (not enforced): " + " | ".join(notes))


if __name__ == "__main__":
    problems, notes = compare()
    print("Edge problems (strict):", *(problems or ["none"]), sep="\n  ")
    print("Label notes (loose):", *(notes or ["none"]), sep="\n  ")
    unittest.main()
