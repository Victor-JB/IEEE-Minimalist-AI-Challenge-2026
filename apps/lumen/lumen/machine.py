"""The behavior state machine from docs/STATE_MACHINE.md (v1), as data.

STATES says what each state runs (models + rates) and which animation pool it draws from.
TRANSITIONS says which event moves it where. Change behavior here, not in the main loop.

Pure logic: no camera, models or arm, so tests/test_machine.py can drive it with a fake clock.
"""
import random
from dataclasses import dataclass, field

from . import config as C
from . import events as E


@dataclass(frozen=True)
class State:
    models: dict                 # model name -> Hz, or "per_pose" (run when the sweep pauses)
    pool: str                    # animation pool in behavior/pools.py
    parent: str = None           # sub-states of ILLUMINATE
    timer: tuple = None          # (seconds | (lo, hi), event) fired after that long in the state
    light: float = None          # lamp brightness on enter (None = leave as is)


STATES = {
    "BOOT":      State({}, "boot"),
    "SLEEP":     State({"gate": 2}, "asleep", light=0.0),
    "IDLE":      State({"gate": 10, "face": 1}, "bored", light=0.0),
    "NOTICE":    State({"face": 10}, "notice", timer=(C.NOTICE_TIMEOUT_S, E.TIMEOUT)),
    "STANDBY":   State({"face": 5, "hand": 10}, "attentive"),
    "LOST":      State({"face": 5}, "looking_around", timer=(C.LOST_TIMEOUT_S, E.TIMEOUT)),
    "ACK":       State({}, "acknowledge"),
    "SEARCH":    State({"book": "per_pose", "hand": 2}, "searching"),
    "AIM":       State({"book": 3}, "found"),
    "NOT_FOUND": State({}, "quizzical"),
    # ILLUMINATE is a composite: entering it lands in HOLD.
    "ILLUMINATE": State({}, "holding", light=1.0),
    "HOLD":      State({"hand": 8, "book": 0.5, "gate": 5}, "holding", parent="ILLUMINATE",
                       timer=(C.CHECKIN_EVERY_S, E.CHECKIN_DUE)),
    "ADJUST":    State({"hand": 8}, "adjust", parent="ILLUMINATE"),
    "CHECKIN":   State({"face": 10}, "checkin", parent="ILLUMINATE"),
    "REACQUIRE": State({"book": 3}, "peek", parent="ILLUMINATE",
                       timer=(C.REACQUIRE_TIMEOUT_S, E.TIMEOUT)),
    "GOODBYE":   State({}, "done", light=0.0),
    "FAULT":     State({}, "fault", light=0.2),
}
INITIAL_CHILD = {"ILLUMINATE": "HOLD"}


def _face_or_idle(m):
    return "STANDBY" if m.ctx.get("face_visible") else "IDLE"


# (state, event) -> target state, or a function of the machine returning one.
# Lookup order: the state itself, then its parent, then "*" (any state).
TRANSITIONS = {
    ("BOOT", E.READY): "IDLE",
    ("IDLE", E.NO_MOTION): "SLEEP",
    ("SLEEP", E.MOTION): "IDLE",
    ("IDLE", E.FACE_SEEN): "NOTICE",
    ("NOTICE", E.FACE_CONFIRMED): "STANDBY",
    ("NOTICE", E.TIMEOUT): "IDLE",
    ("STANDBY", E.FACE_LOST): "LOST",
    ("LOST", E.FACE_SEEN): "STANDBY",
    ("LOST", E.TIMEOUT): "IDLE",
    ("STANDBY", E.POINT): "ACK",
    ("ACK", E.CLIP_DONE): "SEARCH",
    ("SEARCH", E.PALM): "STANDBY",
    ("SEARCH", E.BOOK_FOUND): "AIM",
    ("SEARCH", E.SWEEP_DONE): "NOT_FOUND",
    ("NOT_FOUND", E.CLIP_DONE): "STANDBY",
    ("AIM", E.AT_POSE): "ILLUMINATE",
    ("HOLD", E.CLOSER): "ADJUST",
    ("HOLD", E.FURTHER): "ADJUST",
    ("ADJUST", E.MOVE_DONE): "HOLD",
    ("HOLD", E.CHECKIN_DUE): "CHECKIN",
    ("CHECKIN", E.CLIP_DONE): "HOLD",
    ("HOLD", E.BOOK_LOST): "REACQUIRE",
    ("REACQUIRE", E.BOOK_FOUND): "HOLD",
    ("REACQUIRE", E.TIMEOUT): "NOT_FOUND",
    ("ILLUMINATE", E.PALM): "GOODBYE",
    ("ILLUMINATE", E.NOBODY): "GOODBYE",
    ("GOODBYE", E.CLIP_DONE): _face_or_idle,
    ("*", E.FAULT): "FAULT",
    ("FAULT", E.RECOVERED): "BOOT",
}


@dataclass
class Transition:
    t: float
    src: str
    dst: str
    event: str


class Machine:
    """Handles one event at a time. Callbacks: on_enter(state, machine), on_exit(state, machine)."""

    def __init__(self, on_enter=None, on_exit=None, start="BOOT", now=0.0, rng=None):
        self.on_enter, self.on_exit = on_enter, on_exit
        self.rng = rng or random.Random()
        self.ctx = {"face_visible": False, "adjust": None}
        self.history = []
        self.state, self.entered_at, self.timer_at = None, now, None
        self._enter(start, now)

    # -- queries ---------------------------------------------------------------
    @property
    def spec(self):
        return STATES[self.state]

    @property
    def models(self):
        return self.spec.models

    @property
    def path(self):
        """e.g. 'ILLUMINATE/HOLD' for the HUD."""
        p = self.spec.parent
        return f"{p}/{self.state}" if p else self.state

    # -- driving ---------------------------------------------------------------
    def handle(self, event, now):
        """Apply one event. Returns the Transition, or None if this state ignores it."""
        target = self._lookup(event)
        if target is None:
            return None
        if callable(target):
            target = target(self)
        if event in (E.CLOSER, E.FURTHER):
            self.ctx["adjust"] = event
        return self._go(target, event, now)

    def tick(self, now):
        """Fire the current state's timer if it is due."""
        if self.timer_at is not None and now >= self.timer_at:
            self.timer_at = None
            return self.handle(self.spec.timer[1], now)
        return None

    # -- internals -------------------------------------------------------------
    def _lookup(self, event):
        for key in (self.state, self.spec.parent, "*"):
            if key and (key, event) in TRANSITIONS:
                return TRANSITIONS[(key, event)]
        return None

    def _go(self, target, event, now):
        src = self.state
        target = INITIAL_CHILD.get(target, target)
        old_parent, new_parent = self.spec.parent, STATES[target].parent
        if self.on_exit:
            self.on_exit(src, self)
            if old_parent and old_parent != new_parent:
                self.on_exit(old_parent, self)
        entering_parent = new_parent if new_parent and new_parent != old_parent else None
        self._enter(target, now, entering_parent)
        tr = Transition(now, src, target, event)
        self.history.append(tr)
        return tr

    def _enter(self, name, now, entering_parent=None):
        self.state, self.entered_at = name, now
        timer = STATES[name].timer
        if timer:
            secs = timer[0]
            if isinstance(secs, tuple):
                secs = self.rng.uniform(*secs)
            self.timer_at = now + secs
        else:
            self.timer_at = None
        if self.on_enter:
            if entering_parent:
                self.on_enter(entering_parent, self)
            self.on_enter(name, self)
