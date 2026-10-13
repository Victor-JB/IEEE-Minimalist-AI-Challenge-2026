"""Headless full-loop simulation of the demo, no camera, models or arm.

Drives the real Machine + Director + Perception (debouncers, model scheduler, gesture rules)
with fake detectors that read a scripted "world" (is a face there, which hand pose, is the
book visible, is anything moving), a fake motion worker (clips/moves finish after a set time),
and a fake clock. It prints the state path and writes a transitions CSV in the same format
as the app's lumen/translog.py, so tools/scorecard.py can be tried before the board is back.

    python3 apps/lumen/tools/simulate.py                         # print the path
    python3 apps/lumen/tools/simulate.py --csv sim.csv --power-csv sim_power.csv
    python3 apps/lumen/tools/scorecard.py sim.csv --power sim_power.csv --idle-watts 2.0

--power-csv writes a SYNTHETIC power trace (a toy model, see synthetic_watts) only to
exercise the scorecard's power path. Never put those numbers in RESULTS.md.
"""
import argparse
import csv
import inspect
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
if str(APP) not in sys.path:
    sys.path.insert(0, str(APP))

from lumen import events as E                      # noqa: E402
from lumen.behavior.director import Director       # noqa: E402
from lumen.behavior.expressive import Expressive   # noqa: E402
from lumen.io.light import Light                   # noqa: E402
from lumen.machine import Machine                  # noqa: E402
from lumen.perception import Perception            # noqa: E402
from lumen.perception import gestures as G         # noqa: E402
from lumen.translog import TransitionLog           # noqa: E402

FPS = 15.0                 # about what the board's camera loop does
FRAME_SHAPE = (480, 640, 3)
CLIP_S = 1.6               # every fake clip lasts this long (divided by its time_scale)
SWEEP_MOVE_S = 1.0         # same as the real worker's move between sweep pauses

# The 5-minute demo (docs/DEMO_MEASUREMENT.md), then leave / sleep / fault / reset.
# (t seconds, what the person or the rig does, world changes or an action)
SCENARIO = [
    (0.0,   "power on (BOOT homes the arm)", {}),
    (8.0,   "walk up and sit at the desk", dict(face=True, motion=True)),
    (16.0,  "point: find my book (no book on the desk yet)", dict(gesture=G.POINT)),
    (17.0,  "hand down", dict(gesture=None)),
    (30.0,  "put the book on the desk", dict(book=True)),
    (32.0,  "point again", dict(gesture=G.POINT)),
    (33.0,  "hand down", dict(gesture=None)),
    (48.0,  "thumbs up: closer", dict(gesture=G.THUMBS_UP)),
    (48.8,  "hand down", dict(gesture=None)),
    (56.0,  "thumbs down: further", dict(gesture=G.THUMBS_DOWN)),
    (56.8,  "hand down", dict(gesture=None)),
    (70.0,  "cover the book with a hand", dict(book=False)),
    (76.0,  "uncover it (long enough to be lost even if a check-in interrupts)", dict(book=True)),
    (95.0,  "open palm 1 s: done", dict(gesture=G.OPEN_PALM)),
    (96.5,  "hand down", dict(gesture=None)),
    (105.0, "walk away", dict(face=False, motion=False, book=False)),
    (430.0, "someone walks past (wakes from SLEEP)", dict(motion=True)),
    (432.0, "gone", dict(motion=False)),
    (450.0, "arm stall (worker reports FAULT)", "fault"),
    (480.0, "arm stall again within 2 min (latched)", "fault"),
    (500.0, "operator reset: open palm held 3 s", dict(gesture=G.OPEN_PALM)),
    (504.0, "hand down (restart Lumen if the palm did not clear the latch)", "restart_if_latched"),
    (520.0, "end", {}),
]


class Clock:
    def __init__(self, t=0.0):
        self.t = t


@dataclass
class World:
    face: bool = False
    gesture: str = None
    book: bool = False
    motion: bool = False


@dataclass
class Det:
    x: float
    y: float
    w: float
    h: float
    score: float = 0.9

    @property
    def center(self):
        return (self.x + self.w / 2, self.y + self.h / 2)


def _hand(gesture):
    """21 landmarks (x, y, z) that gestures.classify() reads as `gesture`. Wrist at the origin,
    fingers up (image y grows downward)."""
    lm = [(0.0, 0.0, 0.0)] * 21
    xs = {"index": -15, "middle": 0, "ring": 15, "pinky": 30}
    ids = {"index": (5, 6, 7, 8), "middle": (9, 10, 11, 12), "ring": (13, 14, 15, 16), "pinky": (17, 18, 19, 20)}
    extended = {G.OPEN_PALM: set(xs), G.POINT: {"index"}}.get(gesture, set())
    for f, (mcp, pip, dip, tip) in ids.items():
        x = xs[f]
        lm[mcp], lm[pip] = (x, -30, 0), (x, -40, 0)
        lm[dip], lm[tip] = ((x, -60, 0), (x, -80, 0)) if f in extended else ((x, -30, 0), (x, -20, 0))
    thumb = {G.OPEN_PALM: [(-10, -5), (-25, -10), (-35, -15), (-50, -20)],
             G.THUMBS_UP: [(-10, -5), (-20, -25), (-25, -45), (-25, -65)],
             G.THUMBS_DOWN: [(-10, -5), (-20, -15), (-30, 0), (-30, 15)]}.get(
        gesture, [(-5, -5), (-10, -15), (-12, -22), (-10, -25)])        # tucked
    for i, (x, y) in enumerate(thumb, start=1):
        lm[i] = (x, y, 0)
    return [(float(x), float(y), float(z)) for x, y, z in lm]


HANDS = {g: _hand(g) for g in (G.POINT, G.THUMBS_UP, G.THUMBS_DOWN, G.OPEN_PALM)}


class FakeFace:
    def __init__(self, world):
        self.world, self.target, self.calls = world, None, 0

    def detect(self, frame, now):
        self.calls += 1
        faces = [Det(270, 190, 100, 100)] if self.world.face else []   # centered: look-at settles
        self.target = faces[0] if faces else None
        return faces


class FakeHands:
    def __init__(self, world):
        self.world, self.calls = world, 0

    def detect(self, frame, roi=None):
        self.calls += 1
        return [HANDS[self.world.gesture]] if self.world.gesture else []


class FakeBook:
    def __init__(self, world):
        self.world, self.calls = world, 0

    def detect(self, frame, roi=None):
        self.calls += 1
        return [(300, 300, 120, 160, 0.8)] if self.world.book else []


class FakeGate:
    def __init__(self, world):
        self.world, self.calls = world, 0

    def update(self, frame):
        self.calls += 1
        return self.world.motion


class FakeWorker:
    """MotionWorker's command API with scripted completion times on the fake clock.
    A new command interrupts the previous one (its pending reports are dropped)."""

    def __init__(self, clock):
        self.clock, self.pending, self.base, self.offset = clock, [], None, (0, 0, 0)
        self.commands = []

    def _replace(self, kind, events):
        self.commands.append((self.clock.t, kind))
        self.pending = [p for p in self.pending if p[1] == E.FAULT] + events

    def play(self, ref, amplitude, time_scale, token):
        self._replace("play " + ref, [(self.clock.t + CLIP_S / time_scale, E.CLIP_DONE, token, ref)])

    def goto(self, pose, duration, token, done=E.MOVE_DONE):
        self._replace("goto", [(self.clock.t + duration, done, token, None)])

    def path(self, segments, token, done=E.MOVE_DONE):
        self._replace("path", [(self.clock.t + sum(d for _, d in segments), done, token, None)])

    def sweep(self, poses, dwell, token):
        t, evs = self.clock.t, []
        for i, _ in enumerate(poses):
            t += SWEEP_MOVE_S
            evs.append((t, E.AT_POSE, token, i))
            t += dwell
        evs.append((t, E.SWEEP_DONE, token, None))
        self._replace("sweep", evs)

    def relax(self):
        self._replace("relax", [])

    def connect(self):
        self.commands.append((self.clock.t, "connect"))

    def set_base(self, pose):
        self.base = list(pose)

    def set_offset(self, offset):
        self.offset = tuple(offset)

    def fault(self, reason):
        self.pending.append((self.clock.t, E.FAULT, None, reason))

    def poll_events(self):
        due = sorted((p for p in self.pending if p[0] <= self.clock.t), key=lambda p: p[0])
        self.pending = [p for p in self.pending if p[0] > self.clock.t]
        return [(event, token, info) for _, event, token, info in due]


# Toy power model for --power-csv (made-up numbers, only to exercise the scorecard).
SYN_IDLE_W, SYN_APP_W, SYN_LIGHT_W = 2.0, 0.3, 0.8
SYN_MJ = {"face": 90.0, "hand": 40.0, "book": 150.0, "gate": 1.0}


def synthetic_watts(light_level, calls_in_window, window_s):
    return (SYN_IDLE_W + SYN_APP_W + SYN_LIGHT_W * light_level
            + sum(SYN_MJ[k] * n for k, n in calls_in_window.items()) / 1000 / window_s)


@dataclass
class Result:
    transitions: list           # (t, src, event, dst path)
    steps: list                 # (t, label)
    counts: dict                # scheduler counts (what the app logs)
    calls: dict                 # actual detector calls
    faults: int


def simulate(scenario=SCENARIO, csv_path=None, power_csv=None, seed=0, fps=FPS, start_wall=None,
             out=print):
    clock, world = Clock(), World()
    rng = random.Random(seed)
    start_wall = float(int(time.time() if start_wall is None else start_wall))
    expressive = Expressive(random.Random(rng.random()), now=0.0)
    worker, light = FakeWorker(clock), Light()
    det = dict(face=FakeFace(world), hand=FakeHands(world), book=FakeBook(world), gate=FakeGate(world))
    perception = Perception(face=det["face"], hands=det["hand"], book=det["book"], gate=det["gate"])
    log = TransitionLog(csv_path) if csv_path else None
    power = None
    if power_csv:
        pf = open(power_csv, "w", newline="")
        power = csv.writer(pf)
        power.writerow(["timestamp", "watts"])
    transitions, steps = [], []
    _takes_ctx = bool(inspect.signature(perception.on_state_change).parameters)

    def boot():
        director = Director(worker, light, expressive, rng=random.Random(rng.random()))

        def entered(state, m):
            director.on_enter(state, m)
            if _takes_ctx:
                perception.on_state_change(m.ctx)
            else:
                perception.on_state_change()
        machine = Machine(on_enter=entered, on_exit=director.on_exit, now=clock.t,
                          rng=random.Random(rng.random()))
        return director, machine

    def record(tr, machine, director):
        transitions.append((tr.t, tr.src, tr.event, machine.path))
        if log:
            log.write(tr, machine.models, expressive.energy, perception.sched.counts, wall=start_wall + tr.t)
        out(f"{tr.t:8.2f}  {tr.src:>10} --{tr.event}--> {machine.path}")
        if tr.dst == "FAULT" and director.faults > 1:
            out(f"{'':8}  FAULT #{director.faults}: latched")

    director, machine = boot()
    script = sorted(scenario, key=lambda s: s[0])
    end_t = script[-1][0]
    frame = type("Frame", (), {"shape": FRAME_SHAPE})()
    dt = 1.0 / fps
    n, last_sample, last_calls = 0, 0.0, {k: 0 for k in det}
    i = 0
    while clock.t <= end_t:
        now = clock.t
        while i < len(script) and script[i][0] <= now:
            t_s, label, change = script[i]
            steps.append((t_s, label))
            out(f"{t_s:8.2f}  # {label}")
            if change == "fault":
                worker.fault("simulated STALL")
            elif change == "restart_if_latched":
                world.gesture = None
                if machine.state == "FAULT":   # no RESET palm in this version: the operator restarts Lumen
                    old = machine.state
                    director, machine = boot()
                    tr = type("Tr", (), dict(t=now, src=old, dst=machine.state, event="RESTART"))()
                    record(tr, machine, director)
            else:
                for k, v in change.items():
                    setattr(world, k, v)
            i += 1

        # the app's main loop, one frame (lumen/app.py run())
        events = perception.step(frame, now, machine.models, machine.ctx)
        machine.ctx["at_pause_pose"] = False
        events += director.poll_events(now, machine, perception)
        director.note_events(events, now)
        for tr in [machine.handle(ev, now) for ev in events] + [machine.tick(now)]:
            if tr:
                record(tr, machine, director)
        director.tick(now, machine, perception, frame.shape)

        if power and now - last_sample >= 1.0 - 1e-9:
            calls = {k: d.calls - last_calls[k] for k, d in det.items()}
            last_calls = {k: d.calls for k, d in det.items()}
            power.writerow([f"{start_wall + now:.3f}", f"{synthetic_watts(light.level, calls, now - last_sample):.3f}"])
            last_sample = now
        n += 1
        clock.t = n * dt
    if log:
        log.close()
    if power:
        pf.close()
    res = Result(transitions, steps, dict(perception.sched.counts), {k: d.calls for k, d in det.items()},
                 director.faults)
    out("path: " + " > ".join(["BOOT"] + [p for _, _, _, p in transitions]))
    out(f"inferences logged (scheduler): {res.counts}")
    out(f"detector calls (actual):       {res.calls}")
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", help="write the transitions CSV here (overwritten)")
    ap.add_argument("--power-csv", help="write a SYNTHETIC timestamp,watts trace here")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--fps", type=float, default=FPS)
    ap.add_argument("--quiet", action="store_true", help="only print the path summary")
    a = ap.parse_args(argv)
    if a.csv and Path(a.csv).exists():
        Path(a.csv).unlink()
    printer = (lambda s: s.startswith(("path:", "inferences", "detector")) and print(s)) if a.quiet else print
    simulate(csv_path=a.csv, power_csv=a.power_csv, seed=a.seed, fps=a.fps, out=printer)


if __name__ == "__main__":
    main()
