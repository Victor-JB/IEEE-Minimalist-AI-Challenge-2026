"""The director turns states into motion and light: on_enter picks what the arm does, tick()
keeps looping pools going and steers the look-at, poll_events() turns arm reports into events.
"""
import random

from .. import config as C
from .. import events as E
from ..machine import INITIAL_CHILD, STATES
from . import modifiers as M
from .look_at import LookAt, face_error

ONE_SHOT = {"NOTICE", "ACK", "NOT_FOUND", "CHECKIN", "GOODBYE", "FAULT"}   # CLIP_DONE ends the state
LOOK_AT = {"NOTICE", "STANDBY", "LOST", "CHECKIN"}                          # follow the target face
GAP_S = {"bored": (2.0, 6.0), "attentive": (1.5, 4.0), "holding": (4.0, 9.0),
         "looking_around": (0.5, 1.5), "asleep": (20.0, 40.0)}               # pause between looping clips
CENTER_FIRST = {"NOTICE", "STANDBY"}   # center the face before any clip plays here
CENTER_TIMEOUT_S = 4.0                 # ...but start the clips anyway after this long
INTERACTIONS = {E.POINT, E.CLOSER, E.FURTHER, E.PALM, E.FACE_CONFIRMED}
ADJUST_STEP = (3.0, -4.0, 1.0)   # (shoulder, elbow, head) deg per CLOSER; TODO: tune on the arm


class Director:
    def __init__(self, worker, light, expressive, rng=None):
        self.w, self.light, self.x = worker, light, expressive
        self.rng = rng or random.Random()
        self.token, self.busy, self.next_clip_at = 0, False, 0.0
        self.faults, self.last_fault_t, self.fault_reason = 0, -1e9, None
        self.aim_pose = list(C.HOME)
        self.look, self.last_tick = LookAt(), None

    # -- state machine callbacks ------------------------------------------------
    def on_enter(self, state, m):
        spec = STATES[state]
        if spec.light is not None:
            self.light.set(spec.light)
        if state in INITIAL_CHILD:          # composite (ILLUMINATE): its child does the work
            return
        if state not in LOOK_AT:            # NOTICE -> STANDBY -> LOST keep the face centered
            self.look.reset()
            self.w.set_offset(self.look.offset)
        self.token += 1
        self.busy = True
        tok = self.token
        if state == "BOOT":
            self.w.connect()                # new arm session if a fault closed the last one
            self.w.set_base(C.HOME)
            self.w.goto(C.HOME, 3.0, tok)
        elif state == "SEARCH":
            self.w.sweep(self._sweep_poses(), dwell=0.6, token=tok)
        elif state == "AIM":
            # TODO(victor): pose that points the light at the book, from the book's position in
            # the image + the camera-to-head offset. For now: lean the head down over the desk.
            self.aim_pose = [C.HOME[0] + 5, C.HOME[1] - 5, C.HOME[2] + 25]
            self._expressive_goto(self.aim_pose, 1.5, tok, E.AT_POSE, "aim")
        elif state == "HOLD":
            self.w.set_base(self.aim_pose)
            self.busy = False
        elif state == "ADJUST":
            sign = 1 if m.ctx.get("adjust") == E.CLOSER else -1
            self.light.step(0.15 * sign)
            self.aim_pose = [a + sign * d for a, d in zip(self.aim_pose, ADJUST_STEP)]
            self._expressive_goto(self.aim_pose, 0.8, tok, E.MOVE_DONE, "adjust")
        elif state == "FAULT":
            if m.entered_at - self.last_fault_t > C.FAULT_FORGET_S:
                self.faults = 0
            self.faults += 1
            self.last_fault_t = m.entered_at
            self.w.relax()
        elif state in CENTER_FIRST:         # hold still and center the face; tick() starts the clips
            self.w.set_base(C.HOME)
            self.busy = False
        elif state in ("IDLE", "SLEEP"):
            self.w.set_base(C.HOME)
            self._play(spec.pool, tok)
        else:
            self._play(spec.pool, tok)

    def on_exit(self, state, m):
        pass

    # -- per frame ---------------------------------------------------------------
    def tick(self, now, m, perception, frame_shape):
        self.x.update(now)
        dt, self.last_tick = (0.0 if self.last_tick is None else min(0.2, now - self.last_tick)), now
        # Steer only between clips: a clip moves the head (and camera) itself.
        if m.state in LOOK_AT and not self.busy:
            h, w = frame_shape[:2]
            err = face_error(perception.target, w, h) if perception.target is not None else None
            self.w.set_offset(self.look.update(err, now, dt))
        pool = m.spec.pool
        waiting = (m.state in CENTER_FIRST and not self.look.centered(now)
                   and now - m.entered_at < CENTER_TIMEOUT_S)
        if (not self.busy and not waiting and m.state not in ONE_SHOT and pool in GAP_S
                and now >= self.next_clip_at):
            self.token += 1
            self.busy = True
            self._play(pool, self.token)
        # TODO(victor): mirror mode in STANDBY: when perception.gesture is a wave, offset the head
        # with the hand's x/y and play from the "mirror" pool.

    def poll_events(self, now, m, perception=None):
        """Arm reports -> machine events. Stale reports from interrupted commands are dropped."""
        out = []
        for event, token, info in self.w.poll_events():
            if event == E.FAULT:
                self.fault_reason = info
                out.append(E.FAULT)
                continue
            if token != self.token:
                continue
            if event == E.STEP:                        # a mark inside one of our plans
                self._on_step(m, info, now)
                continue
            if event == E.AT_POSE and m.state == "SEARCH":
                m.ctx["at_pause_pose"] = True          # lets the book model run this frame
                continue
            self.busy = False
            lo, hi = GAP_S.get(m.spec.pool, (0, 0))
            self.next_clip_at = now + self.rng.uniform(lo, hi)
            if m.state == "BOOT" and event == E.MOVE_DONE:
                out.append(E.READY)
            else:
                out.append(event)
        if m.state == "FAULT" and self.faults <= 1 and now - m.entered_at > C.FAULT_COOLDOWN_S:
            out.append(E.RECOVERED)
        return out

    def note_events(self, events, now):
        if E.RESET in events:
            self.faults = 0                 # the user cleared a latched fault
        if any(e in INTERACTIONS for e in events):
            self.x.bump(now)

    # -- helpers -----------------------------------------------------------------
    def _play(self, pool, tok):
        choice = self.x.pick(pool)
        if choice is None:
            self.busy = False
            return
        ref, amp, ts = choice
        self.w.play(ref, amp, ts, tok)

    # -- modifier rules (behavior/modifiers.py) ---------------------------------------
    def _arm_pose(self):
        """Where the arm is now (last streamed setpoint), to plan the next move from."""
        return list(getattr(self.w, "current", None) or self.w.base)

    def _on_step(self, m, index, now):
        """A plan reached one of its marks (see the per-state plans below)."""

    def _expressive_goto(self, pose, duration, tok, done, style):
        """goto dressed by the modifier rules (glance / windup / hesitation), inside the caps."""
        plan = M.expressive_move(self._arm_pose(), pose, duration, self.rng,
                                 M.gamma_for(self.x.energy), style)
        self.w.seq(plan, tok, done=done)

    def _sweep_poses(self):
        """3-5 pauses across the desk. TODO: needs the 4th (yaw) joint for a real left-right sweep;
        with pitch only, this sweeps near-to-far."""
        base = list(C.HOME)
        return [[base[0] + s, base[1] - 10 + e, base[2] + 30] for s, e in ((-5, 10), (0, 0), (5, -10))]
