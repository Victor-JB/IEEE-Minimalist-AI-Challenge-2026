"""The director turns states into motion and light: on_enter picks what the arm does, tick()
keeps looping pools going and steers the look-at, poll_events() turns arm reports into events.
"""
import random

from .. import config as C
from .. import events as E
from ..machine import INITIAL_CHILD, STATES
from . import aim as G
from .look_at import LookAt, face_error
import kinematics as K                 # arm/python/kinematics.py (on sys.path via config -> paths)

ONE_SHOT = {"NOTICE", "ACK", "NOT_FOUND", "CHECKIN", "GOODBYE", "FAULT"}   # CLIP_DONE ends the state
LOOK_AT = {"NOTICE", "STANDBY", "LOST", "CHECKIN"}                          # follow the target face
GAP_S = {"bored": (2.0, 6.0), "attentive": (1.5, 4.0), "holding": (4.0, 9.0),
         "looking_around": (0.5, 1.5), "asleep": (20.0, 40.0)}               # pause between looping clips
CENTER_FIRST = {"NOTICE", "STANDBY"}   # center the face before any clip plays here
CENTER_TIMEOUT_S = 4.0                 # ...but start the clips anyway after this long
INTERACTIONS = {E.POINT, E.CLOSER, E.FURTHER, E.PALM, E.FACE_CONFIRMED}
BOOK_PER_POSE = 2                # book inferences per settled sweep pause (BOOK_FOUND needs 2 in a row)


class Director:
    def __init__(self, worker, light, expressive, rng=None):
        self.w, self.light, self.x = worker, light, expressive
        self.rng = rng or random.Random()
        self.token, self.busy, self.next_clip_at = 0, False, 0.0
        self.faults, self.last_fault_t, self.fault_reason = 0, -1e9, None
        self.aim_pose = list(C.HOME)        # (shoulder, elbow, head[, roll]) the light holds at
        self.aim_target = None              # desk point (m, arm frame) the light is on
        self.look, self.last_tick = LookAt(), None
        self.sweep_plan, self.view, self.pause_frames = None, None, 0
        self.perception, self.frame_wh = None, (640, 480)

    # -- state machine callbacks ------------------------------------------------
    def on_enter(self, state, m):
        spec = STATES[state]
        if spec.light is not None:
            self.light.set(spec.light)
        if state == "CHECKIN":              # LightCoupling: dim while looking at the user
            self.light.couple(C.LIGHT_CHECKIN_SCALE)
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
            self.view, self.pause_frames = None, 0
            self.w.sweep(self._sweep_poses(), dwell=0.6, token=tok)
        elif state == "AIM":
            self.aim_pose, self.aim_target = self._plan_aim()
            dur = G.move_time(self._current(), self.aim_pose, 1.0)
            self.w.goto(self.aim_pose, dur, tok, done=E.AT_POSE)
        elif state == "HOLD":
            self.w.set_base(self.aim_pose)
            self.busy = False
        elif state == "ADJUST":
            sign = 1 if m.ctx.get("adjust") == E.CLOSER else -1
            self.light.step(0.15 * sign)
            self.w.path(self._adjust_path(sign * C.ADJUST_STEP_M), tok)
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
        if state == "CHECKIN":
            self.light.couple(1.0)

    # -- per frame ---------------------------------------------------------------
    def tick(self, now, m, perception, frame_shape):
        self.x.update(now)
        self.perception, self.frame_wh = perception, (frame_shape[1], frame_shape[0])
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
            if event == E.AT_POSE and m.state == "SEARCH":
                # The worker only reports a pause once the arm has settled there: the book model
                # may run on the next BOOK_PER_POSE frames, and AIM knows where the camera was.
                plan = self.sweep_plan or []
                self.view = plan[info] if isinstance(info, int) and info < len(plan) else None
                self.pause_frames = BOOK_PER_POSE
                continue
            self.busy = False
            lo, hi = GAP_S.get(m.spec.pool, (0, 0))
            self.next_clip_at = now + self.rng.uniform(lo, hi)
            if m.state == "BOOT" and event == E.MOVE_DONE:
                out.append(E.READY)
            else:
                out.append(event)
        if self.pause_frames and m.state == "SEARCH":
            m.ctx["at_pause_pose"] = True              # lets the book model run on the next frame
            self.pause_frames -= 1
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

    def _current(self):
        return list(getattr(self.w, "current", C.HOME))

    def _sweep_poses(self):
        """SEARCH pauses near to far with a small J5 left/right look each (behavior/aim.py
        sweep_poses: FK + camera FOV give the coverage). Planned once: it only depends on config.
        Yaw seam: aim.sweep_poses is where yaw columns go once that joint exists."""
        if self.sweep_plan is None:
            self.sweep_plan = G.sweep_poses()
        return [list(q) + [r] for q, r in self.sweep_plan]

    def _plan_aim(self):
        """AIM pose: the book's centre in the image, seen from the settled sweep pause, through the
        camera model onto the desk, then IK to light it from a comfortable height.
        Returns (pose with roll, desk target or None)."""
        view = self.view or (tuple(self._current()[:3]), getattr(self.w, "base_roll", 0.0))
        book = getattr(self.perception, "last_book", None)
        sol = None
        if book is not None:
            sol = G.aim_at_book(view[0], view[1], book, self.frame_wh, ref=view[0])
        if sol is None:                     # no usable box: light where the camera was looking
            centre = G.desk_point(view[0], view[1], self.frame_wh[0] / 2, self.frame_wh[1] / 2, *self.frame_wh)
            sol = K.aim(centre, ref=view[0]) if centre is not None else None
        if sol is None:                     # nothing reachable: lean the head down over the desk
            return [C.HOME[0] + 5, C.HOME[1] - 5, C.HOME[2] + 25], None
        return list(sol.q) + [sol.roll], sol.target

    def _adjust_path(self, delta):
        """CLOSER (+) / FURTHER (-): the light moves delta m along its axis, still on the book,
        clamped to what IK can reach; a small counter-move first (anticipation). [(pose, s)]."""
        q, roll = self.aim_pose[:3], (self.aim_pose[3] if len(self.aim_pose) > 3 else 0.0)
        target = self.aim_target or K.beam_hit(q, roll, C.BOOK_TOP_Z)
        if target is None:
            return [(list(self.aim_pose), 0.3)]
        self.aim_target = target
        sol, moved = K.move_along_beam(q, roll, target, delta)
        new = list(sol.q) + [sol.roll]
        path = []
        if moved:
            wind, back = K.move_along_beam(q, roll, target, -moved * C.ANTICIPATION)
            if back:
                path.append((list(wind.q) + [wind.roll], C.ANTICIPATION_S))
        path.append((new, G.move_time(path[-1][0] if path else self._current(), new, 0.8)))
        self.aim_pose = new
        return path
