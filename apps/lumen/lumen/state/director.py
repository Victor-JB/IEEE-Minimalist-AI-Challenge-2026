"""The director turns states into motion and light: on_enter picks what the arm does, tick()
keeps looping pools going and steers the look-at, poll_events() turns arm reports into events.
"""
import random

from ..light import config as light_cfg
from ..movement import config as move_cfg
from . import config as state_cfg
from .. import events as E
from .machine import INITIAL_CHILD, STATES
from ..movement import modifiers as M
from ..movement import aim as G
from ..movement.look_at import LookAt, face_error
import kinematics as K                 # arm/python/kinematics.py (on sys.path via config -> paths)
# Every movement number used here (pauses between clips, SLEEP/wake clips, CHECK-IN, REACQUIRE,
# mirror) is in movement/config.py; the state timers are in state/config.py.
from ..movement.config import (CENTER_TIMEOUT_S, CHECKIN_DEFAULT_UP, CHECKIN_LOOK_S,  # noqa: E402
                               CHECKIN_MAX_UP, CAMERA_HALF_VFOV_DEG, FALSE_ALARM_CLIP, GAP_S, MIRROR_S,
                               REACQUIRE_DWELL_S, REACQUIRE_PEEKS, SLEEP_BREATHE, SLEEP_BREATHE_S,
                               SLEEP_ENTRY, WAKE_AMPLITUDE, WAKE_CLIP, WAKE_WINDOW_S)
from ..ai.book.config import BOOK_PER_POSE   # noqa: E402

ONE_SHOT = {"NOTICE", "ACK", "NOT_FOUND", "CHECKIN", "GOODBYE", "FAULT"}   # CLIP_DONE ends the state
LOOK_AT = {"NOTICE", "STANDBY", "LOST", "CHECKIN"}                          # follow the target face
CENTER_FIRST = {"NOTICE", "STANDBY"}   # center the face before any clip plays here (move_cfg.CENTER_TIMEOUT_S)
INTERACTIONS = {E.POINT, E.CLOSER, E.FURTHER, E.PALM, E.FACE_CONFIRMED, E.WAVE}


class Director:
    def __init__(self, worker, light, expressive, rng=None, power=None):
        self.w, self.light, self.x, self.power = worker, light, expressive, power
        self.rng = rng or random.Random()
        self.token, self.busy, self.next_clip_at = 0, False, 0.0
        self.faults, self.last_fault_t, self.fault_reason = 0, -1e9, None
        self.aim_pose = list(move_cfg.HOME)        # (shoulder, elbow, head[, roll]) the light holds at
        self.aim_target = None              # desk point (m, arm frame) the light is on
        self.look, self.last_tick = LookAt(), None
        self.slumped = False                # SLEEP left the arm at go_to_sleep's end pose
        self.wake_tok = None                # token of a wake-up still playing (it may outlive NOTICE)
        self.user_pitch = None              # camera pitch (sum of joints) that last centered the face
        self.checkin_phase = None           # CHECK-IN plan: "up" -> "look" -> "back"
        self.mirror_until, self.last_wave_t = 0.0, None
        self.sweep_plan, self.view, self.pause_frames = None, None, 0
        self.perception, self.frame_wh = None, (640, 480)

    # -- state machine callbacks ------------------------------------------------
    def on_enter(self, state, m):
        spec = STATES[state]
        if spec.light is not None:
            self.light.set(spec.light)
        if state == "CHECKIN":              # LightCoupling: dim while looking at the user
            self.light.couple(light_cfg.LIGHT_CHECKIN_SCALE)
        if state in INITIAL_CHILD:          # composite (ILLUMINATE): its child does the work
            return
        if state not in LOOK_AT:            # NOTICE -> STANDBY -> LOST keep the face centered
            self.look.reset()
            self.w.set_offset(self.look.offset)
        self.token += 1
        self.busy = True
        tok = self.token
        if self._enter_behavior(state, m, tok):   # per-state behaviors below, kept out of this chain
            return
        if state == "BOOT":
            self.w.connect()                # new arm session if a fault closed the last one
            self.w.set_base(move_cfg.HOME)
            self.w.goto(move_cfg.HOME, move_cfg.BOOT_HOME_S, tok)
        elif state == "SEARCH":
            self.view, self.pause_frames = None, 0
            self.w.sweep(self._sweep_poses(), dwell=move_cfg.SWEEP_DWELL_S, token=tok)
        elif state == "AIM":
            self.aim_pose, self.aim_target = self._plan_aim()
            dur = G.move_time(self._current(), self.aim_pose, 1.0)
            self.w.goto(self.aim_pose, dur, tok, done=E.AT_POSE)
        elif state == "HOLD":
            self.w.set_base(self.aim_pose)
            self.busy = False
        elif state == "ADJUST":
            sign = 1 if m.ctx.get("adjust") == E.CLOSER else -1
            self.light.step(light_cfg.LIGHT_ADJUST_STEP * sign)
            self.w.path(self._adjust_path(sign * move_cfg.ADJUST_STEP_M), tok)
        elif state == "CHECKIN":
            self._enter_checkin(tok)
        elif state == "REACQUIRE":
            self._enter_reacquire(tok)
        elif state == "FAULT":
            if m.entered_at - self.last_fault_t > state_cfg.FAULT_FORGET_S:
                self.faults = 0
            self.faults += 1
            self.last_fault_t = m.entered_at
            self.w.relax()
        elif state in CENTER_FIRST:         # hold still and center the face; tick() starts the clips
            self.w.set_base(move_cfg.HOME)
            self.busy = False
        elif state == "IDLE":
            self.w.set_base(move_cfg.HOME)
            self._play(spec.pool, tok)
        else:
            self._play(spec.pool, tok)

    def on_exit(self, state, m):
        if state == "CHECKIN":
            self.light.couple(1.0)
        elif state == "SLEEP" and self.power:
            self.power.wake()
        elif state == "REACQUIRE":
            self._exit_reacquire()

    # -- per frame ---------------------------------------------------------------
    def tick(self, now, m, perception, frame_shape):
        self.x.update(now)
        self.perception, self.frame_wh = perception, (frame_shape[1], frame_shape[0])
        dt, self.last_tick = (0.0 if self.last_tick is None else min(0.2, now - self.last_tick)), now
        # Steer only between clips: a clip moves the head (and camera) itself.
        mirroring = self._mirror(m, perception, frame_shape, now, dt)
        if m.state in LOOK_AT and not self.busy and not mirroring:
            h, w = frame_shape[:2]
            err = face_error(perception.target, w, h) if perception.target is not None else None
            self.w.set_offset(self.look.update(err, now, dt))
            self._note_user_pitch(perception, err)
        self._steer_checkin(m, perception, frame_shape, now, dt)
        pool = m.spec.pool
        waiting = (m.state in CENTER_FIRST and not self.look.centered(now)
                   and now - m.entered_at < CENTER_TIMEOUT_S)
        if (not self.busy and not waiting and m.state not in ONE_SHOT and pool in GAP_S
                and now >= self.next_clip_at):
            self.token += 1
            self.busy = True
            self._loop_clip(pool, self.token)

    def poll_events(self, now, m, perception=None):
        """Arm reports -> machine events. Stale reports from interrupted commands are dropped."""
        out = []
        for event, token, info in self.w.poll_events():
            if event == E.FAULT:
                self.fault_reason = info
                out.append(E.FAULT)
                continue
            if token is not None and token == self.wake_tok:
                self.wake_tok = None
                if token != self.token:            # the wake-up outlived NOTICE (now STANDBY): done
                    self.busy = False
                    continue
            if token != self.token:
                continue
            if event == E.STEP:                        # a mark inside one of our plans
                self._on_step(m, info, now)
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
        if m.state == "FAULT" and self.faults <= 1 and now - m.entered_at > state_cfg.FAULT_COOLDOWN_S:
            if not getattr(self.w, "hot", False):   # OVERTEMP: and cooled below OVERTEMP_CLEAR_C
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

    def _loop_clip(self, pool, tok):
        """The next clip of a looping pool."""
        if self.slumped and pool == "bored":         # woken by motion, nobody came: get up
            self._wake_up(tok)
        elif pool == "asleep":                          # a very slow, shallow breathe (IdleBreathe)
            amp, ts = SLEEP_BREATHE
            self.w.seq(M.idle_breathe(SLEEP_BREATHE_S, self.rng, 0.0, amp, ts), tok)
        else:
            self._play(pool, tok)

    # -- per-state behaviors ------------------------------------------------------------
    def _enter_behavior(self, state, m, tok):
        """States (or cases) handled here instead of on_enter's chain. True = handled."""
        carry = self.wake_tok is not None and state in CENTER_FIRST
        if not carry:
            self.wake_tok = None                      # whatever plays next interrupts it
        if state == "BOOT":
            self.slumped = False
        if state == "SLEEP":
            self._enter_sleep(tok)
            return True
        if state == "IDLE":
            return self._enter_idle(m, tok)
        if state in CENTER_FIRST and carry:           # let the wake-up finish, then center the face
            self.w.set_base(move_cfg.HOME)
            return True
        if state == "NOTICE" and (self.slumped or self._woke_recently(m)):
            self._wake_up(tok)
            return True
        return False

    def _woke_recently(self, m):
        """SLEEP -> IDLE -> here, with IDLE lasting at most WAKE_WINDOW_S."""
        idle, sleep = m.came_from(1), m.came_from(2)
        return (idle is not None and idle[0] == "IDLE" and sleep is not None and sleep[0] == "SLEEP"
                and idle[2] - idle[1] <= WAKE_WINDOW_S)

    def _enter_idle(self, m, tok):
        prev = m.came_from()
        if prev and prev[0] == "SLEEP" and self.slumped:
            # Motion woke it: stay slumped a moment. A face now gets the groggy wake-up in NOTICE;
            # nobody by WAKE_WINDOW_S and the IDLE loop wakes up anyway (_loop_clip).
            self.busy = False
            self.next_clip_at = m.entered_at + WAKE_WINDOW_S
            return True
        if prev and prev[0] == "NOTICE" and prev[3] == E.TIMEOUT:
            self.w.set_base(move_cfg.HOME)
            self._play_ref(FALSE_ALARM_CLIP, tok)    # then the bored loop, after its gap
            return True
        return False

    def _wake_up(self, tok):
        """The groggy wake-up from the slumped pose; rebases on its end (home)."""
        self.slumped, self.wake_tok = False, tok
        self.w.seq([("play", WAKE_CLIP, WAKE_AMPLITUDE, 1.0), ("rebase",)], tok)

    # -- mirror mode (STANDBY) ------------------------------------------------------------
    def _mirror(self, m, perception, frame_shape, now, dt):
        """A wave in STANDBY: play from the "mirror" pool and let the head follow the hand for
        MIRROR_S (the energy bump comes from E.WAVE in INTERACTIONS). True while mirroring.
        Pitch only: hand x is ignored until the yaw joint exists."""
        if m.state != "STANDBY":
            self.mirror_until = 0.0
            return False
        wave_t = getattr(perception, "wave_t", None)
        if wave_t is not None and wave_t != self.last_wave_t and now - wave_t < 0.5:   # a fresh one
            self.last_wave_t = wave_t
            self.mirror_until = now + MIRROR_S
            self.token += 1
            self.busy = True
            self._play("mirror", self.token)
        if now >= self.mirror_until:
            return False
        hand = perception.hand_lm[0] if getattr(perception, "hand_lm", None) else None
        err = None
        if hand:
            h, w = frame_shape[:2]
            cx = sum(p[0] for p in hand) / len(hand)
            cy = sum(p[1] for p in hand) / len(hand)
            err = (2 * cx / w - 1, 2 * cy / h - 1)
        self.w.set_offset(self.look.update(err, now, dt))      # the worker keeps it during the clip
        return True

    # -- CHECK-IN -----------------------------------------------------------------------
    def _note_user_pitch(self, perception, err):
        """Remember where the face is (camera pitch that centers it), for CHECK-IN later."""
        if err is None or not getattr(perception, "faces", True):
            return
        self.user_pitch = sum(self.w.base) + self.look.head + err[1] * CAMERA_HALF_VFOV_DEG

    def _checkin_pose(self):
        """The aim pose with the head pitched (most of the way) toward the last-seen face."""
        aim = list(self.aim_pose)
        if self.user_pitch is None:
            turn = CHECKIN_DEFAULT_UP
        else:
            turn = self.user_pitch - sum(aim[:3])              # head change that centers the face
        share = min(1.0, M.jitter(self.rng, 0.6 + 0.4 * M.gamma_for(self.x.energy, social=True)))
        turn = max(-CHECKIN_MAX_UP, min(CHECKIN_MAX_UP, share * turn))
        return M.clamp_pose([aim[0], aim[1], aim[2] + turn] + aim[3:])   # (keeps a roll, if any)

    def _enter_checkin(self, tok):
        """Up toward the user, mark 0 (track the face), hold, mark 1, glide back to the aim pose
        (clearing the look-at offset). CLIP_DONE at the end returns to HOLD."""
        self.checkin_phase = "up"
        look = self._checkin_pose()
        plan = [("goto", look, M.jitter(self.rng, 0.5)), ("mark", 0), ("wait", CHECKIN_LOOK_S),
                ("mark", 1), ("goto", list(self.aim_pose), M.jitter(self.rng, 0.6), True)]
        self.w.seq(M.respect_caps(plan, self._arm_pose()), tok)

    def _checkin_step(self, index):
        self.look.reset()
        self.w.set_offset(self.look.offset)
        self.checkin_phase = "look" if index == 0 else "back"

    def _steer_checkin(self, m, perception, frame_shape, now, dt):
        """While looking up: center the face with the look-at, and refine where the user is."""
        if m.state != "CHECKIN":
            self.checkin_phase = None
            return
        if self.checkin_phase != "look":
            return
        h, w = frame_shape[:2]
        err = face_error(perception.target, w, h) if perception.target is not None else None
        self.w.set_offset(self.look.update(err, now, dt))
        self._note_user_pitch(perception, err)

    # -- REACQUIRE ----------------------------------------------------------------------
    def reacquire_plan(self, start):
        """Peek around the aim pose, each peek wider; the first one hesitant (it's a guess)."""
        aim = list(self.aim_pose)
        peeks = [M.clamp_pose([a + d for a, d in zip(aim, off)] + aim[3:]) for off in REACQUIRE_PEEKS]
        gamma = M.gamma_for(self.x.energy)
        plan = M.hesitation(start, peeks[0], 0.3, self.rng, gamma) + [("wait", REACQUIRE_DWELL_S)]
        for p in peeks[1:]:                       # 0.3 s asks for "as fast as the caps allow"
            plan += [("goto", p, 0.3), ("wait", REACQUIRE_DWELL_S)]
        return M.respect_caps(plan + [("goto", aim, 0.3)], start)

    def _enter_reacquire(self, tok):
        self.w.seq(self.reacquire_plan(self._arm_pose()), tok, done=E.SWEEP_DONE)

    def _exit_reacquire(self):
        """Found it (or leaving): glide back to the aim pose. Whatever the next state plays
        interrupts this; HOLD just holds there. Token = REACQUIRE's, so its done is dropped."""
        self.w.seq(M.respect_caps([("goto", list(self.aim_pose), 0.5)], self._arm_pose()), self.token,
                   done=E.MOVE_DONE)

    # -- SLEEP ------------------------------------------------------------------------
    def _enter_sleep(self, tok):
        """Save power (camera to SLEEP_CAMERA_FPS, opt-in CPU governor), slump once, then breathe
        slowly around the slumped pose (rebase: the hold must not pull the lamp back up)."""
        if self.power:
            self.power.sleep()
        self.slumped = True
        self.w.seq([("play", SLEEP_ENTRY, 1.0, 1.0), ("rebase",)], tok)

    # -- modifier rules (movement/modifiers.py) ---------------------------------------
    def _arm_pose(self):
        """Where the arm is now (last streamed setpoint), to plan the next move from."""
        return list(getattr(self.w, "current", None) or self.w.base)

    def _on_step(self, m, index, now):
        """A plan reached one of its marks (see the per-state plans below)."""
        if m.state == "CHECKIN":
            self._checkin_step(index)

    def _play_ref(self, ref, tok):
        """One named clip with the usual amplitude / speed noise (not drawn from a pool)."""
        amp = self.rng.uniform(*move_cfg.AMPLITUDE_RANGE)
        ts = self.rng.uniform(*move_cfg.TIME_SCALE_RANGE)
        self.w.play(ref, amp, ts, tok)

    def _current(self):
        return list(getattr(self.w, "current", move_cfg.HOME))

    def _sweep_poses(self):
        """SEARCH pauses near to far with a small J5 left/right look each (movement/aim.py
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
            return [move_cfg.HOME[0] + 5, move_cfg.HOME[1] - 5, move_cfg.HOME[2] + 25], None
        return list(sol.q) + [sol.roll], sol.target

    def _adjust_path(self, delta):
        """CLOSER (+) / FURTHER (-): the light moves delta m along its axis, still on the book,
        clamped to what IK can reach; a small counter-move first (anticipation). [(pose, s)]."""
        q, roll = self.aim_pose[:3], (self.aim_pose[3] if len(self.aim_pose) > 3 else 0.0)
        target = self.aim_target or K.beam_hit(q, roll, move_cfg.BOOK_TOP_Z)
        if target is None:
            return [(list(self.aim_pose), 0.3)]
        self.aim_target = target
        sol, moved = K.move_along_beam(q, roll, target, delta)
        new = list(sol.q) + [sol.roll]
        path = []
        if moved:
            wind, back = K.move_along_beam(q, roll, target, -moved * move_cfg.ANTICIPATION)
            if back:
                path.append((list(wind.q) + [wind.roll], move_cfg.ANTICIPATION_S))
        path.append((new, G.move_time(path[-1][0] if path else self._current(), new, 0.8)))
        self.aim_pose = new
        return path
