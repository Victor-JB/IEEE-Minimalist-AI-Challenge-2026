"""Perception: frames in, debounced events out. The state machine never sees raw detections."""
from .. import config as C
from .. import events as E
from .gestures import classify, POINT as G_POINT, THUMBS_UP, THUMBS_DOWN, OPEN_PALM
from .scheduler import ModelScheduler


class Perception:
    def __init__(self, face=None, hands=None, book=None, gate=None):
        self.face, self.hands, self.book, self.gate = face, hands, book, gate
        self.sched = ModelScheduler()
        self.faces, self.target, self.hand_lm, self.gesture, self.books = [], None, [], None, []
        self.face_seen = E.KofN(C.FACE_SEEN_K, C.FACE_SEEN_N)
        self.face_confirm = E.KofN(C.FACE_CONFIRM_K, C.FACE_CONFIRM_N)
        self.face_absent = E.AbsentFor(C.FACE_LOST_S)
        self.no_motion = E.AbsentFor(C.NO_MOTION_SLEEP_S)
        self.nobody = E.AbsentFor(C.NOBODY_S)
        self.book_absent = E.AbsentFor(C.BOOK_LOST_S)
        self.book_hits = 0
        self.last_motion_t = -1e9
        self.held = {G_POINT: E.HeldFor(C.GESTURE_HOLD_S),
                     THUMBS_UP: E.HeldFor(C.GESTURE_HOLD_S, C.GESTURE_REPEAT_S),
                     THUMBS_DOWN: E.HeldFor(C.GESTURE_HOLD_S, C.GESTURE_REPEAT_S),
                     OPEN_PALM: E.HeldFor(C.PALM_HOLD_S)}
        self.gesture_event = {G_POINT: E.POINT, THUMBS_UP: E.CLOSER, THUMBS_DOWN: E.FURTHER, OPEN_PALM: E.PALM}

    def step(self, frame, now, rates, ctx):
        """Run the models the state asks for on this frame; return a list of events."""
        out = []
        ran = self.sched.due(rates, now, ctx.get("at_pause_pose", False))
        moving = False
        if "gate" in ran and self.gate:
            moving = self.gate.update(frame)
            if moving:
                self.last_motion_t = now
                out.append(E.MOTION)
            if self.no_motion.update(moving, now):
                out.append(E.NO_MOTION)
        if "face" in ran and rates.get("gate") and now - self.last_motion_t > 1.0:
            ran = [m for m in ran if m != "face"]               # gated: nothing moved for 1 s, skip the face model

        if "face" in ran and self.face:
            self.faces = self.face.detect(frame, now)
            self.target = self.face.target
            seen = bool(self.faces)
            ctx["face_visible"] = seen
            if self.face_seen.update(seen) and seen:
                out.append(E.FACE_SEEN)
            if self.face_confirm.update(seen):
                out.append(E.FACE_CONFIRMED)
            if self.face_absent.update(seen, now):
                out.append(E.FACE_LOST)

        if "hand" in ran and self.hands:
            self.hand_lm = self.hands.detect(frame, ctx.get("hand_roi"))
            self.gesture = classify(self.hand_lm[0]) if self.hand_lm else None
            for g, deb in self.held.items():
                if deb.update(self.gesture == g, now):
                    out.append(self.gesture_event[g])

        if "book" in ran and self.book:
            self.books = self.book.detect(frame, ctx.get("book_roi"))
            self.book_hits = self.book_hits + 1 if self.books else 0
            if self.book_hits == C.BOOK_FOUND_CONSECUTIVE:
                out.append(E.BOOK_FOUND)
            if self.book_absent.update(bool(self.books), now):
                out.append(E.BOOK_LOST)

        anyone = bool(self.faces) or bool(self.hand_lm) or moving
        if self.nobody.update(anyone, now):
            out.append(E.NOBODY)
        return out

    def on_state_change(self):
        """Reset debouncers so a state never inherits a half-counted event from the last one."""
        self.face_confirm.reset()
        self.faces, self.hand_lm, self.books, self.book_hits = [], [], [], 0
        for deb in self.held.values():
            deb.update(False, 0)
