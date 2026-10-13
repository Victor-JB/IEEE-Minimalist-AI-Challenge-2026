"""Perception: frames in, debounced events out. The state machine never sees raw detections."""
from .book import config as book_cfg
from . import config as ai_cfg
from .hands import config as hands_cfg
from .. import events as E
from .hands.gestures import classify, hand_size, WaveDetector, POINT as G_POINT, THUMBS_UP, THUMBS_DOWN, OPEN_PALM
from .scheduler import ModelScheduler

from .config import BOOK_ROI_FROM_BOOK, HAND_ROI_FROM_BOOK, HAND_ROI_FROM_FACE   # edit in ai/config.py


def grow(box, margins, shape):
    """(x, y, w, h) box grown by margins (in box widths/heights), clamped to the frame."""
    x, y, w, h = box[:4]
    fh, fw = shape[:2]
    x0, y0 = max(0, int(x - margins[0] * w)), max(0, int(y - margins[1] * h))
    x1, y1 = min(fw, int(x + w + margins[2] * w)), min(fh, int(y + h + margins[3] * h))
    return (x0, y0, x1 - x0, y1 - y0) if x1 > x0 and y1 > y0 else None


class Perception:
    def __init__(self, face=None, hands=None, book=None, gate=None):
        self.face, self.hands, self.book, self.gate = face, hands, book, gate
        self.sched = ModelScheduler()
        self.faces, self.target, self.hand_lm, self.gesture, self.books = [], None, [], None, []
        self.face_seen = E.KofN(ai_cfg.FACE_SEEN_K, ai_cfg.FACE_SEEN_N)
        self.face_confirm = E.KofN(ai_cfg.FACE_CONFIRM_K, ai_cfg.FACE_CONFIRM_N)
        self.face_absent = E.AbsentFor(ai_cfg.FACE_LOST_S)
        self.no_motion = E.AbsentFor(ai_cfg.NO_MOTION_SLEEP_S)
        self.nobody = E.AbsentFor(ai_cfg.NOBODY_S)
        self.book_hits = self.book_misses = 0
        self.book_last_seen = None
        self.last_motion_t = -1e9
        self.held = {G_POINT: E.HeldFor(hands_cfg.GESTURE_HOLD_S),
                     THUMBS_UP: E.HeldFor(hands_cfg.GESTURE_HOLD_S, hands_cfg.GESTURE_REPEAT_S),
                     THUMBS_DOWN: E.HeldFor(hands_cfg.GESTURE_HOLD_S, hands_cfg.GESTURE_REPEAT_S),
                     OPEN_PALM: E.HeldFor(hands_cfg.PALM_HOLD_S)}
        self.reset_palm = E.HeldFor(hands_cfg.RESET_HOLD_S)
        self.wave = WaveDetector(hands_cfg.WAVE_REVERSALS, hands_cfg.WAVE_WINDOW_S, hands_cfg.WAVE_SWING)
        self.wave_t = None                  # when the last wave was detected
        self.gesture_event = {G_POINT: E.POINT, THUMBS_UP: E.CLOSER, THUMBS_DOWN: E.FURTHER, OPEN_PALM: E.PALM}

    def step(self, frame, now, rates, ctx):
        """Run the models the state asks for on this frame; return a list of events."""
        out = []
        if self.book_misses and isinstance(rates.get("book"), (int, float)):
            rates = {**rates, "book": max(rates["book"], book_cfg.BOOK_CONFIRM_HZ)}   # confirm a miss quickly
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
            self.sched.cancel("face")                           # so the logged count is real inferences

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
            if self.reset_palm.update(self.gesture == OPEN_PALM, now):
                out.append(E.RESET)
            lm = self.hand_lm[0] if self.hand_lm and len(self.hand_lm[0]) == 21 else None
            if self.wave.update(lm[0][0] if lm else None, hand_size(lm) if lm else 0.0, now):
                self.wave_t = now
                out.append(E.WAVE)

        if "book" in ran and self.book:
            self.books = self.book.detect(frame, ctx.get("book_roi"))
            if self.books:
                self.book_hits, self.book_misses, self.book_last_seen = self.book_hits + 1, 0, now
                self.last_book = max(self.books, key=lambda b: b[4])
            else:
                self.book_hits = 0
                self.book_misses += 1
            if self.book_hits == book_cfg.BOOK_FOUND_CONSECUTIVE:
                out.append(E.BOOK_FOUND)
            # Lost = several quick misses in a row AND gone long enough; one missed 0.5 Hz check isn't.
            if (self.book_misses == book_cfg.BOOK_LOST_MISSES and self.book_last_seen is not None
                    and now - self.book_last_seen >= book_cfg.BOOK_LOST_S):
                out.append(E.BOOK_LOST)

        anyone = bool(self.faces) or bool(self.hand_lm) or moving
        if self.nobody.update(anyone, now):
            out.append(E.NOBODY)
        self._set_rois(frame.shape, ctx)     # crops for the next frame, from this frame's results
        return out

    def _set_rois(self, shape, ctx):
        """Where to look for hands and the book: near the target face, or near the last book."""
        face = self.target if self.faces else None
        book = getattr(self, "last_book", None)
        if face is not None:
            ctx["hand_roi"] = grow((face.x, face.y, face.w, face.h), HAND_ROI_FROM_FACE, shape)
        elif book is not None:
            ctx["hand_roi"] = grow(book, HAND_ROI_FROM_BOOK, shape)
        else:
            ctx["hand_roi"] = None
        # A missed check widens to the full frame (REACQUIRE); a hit crops back down.
        ctx["book_roi"] = grow(book, BOOK_ROI_FROM_BOOK, shape) if book is not None and not self.book_misses else None

    def on_state_change(self, ctx=None):
        """Reset debouncers so a state never inherits a half-counted event from the last one."""
        if ctx is not None:
            ctx["face_visible"] = False      # stale: only the new state's own face runs count
        self.face_confirm.reset()
        self.wave.reset()
        self.faces, self.hand_lm, self.books, self.book_hits, self.book_misses = [], [], [], 0, 0
        for deb in (*self.held.values(), self.reset_palm):
            deb.update(False, 0)
