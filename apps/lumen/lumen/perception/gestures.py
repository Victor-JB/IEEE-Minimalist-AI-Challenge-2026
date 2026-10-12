"""Static gestures from 21 MediaPipe hand landmarks, by rules (no model needed).

Landmark ids: 0 wrist; thumb 1-4; index 5-8; middle 9-12; ring 13-16; pinky 17-20.
A finger is extended when its tip is farther from the wrist than its middle joint.
Image y grows downward. If rules prove flaky, swap in a tiny MLP on the 63 values
(the distillation rung of the optimization ladder) behind the same classify() signature.
"""
import math

POINT, THUMBS_UP, THUMBS_DOWN, OPEN_PALM, NONE = "point", "thumbs_up", "thumbs_down", "open_palm", None
FINGERS = {"index": (6, 8), "middle": (10, 12), "ring": (14, 16), "pinky": (18, 20)}


def _d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def classify(lm):
    """One hand's 21 landmarks -> gesture name or None."""
    if not lm or len(lm) != 21:
        return NONE
    wrist = lm[0]
    ext = {f: _d(lm[tip], wrist) > _d(lm[pip], wrist) * 1.1 for f, (pip, tip) in FINGERS.items()}
    thumb_out = _d(lm[4], lm[5]) > _d(lm[3], lm[5]) * 1.2 and _d(lm[4], wrist) > _d(lm[2], wrist)
    n = sum(ext.values())
    if n == 4 and thumb_out:
        return OPEN_PALM
    if ext["index"] and n == 1:
        return POINT
    if n == 0 and thumb_out:
        hand = _d(wrist, lm[9])                       # hand size, for a scale-free threshold
        dy = lm[4][1] - lm[5][1]
        if dy < -0.5 * hand:
            return THUMBS_UP
        if dy > 0.5 * hand:
            return THUMBS_DOWN
    return NONE
