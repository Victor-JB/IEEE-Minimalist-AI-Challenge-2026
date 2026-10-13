"""Demo HUD on the MJPEG stream: state, where it can go next, what the arm is doing, active
models, light, energy.

The deck's demo slide promises this is on screen the whole time."""
import time

import cv2

from . import thinking
from .perception.hands import CONNECTIONS

GREEN, AMBER, WHITE = (80, 220, 80), (58, 162, 240), (235, 235, 235)


def fault_lines(reason, width=46):
    """The FAULT reason ('CODE: detail') wrapped into at most two short HUD lines."""
    text = f"fault {reason}"
    return [text[:width]] + ([text[width:2 * width].strip()] if len(text) > width else [])


def draw_hands(frame, hands, label=None):
    """Skeleton of each hand (21 landmarks in frame pixels), label above the first one."""
    for i, lm in enumerate(hands):
        pts = [(int(x), int(y)) for x, y, *_ in lm]
        for a, b in CONNECTIONS:
            cv2.line(frame, pts[a], pts[b], GREEN, 2)
        for p in pts:
            cv2.circle(frame, p, 3, AMBER, -1)
        if label and i == 0:
            x, y = min(p[0] for p in pts), max(min(p[1] for p in pts) - 10, 20)
            cv2.putText(frame, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(frame, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, AMBER, 2)


def draw(frame, m, perception, light, energy, fps, watts=None, fault=None, now=None, doing=None):
    for d in perception.faces:
        color = AMBER if d is perception.target else GREEN
        cv2.rectangle(frame, (d.x, d.y), (d.x + d.w, d.y + d.h), color, 2)
    for (x, y, w, h, *_ ) in perception.books:
        cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), (240, 160, 60), 2)
    roi = m.ctx.get("hand_roi")
    if roi is not None and m.models.get("hand"):
        x, y, w, h = (int(v) for v in roi[:4])
        cv2.rectangle(frame, (x, y), (x + w, y + h), WHITE, 1)          # where the palm detector looks
    draw_hands(frame, perception.hand_lm, perception.gesture or "hand")
    models = " ".join(f"{k}:{v}" for k, v in m.models.items()) or "none"
    lines = thinking.lines(m, time.monotonic() if now is None else now, doing)
    lines += [f"models {models}", f"light {light.level:.0%}  energy {energy:.2f}  {fps:.0f} fps"]
    if perception.gesture:
        lines.append(f"gesture {perception.gesture}")
    if watts is not None:
        lines.append(f"{watts:.2f} W")
    if fault:
        lines += fault_lines(fault)
    for i, text in enumerate(lines):
        cv2.putText(frame, text, (10, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(frame, text, (10, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, WHITE if i else AMBER, 2)
