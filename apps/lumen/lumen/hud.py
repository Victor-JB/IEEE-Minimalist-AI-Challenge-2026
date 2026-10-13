"""Demo HUD on the MJPEG stream: state, active models, inference rates, light, energy.

The deck's demo slide promises this is on screen the whole time."""
import cv2

GREEN, AMBER, WHITE = (80, 220, 80), (58, 162, 240), (235, 235, 235)


def fault_lines(reason, width=46):
    """The FAULT reason ('CODE: detail') wrapped into at most two short HUD lines."""
    text = f"fault {reason}"
    return [text[:width]] + ([text[width:2 * width].strip()] if len(text) > width else [])


def draw(frame, m, perception, light, energy, fps, watts=None, fault=None):
    for d in perception.faces:
        color = AMBER if d is perception.target else GREEN
        cv2.rectangle(frame, (d.x, d.y), (d.x + d.w, d.y + d.h), color, 2)
    for (x, y, w, h, *_ ) in perception.books:
        cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), (240, 160, 60), 2)
    models = " ".join(f"{k}:{v}" for k, v in m.models.items()) or "none"
    lines = [f"STATE {m.path}", f"models {models}", f"light {light.level:.0%}  energy {energy:.2f}  {fps:.0f} fps"]
    if perception.gesture:
        lines.append(f"gesture {perception.gesture}")
    if watts is not None:
        lines.append(f"{watts:.2f} W")
    if fault:
        lines += fault_lines(fault)
    for i, text in enumerate(lines):
        cv2.putText(frame, text, (10, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(frame, text, (10, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, WHITE if i else AMBER, 2)
