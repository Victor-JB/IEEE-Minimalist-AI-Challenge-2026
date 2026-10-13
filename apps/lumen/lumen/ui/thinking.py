"""What Lumen is thinking, as short HUD lines: the state and its timer, where it can go next,
and what the arm is doing. Pure text (no cv2) so it is testable; hud.py draws it."""
from ..state.machine import STATES, TRANSITIONS


def next_moves(m):
    """[(event, target)] the current state can take, most specific first (state, then parent).
    The "*" -> FAULT edge is left out: it exists everywhere."""
    seen, out = set(), []
    for key in (m.state, m.spec.parent):
        for (src, event), target in TRANSITIONS.items():
            if key and src == key and event not in seen:
                seen.add(event)
                if callable(target):
                    target = getattr(target, "label", target.__name__)
                out.append((event, target))
    return out


def timer_left(m, now):
    """(event, seconds) of the state's pending timer, or None."""
    if m.timer_at is None:
        return None
    return STATES[m.state].timer[1], max(0.0, m.timer_at - now)


def lines(m, now, doing=None, width=46):
    """e.g. ["STATE ILLUMINATE/HOLD  3.2s",
             "next: ADJUST (CLOSER/FURTHER) | CHECKIN (in 6.5s)",
             "      REACQUIRE (BOOK_LOST) | GOODBYE (PALM/NOBODY)",
             "arm: clip idle_breathe/calm"]"""
    t = timer_left(m, now)
    why = {}                                  # target -> what takes it there, in table order
    for event, target in next_moves(m):
        label = f"in {t[1]:.1f}s" if t and event == t[0] else event
        why.setdefault(target, []).append(label)
    out, line = [f"STATE {m.path}  {now - m.entered_at:.1f}s"], "next:"
    for target, events in why.items():
        item = f"{target} ({'/'.join(events)})"
        if line.strip() not in ("next:", "") and len(line) + len(item) + 3 > width:
            out.append(line)
            line = "     "
        line += (" | " if line.strip() not in ("next:", "") else " ") + item
    out.append(line)
    out.append(f"arm: {doing or 'hold'}")
    return out
