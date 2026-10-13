"""Event names and the debouncers that turn noisy per-frame detections into clean events."""
from collections import deque

# Perception
FACE_SEEN, FACE_CONFIRMED, FACE_LOST = "FACE_SEEN", "FACE_CONFIRMED", "FACE_LOST"
MOTION, NO_MOTION, NOBODY = "MOTION", "NO_MOTION", "NOBODY"
POINT, CLOSER, FURTHER, PALM = "POINT", "CLOSER", "FURTHER", "PALM"   # PALM = STOP in SEARCH, EXIT in ILLUMINATE
BOOK_FOUND, BOOK_LOST = "BOOK_FOUND", "BOOK_LOST"
WAVE = "WAVE"     # wrist waving side to side; no transition, the director mirrors it in STANDBY
# Behavior / motion
CLIP_DONE, MOVE_DONE, SWEEP_DONE, AT_POSE = "CLIP_DONE", "MOVE_DONE", "SWEEP_DONE", "AT_POSE"
STEP = "STEP"     # a "mark" inside a worker plan; the director consumes it, the machine never sees it
# Timers and system
TIMEOUT, CHECKIN_DUE, READY, FAULT, RECOVERED = "TIMEOUT", "CHECKIN_DUE", "READY", "FAULT", "RECOVERED"
MANUAL, AUTO = "MANUAL", "AUTO"           # web page mode switch: desk controls take / give back the arm
RESET = "RESET"                          # long open palm in FAULT: clear a latched fault and reboot


class KofN:
    """True once k of the last n observations were positive."""
    def __init__(self, k, n):
        self.k, self.hist = k, deque(maxlen=n)

    def update(self, positive):
        self.hist.append(bool(positive))
        return sum(self.hist) >= self.k

    def reset(self):
        self.hist.clear()


class HeldFor:
    """Fires once when a condition has held for `seconds`, then every `repeat` s while it holds."""
    def __init__(self, seconds, repeat=None):
        self.seconds, self.repeat = seconds, repeat
        self.since = self.last_fire = None

    def update(self, active, now):
        if not active:
            self.since = self.last_fire = None
            return False
        if self.since is None:
            self.since = now
        if now - self.since < self.seconds:
            return False
        if self.last_fire is None or (self.repeat and now - self.last_fire >= self.repeat):
            self.last_fire = now
            return True
        return False


class AbsentFor:
    """Fires once when nothing has been seen for `seconds`; re-arms when seen again."""
    def __init__(self, seconds):
        self.seconds, self.last_seen, self.fired = seconds, None, False

    def update(self, seen, now):
        if seen or self.last_seen is None:
            self.last_seen, self.fired = now, False
            return False
        if not self.fired and now - self.last_seen >= self.seconds:
            self.fired = True
            return True
        return False
