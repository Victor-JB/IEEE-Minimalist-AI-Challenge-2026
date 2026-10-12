"""Center the target face in the image by steering the head.

The arm has three pitch joints (shoulder, elbow, head) and no yaw, so it can only center
up/down; left/right needs the 4th joint once it is commissioned (add it as YAW here).

The camera rides in the head, so moving the head moves the face in the image: the offset is
integrated (it keeps moving while the face is off center and stops once it is centered),
instead of being proportional to the error (which settles short of center).
"""
RATE_DEG_S = 15.0    # head speed for a face at the top/bottom edge of the frame
DEADBAND = 0.08      # normalized error treated as centered, so the lamp doesn't twitch
MAX_DEG = 25.0       # most the head may turn away from the pose it is tracking around
CENTER_HOLD_S = 0.4  # centered this long -> `centered` is True


def face_error(det, width, height):
    """Normalized (x, y) in [-1, 1]; +y means the face is below center."""
    cx, cy = det.center
    return (2 * cx / width - 1, 2 * cy / height - 1)


class LookAt:
    """(shoulder, elbow, head) offset that brings the face to the image center."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.head, self.centered_since = 0.0, None

    @property
    def offset(self):
        return (0.0, 0.0, self.head)

    def centered(self, now):
        return self.centered_since is not None and now - self.centered_since >= CENTER_HOLD_S

    def update(self, err, now, dt):
        """err from face_error, or None when no face is visible (hold where it is)."""
        if err is None:
            self.centered_since = None
            return self.offset
        ey = err[1]
        if abs(ey) < DEADBAND:
            self.centered_since = self.centered_since or now
        else:
            self.centered_since = None
            self.head += RATE_DEG_S * ey * dt        # +head = look down (motion_lib convention)
            self.head = max(-MAX_DEG, min(MAX_DEG, self.head))
        return self.offset
