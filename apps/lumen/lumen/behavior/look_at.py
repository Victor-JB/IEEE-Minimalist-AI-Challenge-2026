"""Turn the target face's position in the image into joint offsets.

The arm has three pitch joints (shoulder, elbow, head) and no yaw, so it can only track
up/down; left/right needs the 4th joint once it is commissioned (add it as YAW here).
"""
GAIN_DEG = 12.0      # head offset for a face at the top/bottom edge of the frame
DEADBAND = 0.08      # ignore small errors so the lamp doesn't twitch


def face_error(det, width, height):
    """Normalized (x, y) in [-1, 1]; +y means the face is below center."""
    cx, cy = det.center
    return (2 * cx / width - 1, 2 * cy / height - 1)


def look_offset(err):
    """(shoulder, elbow, head) offsets in degrees for a normalized face error."""
    _, ey = err
    if abs(ey) < DEADBAND:
        ey = 0.0
    return (0.0, 0.0, GAIN_DEG * ey)   # +head = look down (motion_lib convention)
