"""Face detection (Qualcomm Lightweight-Face-Detection, already working in inference/) plus
target selection: follow the largest face, switch only if another stays clearly larger."""
from .. import config as C
from .. import paths  # noqa: F401  (puts inference/ on sys.path)


class FaceTracker:
    def __init__(self, threshold=0.5):
        from detector import FaceDetector       # inference/detector.py
        self.detector = FaceDetector(score_threshold=threshold)
        self.target = None
        self._challenger_since = None

    def detect(self, frame, now):
        faces = self.detector.detect(frame)
        self.target = self._choose(faces, now)
        return faces

    def _choose(self, faces, now):
        if not faces:
            self._challenger_since = None
            return None
        biggest = max(faces, key=lambda d: d.w * d.h)
        if self.target is None:
            return biggest
        # the old target's nearest match this frame
        tx, ty = self.target.center
        current = min(faces, key=lambda d: (d.center[0] - tx) ** 2 + (d.center[1] - ty) ** 2)
        if biggest is current or biggest.w * biggest.h < C.SWITCH_FACE_RATIO * current.w * current.h:
            self._challenger_since = None
            return current
        self._challenger_since = self._challenger_since or now
        if now - self._challenger_since >= C.SWITCH_FACE_HOLD_S:
            self._challenger_since = None
            return biggest
        return current
