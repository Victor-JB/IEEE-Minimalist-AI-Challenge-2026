"""Small, aspect-independent features for a landmark-only gesture classifier.

HaGRID landmarks are normalized by image width and height independently. The
same features must therefore accept either those coordinates or camera pixels.
No camera frames, image dimensions, or new inference model are needed at runtime.
"""
import numpy as np

CHAINS = np.asarray([[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12],
                     [13, 14, 15, 16], [17, 18, 19, 20]])
LABELS = (None, "point", "open_palm", "thumbs_up", "thumbs_down")
FEATURE_VERSION = 1


def features(points):
    """Return float32 features; input shape (..., 21, >=2), output (..., 106)."""
    p = np.asarray(points, dtype=np.float32)[..., :2]
    if p.shape[-2:] != (21, 2):
        raise ValueError("expected 21 hand landmarks")
    lo, hi = p.min(axis=-2, keepdims=True), p.max(axis=-2, keepdims=True)
    q = (p - (lo + hi) * .5) / np.maximum((hi - lo) * .5, 1e-6)

    # Coordinates in a palm basis survive independent x/y scaling, reflection,
    # and rotation. Clipping limits instability when a hand is nearly edge-on.
    u, v = q[..., 5, :] - q[..., 17, :], q[..., 9, :] - q[..., 0, :]
    det = u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]
    floor = np.maximum(np.linalg.norm(u, axis=-1) * np.linalg.norm(v, axis=-1) * .01, 1e-8)
    det = np.where(det >= 0, np.maximum(det, floor), np.minimum(det, -floor))
    delta = q - q[..., 0:1, :]
    a = (delta[..., 0] * v[..., None, 1] - delta[..., 1] * v[..., None, 0]) / det[..., None]
    b = (u[..., None, 0] * delta[..., 1] - u[..., None, 1] * delta[..., 0]) / det[..., None]
    palm = np.clip(np.stack((a, b), axis=-1), -4., 4.) * .25

    fingers = q[..., CHAINS, :]
    bones = np.diff(fingers, axis=-2)
    lengths = np.maximum(np.linalg.norm(bones, axis=-1), 1e-6)
    unit = bones / lengths[..., None]
    cosine = np.sum(unit[..., :-1, :] * unit[..., 1:, :], axis=-1)
    straight = np.linalg.norm(fingers[..., -1, :] - fingers[..., 0, :], axis=-1) / lengths.sum(axis=-1)
    tip_dist = np.linalg.norm(fingers[..., -1, :] - q[..., None, 0, :], axis=-1)
    pip_dist = np.maximum(np.linalg.norm(fingers[..., 1, :] - q[..., None, 0, :], axis=-1), .01)
    ratio = np.clip(tip_dist / pip_dist, 0., 3.) / 3.
    thumb_dir = q[..., 4, :] - q[..., 2, :]
    prefix = p.shape[:-2]
    return np.concatenate((q.reshape(*prefix, 42), palm.reshape(*prefix, 42),
                           cosine.reshape(*prefix, 10), straight, ratio, thumb_dir), axis=-1).astype(np.float32)


class LandmarkClassifier:
    """Two small matrix multiplies; the trained artifact is loaded only once."""

    def __init__(self, path):
        with np.load(path, allow_pickle=False) as weights:
            if int(weights["feature_version"]) != FEATURE_VERSION:
                raise ValueError("incompatible gesture feature version")
            self.weights = {name: weights[name].astype(np.float32) for name in ("w1", "b1", "w2", "b2")}
        w1, b1, w2, b2 = (self.weights[key] for key in ("w1", "b1", "w2", "b2"))
        if (w1.ndim != 2 or w1.shape[0] != 106 or b1.shape != (w1.shape[1],)
                or w2.shape != (w1.shape[1], len(LABELS)) or b2.shape != (len(LABELS),)
                or not all(np.isfinite(w).all() for w in self.weights.values())):
            raise ValueError("invalid gesture model dimensions or values")

    def probabilities(self, points):
        x = features(points)
        h = np.maximum(x @ self.weights["w1"] + self.weights["b1"], 0.)
        logits = h @ self.weights["w2"] + self.weights["b2"]
        exp = np.exp(logits - logits.max(axis=-1, keepdims=True))
        return exp / exp.sum(axis=-1, keepdims=True)

    def classify(self, points, threshold=.5):
        try:
            p = np.asarray(points, dtype=np.float32)
        except (TypeError, ValueError):
            return None
        if (p.ndim != 2 or p.shape[0] != 21 or p.shape[1] < 2
                or not np.isfinite(p).all() or np.ptp(p[:, :2], axis=0).max() < 1e-8):
            return None
        probs = self.probabilities(points)
        best = int(np.argmax(probs))
        return LABELS[best] if probs[best] >= threshold else None
