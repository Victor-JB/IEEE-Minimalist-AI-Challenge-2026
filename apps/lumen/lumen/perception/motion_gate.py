"""The free 'model': is anything moving? Frame difference on an 80x60 gray thumbnail (<1 ms).

Nothing moving -> no neural network runs (optimization ladder rung 2)."""
import cv2
import numpy as np


class MotionGate:
    def __init__(self, threshold):
        self.threshold, self.prev, self.score = threshold, None, 0.0

    def update(self, frame):
        small = cv2.resize(frame, (80, 60), interpolation=cv2.INTER_AREA)
        if small.ndim == 3:
            small = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        small = small.astype(np.int16)
        moving = False
        if self.prev is not None:
            self.score = float(np.abs(small - self.prev).mean())
            moving = self.score > self.threshold
        self.prev = small
        return moving
