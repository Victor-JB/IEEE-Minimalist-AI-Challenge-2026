"""The camera feed of the one web page (:8080), with Lumen's HUD drawn on it.

Lumen owns the camera, so the page never opens it: the main loop publishes each annotated frame
here, and the page's /video.mjpg (desk/server.py) streams them. The JPEG encode only runs while
someone is watching. Same interface as desk/video.VideoFeed (watch, next_frame, error, fps, clients).
"""
import threading
import time

import cv2


class HudFeed:
    def __init__(self, quality=70):
        self.quality = quality
        self.cond = threading.Condition()
        self.jpeg, self.error, self.clients, self.seq, self.fps = None, None, 0, 0, 0.0
        self._frames, self._t0 = 0, time.monotonic()

    def watch(self):
        feed = self

        class Viewer:
            def __enter__(self):
                with feed.cond:
                    feed.clients += 1
                return feed

            def __exit__(self, *exc):
                with feed.cond:
                    feed.clients -= 1
        return Viewer()

    @property
    def watched(self):
        return self.clients > 0

    def publish(self, frame):
        """One BGR frame from the main loop (skipped when nobody is watching)."""
        if not self.clients:
            return
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        now = time.monotonic()
        self._frames += 1
        if now - self._t0 >= 1.0:
            self.fps, self._frames, self._t0 = self._frames / (now - self._t0), 0, now
        with self.cond:
            self.jpeg, self.seq = buf.tobytes(), self.seq + 1
            self.cond.notify_all()

    def next_frame(self, last_seq, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq or self.error, timeout=timeout)
            return self.seq, self.jpeg
