"""The desk's camera feed: opened while someone watches, released 10 s after the last viewer.

The camera is luxoiii's (it rides in the head), so it is only free while luxoiii is stopped; the
page shows the error instead of a picture when it can't be opened.
"""
import threading
import time

IDLE_RELEASE_S = 10.0
FPS = 15


class VideoFeed:
    def __init__(self, source="auto", quality=70):
        self.source, self.quality = source, quality
        self.cond = threading.Condition()
        self.jpeg, self.error, self.clients, self.seq = None, None, 0, 0
        self.fps = 0.0
        self._thread = None

    def watch(self):
        """Context for one viewer; starts the camera thread if it isn't running."""
        feed = self

        class Viewer:
            def __enter__(self):
                with feed.cond:
                    feed.clients += 1
                    if feed._thread is None or not feed._thread.is_alive():
                        feed._thread = threading.Thread(target=feed._run, daemon=True, name="desk-video")
                        feed._thread.start()
                return feed

            def __exit__(self, *exc):
                with feed.cond:
                    feed.clients -= 1
        return Viewer()

    def next_frame(self, last_seq, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq or self.error, timeout=timeout)
            return self.seq, self.jpeg

    def _run(self):
        try:
            import cv2
            from ..io.camera import Camera
            cam = Camera(self.source)
        except Exception as e:
            with self.cond:
                self.error = f"Camera unavailable ({e}). Is luxoiii running?"
                self.cond.notify_all()
            time.sleep(3)                             # don't retry in a tight loop
            with self.cond:
                self.error = None
            return
        with self.cond:
            self.error = None
        idle_since, frames, t0 = None, 0, time.monotonic()
        try:
            while True:
                with self.cond:
                    watching = self.clients > 0
                now = time.monotonic()
                if not watching:
                    idle_since = idle_since or now
                    if now - idle_since > IDLE_RELEASE_S:
                        return
                    time.sleep(0.2)
                    continue
                idle_since = None
                frame = cam.read()
                if frame is None:
                    raise RuntimeError("camera stopped delivering frames")
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                frames += 1
                if now - t0 >= 1.0:
                    self.fps, frames, t0 = frames / (now - t0), 0, now
                if ok:
                    with self.cond:
                        self.jpeg, self.seq = buf.tobytes(), self.seq + 1
                        self.cond.notify_all()
                time.sleep(max(0.0, 1.0 / FPS - (time.monotonic() - now)))
        except Exception as e:
            with self.cond:
                self.error = f"Camera error: {e}"
                self.cond.notify_all()
        finally:
            cam.release()
