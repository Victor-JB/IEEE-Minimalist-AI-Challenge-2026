"""The desk inside luxoiii: the control page on luxoiii's own stream port (http://<board>:8080/desk).

One process, one arm owner at a time. Connect on the page pauses luxoiii: the motion worker
releases the arm (no FAULT), the main loop stops perceiving and commanding, and the desk's arm
thread takes the session. Disconnect hands it back: the desk closes and luxoiii re-enters BOOT
(reconnect, go home). The camera stays luxoiii's; the desk's video is the frame luxoiii just read.

    handoff = Handoff(worker)          # starts the desk's (idle) arm thread
    handoff.manual                     # main loop: True -> skip perception/behavior/light
    handoff.feed.publish(frame)        # main loop: every frame, for /video.mjpg
"""
import threading
import time

import cv2

from .arm import DeskArm

RELEASE_WAIT_S = 3.0          # for the worker to close its session
CONNECT_TRIES = 4             # the firmware may take ~2 s to drop the old owner
CONNECT_WAIT_S = 1.5


class SharedFeed:
    """VideoFeed's interface (watch, next_frame, error, fps, clients) over frames luxoiii publishes."""

    def __init__(self, quality=70):
        self.quality = quality
        self.cond = threading.Condition()
        self.jpeg, self.error, self.clients, self.seq = None, None, 0, 0
        self.fps, self._n, self._t0 = 0.0, 0, time.monotonic()

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

    def publish(self, frame):
        """Encode only while someone watches."""
        if not self.clients:
            return
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        now = time.monotonic()
        self._n += 1
        if now - self._t0 >= 1.0:
            self.fps, self._n, self._t0 = self._n / (now - self._t0), 0, now
        if ok:
            with self.cond:
                self.jpeg, self.seq = buf.tobytes(), self.seq + 1
                self.cond.notify_all()

    def next_frame(self, last_seq, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout=timeout)
            return self.seq, self.jpeg


class Handoff:
    """DeskArm's interface for the desk server, with connect/disconnect moving the arm between
    luxoiii's motion worker and the desk."""

    def __init__(self, worker, desk=None):
        self.worker = worker
        self.desk = desk or DeskArm()
        self.desk.start()
        self.feed = SharedFeed()
        self.manual = False
        self.resume = False           # main loop: restart the state machine (BOOT) and clear it
        self._busy = threading.Lock()
        self._note = None

    def snapshot(self):
        s = self.desk.snapshot()
        if self._note:
            s["notice"] = self._note
        elif not self.manual:
            s["notice"] = "luxoiii is driving the arm. Connect to take manual control (pauses luxoiii)."
        s["luxoiii"] = "paused" if self.manual else "running"
        return s

    def history_since(self, t):
        return self.desk.history_since(t)

    def send(self, cmd, *args):
        if cmd == "connect":
            return self._spawn(self._take)
        if cmd == "disconnect":
            return self._spawn(self._give_back)
        if self.manual:
            self.desk.send(cmd, *args)

    def _spawn(self, fn):
        threading.Thread(target=fn, daemon=True, name="desk-handoff").start()

    def _take(self):
        if not self._busy.acquire(blocking=False):
            return
        try:
            if self.manual and self.desk.arm:
                return
            self._note = "Pausing luxoiii and releasing the arm..."
            self.manual = True
            self.worker.release()
            deadline = time.monotonic() + RELEASE_WAIT_S
            while self.worker.arm is not None and time.monotonic() < deadline:
                time.sleep(0.05)
            for _ in range(CONNECT_TRIES):
                self._note = "Connecting the desk to the arm..."
                self.desk.send("connect")
                deadline = time.monotonic() + CONNECT_WAIT_S
                while self.desk.arm is None and time.monotonic() < deadline:
                    time.sleep(0.05)
                if self.desk.arm is not None:
                    break
            self._note = None
            if self.desk.arm is None:
                with self.desk.lock:
                    notice = self.desk.notice
                self._back_to_luxoiii()
                self._note = f"Desk couldn't take the arm ({notice}); luxoiii resumed."
                threading.Timer(6.0, self._clear_note).start()
        finally:
            self._busy.release()

    def _give_back(self):
        with self._busy:
            if not self.manual:
                return
            self.desk.send("disconnect")
            deadline = time.monotonic() + RELEASE_WAIT_S
            while self.desk.arm is not None and time.monotonic() < deadline:
                time.sleep(0.05)
            self._back_to_luxoiii()

    def _back_to_luxoiii(self):
        self.manual = False
        self.resume = True            # the main loop re-enters BOOT, which reconnects the worker

    def _clear_note(self):
        self._note = None
