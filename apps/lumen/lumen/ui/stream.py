"""MJPEG-over-HTTP stream of the latest frame, viewable in any browser.

Lighter than X forwarding: frames go out as JPEGs instead of raw pixels.

    streamer = FrameStreamer(port=8080)  # serves http://<host>:8080/
    streamer.update(frame)               # call once per frame
"""

import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

BOUNDARY = b"frame"
PAGE = b"""<!doctype html><title>robot camera</title>
<body style="margin:0;background:#111;display:grid;place-items:center;height:100vh">
<img src="/stream" style="max-width:100%;max-height:100vh"></body>"""


class FrameStreamer:
    def __init__(self, port=8080, quality=70):
        self.quality = quality
        self._jpeg = None
        self._cond = threading.Condition()
        self._clients = 0

        streamer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(PAGE)
                elif self.path == "/stream":
                    streamer._serve(self)
                else:
                    self.send_error(404)

            def log_message(self, *args):  # keep the terminal for detections
                pass

        self.server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        print(f"Streaming at http://{socket.gethostname()}:{port}/")

    def update(self, frame):
        """Publish a BGR frame; skips the JPEG encode when nobody is watching."""
        if not self._clients:
            return
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            with self._cond:
                self._jpeg = buf.tobytes()
                self._cond.notify_all()

    def _serve(self, handler):
        handler.send_response(200)
        handler.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY.decode()}")
        handler.send_header("Cache-Control", "no-cache")
        handler.end_headers()
        with self._cond:
            self._clients += 1
        try:
            while True:
                with self._cond:
                    self._cond.wait(timeout=5)
                    jpeg = self._jpeg
                if jpeg is None:
                    continue
                handler.wfile.write(
                    b"--" + BOUNDARY + b"\r\nContent-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(jpeg)}\r\n\r\n".encode() + jpeg + b"\r\n"
                )
        except (BrokenPipeError, ConnectionResetError):
            pass  # browser tab closed
        finally:
            with self._cond:
                self._clients -= 1

    def close(self):
        self.server.shutdown()
