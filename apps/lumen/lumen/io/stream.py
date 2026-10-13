"""MJPEG-over-HTTP stream of the latest frame, viewable in any browser.

Lighter than X forwarding: frames go out as JPEGs instead of raw pixels.

    streamer = FrameStreamer(port=8080)  # serves http://<host>:8080/
    streamer.update(frame)               # call once per frame

With a desk (lumen/desk/shared.py Handoff) the same port also serves the arm control page at
/desk and its JSON API (lumen/desk/server.py); the HUD stays at / and /stream.
"""

import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

BOUNDARY = b"frame"
PAGE = b"""<!doctype html><title>robot camera</title>
<body style="margin:0;background:#111;display:grid;place-items:center;height:100vh">
<img src="/stream" style="max-width:100%;max-height:100vh"></body>"""
DESK_LINK = (b'<a href="/desk" style="position:fixed;top:12px;right:12px;padding:8px 14px;border-radius:6px;'
             b'background:#2a6df4;color:#fff;font:600 14px system-ui,sans-serif;text-decoration:none">'
             b'Manual control</a>')


class FrameStreamer:
    def __init__(self, port=8080, quality=70, desk=None, video=None):
        self.quality = quality
        self._jpeg = None
        self._cond = threading.Condition()
        self._clients = 0

        streamer = self
        page = PAGE.replace(b"</body>", DESK_LINK + b"</body>") if desk else PAGE
        if desk:
            from ..desk.server import make_handler
            Base = make_handler(desk, video)         # HTTP/1.1: every reply here needs a length
        else:
            Base = BaseHTTPRequestHandler

        class Handler(Base):
            def do_GET(self):
                path = self.path.split("?")[0]
                if path == "/":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(page)))
                    self.end_headers()
                    self.wfile.write(page)
                elif path == "/stream":
                    self.close_connection = True
                    streamer._serve(self)
                elif desk and path == "/desk":
                    self.path = "/"
                    super().do_GET()
                elif desk:
                    super().do_GET()
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
        handler.send_header("Connection", "close")
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
