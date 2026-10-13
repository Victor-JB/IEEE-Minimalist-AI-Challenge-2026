"""HTTP side of the desk: the page, its JSON API, the MJPEG feed, poses and recordings.

    GET  /                  the page            GET  /state         arm snapshot (polled ~10 Hz)
    GET  /history?since=t   plot samples        GET  /video.mjpg    camera
    GET  /clips  /poses  /recordings            GET  /recordings/<name>  download
    GET  /replay/<name>     a recording's measured angles, decimated, for the replay slider
    GET  /cad/<file>        the Fusion ARM v29 meshes + manifest (arm/cad) for the 3D view
    POST /cmd {"cmd": ..., "args": [...]}       queued to the arm thread (see DeskArm._cmd_*)
    POST /poses {"name"} (teach current)        POST /poses/delete {"name"}
"""
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

from .. import paths
from .arm import JOINTS, MOTOR

PAGE = Path(__file__).with_name("page.html")
POSES = paths.ARM / "clips" / "taught_poses.json"
RECORDING_DIRS = (paths.LOGS, paths.ARM / "recordings")
NAME = re.compile(r"^[A-Za-z0-9_.-]+\.jsonl$")
CAD = paths.ARM / "cad"
CAD_NAME = re.compile(r"^[A-Za-z0-9_.-]+\.(stl|json)$")
COMMANDS = {"connect", "disconnect", "power", "relax", "stop", "jog", "goto", "clip", "roll", "roll_off", "light"}
ARM_COMMANDS = COMMANDS - {"stop"}       # refused in Auto: Lumen owns the arm then (STOP always goes through)
REPLAY_POINTS = 1500


def load_poses():
    try:
        return json.loads(POSES.read_text())
    except (OSError, ValueError):
        return {}


def recordings():
    files = [f for d in RECORDING_DIRS if d.exists() for f in d.glob("*.jsonl")]
    files.sort(key=lambda f: f.stat().st_mtime, reverse=True)
    return [dict(name=f.name, size=f.stat().st_size, modified=f.stat().st_mtime) for f in files[:60]]


def recording_path(name):
    if not NAME.match(name):
        return None
    for d in RECORDING_DIRS:
        f = d / name
        if f.is_file():
            return f
    return None


def replay(path):
    """[[t, [shoulder, elbow, head] or None], ...] from a recording's arm samples, decimated."""
    out = []
    with open(path) as f:
        for line in f:
            if '"sample"' not in line:
                continue
            d = json.loads(line)
            s = d.get("sample") or {}
            if d.get("type") != "sample" or s.get("kind") != "arm":
                continue
            ax = s["axes"]
            ok = all(ax[m]["angle"] != -2147483648 for m in MOTOR)
            out.append([round(d["time"], 3), [ax[m]["angle"] / 1000 for m in MOTOR] if ok else None])
    step = max(1, len(out) // REPLAY_POINTS)
    return out[::step]


def make_handler(desk, video, app=None, poses_lock=threading.Lock()):
    """app: the Lumen web page (ui/web.py) when the desk runs inside Lumen; None standalone."""
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def _send(self, code, body, kind="application/json", cache="no-store"):
            data = body if isinstance(body, bytes) else (body if isinstance(body, str) else json.dumps(body)).encode()
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", cache)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _body(self):
            n = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(min(n, 65536)) or b"{}")

        def do_GET(self):
            url = urlparse(self.path)
            p = url.path
            if p in ("/", "/index.html"):
                return self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
            if p == "/state":
                s = desk.snapshot()
                s["video"] = dict(error=video.error, fps=round(video.fps, 1), viewers=video.clients)
                s["mode"] = app.mode if app else "manual"
                s["lumen"] = app.status if app else None
                return self._send(200, s)
            if p == "/history":
                since = float(parse_qs(url.query).get("since", ["0"])[0])
                return self._send(200, desk.history_since(since))
            if p == "/video.mjpg":
                return self._video()
            if p == "/clips":
                from player import list_clips
                return self._send(200, list_clips())
            if p == "/poses":
                return self._send(200, load_poses())
            if p == "/recordings":
                return self._send(200, recordings())
            if p.startswith("/recordings/"):
                f = recording_path(unquote(p.split("/", 2)[2]))
                if not f:
                    return self._send(404, {"error": "no such recording"})
                return self._send(200, f.read_bytes(), "application/x-ndjson")
            if p.startswith("/cad/"):
                name = unquote(p.split("/", 2)[2])
                f = CAD / name
                if not CAD_NAME.match(name) or not f.is_file():
                    return self._send(404, {"error": "no such CAD file"})
                kind = "application/json" if name.endswith(".json") else "model/stl"
                return self._send(200, f.read_bytes(), kind, cache="max-age=86400")
            if p.startswith("/replay/"):
                f = recording_path(unquote(p.split("/", 2)[2]))
                if not f:
                    return self._send(404, {"error": "no such recording"})
                return self._send(200, dict(name=f.name, joints=JOINTS, samples=replay(f)))
            self._send(404, {"error": "not found"})

        def do_POST(self):
            try:
                body = self._body()
                p = urlparse(self.path).path
                if p == "/cmd":
                    cmd, args = body.get("cmd"), body.get("args") or []
                    if cmd == "mode" and app and isinstance(args, list) and args:
                        app.request(str(args[0]))
                        return self._send(200, {"ok": True})
                    if cmd not in COMMANDS or not isinstance(args, list):
                        return self._send(400, {"error": f"unknown command {cmd!r}"})
                    if app and app.mode != "manual" and cmd in ARM_COMMANDS:
                        return self._send(409, {"error": "Lumen has the arm: switch to Manual first"})
                    desk.send(cmd, *args)
                    return self._send(200, {"ok": True})
                if p == "/poses":
                    name = str(body.get("name", "")).strip()[:40]
                    s = desk.snapshot()
                    pose = [j["angle"] for j in s["joints"]]
                    if not name or None in pose:
                        return self._send(400, {"error": "need a name and healthy angles on all joints"})
                    roll = (s["roll"] or {}).get("target") if s["roll"] and s["roll"].get("enabled") else None
                    with poses_lock:
                        poses = load_poses()
                        poses[name] = dict(pose=[round(a, 2) for a in pose], roll=roll,
                                           saved=time.strftime("%Y-%m-%d %H:%M:%S"))
                        POSES.write_text(json.dumps(poses, indent=2) + "\n")
                    return self._send(200, poses)
                if p == "/poses/delete":
                    with poses_lock:
                        poses = load_poses()
                        poses.pop(str(body.get("name")), None)
                        POSES.write_text(json.dumps(poses, indent=2) + "\n")
                    return self._send(200, poses)
                self._send(404, {"error": "not found"})
            except (ValueError, TypeError) as e:
                self._send(400, {"error": str(e)})

        def _video(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            seq = -1
            try:
                with video.watch():
                    while True:
                        seq, jpeg = video.next_frame(seq)
                        if jpeg is None:
                            if video.error:
                                time.sleep(1)
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass                                     # the page polls ~10 times a second

    return Handler


def serve(desk, video, port, app=None):
    server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(desk, video, app))
    server.daemon_threads = True
    return server
