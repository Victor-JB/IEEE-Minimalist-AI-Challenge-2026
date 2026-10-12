"""python -m lumen.desk  (from apps/lumen, with the board's venv)

    python -m lumen.desk                    # http://<board>:8081/
    python -m lumen.desk --port 9000 --camera /dev/video2
"""
import argparse
import socket

from .arm import DeskArm
from .server import serve
from .video import VideoFeed


def main():
    p = argparse.ArgumentParser(prog="lumen.desk", description="Browser control page for the arm.")
    p.add_argument("--port", type=int, default=8081)
    p.add_argument("--camera", default="auto", help="'auto', an index, or a /dev/video path")
    args = p.parse_args()
    desk = DeskArm()
    desk.start()
    server = serve(desk, VideoFeed(args.camera), args.port)
    print(f"Lumen Desk at http://{socket.gethostname()}:{args.port}/  (Ctrl+C releases the arm)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        desk.stop()
        desk.join(timeout=2)
        server.server_close()


if __name__ == "__main__":
    main()
