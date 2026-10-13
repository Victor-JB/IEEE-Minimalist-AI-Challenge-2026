"""python -m luxoiii  (from apps/luxoiii; the systemd service runs exactly this)

    python -m luxoiii --fake-arm --stream 8080      # laptop/webcam: full behavior, no board
    python -m luxoiii --headless --stream 8080      # on the Arduino Q (what boot runs)
    PERF=1 python -m luxoiii --headless             # + timing report on exit
"""
import argparse

from .app import run


def main():
    p = argparse.ArgumentParser(prog="luxoiii")
    p.add_argument("--camera", default="auto", help="'auto', an index, or a /dev/video path")
    p.add_argument("--fake-arm", action="store_true", help="no board: simulate arm timing")
    p.add_argument("--no-face", action="store_true", help="skip the face model (debug)")
    p.add_argument("--headless", action="store_true", help="print status instead of a window")
    p.add_argument("--stream", nargs="?", type=int, const=8080, metavar="PORT",
                   help="serve the HUD video at http://<board>:PORT/")
    args = p.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
