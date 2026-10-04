"""USB camera capture via OpenCV, on Linux (V4L2) and Windows (DirectShow).

Run directly to check the feed:
    python inference/camera.py                       # opens camera 0
    python inference/camera.py --camera 1            # another index
    python inference/camera.py --camera /dev/video2  # Linux device path
    python inference/camera.py --probe               # list cameras
    python inference/camera.py --headless            # no window, print fps
"""

import argparse
import glob
import os
import sys
import time

import cv2

if sys.platform == "win32":
    # DirectShow opens fast on Windows; the default (MSMF) can take seconds.
    BACKEND = cv2.CAP_DSHOW
elif sys.platform.startswith("linux"):
    BACKEND = cv2.CAP_V4L2
else:
    BACKEND = cv2.CAP_ANY

# Some cameras send black or failed frames while auto-exposure settles.
WARMUP_FRAMES = 5


def parse_source(source):
    """'0' -> 0 (index); anything else (e.g. '/dev/video0') stays a device path."""
    return int(source) if str(source).isdigit() else source


def display_available():
    """False on a headless Linux box (e.g. over SSH), where cv2.imshow would crash."""
    if sys.platform.startswith("linux"):
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    return True


class Camera:
    def __init__(self, source=0, width=640, height=480, fps=30):
        source = parse_source(source)
        self.cap = cv2.VideoCapture(source, BACKEND)
        if not self.cap.isOpened():
            hint = "Try --probe to list cameras."
            if sys.platform.startswith("linux"):
                hint += (
                    " If the device exists but won't open, add yourself to the"
                    " 'video' group: sudo usermod -aG video $USER (then log back in)."
                )
            raise RuntimeError(f"Could not open camera {source!r}. {hint}")

        # FOURCC must be set before the resolution so V4L2 picks an MJPG mode.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # Keep the driver buffer small so we always process the newest frame.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        for _ in range(WARMUP_FRAMES):
            self.cap.read()

        # set() is only a request; report what the camera actually gave us.
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(
            f"Camera {source!r} ({self.cap.getBackendName()}): "
            f"{self.width}x{self.height} @ {self.cap.get(cv2.CAP_PROP_FPS):.0f} fps requested"
        )

    def read(self):
        """Return the next frame as an (H, W, 3) uint8 BGR array, or None on failure."""
        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self):
        self.cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.release()


def probe(max_index=4):
    """Print which cameras open and return a frame."""
    if sys.platform.startswith("linux"):
        # A UVC camera usually creates two nodes; only one of them gives frames.
        sources = sorted(
            glob.glob("/dev/video*"), key=lambda p: int(p[len("/dev/video") :])
        )
    else:
        sources = range(max_index)

    for source in sources:
        cap = cv2.VideoCapture(source, BACKEND)
        ok = cap.isOpened() and cap.read()[0]
        print(f"  {source}: {'OK' if ok else '-'}")
        cap.release()

    # Names that survive replugging; pass one of these to --camera on the robot.
    for path in sorted(glob.glob("/dev/v4l/by-id/*")):
        print(f"  {path} -> {os.path.realpath(path)}")


class FpsCounter:
    """Exponentially smoothed frames-per-second."""

    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.fps = 0.0
        self._last = time.perf_counter()

    def tick(self):
        now = time.perf_counter()
        dt, self._last = now - self._last, now
        if dt > 0:
            self.fps = (
                (1 - self.alpha) * self.fps + self.alpha * (1 / dt)
                if self.fps
                else 1 / dt
            )
        return self.fps


def main():
    parser = argparse.ArgumentParser(description="Show the live camera feed.")
    parser.add_argument(
        "--camera",
        "--index",
        default="0",
        help="index (0) or device path (/dev/video0, /dev/v4l/by-id/...)",
    )
    parser.add_argument(
        "--probe", action="store_true", help="list working cameras and exit"
    )
    parser.add_argument(
        "--headless", action="store_true", help="no window; print fps instead"
    )
    args = parser.parse_args()

    if args.probe:
        probe()
        return

    headless = args.headless or not display_available()
    last_print = 0.0
    with Camera(args.camera) as cam:
        fps = FpsCounter()  # start after the (slow) camera open
        while True:
            frame = cam.read()
            if frame is None:
                print("Frame grab failed; is the camera unplugged?")
                break
            fps.tick()

            if headless:
                if time.perf_counter() - last_print > 1:
                    print(f"{fps.fps:.1f} fps, frame {frame.shape}")
                    last_print = time.perf_counter()
                continue

            cv2.putText(
                frame,
                f"{fps.fps:.1f} fps",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.imshow("camera (q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    if not headless:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:  # Ctrl+C is the only way out in headless mode
        pass
