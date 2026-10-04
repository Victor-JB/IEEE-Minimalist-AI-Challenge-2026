"""USB camera capture via OpenCV.

Run directly to check the feed:
    python inference/camera.py            # opens camera 0
    python inference/camera.py --index 1  # pick another camera
    python inference/camera.py --probe    # list which indices work
"""

import argparse
import sys
import time

import cv2

# DirectShow opens fast on Windows; the default (MSMF) can take seconds.
BACKEND = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY


class Camera:
    def __init__(self, index=0, width=640, height=480, fps=30):
        self.cap = cv2.VideoCapture(index, BACKEND)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {index}. Try --probe to list cameras.")

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # Keep the driver buffer small so we always process the newest frame.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # set() is only a request; report what the camera actually gave us.
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Camera {index}: {self.width}x{self.height} @ {self.cap.get(cv2.CAP_PROP_FPS):.0f} fps requested")

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
    """Print which camera indices open and return a frame."""
    for i in range(max_index):
        cap = cv2.VideoCapture(i, BACKEND)
        ok = cap.isOpened() and cap.read()[0]
        print(f"  index {i}: {'OK' if ok else '-'}")
        cap.release()


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
            self.fps = (1 - self.alpha) * self.fps + self.alpha * (1 / dt) if self.fps else 1 / dt
        return self.fps


def main():
    parser = argparse.ArgumentParser(description="Show the live camera feed.")
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--probe", action="store_true", help="list working camera indices and exit")
    args = parser.parse_args()

    if args.probe:
        probe()
        return

    fps = FpsCounter()
    with Camera(args.index) as cam:
        while True:
            frame = cam.read()
            if frame is None:
                print("Frame grab failed; is the camera unplugged?")
                break

            cv2.putText(frame, f"{fps.tick():.1f} fps", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("camera (q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
