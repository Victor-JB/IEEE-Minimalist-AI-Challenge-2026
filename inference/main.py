"""Live face detection: camera -> detector -> on-screen boxes.

    python inference/main.py                       # camera 0 with detection
    python inference/main.py --camera /dev/video2  # another camera
    python inference/main.py --no-detect           # camera only
    python inference/main.py --headless            # no window, print detections
    PERF=1 python inference/main.py --headless     # + timing/system report at exit (see perf/)
"""

import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "perf"))
from perfstats import count, timed  # noqa: E402  (no-op unless PERF=1)

from camera import Camera, FpsCounter, display_available  # noqa: E402
from detector import FaceDetector  # noqa: E402


def draw(frame, detections, fps, infer_ms):
    for d in detections:
        cv2.rectangle(frame, (d.x, d.y), (d.x + d.w, d.y + d.h), (0, 255, 0), 2)
        cv2.putText(frame, f"{d.score:.2f}", (d.x, d.y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    status = f"{fps:.1f} fps"
    if infer_ms is not None:
        status += f" | infer {infer_ms:.1f} ms | faces {len(detections)}"
    cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", "--index", default="0", help="index (0) or device path (/dev/video0)")
    parser.add_argument("--no-detect", action="store_true", help="show the camera feed only")
    parser.add_argument("--headless", action="store_true", help="no window; print to the terminal")
    parser.add_argument("--threshold", type=float, default=0.5)
    args = parser.parse_args()

    headless = args.headless or not display_available()
    detector = None if args.no_detect else FaceDetector(score_threshold=args.threshold)
    last_print = 0.0

    with Camera(args.camera) as cam:
        fps = FpsCounter()  # start after the (slow) camera open
        while True:
            with timed("camera.read"):
                frame = cam.read()
            if frame is None:
                print("Frame grab failed; is the camera unplugged?")
                break
            count("frames")

            detections, infer_ms = [], None
            if detector:
                t0 = time.perf_counter()
                detections = detector.detect(frame)
                infer_ms = (time.perf_counter() - t0) * 1000
                count("faces", len(detections))
            fps.tick()

            # TODO: pick a target face (e.g. largest box) and turn its center
            # into a normalized [-1, 1] error for the robot's head motors.

            if headless:
                if time.perf_counter() - last_print > 1:
                    infer = f", infer {infer_ms:.1f} ms" if infer_ms is not None else ""
                    print(f"{fps.fps:.1f} fps{infer}, faces: {[(d.center, round(d.score, 2)) for d in detections]}")
                    last_print = time.perf_counter()
                continue

            with timed("display"):
                draw(frame, detections, fps.fps, infer_ms)
                cv2.imshow("face detection (q to quit)", frame)
                key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    if not headless:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:  # Ctrl+C is the only way out in headless mode
        pass
