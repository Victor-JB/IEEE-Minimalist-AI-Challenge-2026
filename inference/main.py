"""Live face detection: camera -> detector -> on-screen boxes.

    python inference/main.py              # camera 0 with detection
    python inference/main.py --index 1    # another camera
    python inference/main.py --no-detect  # camera only
"""

import argparse
import time

import cv2

from camera import Camera, FpsCounter
from detector import FaceDetector


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
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--no-detect", action="store_true", help="show the camera feed only")
    parser.add_argument("--threshold", type=float, default=0.5)
    args = parser.parse_args()

    detector = None if args.no_detect else FaceDetector(score_threshold=args.threshold)
    fps = FpsCounter()

    with Camera(args.index) as cam:
        while True:
            frame = cam.read()
            if frame is None:
                print("Frame grab failed; is the camera unplugged?")
                break

            detections, infer_ms = [], None
            if detector:
                t0 = time.perf_counter()
                detections = detector.detect(frame)
                infer_ms = (time.perf_counter() - t0) * 1000

            # TODO: pick a target face (e.g. largest box) and turn its center
            # into a normalized [-1, 1] error for the robot's head motors.

            draw(frame, detections, fps.tick(), infer_ms)
            cv2.imshow("face detection (q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
