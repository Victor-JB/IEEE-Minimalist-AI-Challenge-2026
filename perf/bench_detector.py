"""Sandboxed detector benchmark: inference is the only thing running.

    python perf/bench_detector.py                          # 4 threads, 30 s, synthetic frame
    python perf/bench_detector.py --threads 1 2 4          # sweep thread counts
    python perf/bench_detector.py --camera 0               # use one real camera frame
    python perf/bench_detector.py --model other.tflite     # before/after a model change
    python perf/bench_detector.py --watts 3.4 --idle-watts 2.1   # add energy per inference

Power: power the board through a USB-C power meter, run with a --duration long
enough for the meter's average to settle (30 s is plenty), and pass the average
with --watts. With --threads sweeps, give one --watts value per thread count.
Results print as a markdown table row you can paste into perf/RESULTS.md.
"""

import argparse
import os
import platform
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "inference"))
from detector import MODEL_PATH, FaceDetector  # noqa: E402


def get_frame(args):
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            sys.exit(f"Could not read {args.image}")
        return frame, Path(args.image).name
    if args.camera is not None:
        from camera import Camera

        with Camera(args.camera) as cam:
            frame = cam.read()
        return frame, f"camera {args.camera} {frame.shape[1]}x{frame.shape[0]}"
    # Latency of the model itself does not depend on content; postprocessing barely does.
    rng = np.random.default_rng(0)
    return rng.integers(0, 256, (480, 640, 3), np.uint8), "synthetic 640x480"


def pct(values, p):
    return float(np.percentile(values, p))


def bench(model_path, frame, threads, duration, warmup=10):
    det = FaceDetector(model_path=model_path, num_threads=threads)
    for _ in range(warmup):
        det.detect(frame)

    pre, inv, post = [], [], []
    end = time.perf_counter() + duration
    while time.perf_counter() < end:
        t0 = time.perf_counter()
        tensor = det._preprocess(frame)
        t1 = time.perf_counter()
        det.interpreter.set_tensor(det.input_index, tensor)
        det.interpreter.invoke()
        t2 = time.perf_counter()
        det._postprocess()
        t3 = time.perf_counter()
        pre.append(t1 - t0)
        inv.append(t2 - t1)
        post.append(t3 - t2)

    ms = lambda xs: np.array(xs) * 1000  # noqa: E731
    total = ms(pre) + ms(inv) + ms(post)
    return {
        "threads": threads,
        "n": len(total),
        "pre": pct(ms(pre), 50),
        "invoke_p50": pct(ms(inv), 50),
        "invoke_p90": pct(ms(inv), 90),
        "post": pct(ms(post), 50),
        "total_p50": pct(total, 50),
        "total_p90": pct(total, 90),
        "fps": len(total) / (sum(total) / 1000),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default=str(MODEL_PATH))
    parser.add_argument("--threads", type=int, nargs="+", default=[os.cpu_count()])
    parser.add_argument("--duration", type=float, default=30, help="seconds per thread count")
    parser.add_argument("--image", help="benchmark on this image")
    parser.add_argument("--camera", help="benchmark on one frame from this camera")
    parser.add_argument("--watts", type=float, nargs="+", help="avg board power per run (W)")
    parser.add_argument("--idle-watts", type=float, help="board power at idle (W)")
    args = parser.parse_args()

    if args.watts and len(args.watts) != len(args.threads):
        sys.exit("Give one --watts value per --threads value.")

    frame, source = get_frame(args)
    size_kb = Path(args.model).stat().st_size / 1024
    print(f"model {Path(args.model).name} ({size_kb:.0f} KB), input: {source}, "
          f"{platform.machine()} {os.cpu_count()} cores, {args.duration:.0f} s per run\n")

    header = ("| model | threads | pre ms | invoke p50 / p90 ms | post ms | total p50 / p90 ms "
              "| inf/s | W | mJ/inf | mJ/inf above idle |")
    print(header)
    print("|" + "---|" * (header.count("|") - 1))
    for i, threads in enumerate(args.threads):
        r = bench(args.model, frame, threads, args.duration)
        watts = args.watts[i] if args.watts else None
        energy = f"{watts / r['fps'] * 1000:.0f}" if watts else "-"
        above = f"{(watts - args.idle_watts) / r['fps'] * 1000:.0f}" if watts and args.idle_watts else "-"
        print(f"| {Path(args.model).stem} | {threads} | {r['pre']:.1f} | {r['invoke_p50']:.1f} / {r['invoke_p90']:.1f} "
              f"| {r['post']:.1f} | {r['total_p50']:.1f} / {r['total_p90']:.1f} | {r['fps']:.1f} "
              f"| {watts or '-'} | {energy} | {above} |")


if __name__ == "__main__":
    main()
