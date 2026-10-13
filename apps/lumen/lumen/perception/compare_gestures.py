"""Rules (gestures.classify) vs MediaPipe's canned classifier on a folder of hand photos.

    python -m lumen.perception.compare_gestures photos/              # one line per image + a summary
    python -m lumen.perception.compare_gestures photos/ --out annotated/
    python -m lumen.perception.compare_gestures photos/ --repeat 50  # also time detect(), tracking off vs on

Put photos in subfolders named after the expected gesture (point, thumbs_up, thumbs_down,
open_palm, none) to get an accuracy count for each classifier. Run from apps/lumen.
"""
import argparse
import time
from collections import Counter
from pathlib import Path

import cv2

from .. import hud
from .gestures import classify
from .hands import CANNED_TO_RULES, CannedGestures, HandLandmarker

EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def bench(frame, repeat):
    """Mean ms per detect() on one frame: palm detector every call vs tracking."""
    out = {}
    for tracking in (False, True):
        hands = HandLandmarker(tracking=tracking)
        hands.detect(frame)                         # warm up (and seed the track)
        t0 = time.perf_counter()
        for _ in range(repeat):
            hands.detect(frame)
        out["tracking" if tracking else "palm every call"] = (time.perf_counter() - t0) / repeat * 1000
    return out


def main():
    p = argparse.ArgumentParser(prog="compare_gestures")
    p.add_argument("folder", type=Path)
    p.add_argument("--out", type=Path, help="write annotated copies here")
    p.add_argument("--repeat", type=int, default=0, help="time detect() this many times per image")
    args = p.parse_args()

    hands, canned = HandLandmarker(tracking=False), CannedGestures()
    if not hands.available:
        raise SystemExit("hand models did not load")
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
    tally, agree, n = Counter(), 0, 0
    for path in sorted(f for f in args.folder.rglob("*") if f.suffix.lower() in EXTS):
        frame = cv2.imread(str(path))
        if frame is None:
            continue
        expected = path.parent.name if path.parent != args.folder else None
        hands.detect(frame)
        if not hands.last:
            print(f"{path.relative_to(args.folder)}: no hand")
            tally["no hand"] += 1
            continue
        h = hands.last[0]
        rule = classify(h.points)
        label, prob = canned.classify(h.points, h.right)
        mapped = CANNED_TO_RULES.get(label)
        n, agree = n + 1, agree + (rule == mapped)
        if expected:
            want = None if expected == "none" else expected
            tally["rules ok"] += rule == want
            tally["canned ok"] += mapped == want
            tally["labelled"] += 1
        line = f"{path.relative_to(args.folder)}: rules {rule}  canned {label} {prob:.2f}  presence {h.score:.2f}"
        if args.repeat:
            line += "  " + "  ".join(f"{k} {v:.1f} ms" for k, v in bench(frame, args.repeat).items())
        print(line)
        if args.out:
            hud.draw_hands(frame, [h.points], f"{rule} / {label}")
            cv2.imwrite(str(args.out / path.name), frame)
    print(f"\n{n} hands, rules and canned agree on {agree}" + (f"; {dict(tally)}" if tally else ""))


if __name__ == "__main__":
    main()
