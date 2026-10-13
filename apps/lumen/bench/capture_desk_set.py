"""Capture the real lamp-camera book eval set, and pre-label it for hand correction.

    python bench/capture_desk_set.py capture --camera auto --count 150 --every 2   # on the board
    python bench/capture_desk_set.py prelabel --model yolox_s_640_fp32              # on a PC
    python bench/eval_detector.py --data bench/data/desk --model app --sweep 0.3 0.4 0.5

capture: saves a frame every --every seconds into bench/data/desk/images/ (gitignored) while you
  rearrange the scene. Move the lamp head by hand (or run Lumen's SEARCH sweep) between shots.
prelabel: runs a big, slow detector (the 640 fp32 model is fine on a PC) at a low threshold and
  writes bench/data/desk/annotations.json. Open it next to the images and fix it by hand: delete
  wrong boxes, add missed ones, move far/sliver books to "ignore". Never evaluate a model on labels
  it wrote itself without that correction pass.

What to shoot (~150 frames, the camera exactly as mounted in the lamp head, at Lumen's resolution):
  - ~100 with books: 1, 2 and 3+ books; closed, open (pages up), spine-up, stacked; hardcover,
    paperback, notebook-like covers; the poses of the SEARCH sweep (3-5 pause poses, straight down
    and oblique ~30-45 deg), and the AIM/HOLD pose right over a book (book fills 30-70% of frame).
  - ~50 negatives: the same desk with no book: laptop, tablet, phone, papers, a magazine, an
    envelope, a keyboard, a mug, a hand on the desk. These are the false BOOK_FOUND traps.
  - Lighting: lamp LED off and on (full and dim), daylight, evening room light; one session with
    the hand making gestures over the book (ILLUMINATE).
  - Hold out the scenes: shoot two different desks/sessions and keep one only for final numbers.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2

BENCH = Path(__file__).resolve().parent
DESK = BENCH / "data" / "desk"


def capture(args):
    sys.path.insert(0, str(BENCH.parent))
    from lumen.io.camera import Camera

    out = DESK / "images"
    out.mkdir(parents=True, exist_ok=True)
    start = len(list(out.glob("*.jpg")))
    with Camera(args.camera) as cam:
        for i in range(start, start + args.count):
            time.sleep(args.every)
            frame = cam.read()
            if frame is None:
                sys.exit("camera read failed")
            cv2.imwrite(str(out / f"{i:04d}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            print(f"{i:04d}.jpg {frame.shape[1]}x{frame.shape[0]}", flush=True)


def prelabel(args):
    sys.path.insert(0, str(BENCH))
    from book_candidates import load

    det = load(args.model, score_threshold=args.threshold)
    entries = []
    for f in sorted((DESK / "images").glob("*.jpg")):
        boxes = [[x, y, w, h] for x, y, w, h, _ in det.detect(cv2.imread(str(f)))]
        entries.append({"file": f.name, "boxes": boxes, "ignore": []})
    path = DESK / "annotations.json"
    if path.exists() and not args.force:
        sys.exit(f"{path} exists (hand-corrected?); pass --force to overwrite")
    path.write_text(json.dumps({"source": f"lamp camera; pre-labelled by {args.model}, CORRECT BY HAND",
                                "images": entries}, indent=1))
    print(f"wrote {path}: {len(entries)} images, {sum(len(e['boxes']) for e in entries)} boxes to check")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("--camera", default="auto")
    c.add_argument("--count", type=int, default=150)
    c.add_argument("--every", type=float, default=2.0, help="seconds between frames")
    p = sub.add_parser("prelabel")
    p.add_argument("--model", default="yolox_s_640_fp32")
    p.add_argument("--threshold", type=float, default=0.25)
    p.add_argument("--force", action="store_true")
    args = parser.parse_args()
    capture(args) if args.cmd == "capture" else prelabel(args)


if __name__ == "__main__":
    main()
