"""Book detector accuracy + latency on a folder of annotated images.

    python bench/eval_detector.py                                  # shipped model, COCO stand-in set
    python bench/eval_detector.py --model effdet_lite0_320_int8    # a candidate from bench/models/
    python bench/eval_detector.py --data bench/data/desk           # the real lamp-camera set
    python bench/eval_detector.py --sweep 0.3 0.4 0.5 0.6          # several score thresholds, one pass
    python bench/eval_detector.py --roi 0.5                        # crop around each GT book (AIM / HOLD)

Metrics (per score threshold):
  recall      matched books / annotated books, a match being IoU >= 0.5 (each book matched once)
  img hit     positive images with at least one matched book: what BOOK_FOUND needs
  FP/frame    unmatched detections per image, over all images. A detection that overlaps an
              "ignore" box (IoU >= 0.5, or mostly inside it) is neither a hit nor a false positive.
  neg FP img  share of book-free images with any detection: what a false BOOK_FOUND needs
  latency     detect() wall time (preprocess + invoke + postprocess), p50 / p90 ms, this machine.
              Mac latency is NOT board latency: re-run on the UNO Q (and with bench_detector.py).

Annotation format: <data>/annotations.json next to <data>/images/
    {"images": [
        {"file": "0001.jpg", "boxes": [[x, y, w, h], ...], "ignore": [[x, y, w, h], ...]},
        {"file": "0002.jpg", "boxes": [], "ignore": []}            <- a negative frame
    ]}
  x, y = top-left corner, in pixels of that image. "ignore" is optional: books too small or too
  cut off to count (a far shelf, a sliver at the frame edge). Images with no "boxes" are negatives.
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from book_candidates import load  # noqa: E402

DEFAULT_DATA = Path(__file__).resolve().parent / "data" / "coco_books"


def iou(a, b):
    """IoU of two (x, y, w, h) boxes."""
    w = min(a[0] + a[2], b[0] + b[2]) - max(a[0], b[0])
    h = min(a[1] + a[3], b[1] + b[3]) - max(a[1], b[1])
    if w <= 0 or h <= 0:
        return 0.0
    inter = w * h
    return inter / (a[2] * a[3] + b[2] * b[3] - inter)


def inside(det, box):
    """Share of det's area that lies inside box."""
    w = min(det[0] + det[2], box[0] + box[2]) - max(det[0], box[0])
    h = min(det[1] + det[3], box[1] + box[3]) - max(det[1], box[1])
    return max(w, 0) * max(h, 0) / max(det[2] * det[3], 1e-9)


def match(dets, boxes, ignore, iou_threshold=0.5):
    """(true positives, false positives) for one image; dets sorted best first."""
    used, tp, fp = set(), 0, 0
    for d in sorted(dets, key=lambda d: -d[4]):
        best, best_iou = None, iou_threshold
        for j, g in enumerate(boxes):
            v = iou(d, g)
            if j not in used and v >= best_iou:
                best, best_iou = j, v
        if best is not None:
            used.add(best)
            tp += 1
        elif not any(iou(d, g) >= iou_threshold or inside(d, g) >= 0.5 for g in ignore):
            fp += 1
    return tp, fp


def score(results, threshold):
    """Metrics at one score threshold from cached (entry, detections) pairs."""
    n_gt = tp = fp = hits = pos = neg = neg_fp = 0
    for entry, dets in results:
        dets = [d for d in dets if d[4] >= threshold]
        t, f = match(dets, entry["boxes"], entry.get("ignore", []))
        n_gt, tp, fp = n_gt + len(entry["boxes"]), tp + t, fp + f
        if entry["boxes"]:
            pos, hits = pos + 1, hits + (t > 0)
        else:
            neg, neg_fp = neg + 1, neg_fp + (f > 0)
    return {"threshold": threshold, "recall": tp / max(n_gt, 1), "img_hit": hits / max(pos, 1),
            "fp_per_frame": fp / max(len(results), 1), "neg_fp_img": neg_fp / max(neg, 1),
            "books": n_gt, "pos": pos, "neg": neg}


def grow(box, pad, shape):
    """A crop around a GT book like Perception's book_roi: pad box widths on each side."""
    x, y, w, h = box
    x0, y0 = max(0, int(x - pad * w)), max(0, int(y - pad * h))
    x1, y1 = min(shape[1], int(x + w + pad * w)), min(shape[0], int(y + h + pad * h))
    return x0, y0, x1 - x0, y1 - y0


def run(args):
    data = Path(args.data)
    entries = json.loads((data / "annotations.json").read_text())["images"][: args.limit]
    det = load(args.model, score_threshold=min(args.sweep), num_threads=args.threads)
    if not det.available:
        sys.exit(f"model {args.model} did not load")
    frame = cv2.imread(str(data / "images" / entries[0]["file"]))
    for _ in range(5):                                  # warm up (allocations, XNNPACK packing)
        det.detect(frame)

    results, ms = [], []
    for entry in entries:
        frame = cv2.imread(str(data / "images" / entry["file"]))
        if frame is None:
            sys.exit(f"cannot read {entry['file']}")
        if args.roi is not None:
            # One crop per GT book; the crop is the whole "image", so books outside it don't count.
            for g in entry["boxes"]:
                roi = grow(g, args.roi, frame.shape)
                t0 = time.perf_counter()
                dets = det.detect(frame, roi)
                ms.append((time.perf_counter() - t0) * 1000)
                others = [b for b in entry["boxes"] + entry.get("ignore", []) if b is not g]
                results.append(({"boxes": [g], "ignore": others}, dets))
            continue
        t0 = time.perf_counter()
        dets = det.detect(frame)
        ms.append((time.perf_counter() - t0) * 1000)
        results.append((entry, dets))
    return results, np.array(ms)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default="app", help="'app', a bench/models/ name, or a model folder")
    parser.add_argument("--data", default=str(DEFAULT_DATA))
    parser.add_argument("--sweep", type=float, nargs="+", default=[0.5], help="score thresholds")
    parser.add_argument("--threads", type=int, default=os.cpu_count())
    parser.add_argument("--roi", type=float, help="crop around each GT book, padded by this many box sizes")
    parser.add_argument("--limit", type=int, help="first N images only")
    parser.add_argument("--json", help="also append the results to this JSON-lines file")
    args = parser.parse_args()

    results, ms = run(args)
    p50, p90 = np.percentile(ms, 50), np.percentile(ms, 90)
    rows = [score(results, t) for t in args.sweep]
    r = rows[0]
    print(f"{args.model} on {Path(args.data).name}{' roi %.1f' % args.roi if args.roi is not None else ''}: "
          f"{r['pos']} positive / {r['neg']} negative images, {r['books']} books, {args.threads} threads\n")
    print("| model | thr | recall @0.5 | img hit | FP/frame | neg FP img | p50 / p90 ms |")
    print("|---|---|---|---|---|---|---|")
    for r in rows:
        print(f"| {args.model} | {r['threshold']:.2f} | {r['recall']:.3f} | {r['img_hit']:.3f} "
              f"| {r['fp_per_frame']:.3f} | {r['neg_fp_img']:.3f} | {p50:.1f} / {p90:.1f} |")
    if args.json:
        with open(args.json, "a") as f:
            for r in rows:
                f.write(json.dumps({"model": args.model, "data": Path(args.data).name, "roi": args.roi,
                                    "threads": args.threads, "p50_ms": p50, "p90_ms": p90, **r}) + "\n")


if __name__ == "__main__":
    main()
