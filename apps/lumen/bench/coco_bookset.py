"""Build the stand-in book eval set (and a calibration set) from COCO 2017.

    python bench/coco_bookset.py                       # eval set -> bench/data/coco_books/
    python bench/coco_bookset.py --calib 200           # + int8 calibration images -> bench/data/coco_calib/

Everything lands in bench/data/ (gitignored). Only the standard library is needed.
The annotations zip (~250 MB) is downloaded once from cocodataset.org and cached in
bench/data/; images are fetched one by one from images.cocodataset.org.

Eval set (val2017, never used for calibration or training):
  positives  every image with at least one "book" box covering >= --min-area of the image
  negatives  --negatives random images with no book at all (seeded, so the set is reproducible)
  Books smaller than --min-area, and crowd boxes, become "ignore" regions: missing them is
  not a miss, and detecting them is not a false positive. A desk book seen by the lamp is
  large; a spine on a far-away shelf is not what Lumen is looking for.

Calibration set (train2017, disjoint from the eval set): half images with books, half random.

COCO images are CC BY 4.0 (and Flickr terms); keep them out of git. Output format is the one
eval_detector.py reads, documented there.
"""

import argparse
import json
import random
import sys
import urllib.request
import zipfile
from pathlib import Path

DATA = Path(__file__).resolve().parent / "data"
ANN_URL = "http://images.cocodataset.org/annotations/annotations_trainval2017.zip"
IMG_URL = "http://images.cocodataset.org/{split}/{file}"
BOOK = 84  # COCO category id of "book" (index 73 in the contiguous 80-class list)


def fetch(url, dest):
    if dest.exists():
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    with urllib.request.urlopen(url, timeout=60) as r, open(tmp, "wb") as f:
        while chunk := r.read(1 << 20):
            f.write(chunk)
    tmp.rename(dest)
    return dest


def load_instances(split):
    """instances_<split>.json, extracted from the cached annotations zip."""
    path = DATA / "annotations" / f"instances_{split}.json"
    if not path.exists():
        zip_path = fetch(ANN_URL, DATA / "annotations_trainval2017.zip")
        print(f"extracting {path.name}")
        with zipfile.ZipFile(zip_path) as z:
            z.extract(f"annotations/{path.name}", DATA)
    return json.loads(path.read_text())


def book_boxes(coco):
    """image id -> list of (bbox [x, y, w, h], iscrowd) for books."""
    out = {}
    for a in coco["annotations"]:
        if a["category_id"] == BOOK:
            out.setdefault(a["image_id"], []).append(([round(v, 1) for v in a["bbox"]], a["iscrowd"]))
    return out


def any_objects(coco):
    return {a["image_id"] for a in coco["annotations"]}


def download_images(split, images, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    for i, im in enumerate(images, 1):
        fetch(IMG_URL.format(split=split, file=im["file_name"]), out_dir / im["file_name"])
        if i % 25 == 0 or i == len(images):
            print(f"  {i}/{len(images)} images", end="\r" if i < len(images) else "\n")


def build_eval(args):
    coco = load_instances("val2017")
    books = book_boxes(coco)
    entries, positives = [], 0
    for im in coco["images"]:
        if im["id"] not in books:
            continue
        min_area = args.min_area * im["width"] * im["height"]
        counted = [b for b, crowd in books[im["id"]] if not crowd and b[2] * b[3] >= min_area]
        ignored = [b for b, crowd in books[im["id"]] if crowd or b[2] * b[3] < min_area]
        if counted:
            entries.append({"file": im["file_name"], "boxes": counted, "ignore": ignored})
            positives += 1
    # Negatives: images with objects but no book at all (empty COCO images are rare and odd).
    annotated = any_objects(coco)
    pool = sorted((im for im in coco["images"] if im["id"] not in books and im["id"] in annotated),
                  key=lambda im: im["id"])
    rng = random.Random(args.seed)
    for im in rng.sample(pool, args.negatives):
        entries.append({"file": im["file_name"], "boxes": [], "ignore": []})

    out = DATA / "coco_books"
    by_name = {im["file_name"]: im for im in coco["images"]}
    print(f"eval set: {positives} positive images ({sum(len(e['boxes']) for e in entries)} books "
          f">= {args.min_area:.1%} of the image), {args.negatives} negatives -> {out}")
    download_images("val2017", [by_name[e["file"]] for e in entries], out / "images")
    (out / "annotations.json").write_text(json.dumps({
        "source": "COCO val2017 (CC BY 4.0), built by bench/coco_bookset.py",
        "min_area": args.min_area, "seed": args.seed, "images": entries}, indent=1))


def build_calib(args):
    coco = load_instances("train2017")
    books = book_boxes(coco)
    rng = random.Random(args.seed)
    ims = sorted(coco["images"], key=lambda im: im["id"])
    with_books = [im for im in ims if im["id"] in books]
    others = [im for im in ims if im["id"] not in books]
    half = args.calib // 2
    chosen = rng.sample(with_books, half) + rng.sample(others, args.calib - half)
    out = DATA / "coco_calib"
    print(f"calibration set: {len(chosen)} train2017 images -> {out}")
    download_images("train2017", chosen, out)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--min-area", type=float, default=0.01, help="smallest counted book, fraction of image")
    parser.add_argument("--negatives", type=int, default=150, help="images with no book")
    parser.add_argument("--calib", type=int, default=0, help="also fetch N calibration images (train2017)")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    build_eval(args)
    if args.calib:
        build_calib(args)


if __name__ == "__main__":
    sys.exit(main())
