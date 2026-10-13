"""Candidate book detectors, all behind the app's BookDetector interface.

    python bench/book_candidates.py fetch          # download the published candidates -> bench/models/
    python bench/book_candidates.py list           # what is there

Each candidate is a folder bench/models/<name>/ with one .tflite and a metadata.json whose
"lumen" block says how to feed and decode it (the same block the shipped model has).
YOLOX models made by export_yolox.py land there too. bench/models/ is gitignored.

Decoders:
  boxes_scores  BookDetector itself (Qualcomm AI Hub YOLO layout, and our YOLOX exports)
  effdet        MediaPipe EfficientDet-Lite: raw anchor offsets + 90-class scores, decoded here

Sources and licenses (checked 2026-10-12):
  yolox_s_640_*   Qualcomm AI Hub Yolo-X v0.64.0 (YOLOX-S), huggingface.co/qualcomm/Yolo-X,
                  Apache-2.0 (Megvii YOLOX). Public S3 asset, no login.
  effdet_lite*    Google MediaPipe object detector models, storage.googleapis.com/mediapipe-models,
                  Apache-2.0. int8 = Google's own post-training full-integer quantization.
  Not used: Qualcomm YOLOv5/v8/v10/v11 (AGPL-3.0), YOLOv6/v7 (GPL-3.0), Ultralytics (AGPL-3.0);
  Qualcomm SSD-MobilenetV2 / EfficientDet / Yolo-NAS repos need a Hugging Face login.
"""

import json
import sys
import urllib.request
import zipfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))     # apps/lumen
from lumen.perception.book import BookDetector  # noqa: E402

BENCH = Path(__file__).resolve().parent
CANDIDATES = BENCH / "models"

QC = "https://qaihub-public-assets.s3.us-west-2.amazonaws.com/qai-hub-models/models/yolox/releases/v0.64.0/"
MP = "https://storage.googleapis.com/mediapipe-models/object_detector/"
PUBLISHED = {
    # name: (url, file inside the zip or None, lumen block)
    "yolox_s_640_w8a8": (QC + "yolox-tflite-w8a8.zip", "yolox-tflite-w8a8/yolox.tflite",
                         {"decoder": "boxes_scores", "book_class": 73, "channels": "bgr", "input_range": [0, 1]}),
    "yolox_s_640_fp32": (QC + "yolox-tflite-float.zip", "yolox-tflite-float/yolox.tflite",
                         {"decoder": "boxes_scores", "book_class": 73, "channels": "bgr", "input_range": [0, 1]}),
    "effdet_lite0_320_int8": (MP + "efficientdet_lite0/int8/latest/efficientdet_lite0.tflite", None,
                              {"decoder": "effdet", "book_class": 83, "channels": "rgb", "input_range": [-1, 1]}),
    "effdet_lite0_320_fp32": (MP + "efficientdet_lite0/float32/latest/efficientdet_lite0.tflite", None,
                              {"decoder": "effdet", "book_class": 83, "channels": "rgb", "input_range": [-1, 1]}),
    "effdet_lite2_448_int8": (MP + "efficientdet_lite2/int8/latest/efficientdet_lite2.tflite", None,
                              {"decoder": "effdet", "book_class": 83, "channels": "rgb", "input_range": [-1, 1]}),
    "effdet_lite2_448_fp32": (MP + "efficientdet_lite2/float32/latest/efficientdet_lite2.tflite", None,
                              {"decoder": "effdet", "book_class": 83, "channels": "rgb", "input_range": [-1, 1]}),
}
SOURCES = {"yolox": "Qualcomm AI Hub Yolo-X v0.64.0 (YOLOX-S), Apache-2.0",
           "effdet": "Google MediaPipe EfficientDet-Lite, Apache-2.0"}


def fetch():
    for name, (url, member, lumen) in PUBLISHED.items():
        out = CANDIDATES / name
        model = out / "model.tflite"
        if model.exists():
            print(f"have {name}")
            continue
        out.mkdir(parents=True, exist_ok=True)
        tmp = out / ("download.zip" if member else "download.tflite")
        print(f"fetching {name} from {url}")
        urllib.request.urlretrieve(url, tmp)
        if member:
            with zipfile.ZipFile(tmp) as z, z.open(member) as src, open(model, "wb") as dst:
                dst.write(src.read())
            tmp.unlink()
        else:
            tmp.rename(model)
        lumen = {**lumen, "source": SOURCES[name.split("_")[0]], "url": url}
        (out / "metadata.json").write_text(json.dumps({"model_files": {"model.tflite": {}}, "lumen": lumen}, indent=2))


def efficientdet_anchors(size, levels=(3, 4, 5, 6, 7), anchor_scale=3.0,
                         aspects=((1.0, 1.0), (1.4, 0.7), (0.7, 1.4))):
    """AutoML EfficientDet anchors as (N, 4) yc, xc, h, w in input pixels, in output order:
    level, then grid row-major, then 3 octave scales x 3 aspect ratios. The Lite models use
    anchor_scale 3 (full EfficientDet uses 4); checked on COCO persons: box/GT size ratio 1.00."""
    out, feat = [], size
    for level in range(1, max(levels) + 1):
        feat = (feat - 1) // 2 + 1                     # AutoML's feature map size at this level
        if level not in levels:
            continue
        stride = size / feat                           # not 2**level when size isn't a multiple of it
        centers = np.arange(stride / 2, size, stride)
        yc, xc = np.meshgrid(centers, centers, indexing="ij")
        per_cell = []
        for octave in range(3):
            base = anchor_scale * stride * 2 ** (octave / 3)
            for ax, ay in aspects:
                per_cell.append(np.stack([yc, xc, np.full_like(yc, base * ay), np.full_like(xc, base * ax)], -1))
        out.append(np.stack(per_cell, 2).reshape(-1, 4))
    return np.concatenate(out)


class EfficientDetBook(BookDetector):
    """MediaPipe EfficientDet-Lite: decode anchor offsets, take the book column, then NMS."""

    def __init__(self, model_dir, **kwargs):
        super().__init__(model_dir, **kwargs)
        if not self.available:
            return
        outs = sorted(self.outputs.values(), key=lambda d: d["shape"][-1])
        self.box_out, self.score_out = outs[0], outs[-1]               # [.., 4] and [.., 90]
        self.anchors = efficientdet_anchors(self.input_w)
        assert len(self.anchors) == self.box_out["shape"][1], "anchor count mismatch"

    def _postprocess(self):
        from lumen.perception.book import nms

        scores = self.interpreter.get_tensor(self.score_out["index"])[0][:, self.book_class]  # probabilities
        keep = scores >= self.score_threshold
        if not keep.any():
            return []
        t = self.interpreter.get_tensor(self.box_out["index"])[0][keep]      # ty, tx, th, tw
        a = self.anchors[keep]
        yc, xc = t[:, 0] * a[:, 2] + a[:, 0], t[:, 1] * a[:, 3] + a[:, 1]
        h, w = np.exp(t[:, 2]) * a[:, 2], np.exp(t[:, 3]) * a[:, 3]
        boxes = np.stack([xc - w / 2, yc - h / 2, xc + w / 2, yc + h / 2], -1) / self.scale
        boxes = np.clip(boxes, 0, [self.crop_w, self.crop_h, self.crop_w, self.crop_h])
        scores = scores[keep]
        return [(int(b[0]), int(b[1]), int(b[2] - b[0]), int(b[3] - b[1]), float(scores[i]))
                for i, b in ((i, boxes[i]) for i in nms(boxes, scores, self.nms_threshold))]


def load(name_or_dir, **kwargs):
    """A detector for bench/models/<name>, any folder with metadata.json, or 'app' (the shipped model)."""
    if name_or_dir == "app":
        return BookDetector(**kwargs)
    path = Path(name_or_dir)
    if not path.exists():
        path = CANDIDATES / name_or_dir
    decoder = json.loads((path / "metadata.json").read_text())["lumen"]["decoder"]
    return (EfficientDetBook if decoder == "effdet" else BookDetector)(path, **kwargs)


def main():
    if sys.argv[1:] == ["fetch"]:
        fetch()
    for d in sorted(CANDIDATES.glob("*/metadata.json")):
        size = sum(f.stat().st_size for f in d.parent.glob("*.tflite")) / 1e6
        print(f"{d.parent.name:<28}{size:>7.1f} MB")


if __name__ == "__main__":
    main()
