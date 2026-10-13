"""Export Megvii YOLOX (nano / tiny) to a book-only TFLite at any input size, fp32 and int8.

    python bench/export_yolox.py --yolox-src ~/src/YOLOX-0.3.0 --weights yolox_nano.pth --size 320
    python bench/export_yolox.py ... --weights my_finetune.pth --classes 1     # 1-class fine-tune

Output: bench/models/yolox_<variant>_<size>_{fp32,int8}/ with model.tflite + metadata.json,
ready for eval_detector.py --model <name>, and for the app (copy the int8 folder to
models/book_det-tflite-w8a8/).

Not part of the app: a dev-machine tool, needs extra packages (Mac or Linux, Python 3.12):
    pip install torch onnx onnxscript onnx2tf tensorflow onnx-graphsurgeon sng4onnx loguru tabulate
Sources (Apache-2.0, no login): code https://github.com/Megvii-BaseDetection/YOLOX (tag 0.3.0),
weights https://github.com/Megvii-BaseDetection/YOLOX/releases/tag/0.1.1rc0 (yolox_nano.pth,
yolox_tiny.pth).

What the exported graph does, so the app's decoder stays trivial (BookDetector, "boxes_scores"):
    input   [1, S, S, 3] BGR 0-255 (uint8 in the int8 model: the raw letterboxed pixels)
    boxes   [1, N, 4]    x1, y1, x2, y2 in input pixels (grid + stride decode done in-graph)
    scores  [1, N + 1]   objectness x P(book): only the book row of the 80-class conv is kept, so
                         the other 79 class maps are never computed. The extra last entry is a
                         constant 1 that pins the int8 range to [0, 1] (see BookHead.forward).
    TFLite drops the output names; BookDetector tells boxes from scores by rank.
int8: TFLite full-integer post-training quantization (per-channel weights, int8 activations,
uint8 I/O), calibrated on bench/data/coco_calib/ (train2017 images, half with books, letterboxed
exactly like the app does it). Build that set with coco_bookset.py --calib 200.
"""

import argparse
import json
import subprocess
import sys
import types
from pathlib import Path

import cv2
import numpy as np
import torch
from torch import nn

BENCH = Path(__file__).resolve().parent
VARIANTS = {"nano": (0.33, 0.25, True), "tiny": (0.33, 0.375, False), "s": (0.33, 0.50, False)}
BOOK_CLASS = 73


def build_yolox(src, variant, num_classes):
    sys.path.insert(0, str(src))
    sys.modules.setdefault("thop", types.SimpleNamespace(profile=None))  # only used for FLOP counts
    from yolox.models import YOLOX, YOLOPAFPN, YOLOXHead

    depth, width, depthwise = VARIANTS[variant]
    in_channels = [256, 512, 1024]
    backbone = YOLOPAFPN(depth, width, in_channels=in_channels, depthwise=depthwise)
    head = YOLOXHead(num_classes, width, in_channels=in_channels, depthwise=depthwise)
    model = YOLOX(backbone, head)
    for m in model.modules():          # as YOLOX's Exp.get_model (init_yolo): the weights were trained so
        if isinstance(m, nn.BatchNorm2d):
            m.eps, m.momentum = 1e-3, 0.03
    return model


class SpaceToDepth(nn.Module):
    """YOLOX's Focus slicing (x[..., ::2, ::2] etc.) as a one-hot 2x2 stride-2 conv: exact, and it
    converts cleanly (onnx2tf mangles the strided slices when it transposes to NHWC)."""

    def __init__(self, focus):
        super().__init__()
        self.conv = focus.conv
        self.s2d = nn.Conv2d(3, 12, 2, stride=2, bias=False)
        w = torch.zeros(12, 3, 2, 2)
        # Focus concatenates top-left, bottom-left, top-right, bottom-right; (dy, dx) of each:
        for k, (dy, dx) in enumerate([(0, 0), (1, 0), (0, 1), (1, 1)]):
            for c in range(3):
                w[k * 3 + c, c, dy, dx] = 1
        self.s2d.weight.data = w

    def forward(self, x):
        return self.conv(self.s2d(x))


class BookHead(nn.Module):
    """YOLOX + in-graph decode -> (boxes xyxy, book score). Input NCHW BGR 0-255."""

    def __init__(self, model, size, book_class):
        super().__init__()
        self.model, self.book_class = model, book_class
        focus = model.backbone.backbone.stem
        check = torch.rand(1, 3, 64, 64)
        model.backbone.backbone.stem = SpaceToDepth(focus)
        with torch.no_grad():
            assert torch.allclose(focus(check), model.backbone.backbone.stem(check), atol=1e-5)
        grids, strides = [], []
        for stride in (8, 16, 32):
            n = size // stride
            yv, xv = torch.meshgrid(torch.arange(n), torch.arange(n), indexing="ij")
            grids.append(torch.stack((xv, yv), 2).view(-1, 2))
            strides.append(torch.full((n * n, 1), stride))
        self.register_buffer("grid", torch.cat(grids).float())
        self.register_buffer("stride", torch.cat(strides).float())

        # Keep only the book row of each class conv: 1 output channel instead of 80.
        head = model.head
        for k, conv in enumerate(head.cls_preds):
            one = nn.Conv2d(conv.in_channels, 1, 1)
            one.weight.data = conv.weight.data[book_class: book_class + 1].clone()
            one.bias.data = conv.bias.data[book_class: book_class + 1].clone()
            head.cls_preds[k] = one

    def forward(self, x):
        # YOLOXHead.forward, rewritten per level NHWC-first so onnx2tf keeps the layout straight.
        # Box offsets and scores stay in separate tensors all the way: in int8 every tensor has one
        # scale, and a concat of offsets (about +-5) with probabilities would leave the scores ~25 levels.
        head, regs, scores = self.model.head, [], []
        flat = lambda t, c: t.permute(0, 2, 3, 1).reshape(1, -1, c)   # noqa: E731  [1,C,H,W] -> [1,HW,C]
        for k, feat in enumerate(self.model.backbone(x)):
            feat = head.stems[k](feat)
            reg_feat = head.reg_convs[k](feat)
            regs.append(flat(head.reg_preds[k](reg_feat), 4))
            obj = head.obj_preds[k](reg_feat).sigmoid()
            cls = head.cls_preds[k](head.cls_convs[k](feat)).sigmoid()
            scores.append(flat(obj * cls, 1))
        reg = torch.cat(regs, 1)                                                 # [1, N, 4]
        xy = (reg[..., :2] + self.grid) * self.stride
        wh = torch.exp(reg[..., 2:4]) * self.stride
        boxes = torch.cat([xy - wh / 2, xy + wh / 2], -1)
        # A constant 1 appended to the scores pins their int8 range to [0, 1]: TFLite's int8 concat
        # shares one scale between inputs and output. Without it the range comes from the highest
        # book score seen in calibration (0.34 for nano at 320) and every surer book saturates there.
        scores = torch.cat(scores + [torch.ones(1, 1, 1)], 1)[..., 0]           # [1, N + 1]
        return boxes, scores


def letterbox(image, size, pad=114):
    h, w = image.shape[:2]
    s = min(size / w, size / h)
    image = cv2.resize(image, (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), pad, np.uint8)
    canvas[: image.shape[0], : image.shape[1]] = image
    return canvas


def calibration(size, n):
    files = sorted((BENCH / "data" / "coco_calib").glob("*.jpg"))[:n]
    if not files:
        sys.exit("no calibration images: run bench/coco_bookset.py --calib 200")
    return np.stack([letterbox(cv2.imread(str(f)), size) for f in files]).astype(np.float32)   # NHWC BGR


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--yolox-src", required=True, help="unpacked YOLOX source (for yolox.models)")
    parser.add_argument("--weights", required=True)
    parser.add_argument("--variant", choices=VARIANTS, help="default: from the weights file name")
    parser.add_argument("--size", type=int, default=320)
    parser.add_argument("--classes", type=int, default=80, help="80 (COCO weights) or 1 (book-only fine-tune)")
    parser.add_argument("--calib", type=int, default=200, help="calibration images for int8")
    parser.add_argument("--name", help="output name stem (default yolox_<variant>_<size>)")
    args = parser.parse_args()

    variant = args.variant or next(v for v in VARIANTS if v in Path(args.weights).stem)
    stem = args.name or f"yolox_{variant}_{args.size}"
    model = build_yolox(Path(args.yolox_src).expanduser(), variant, args.classes)
    ckpt = torch.load(args.weights, map_location="cpu", weights_only=False)
    model.load_state_dict(ckpt.get("model", ckpt))
    model.eval()
    calib = calibration(args.size, args.calib)
    book = BOOK_CLASS if args.classes == 80 else 0
    x = torch.from_numpy(calib[:4]).permute(0, 3, 1, 2).contiguous()
    with torch.no_grad():
        ref = model(x)                                   # upstream decode: cx, cy, w, h, obj, classes
    net = BookHead(model, args.size, book).eval()
    with torch.no_grad():                                # our rewritten head must give the same answer
        boxes, scores = net(x[:1])
    ref_boxes = torch.cat([ref[:1, :, :2] - ref[:1, :, 2:4] / 2, ref[:1, :, :2] + ref[:1, :, 2:4] / 2], -1)
    assert torch.allclose(boxes, ref_boxes, atol=1e-2), "box decode differs from upstream YOLOX"
    assert torch.allclose(scores[:, :-1], ref[:1, :, 4] * ref[:1, :, 5 + book], atol=1e-5), "scores differ"

    work = BENCH / "models" / "_work" / stem
    work.mkdir(parents=True, exist_ok=True)
    onnx_path = work / f"{stem}.onnx"
    torch.onnx.export(net, torch.zeros(1, 3, args.size, args.size), str(onnx_path), opset_version=17,
                      input_names=["image"], output_names=["boxes", "scores"], dynamo=False)
    np.save(work / "calib.npy", calib)
    # onnx2tf checks its conversion on sample images it would otherwise download (a pickled
    # .npy from its GitHub releases) into the working directory: hand it ours instead.
    sample = np.stack([cv2.resize(c, (128, 128)) for c in calib[:20]]) / 255
    np.save(work / "calibration_image_sample_data_20x128x128x3_float32.npy", sample.astype(np.float32))
    # onnx2tf: NCHW -> NHWC graph, fp32 TFLite, then full-integer quantization with uint8 I/O.
    subprocess.run([sys.executable, "-m", "onnx2tf", "-i", str(onnx_path), "-o", str(work / "tf"),
                    "-oiqt", "-qt", "per-channel", "-iqd", "uint8", "-oqd", "uint8",
                    "-cind", "image", str(work / "calib.npy"), "[[[[0,0,0]]]]", "[[[[1,1,1]]]]",
                    "-osd", "-n"], check=True, cwd=work)

    meta_common = {"decoder": "boxes_scores", "channels": "bgr", "input_range": [0, 255], "pad_value": 114,
                   "source": "Megvii YOLOX (Apache-2.0), github.com/Megvii-BaseDetection/YOLOX, "
                             f"weights {Path(args.weights).name}; exported by bench/export_yolox.py",
                   "license": "Apache-2.0", "input_size": args.size, "classes": "book only (COCO class 73)"}
    for precision, produced in [("fp32", f"{stem}_float32.tflite"), ("int8", f"{stem}_full_integer_quant.tflite")]:
        out = BENCH / "models" / f"{stem}_{precision}"
        out.mkdir(parents=True, exist_ok=True)
        (out / "model.tflite").write_bytes((work / "tf" / produced).read_bytes())
        lumen = {**meta_common, "precision": precision}
        if precision == "int8":
            lumen["calibration"] = f"{len(calib)} COCO train2017 images (bench/data/coco_calib), letterboxed"
        (out / "metadata.json").write_text(json.dumps({"model_id": stem, "model_files": {"model.tflite": {}},
                                                       "lumen": lumen}, indent=2))
        print(f"wrote {out} ({(out / 'model.tflite').stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
