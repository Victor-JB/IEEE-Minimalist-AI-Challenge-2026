#!/usr/bin/env python3
"""Train a small landmark classifier using official HaGRID train/val annotations.

Only NumPy is required for training and inference. The public landmarks are
MediaPipe auto-annotations, not a hand-detection ground truth or a substitute
for evaluating actual deployed-model outputs on held-out photographs.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("gesture_features", ROOT / "apps/luxoiii/luxoiii/perception/gesture_features.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
features, LABELS = module.features, module.LABELS
CLASSES = ("palm", "like", "dislike", "one", "point", "fist", "no_gesture", "peace", "ok")
MAPPING = {"one": 1, "point": 1, "palm": 2, "like": 3, "dislike": 4}


def dataset(root, split):
    rows, labels, categories, subjects, inputs = [], [], [], [], []
    for category in CLASSES:
        path = root / f"{split}-{category}.npz"
        if not path.exists():
            continue
        with np.load(path, allow_pickle=False) as data:
            p = data["points"].astype(np.float32)
            good = np.isfinite(p).all(axis=(1, 2)) & (np.ptp(p, axis=1).min(axis=-1) > 1e-5)
            rows.append(p[good])
            labels.extend(MAPPING.get(str(x), 0) for x in data["labels"][good])
            categories.extend(str(x) for x in data["labels"][good])
            subjects.extend(str(x) for x in data["subject_hashes"][good])
        inputs.append({"path": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    return np.concatenate(rows), np.array(labels), np.array(categories), set(subjects), inputs


def augment(p, categories, rng):
    p = p.copy()
    p -= p[:, :1]
    # Assume no fixed source-camera aspect ratio. This random aspect correction
    # matters only for rotated examples; unrotated features remove axis scaling.
    p[:, :, 0] *= rng.uniform(.55, 1.4, len(p))[:, None]
    full = np.isin(categories, ["point", "one", "palm", "fist", "peace", "ok"])
    angle = rng.uniform(-.35, .35, len(p))
    angle[full] = rng.uniform(-np.pi, np.pi, full.sum())
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.stack((c, -s, s, c), axis=-1).reshape(-1, 2, 2)
    p = np.einsum("nij,nkj->nki", rotation, p)
    p[:, :, 0] *= rng.choice([-1., 1.], len(p))[:, None]
    noise_scale = np.ptp(p, axis=1)[:, None] * .003
    p += rng.normal(size=p.shape).astype(np.float32) * noise_scale
    return p


def probabilities(x, weights):
    h = np.maximum(x @ weights["w1"] + weights["b1"], 0.)
    logits = h @ weights["w2"] + weights["b2"]
    ex = np.exp(logits - logits.max(axis=1, keepdims=True))
    return ex / ex.sum(axis=1, keepdims=True), h


def metrics(y, prediction):
    confusion = np.zeros((len(LABELS), len(LABELS)), dtype=int)
    np.add.at(confusion, (y, prediction), 1)
    recalls = np.diag(confusion) / np.maximum(confusion.sum(axis=1), 1)
    return dict(accuracy=float(np.mean(y == prediction)), macro_recall=float(recalls.mean()),
                recalls={str(label): float(recall) for label, recall in zip(LABELS, recalls)},
                confusion=confusion.tolist())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--hidden", type=int, default=48)
    parser.add_argument("--seed", type=int, default=103)
    args = parser.parse_args()
    p, y, categories, train_subjects, train_inputs = dataset(args.data, "train")
    vp, vy, vc, val_subjects, val_inputs = dataset(args.data, "val")
    if train_subjects & val_subjects:
        raise ValueError("training and validation subjects overlap")
    rng = np.random.default_rng(args.seed)
    x, vx = features(p), features(vp)
    dimensions = x.shape[1]
    weights = {"w1": rng.normal(0, np.sqrt(2/dimensions), (dimensions, args.hidden)).astype(np.float32),
               "b1": np.zeros(args.hidden, np.float32),
               "w2": rng.normal(0, np.sqrt(2/args.hidden), (args.hidden, len(LABELS))).astype(np.float32),
               "b2": np.zeros(len(LABELS), np.float32)}
    first = {key: np.zeros_like(value) for key, value in weights.items()}
    second = {key: np.zeros_like(value) for key, value in weights.items()}
    classes = [np.flatnonzero(y == label) for label in range(len(LABELS))]
    steps, best, best_epoch, history, t0 = 0, -1., 0, [], time.perf_counter()
    for epoch in range(args.epochs):
        # Balanced sampling protects the smaller command classes without
        # suppressing the naturally more numerous non-command examples.
        indices = np.concatenate([rng.choice(group, 5000, replace=True) for group in classes])
        rng.shuffle(indices)
        sampled = p[indices].copy()
        augmented = rng.random(len(indices)) < .5
        sampled[augmented] = augment(sampled[augmented], categories[indices][augmented], rng)
        sx, sy = features(sampled), y[indices]
        lr = .0015 * (.2 + .8 * (1 - epoch / args.epochs))
        for start in range(0, len(indices), 256):
            bx, by = sx[start:start+256], sy[start:start+256]
            prob, hidden = probabilities(bx, weights)
            prob[np.arange(len(by)), by] -= 1
            prob /= len(by)
            d_hidden = (prob @ weights["w2"].T) * (hidden > 0)
            gradients = dict(w2=hidden.T @ prob + .0001 * weights["w2"], b2=prob.sum(axis=0),
                             w1=bx.T @ d_hidden + .0001 * weights["w1"], b1=d_hidden.sum(axis=0))
            steps += 1
            for key, grad in gradients.items():
                first[key] = .9 * first[key] + .1 * grad
                second[key] = .999 * second[key] + .001 * grad**2
                weights[key] -= lr * (first[key] / (1 - .9**steps)) / (np.sqrt(second[key] / (1 - .999**steps)) + 1e-8)
        pred = probabilities(vx, weights)[0].argmax(axis=1)
        result = metrics(vy, pred)
        history.append({"epoch": epoch+1, **result})
        if result["macro_recall"] > best:
            best, best_epoch = result["macro_recall"], epoch+1
            args.output.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(args.output, feature_version=np.array(module.FEATURE_VERSION), **weights)
        if epoch % 10 == 0 or epoch == args.epochs - 1:
            print(json.dumps({"epoch": epoch+1, "accuracy": result["accuracy"], "macro_recall": result["macro_recall"], "seconds": round(time.perf_counter()-t0, 1)}), flush=True)
    trained = module.LandmarkClassifier(args.output)
    prob = trained.probabilities(vp)
    pred = prob.argmax(axis=1)
    report = dict(dataset="HaGRIDv2 official auto-annotated 2D landmarks", labels=LABELS,
                  training_split="train", selection_split="val", seed=args.seed,
                  train_samples=len(y), val_samples=len(vy), train_subjects=len(train_subjects),
                  val_subjects=len(val_subjects), subject_overlap=0, inputs=train_inputs+val_inputs,
                  model={"feature_version": module.FEATURE_VERSION, "features": dimensions, "hidden": args.hidden,
                         "bytes": args.output.stat().st_size, "sha256": hashlib.sha256(args.output.read_bytes()).hexdigest()},
                  best_epoch=best_epoch, validation=metrics(vy, pred),
                  validation_by_source_label={label: metrics(vy[vc == label], pred[vc == label]) for label in np.unique(vc)},
                  confidence_thresholds={str(threshold): metrics(vy, np.where(prob.max(axis=1) >= threshold, pred, 0))
                                         for threshold in [.4, .5, .6, .7, .8]},
                  history=history, training_seconds=time.perf_counter()-t0)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({"best_epoch": best_epoch, "validation": report["validation"], "model": report["model"]}), flush=True)


if __name__ == "__main__":
    main()
