#!/usr/bin/env python3
"""Plot paired held-out photo scores; optionally embed them in the demo page."""
import argparse
import gzip
import hashlib
import json
import math
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = Path(__file__).resolve().parents[2]
REPORTS = ROOT / "reports/gestures"
START = "<!-- gesture-comparison-chart:start -->"
END = "<!-- gesture-comparison-chart:end -->"


def paired_scores():
    old, new = [json.loads((REPORTS / name).read_text()) for name in
                ("test-original-score.json", "test-new-score.json")]
    for key in ("manifest_sha256", "cache_sha256", "landmark_threshold",
                "images", "gt_hands", "matched_hands", "missed_hands"):
        if old[key] != new[key]:
            raise ValueError(f"Before/after inputs differ: {key}")
    manifest_bytes = (REPORTS / "test-manifest.json").read_bytes()
    if hashlib.sha256(manifest_bytes).hexdigest() != old["manifest_sha256"]:
        raise ValueError("Photo manifest differs from the scored source")
    cache_bytes = gzip.decompress((REPORTS / "test-landmarks.json.gz").read_bytes())
    if hashlib.sha256(cache_bytes).hexdigest() != old["cache_sha256"]:
        raise ValueError("Landmark cache differs from the scored source")
    manifest = json.loads(manifest_bytes)
    rows = manifest["rows"]
    if manifest["split"] != "test" or len(rows) != old["images"]:
        raise ValueError("Expected the complete held-out photo test")
    metrics = []
    for label, gesture in (("All gestures", None), ("Pointing", "point")):
        counts = []
        for score in (old, new):
            correct = score["correct"] if gesture is None else score["confusion"].get("point -> point", 0)
            total = score["gt_hands"] if gesture is None else sum(
                count for key, count in score["confusion"].items() if key.startswith("point -> "))
            if not 0 <= correct <= total or not total:
                raise ValueError(f"Invalid counts for {label}")
            counts.append((correct, total))
        if counts[0][1] != counts[1][1]:
            raise ValueError(f"Before/after denominators differ: {label}")
        metrics.append(dict(label=label, counts=counts,
                            values=[100 * correct / total for correct, total in counts]))
    for score, value in zip((old, new), metrics[0]["values"]):
        if not math.isclose(value / 100, score["end_to_end_recall"]):
            raise ValueError("Recall does not match correct/total counts")
    return metrics, len(rows), len({row["subject_hash"] for row in rows})


def namespace_svg(svg):
    """Matplotlib glyph IDs otherwise collide with the adjacent training chart."""
    svg = re.sub(r'\bid="([^"]+)"', lambda m: f'id="comparison-{m[1]}"', svg)
    svg = re.sub(r'((?:xlink:)?href=")#([^"]+)(")',
                 lambda m: m[1] + "#comparison-" + m[2] + m[3], svg)
    svg = re.sub(r'url\(#([^)]+)\)', lambda m: f'url(#comparison-{m[1]})', svg)
    svg = svg.replace("<svg ", '<svg class="comparison-chart-svg" ', 1)
    svg = svg.replace("*{stroke-linejoin", ".comparison-chart-svg *{stroke-linejoin")
    ids = re.findall(r'\bid="([^"]+)"', svg)
    refs = re.findall(r'(?:href="#|url\(#)([^"\)]+)', svg)
    if len(ids) != len(set(ids)) or not set(refs) <= set(ids):
        raise ValueError("Comparison SVG has duplicate or unresolved references")
    return "\n".join(line.rstrip() for line in svg.splitlines()) + "\n"


def embed(svg, metrics, images, subjects):
    svg = svg[svg.index("<svg"):]
    end = svg.index(">")
    tag = re.sub(r'\s(?:width|height)="[^"]+"', "", svg[:end])
    svg = tag.replace("<svg", '<svg aria-hidden="true" focusable="false"', 1) + svg[end:]
    overall, pointing = metrics
    old_point, new_point = pointing["counts"]
    gain = overall["values"][1] - overall["values"][0]
    point_gain = pointing["values"][1] - pointing["values"][0]
    figure = f'''  <figure class="comparison-chart" aria-labelledby="comparison-chart-title" aria-describedby="comparison-chart-description">
    <div class="comparison-chart-scroll">
{svg.strip()}
    </div>
    <figcaption>
      <h3 id="comparison-chart-title">We improved pointing by {point_gain:g} percentage points.</h3>
      <p id="comparison-chart-description">We compared our old rules and new classifier on the same model outputs from {images} held-out HaGRID photographs of {subjects} subjects. Overall correct gesture recall rose from {overall['values'][0]:.1f}% to {overall['values'][1]:.1f}% (+{gain:.1f} percentage points); pointing rose from {old_point[0]}/{old_point[1]} ({pointing['values'][0]:.1f}%) to {new_point[0]}/{new_point[1]} ({pointing['values'][1]:.1f}%). Both measures include missed hands. These still-photo results do not measure live camera accuracy.</p>
    </figcaption>
  </figure>'''
    page = ROOT / "apps/luxoiii/luxoiii/desk/preview.html"
    html = page.read_text()
    if html.count(START) != 1 or html.count(END) != 1:
        raise ValueError("Expected one comparison-chart insertion point")
    before, rest = html.split(START)
    _, after = rest.split(END)
    result = before + START + "\n" + figure + "\n" + END + after
    ids = re.findall(r'\bid="([^"]+)"', result)
    if len(ids) != len(set(ids)):
        raise ValueError("Embedded chart would create duplicate HTML IDs")
    page.write_text(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--embed-preview", action="store_true")
    args = parser.parse_args()
    metrics, images, subjects = paired_scores()
    caslon = Path("/System/Library/Fonts/Supplemental/BigCaslon.ttf")
    if caslon.exists():
        font_manager.fontManager.addfont(caslon)
    heading_font = "Big Caslon" if caslon.exists() else "DejaVu Serif"
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 12,
                         "svg.hashsalt": "luxoiii-gesture-comparison-v1"})
    ink, muted, rule, amber, paper = "#514839", "#817767", "#d9d2c5", "#a96b25", "#fcfaf6"
    fig, ax = plt.subplots(figsize=(10, 5.6), facecolor=paper)
    fig.subplots_adjust(left=.095, right=.965, top=.75, bottom=.18)
    ax.set_facecolor(paper)
    fig.text(.095, .93, "Gesture recognition, before & after", fontsize=24,
             fontfamily=heading_font, color=ink)
    fig.text(.095, .865, f"{images} held-out photographs · {subjects} subjects · missed hands included",
             fontsize=12, color=muted)
    ax.set(xlim=(-.55, 1.85), ylim=(0, 100), xticks=[0, 1.3],
           xticklabels=[metric["label"] for metric in metrics], yticks=range(0, 101, 20),
           ylabel="Correct recall (%)")
    ax.tick_params(axis="both", colors=muted, length=0, pad=10)
    ax.tick_params(axis="x", labelsize=14)
    ax.yaxis.label.set_color(muted)
    ax.yaxis.labelpad = 12
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(rule)
    ax.grid(axis="y", color=rule, linewidth=.7)
    ax.set_axisbelow(True)
    for index, (label, color, offset) in enumerate((("Old rules", muted, -.18),
                                                  ("New model", amber, .18))):
        positions = [center + offset for center in (0, 1.3)]
        values = [metric["values"][index] for metric in metrics]
        ax.bar(positions, values, width=.30, color=color, label=label)
        for x, value, metric in zip(positions, values, metrics):
            correct, total = metric["counts"][index]
            ax.annotate(f"{value:.1f}%", (x, value), xytext=(0, 23), textcoords="offset points",
                        ha="center", fontsize=14, color=ink)
            ax.annotate(f"{correct} / {total}", (x, value), xytext=(0, 7), textcoords="offset points",
                        ha="center", fontsize=10, color=muted)
    for x, metric in zip((0, 1.3), metrics):
        gain = metric["values"][1] - metric["values"][0]
        ax.text(x, 92, f"+{gain:.1f} percentage points", ha="center", fontsize=12, color=amber,
                bbox=dict(facecolor=paper, edgecolor="none", pad=4))
    ax.get_xticklabels()[1].set_color(amber)
    ax.legend(loc="lower left", bbox_to_anchor=(0, 1.035), ncol=2, frameon=False,
              fontsize=11, labelcolor=ink, borderaxespad=0, handlelength=1.3)
    fig.text(.095, .045, "Still-photo test on UNO Q model outputs; not live camera accuracy. Full 0–100% scale.",
             fontsize=10, color=muted)
    svg_path = REPORTS / "gesture-comparison.svg"
    fig.savefig(svg_path, metadata={"Date": None})
    svg = namespace_svg(svg_path.read_text())
    svg_path.write_text(svg)
    fig.savefig(REPORTS / "gesture-comparison.png", dpi=180)
    plt.close(fig)
    if args.embed_preview:
        embed(svg, metrics, images, subjects)
    print(f"Verified identical source hashes; plotted {images} images, {subjects} subjects")
    for metric in metrics:
        print(f"{metric['label']}: {metric['counts']} → {metric['values']}")


if __name__ == "__main__":
    main()
