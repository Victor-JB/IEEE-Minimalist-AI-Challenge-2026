#!/usr/bin/env python3
"""Render the recorded validation curve; optionally embed it in the demo page."""
import argparse
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = Path(__file__).resolve().parents[2]
REPORTS = ROOT / "reports/gestures"
START = "<!-- training-accuracy-chart:start -->"
END = "<!-- training-accuracy-chart:end -->"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--embed-preview", action="store_true")
    args = parser.parse_args()
    report = json.loads((REPORTS / "training-report.json").read_text())
    history = report["history"]
    epochs = [row["epoch"] for row in history]
    accuracy = [100 * row["accuracy"] for row in history]
    assert epochs == list(range(1, 101))
    assert all(80 <= value <= 100 for value in accuracy)
    assert report["best_epoch"] == 100

    caslon = Path("/System/Library/Fonts/Supplemental/BigCaslon.ttf")
    if caslon.exists():
        font_manager.fontManager.addfont(caslon)
    heading_font = "Big Caslon" if caslon.exists() else "DejaVu Serif"
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 12,
                         "svg.hashsalt": "luxoiii-training-accuracy-v1"})
    ink, muted, rule, amber, paper = "#514839", "#817767", "#d9d2c5", "#a96b25", "#fcfaf6"
    fig, ax = plt.subplots(figsize=(10, 4.8), facecolor=paper)
    fig.subplots_adjust(left=.085, right=.965, top=.75, bottom=.21)
    ax.set_facecolor(paper)
    fig.text(.085, .925, "Validation accuracy during training", fontsize=24,
             fontfamily=heading_font, color=ink)
    fig.text(.085, .85, "4,447 supplied landmark hands · 100 recorded epochs", fontsize=12, color=muted)
    ax.set(xlim=(1, 100), ylim=(80, 100), xticks=[1, 20, 40, 60, 80, 100],
           yticks=[80, 85, 90, 95, 100], xlabel="Epoch", ylabel="Accuracy (%)")
    ax.tick_params(axis="both", colors=muted, length=0, pad=9)
    ax.xaxis.label.set_color(muted)
    ax.yaxis.label.set_color(muted)
    ax.xaxis.labelpad = 12
    ax.yaxis.labelpad = 12
    for side in ["top", "right"]:
        ax.spines[side].set_visible(False)
    for side in ["left", "bottom"]:
        ax.spines[side].set_color(rule)
    ax.grid(axis="y", color=rule, linewidth=.7)
    ax.set_axisbelow(True)
    ax.plot(epochs, accuracy, color=amber, linewidth=2, label="Validation accuracy")
    ax.scatter([epochs[0], epochs[-1]], [accuracy[0], accuracy[-1]],
               s=40, color=amber, zorder=3, clip_on=False)
    ax.annotate(f"{accuracy[0]:.1f}%", (epochs[0], accuracy[0]), xytext=(12, 0),
                textcoords="offset points", va="center", fontsize=13, color=ink)
    ax.annotate(f"{accuracy[-1]:.1f}%", (epochs[-1], accuracy[-1]), xytext=(-9, -22),
                textcoords="offset points", ha="right", fontsize=13, color=ink)
    fig.text(.085, .025, "Validation on supplied landmarks; not live camera accuracy. Y-axis: 80–100%.",
             fontsize=10, color=muted)
    svg_path = REPORTS / "training-accuracy.svg"
    fig.savefig(svg_path, metadata={"Date": None})
    svg_path.write_text("\n".join(line.rstrip() for line in svg_path.read_text().splitlines()) + "\n")
    fig.savefig(REPORTS / "training-accuracy.png", dpi=180)
    plt.close(fig)

    if args.embed_preview:
        svg = svg_path.read_text()
        svg = svg[svg.index("<svg"):]
        end = svg.index(">")
        tag = re.sub(r'\s(?:width|height)="[^"]+"', "", svg[:end])
        tag = tag.replace("<svg", '<svg class="training-chart-svg" aria-hidden="true" focusable="false"', 1)
        svg = tag + svg[end:]
        svg = svg.replace("*{stroke-linejoin", ".training-chart-svg *{stroke-linejoin")
        page = ROOT / "apps/luxoiii/luxoiii/desk/preview.html"
        html = page.read_text()
        assert html.count(START) == html.count(END) == 1
        before, rest = html.split(START)
        _, after = rest.split(END)
        page.write_text(before + START + "\n" + svg.strip() + "\n" + END + after)
    print(f"Plotted {len(history)} recorded epochs: {accuracy[0]:.4f}% → {accuracy[-1]:.4f}%")


if __name__ == "__main__":
    main()
