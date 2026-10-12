"""Replay real-arm emote traces in the stepper-arm sim and compare.

    python tools/sim/compare_real.py apps/stepper_desk/recordings/emote-traces.json
    python tools/sim/compare_real.py TRACES --out sim/build/compare   # plots + summary.json

TRACES is written by apps/stepper_desk/scripts/emote.py: per clip, the commanded joint
trajectory (shoulder, elbow, head at 50 Hz) and the angles the real arm reported while
streaming it. The same command drives sim/stepper_desk.toml's position actuators; the
report gives RMS error against the command for the real arm (raw, and with its streaming
delay removed) and for the sim. Plots need matplotlib.
"""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
os.environ.setdefault("ROBOT_CONFIG", str(ROOT / "sim" / "stepper_desk.toml"))
sys.path.insert(0, str(ROOT / "motion_lib"))
import mujoco  # noqa: E402
from robot import Robot  # noqa: E402

JOINTS = ["shoulder", "elbow", "head"]


def simulate(model, data, t, q, rate):
    """Feed the command at the control rate; return the sim's joint angles (deg)."""
    sub = int(round(1 / rate / model.opt.timestep))
    mujoco.mj_resetData(model, data)
    data.qpos[:] = np.radians(q[0])
    out = []
    for target in q:
        data.ctrl[:] = np.radians(target)
        for _ in range(sub):
            mujoco.mj_step(model, data)
        out.append(np.degrees(data.qpos.copy()))
    return np.array(out)


def at(times, t, q):
    return np.array([np.interp(times, t, q[:, j]) for j in range(q.shape[1])]).T


def compare(trace, model, data, rate):
    t, q, real = np.array(trace["t"]), np.array(trace["q"]), np.array(trace["real"])
    sim = simulate(model, data, t, q, rate)
    rt, ra = real[:, 0], real[:, 1:4]
    rms = lambda e: np.sqrt(np.mean(e ** 2, 0)).round(2).tolist()
    # The real arm runs a steady streaming delay behind the command: find it (0-300 ms).
    shifts = np.arange(0, 0.301, 0.01)
    delay = float(shifts[np.argmin([np.sqrt(np.mean((at(rt - s, t, q) - ra) ** 2)) for s in shifts])])
    return sim, {
        "real_rms": rms(ra - at(rt, t, q)),
        "real_rms_delay_removed": rms(ra - at(rt - delay, t, q)),
        "real_delay_s": delay,
        "real_max": np.abs(ra - at(rt, t, q)).max(0).round(2).tolist(),
        "sim_rms": rms(sim - q),
        "sim_max": np.abs(sim - q).max(0).round(2).tolist(),
        "real_vs_sim_rms": rms(ra - at(rt, t, sim)),
    }


def plot(traces, sims, summary, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"cmd": "#888888", "real": "#d9480f", "sim": "#1c7ed6", "fixed": "#f59f00"}
    fig, axes = plt.subplots(len(traces), 3, figsize=(15, 2.3 * len(traces)), squeeze=False)
    for row, (clip, trace) in enumerate(traces.items()):
        t, q, real = np.array(trace["t"]), np.array(trace["q"]), np.array(trace["real"])
        for j in range(3):
            ax = axes[row][j]
            ax.plot(t, q[:, j], "--", color=colors["cmd"], lw=1.6, label="command")
            ax.plot(t, sims[clip][:, j], color=colors["sim"], lw=1.4, label="sim")
            ax.plot(real[:, 0], real[:, j + 1], "o", color=colors["real"], ms=2.4, label="real arm")
            ax.set_title(f"{clip} - {JOINTS[j]}", fontsize=9)
            ax.tick_params(labelsize=7)
            ax.grid(alpha=0.25)
    axes[0][0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / "traces.png", dpi=110)

    clips, x, w = list(summary), np.arange(len(summary)), 0.27
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2), sharey=True)
    for j, ax in enumerate(axes):
        ax.bar(x - w, [summary[c]["real_rms"][j] for c in clips], w, color=colors["real"], label="real vs command")
        ax.bar(x, [summary[c]["real_rms_delay_removed"][j] for c in clips], w, color=colors["fixed"],
               label="real vs command, delay removed")
        ax.bar(x + w, [summary[c]["sim_rms"][j] for c in clips], w, color=colors["sim"], label="sim vs command")
        ax.set_xticks(x, [c.replace("/", "\n") for c in clips], fontsize=7)
        ax.set_title(f"{JOINTS[j]}: RMS tracking error (deg)", fontsize=10)
        ax.grid(axis="y", alpha=0.25)
    axes[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / "summary.png", dpi=110)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("traces", help="JSON written by apps/stepper_desk/scripts/emote.py")
    parser.add_argument("--out", default=str(ROOT / "sim" / "build" / "compare"), help="plots and summary.json")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()

    traces = json.loads(Path(args.traces).read_text())
    robot = Robot()
    model = robot.model()
    data = mujoco.MjData(model)
    sims, summary = {}, {}
    for clip, trace in traces.items():
        if len(trace.get("real", [])) < 2:
            print(f"{clip}: no real-arm samples, skipped")
            continue
        sims[clip], summary[clip] = compare(trace, model, data, robot.rate)
        print(clip, json.dumps(summary[clip]))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    if not args.no_plots and summary:
        plot({c: traces[c] for c in summary}, sims, summary, out)
        print(f"plots: {out / 'traces.png'}, {out / 'summary.png'}")


if __name__ == "__main__":
    main()
