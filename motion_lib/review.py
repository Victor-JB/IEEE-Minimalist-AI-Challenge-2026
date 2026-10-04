"""Review images: trajectory plots with a filmstrip of sim frames (instead of GIFs).

    python motion_lib/review.py             # every clip -> review/clips/*.png, plus CATALOG.md
    python motion_lib/review.py nod_yes     # only clips whose name starts with this
    python motion_lib/review.py --gamma     # functional moves at gamma 0 / 0.3 / 1 -> gamma_comparison/

Each clip image: 6 rendered frames across the clip, joint angles (keyframes as faint
lines, light events marked), and joint velocities against the speed limit.
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from checks import derivatives  # noqa: E402
from player import list_clips, load_clip, play, ref  # noqa: E402
from robot import Robot, configurator  # noqa: E402

HERE = Path(__file__).parent
CLIP_DIR = HERE / "review" / "clips"
GAMMA_DIR = HERE / "gamma_comparison"
CATALOG_PATH = HERE / "CATALOG.md"
COLORS = dict(zip(["base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_yaw", "head_roll"],
                  ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf"]))


class FrameRenderer:
    def __init__(self, robot, width=220, height=165):
        # Own model copy, since the eye color is changed per frame to show the light.
        self.model = mujoco.MjModel.from_xml_string(configurator.build_mjcf(robot.cfg))
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height, width)
        self.cam = mujoco.MjvCamera()
        self.cam.lookat[:] = [0.06, 0, 0.26]
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 0.72, 145, -12
        self.eye = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_MATERIAL, "eye")

    def frame(self, q_deg, light=1.0):
        self.data.qpos[:] = np.radians(q_deg)
        mujoco.mj_forward(self.model, self.data)
        glow = np.array([1.0, 0.85, 0.3])
        self.model.mat_rgba[self.eye, :3] = 0.05 + (glow - 0.05) * light
        self.renderer.update_scene(self.data, self.cam)
        return self.renderer.render()


def light_at(traj, t):
    level = 1.0
    for e in sorted(traj.events, key=lambda e: e["t"]):
        if e["t"] <= t and "light" in e:
            level = e["light"]
    return level


def filmstrip(ax_row, traj, renderer):
    for ax, t in zip(ax_row, np.linspace(0, traj.duration, len(ax_row))):
        i = min(int(round(t / traj.dt)), len(traj.t) - 1)
        ax.imshow(renderer.frame(traj.q[i], light_at(traj, t)))
        ax.set_title(f"{t:.2f}s", fontsize=8)
        ax.axis("off")


def plot_joints(ax, traj, joints, keyframe_times=(), title=None):
    for name in joints:
        ax.plot(traj.t, traj.q[:, traj.joint_names.index(name)], color=COLORS.get(name), label=name, lw=1.6)
    for t in keyframe_times:
        ax.axvline(t, color="0.85", lw=0.6, zorder=0)
    for e in traj.events:
        if "light" in e:
            ax.annotate(f"light {e['light']:g}", (e["t"], 1), xycoords=("data", "axes fraction"),
                        fontsize=7, color="goldenrod", ha="center", va="bottom")
            ax.axvline(e["t"], color="goldenrod", lw=0.8, ls=":")
    ax.set_ylabel("joint angle (deg)")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=7, ncol=3, loc="best")
    if title:
        ax.set_title(title, fontsize=9, loc="left")


def plot_velocity(ax, traj, joints, robot):
    vel, _, _ = derivatives(traj)
    for name in joints:
        ax.plot(traj.t[1:], vel[:, traj.joint_names.index(name)], color=COLORS.get(name), lw=1.2)
    for sign in (1, -1):
        ax.axhline(sign * robot.max_vel, color="crimson", ls="--", lw=0.8)
    ax.set_ylabel("velocity (deg/s)")
    ax.set_xlabel("time (s)")
    ax.grid(alpha=0.25)


def clip_figure(clip, robot, renderer, out_path):
    traj = play(clip, robot)
    tags = clip["tags"]
    p = clip["params"]
    v = clip.get("validation", {})
    fig = plt.figure(figsize=(12, 8.2))
    grid = fig.add_gridspec(3, 6, height_ratios=[1.1, 1.6, 1.0], hspace=0.35)
    filmstrip([fig.add_subplot(grid[0, i]) for i in range(6)], traj, renderer)
    ax_q = fig.add_subplot(grid[1, :])
    times = sorted({k["t"] for k in clip["keyframes"]})
    plot_joints(ax_q, traj, clip["joint_mask"], times)
    plot_velocity(fig.add_subplot(grid[2, :], sharex=ax_q), traj, clip["joint_mask"], robot)
    fig.suptitle(
        f"{ref(clip)}   [{clip['mode']}, {tags['category']} / {tags['family']}, valence {tags['valence']:+g}, "
        f"arousal {tags['arousal']:g}]   {v.get('status', 'not validated')}\n"
        f"{clip['description']}\namplitude {p['amplitude']['range']}  time_scale {p['time_scale']['range']}  "
        f"duration {traj.duration:.2f}s" + ("  loopable" if clip["loopable"] else ""),
        fontsize=9, x=0.01, ha="left")
    fig.savefig(out_path, dpi=80, bbox_inches="tight")
    plt.close(fig)


def write_catalog(clips):
    lines = [
        "# Clip catalog",
        "",
        "Generated by `python motion_lib/review.py` from `motion_lib/clips/*.json` "
        "(authored in `author.py`, ranges from `validate.py`). Play any clip in the sim with "
        "`python motion_lib/demo.py --clip <name>/<variant>`.",
        "",
        "- **mode**: additive clips are offsets that play from any pose; absolute clips go entry -> exit pose.",
        "- **amplitude / time_scale**: validated ranges (time_scale 2.0 = twice as slow).",
        "- **v / a**: valence (-1..1) / arousal (0..1).",
        "",
    ]
    for category in ("attitude", "attention", "intention", "emotion"):
        lines += [f"## {category.title()}", "",
                  "| clip | mode | family | v / a | duration | amplitude | time_scale | status | plot | description |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
        for c in clips:
            if c["tags"]["category"] != category:
                continue
            p, t = c["params"], c["tags"]
            end = ""
            if c["mode"] == "absolute":
                end = f" ({'any' if not c['entry_pose'] else 'pose'} -> pose)"
            elif c.get("end_offset"):
                end = " (ends offset)"
            lines.append(
                f"| `{ref(c)}` | {c['mode']}{end}{', loop' if c['loopable'] else ''} | {t['family']} "
                f"| {t['valence']:+g} / {t['arousal']:g} | {c['duration_nominal']:.2f}s "
                f"| {p['amplitude']['range'][0]:g}-{p['amplitude']['range'][1]:g} "
                f"| {p['time_scale']['range'][0]:g}-{p['time_scale']['range'][1]:g} "
                f"| {c.get('validation', {}).get('status', '-')} "
                f"| [png](review/clips/{c['name']}__{c['variant']}.png) | {c['description']} |")
        lines.append("")
    CATALOG_PATH.write_text("\n".join(lines), encoding="utf-8")


def gamma_figures(robot):
    from functional import SCENARIOS, run_scenario

    GAMMA_DIR.mkdir(parents=True, exist_ok=True)
    for name, scenario in SCENARIOS.items():
        gammas = (0.0, 0.3, 1.0)
        fig, axes = plt.subplots(len(gammas), 1, figsize=(12, 9), sharex=True)
        for ax, gamma in zip(axes, gammas):
            traj, log = run_scenario(scenario, robot, gamma)
            plot_joints(ax, traj, robot.joint_names, title=f"gamma = {gamma:g}   ({traj.duration:.1f}s)")
            for t0, t1, label in traj.marks:
                if not label.startswith("function"):
                    ax.axvspan(t0, t1, color="gold", alpha=0.18, lw=0)
                    ax.text((t0 + t1) / 2, 0.02, label, transform=ax.get_xaxis_transform(),
                            fontsize=7, ha="center", rotation=90, va="bottom")
            fired = [entry for entry in log if entry.startswith("+")]
            ax.text(1.0, 1.02, "; ".join(fired)[:150], transform=ax.transAxes, fontsize=7, ha="right")
        axes[-1].set_xlabel("time (s)")
        fig.suptitle(f"{name}: {scenario['description']}  (shaded = inserted by a modifier rule)",
                     fontsize=10, x=0.01, ha="left")
        fig.savefig(GAMMA_DIR / f"{name}.png", dpi=80, bbox_inches="tight")
        plt.close(fig)
        print(f"  gamma_comparison/{name}.png")


def main():
    robot = Robot()
    args = sys.argv[1:]
    if "--gamma" in args:
        gamma_figures(robot)
        return
    prefix = args[0] if args else ""
    CLIP_DIR.mkdir(parents=True, exist_ok=True)
    renderer = FrameRenderer(robot)
    clips = [load_clip(name) for name in list_clips()]
    for c in clips:
        if ref(c).startswith(prefix):
            clip_figure(c, robot, renderer, CLIP_DIR / f"{c['name']}__{c['variant']}.png")
            print(f"  review/clips/{c['name']}__{c['variant']}.png")
    write_catalog(clips)
    print(f"wrote {CATALOG_PATH}")


if __name__ == "__main__":
    main()
