"""Longest straight line the stepper arm's head pivot (3rd joint) can travel.

    python tools/sim/longest_line.py                         # hard limits, safe range, torque-safe
    python tools/sim/longest_line.py --shoulder -29.5 -1     # e.g. keep the shoulder behind upright
    python tools/sim/longest_line.py --plot                  # also writes sim/build/longest_line.png

Planar two-link arm (18.5 cm links, angles from vertical, positive toward the front). A line
is valid when every point has an IK solution on one elbow branch with the joints inside the
limits, without jumps. With --torque, shoulder gravity torque from sim/stepper_desk.toml (CAD
masses) must also stay under the limit: the real shoulder lifts about 1.27 N m. Random search
over reachable endpoints, then local refinement. Run the result on the arm with
apps/stepper_desk/scripts/ik_line.py --line X0 Z0 X1 Z1.
"""

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
L = 0.185


def fk(s, e):
    s, e = math.radians(s), math.radians(e)
    return L * math.sin(s) + L * math.sin(s + e), L * math.cos(s) + L * math.cos(s + e)


def ik(x, z, branch):
    c = (x * x + z * z - 2 * L * L) / (2 * L * L)
    if abs(c) > 1:
        return None
    e = branch * math.acos(c)
    s = math.atan2(x, z) - math.atan2(L * math.sin(e), L + L * math.cos(e))
    return math.degrees(s), math.degrees(e)


def line_path(a, b, lim, branch, n, torque=None, tmax=None):
    path = []
    for t in np.linspace(0, 1, n):
        q = ik(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, branch)
        if q is None or not (lim[0][0] <= q[0] <= lim[0][1] and lim[1][0] <= q[1] <= lim[1][1]):
            return None
        if path and max(abs(q[0] - path[-1][0]), abs(q[1] - path[-1][1])) > 15:
            return None
        path.append(q)
    if torque and max(torque(*q) for q in path[::max(1, n // 15)]) > tmax:
        return None
    return path


def search(lim, torque=None, tmax=None, seed=0, samples=30000):
    rng = np.random.default_rng(seed)
    S, E = rng.uniform(*lim[0], 3000), rng.uniform(*lim[1], 3000)
    pts = [(fk(s, e), 1 if e >= 0 else -1) for s, e in zip(S, E) if not torque or torque(s, e) <= tmax]
    best = (0.0, None)
    for _ in range(samples):
        (a, sa), (b, sb) = pts[rng.integers(len(pts))], pts[rng.integers(len(pts))]
        d = math.dist(a, b)
        if sa == sb and d > best[0] and line_path(a, b, lim, sa, 40, torque, tmax):
            best = (d, (a, b, sa))
    d, (a, b, branch) = best
    for i in range(8000):
        step = 0.004 * (1 - i / 8000) + 0.0003
        na = (a[0] + rng.normal() * step, a[1] + rng.normal() * step)
        nb = (b[0] + rng.normal() * step, b[1] + rng.normal() * step)
        if math.dist(na, nb) > d and line_path(na, nb, lim, branch, 100, torque, tmax):
            d, a, b = math.dist(na, nb), na, nb
    return d, a, b, branch


def shoulder_torque():
    os.environ.setdefault("ROBOT_CONFIG", str(ROOT / "sim" / "stepper_desk.toml"))
    sys.path.insert(0, str(ROOT / "motion_lib"))
    import mujoco
    from robot import Robot
    model = Robot().model()
    data = mujoco.MjData(model)

    def torque(s, e, h=5.3):
        data.qpos[:] = np.radians([s, e, h])
        data.qvel[:] = 0
        mujoco.mj_forward(model, data)
        return abs(data.qfrc_bias[0])
    return torque


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--shoulder", type=float, nargs=2, help="custom shoulder range (deg); runs one torque-safe search")
    p.add_argument("--elbow", type=float, nargs=2, default=(-70, 77))
    p.add_argument("--torque", type=float, default=1.15, help="shoulder gravity-torque cap in N m")
    p.add_argument("--plot", action="store_true")
    args = p.parse_args()
    torque = shoulder_torque()
    if args.shoulder:
        cases = [("custom", (tuple(args.shoulder), tuple(args.elbow)), True)]
    else:
        cases = [("hard limits", ((-32, 36), (-70, 77)), False),
                 ("safe demo range", ((-30, 10), (-70, 77)), False),
                 ("torque-safe", ((-30, 10), (-70, 77)), True)]
    results = []
    for name, lim, capped in cases:
        d, a, b, branch = search(lim, torque if capped else None, args.torque)
        path = line_path(a, b, lim, branch, 200)
        q = np.array(path)
        print(f"{name}: {d * 100:.1f} cm  --line {a[0] * 100:.1f} {a[1] * 100:.1f} {b[0] * 100:.1f} {b[1] * 100:.1f}  "
              f"shoulder {q[:, 0].min():.1f}..{q[:, 0].max():.1f}  elbow {q[:, 1].min():.1f}..{q[:, 1].max():.1f}  "
              f"max shoulder torque {max(torque(*x) for x in path):.2f} N m")
        results.append((name, d, a, b))
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 7))
        S, E = np.meshgrid(np.linspace(-32, 36, 140), np.linspace(-70, 77, 220))
        P = np.array([fk(s, e) for s, e in zip(S.ravel(), E.ravel())]) * 100
        ax.scatter(P[:, 0], P[:, 1], s=1, c="#d0d7de", label="reachable (hard limits)")
        for (name, d, a, b), color in zip(results, ["#868e96", "#1c7ed6", "#2f9e44"]):
            ax.plot([a[0] * 100, b[0] * 100], [a[1] * 100, b[1] * 100], color=color, lw=3, label=f"{name}: {d * 100:.1f} cm")
        hx, hz = fk(-12.04, 63.46)
        ax.plot(hx * 100, hz * 100, "k*", ms=12, label="head pivot at home")
        ax.set_aspect("equal")
        ax.set_xlabel("forward (cm)")
        ax.set_ylabel("up from shoulder pivot (cm)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="lower left")
        out = ROOT / "sim" / "build" / "longest_line.png"
        out.parent.mkdir(exist_ok=True)
        fig.savefig(out, dpi=120)
        print(f"plot: {out}")


if __name__ == "__main__":
    main()
