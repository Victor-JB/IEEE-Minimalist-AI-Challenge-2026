"""Re-sim every clip with the shoulder locked: what each one loses, and how much slower it gets.

    python3 arm/scripts/locked_clip_report.py            # all clips, worst first
    python3 arm/scripts/locked_clip_report.py --lock -15  # at another lock angle

With the shoulder locked, trajectory.fold_locked_shoulder() moves each clip's shoulder motion into
the elbow, so the forearm and head keep their authored angles in the world. Two things can still
go wrong, and this measures both against the same clip with a working shoulder:
  slower   the elbow now carries shoulder + elbow motion, so its speed cap can force the clip
           to play slower (time_scale)
  clipped  the elbow still runs past SAFE_RANGES and is clamped (deg past the limit, worst sample).
           The fold already shrinks the shoulder share to fit; what's left is the clip's own
           elbow/head motion leaving the range.
  kept     how much of the shoulder share survived the fit (1.00 = all of it)
Only what the lock makes worse is called "clipped"; * marks clips that leave the safe range
even with a working shoulder (they were already clamped before).
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
import arm_config as A                      # noqa: E402
from player import list_clips, load_clip, play   # noqa: E402
from trajectory import ConceptRobot, fold_locked_shoulder, trajectory   # noqa: E402

CLIPPED_DEG = 3.0       # more motion than this lost past a limit: the clip looks different
SLOWER = 1.25           # this much slower than with a working shoulder: noticeably sluggish


def _past(raw, ranges):
    return [float(max(0.0, (lo - raw[:, j]).max(), (raw[:, j] - hi).max())) for j, (lo, hi) in enumerate(ranges)]


def lost_past_limits(ref, home):
    """(worst deg past SAFE_RANGES per joint before clamping, the same with a working shoulder,
    share of shoulder motion kept)."""
    sim = ConceptRobot()
    tr = play(load_clip(ref), sim, start=None, amplitude=1.0, time_scale=1.0, enforce_ranges=False)
    delta = tr.q[:, 1:4] - sim.home[1:4]
    folded = fold_locked_shoulder(delta, home)
    lost = _past(np.array(home) + folded, A.SAFE_RANGES)
    free = _past(np.array(home) + delta, [A.SHOULDER_FREE_RANGE] + list(A.SAFE_RANGES[1:]))
    share = delta[:, 0]
    kept = 1.0 if np.abs(share).max() < 1e-6 else float(
        np.dot(folded[:, 1] - delta[:, 1], share) / np.dot(share, share))
    return lost, free, kept


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--lock", type=float, default=A.HOME[0], help="shoulder lock angle (deg)")
    args = p.parse_args()
    rows = []
    for ref in list_clips():
        A.unlock_shoulder()
        home = list(A.HOME)
        home[0] = args.lock
        try:
            t_free, *_ = trajectory(ref, home)
            A.lock_shoulder(args.lock)
            t_lock, q_lock, _, scale = trajectory(ref, home)
            lost, free, kept = lost_past_limits(ref, home)
        except Exception as e:              # a clip the player can't render
            rows.append((ref, None, None, None, f"error: {e}", None))
            continue
        slower = (t_lock[-1] / t_free[-1]) if t_free[-1] > 0 else 1.0
        # Only what the lock makes worse counts: some clips leave the safe range anyway.
        worse = max(0.0, max(lost) - max(free))
        verdict = ("clipped" if worse > CLIPPED_DEG else "slower" if slower > SLOWER else "ok")
        verdict += "" if max(free) <= CLIPPED_DEG else "*"
        rows.append((ref, slower, max(lost), float(t_lock[-1]), verdict, kept))
    A.lock_shoulder(args.lock)
    order = {"error": 0, "clipped": 1, "slower": 2, "ok": 3}
    rows.sort(key=lambda r: (order[r[4].split(":")[0].rstrip("*")], -(r[2] or 0), -(r[1] or 0)))
    print(f"Shoulder locked at {args.lock:.2f} deg; elbow {A.SAFE_RANGES[1]}, head {A.SAFE_RANGES[2]}\n")
    print(f"{'clip':40s} {'verdict':8s} {'lost deg':>8s} {'kept':>5s} {'slower':>7s} {'length s':>8s}")
    for ref, slower, lost, length, verdict, kept in rows:
        if slower is None:
            print(f"{ref:40s} {verdict}")
        else:
            print(f"{ref:40s} {verdict:8s} {lost:8.1f} {kept:5.2f} {slower:6.2f}x {length:8.1f}")
    counts = {k: sum(r[4].startswith(k) for r in rows) for k in order}
    print(f"\n{counts['ok']} ok, {counts['slower']} slower, {counts['clipped']} clipped by the lock, "
          f"{counts['error']} errors (of {len(rows)}).  * = already past the safe range with a working shoulder")


if __name__ == "__main__":
    main()
