"""Sim validation for every clip (sprint brief Part C).

    python motion_lib/validate.py            # every clip
    python motion_lib/validate.py nod_yes    # only clips whose name starts with this

For each clip variant:
  1. schema check of the JSON
  2. amplitude {0.5, 1, 1.5} x time_scale {0.5, 1, 2}, played from home or the entry pose:
     joint limits + margin, velocity / acceleration / jerk limits, collisions, end pose
  3. additive clips (and "from anywhere" clips like home) also from 10 sampled start poses
  4. a failing parameter range is shrunk (bisection between the last passing and the
     first failing value) and written back into the clip file with the reason
  5. peak joint torque at default params
Writes motion_lib/validation_report.md. Re-running never widens a range beyond what
author.py wrote (kept as `authored_range`).
"""

import json
import math
import sys
from pathlib import Path

import numpy as np

from checks import check_clip_schema, check_trajectory
from player import clip_path, list_clips, play, ref
from robot import Robot

AMPS = (0.5, 1.0, 1.5)
SCALES = (0.5, 1.0, 2.0)
N_STARTS = 10
REPORT_PATH = Path(__file__).parent / "validation_report.md"


def sample_starts(robot, n=N_STARTS, seed=0):
    """Collision-free poses scattered around home (within 20% of each joint's range)."""
    rng = np.random.default_rng(seed)
    spread = 0.2 * (robot.hi - robot.lo)
    starts = []
    while len(starts) < n:
        q = robot.safe_clip(robot.home + rng.uniform(-spread, spread), extra=10)
        single = play({"name": "pose", "variant": "", "mode": "absolute", "joint_mask": [],
                       "keyframes": [{"t": 0, "joints": {}}, {"t": 0.04, "joints": {}}]}, robot, start=q)
        if check_trajectory(single, robot).ok:
            starts.append(q)
    return starts


def expected_end(clip, robot, start):
    if clip["mode"] == "absolute":
        return np.array([clip["exit_pose"][n] for n in robot.joint_names])
    end = np.array(start, float)
    for name, offset in (clip.get("end_offset") or {}).items():
        end[robot.index(name)] += offset
    return end


def first_problem(clip, robot, start, amplitude=1.0, time_scale=1.0, hold=0.0):
    """None if the clip plays cleanly from `start` with these params, else the reason."""
    try:
        traj = play(clip, robot, start=start, amplitude=amplitude, time_scale=time_scale,
                    hold=hold, enforce_ranges=False)
    except ValueError as e:
        return str(e)
    report = check_trajectory(traj, robot, expect_end=expected_end(clip, robot, start))
    return report.problems[0] if report.problems else None


def bisect(passes, good, bad, steps=4):
    """Value closest to `bad` that still passes, searching between good and bad."""
    for _ in range(steps):
        mid = (good + bad) / 2
        good, bad = (mid, bad) if passes(mid) else (good, mid)
    # Round to a 0.05 step toward the default (the side that is known to pass).
    if good >= 1:
        return max(1.0, round(math.floor(round(good / 0.05, 6)) * 0.05, 2))
    return min(1.0, round(math.ceil(round(good / 0.05, 6)) * 0.05, 2))


def validate_clip(clip, robot, starts):
    notes = []
    schema = check_clip_schema(clip, robot)
    if schema:
        return {"status": "FAIL", "notes": schema, "grid": {}}

    params = clip["params"]
    for spec in params.values():
        spec.setdefault("authored_range", list(spec["range"]))
    if clip["mode"] == "absolute" and clip.get("entry_pose"):
        home = np.array([clip["entry_pose"][n] for n in robot.joint_names])
    else:
        home = robot.home
    anywhere = clip["mode"] == "additive" or not clip.get("entry_pose")

    def problem(a=1.0, s=1.0, h=0.0):
        return first_problem(clip, robot, home, a, s, h)

    grid = {f"a{a:g}_t{s:g}": problem(a, s) or "pass" for a in AMPS for s in SCALES}
    default = grid["a1_t1"]
    if default != "pass":
        return {"status": "FAIL", "notes": [f"default params: {default}"], "grid": grid}

    # Shrink each range toward the default until it passes, one axis at a time.
    (a_lo, a_hi), (s_lo, s_hi) = params["amplitude"]["authored_range"], params["time_scale"]["authored_range"]
    for name, edge, other in (("amp_hi", a_hi, 1.0), ("amp_lo", a_lo, 1.0)):
        reason = problem(a=edge)
        if reason:
            value = bisect(lambda v: not problem(a=v), 1.0, edge)
            notes.append(f"amplitude {'max' if name == 'amp_hi' else 'min'} {edge:g} -> {value:g}: {reason}")
            a_hi, a_lo = (value, a_lo) if name == "amp_hi" else (a_hi, value)
    for name, edge in (("ts_lo", s_lo), ("ts_hi", s_hi)):
        reason = problem(s=edge)
        if reason:
            value = bisect(lambda v: not problem(s=v), 1.0, edge)
            notes.append(f"time_scale {'min' if name == 'ts_lo' else 'max'} {edge:g} -> {value:g}: {reason}")
            s_lo, s_hi = (value, s_hi) if name == "ts_lo" else (s_lo, value)
    # Corner: big amplitude at high speed is the combination that fails; shrink both together.
    reason = problem(a_hi, s_lo)
    while reason and (a_hi > 1.0 or s_lo < 1.0):
        a_hi, s_lo = max(1.0, round(a_hi - 0.05, 2)), min(1.0, round(s_lo + 0.05, 2))
        if not problem(a_hi, s_lo):
            notes.append(f"corner: amplitude max -> {a_hi:g} with time_scale min -> {s_lo:g}: {reason}")
            break

    h_lo, h_hi = params["hold"]["authored_range"]
    reason = problem(h=h_hi)
    if reason:
        notes.append(f"hold max {h_hi:g} -> 0: {reason}")
        h_hi = 0.0

    params["amplitude"]["range"] = [round(a_lo, 2), round(a_hi, 2)]
    params["time_scale"]["range"] = [round(s_lo, 2), round(s_hi, 2)]
    params["hold"]["range"] = [h_lo, h_hi]

    sampled_failures = []
    if anywhere:
        for i, start in enumerate(starts):
            reason = first_problem(clip, robot, start)
            if reason:
                sampled_failures.append(f"start #{i}: {reason}")
    if sampled_failures:
        notes.append(f"fails from {len(sampled_failures)}/{len(starts)} sampled start poses "
                     f"(first: {sampled_failures[0]}); a selector must check headroom first")

    traj = play(clip, robot, start=home)
    stats = check_trajectory(traj, robot, collisions=False, torque=True).stats
    status = "WARN" if sampled_failures else "PASS"
    return {
        "status": status,
        "notes": notes,
        "grid": grid,
        "sampled_start_failures": len(sampled_failures),
        "peak_velocity_deg_s": round(stats["peak_velocity"], 1),
        "peak_torque_nm": stats["peak_torque"],
    }


def write_report(results, robot):
    counts = {s: sum(r["status"] == s for _, r in results) for s in ("PASS", "WARN", "FAIL")}
    lines = [
        "# Clip validation report",
        "",
        "Generated by `python motion_lib/validate.py`. Limits come from `sim/robot.toml`: "
        f"{robot.max_vel:g} deg/s, {robot.max_acc:g} deg/s^2, {robot.max_jerk:g} deg/s^3, "
        f"{robot.margin:g} deg joint margin, {robot.rate} Hz.",
        "",
        f"**{counts['PASS']} pass, {counts['WARN']} warn, {counts['FAIL']} fail** of {len(results)} clip variants.",
        "",
        "- PASS: clean at default params from home / entry pose and from every sampled start pose.",
        "- WARN: clean from home, but fails from some sampled start poses (usually joint-limit headroom).",
        "- FAIL: fails at default params; do not use until fixed in author.py.",
        "- Grid: amplitude x time_scale, from home or the entry pose. "
        "Ranges are shrunk until every corner passes; reasons are in the notes.",
        "",
        "| clip | variant | status | grid pass | amplitude | time_scale | peak vel | peak torque | notes |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for clip, r in results:
        grid_pass = f"{sum(v == 'pass' for v in r['grid'].values())}/9" if r["grid"] else "-"
        torque = r.get("peak_torque_nm")
        torque_text = "-"
        if torque:
            joint = max(torque, key=torque.get)
            torque_text = f"{torque[joint]:.2f} N*m ({joint})"
        p = clip["params"]
        lines.append(
            f"| {clip['name']} | {clip['variant']} | {r['status']} | {grid_pass} "
            f"| {p['amplitude']['range'][0]:g}-{p['amplitude']['range'][1]:g} "
            f"| {p['time_scale']['range'][0]:g}-{p['time_scale']['range'][1]:g} "
            f"| {r.get('peak_velocity_deg_s', '-')} | {torque_text} | {'<br>'.join(r['notes']) or ''} |"
        )
    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return counts


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else ""
    robot = Robot()
    starts = sample_starts(robot)
    results = []
    for name in list_clips():
        if not name.startswith(prefix):
            continue
        path = clip_path(name)
        clip = json.loads(path.read_text(encoding="utf-8"))
        result = validate_clip(clip, robot, starts)
        clip["validation"] = {k: v for k, v in result.items() if k != "grid"}
        path.write_text(json.dumps(clip, indent=2), encoding="utf-8")
        results.append((clip, result))
        print(f"{result['status']:4}  {ref(clip):36} {'; '.join(result['notes'])[:110]}")
    if not prefix:
        counts = write_report(results, robot)
        print(f"\n{counts} -> {REPORT_PATH}")


if __name__ == "__main__":
    main()
