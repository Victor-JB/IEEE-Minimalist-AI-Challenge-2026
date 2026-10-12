#!/usr/bin/env python3
"""Inverse-kinematics demo: move the head pivot along straight lines, head kept level.

    python3 scripts/ik_line.py                 # preview the joint angles, no connection
    python3 scripts/ik_line.py --execute       # run it (best on the Arduino Q: local link)

The arm is a planar two-link arm in its pitch plane: shoulder and elbow links are both
18.5 cm (the app's 3D view), joint angles are measured from straight up and positive leans
toward the front. For each point of a minimum-jerk path along a line, the elbow-forward IK
solution gives shoulder and elbow; the head joint cancels their sum so the head keeps its
world angle. Points stream to the closed loop at 25 Hz; measured angles go back through
forward kinematics to report how straight the head pivot actually moved.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from arm_client import ArmError, angles  # noqa: E402
from arm_motion import ArmMotion  # noqa: E402

L1 = L2 = 0.185                            # m, pivot to pivot
HOME = (-12.04, 63.46, 5.27)
SAFE = ((-43, 36), (-70, 77), (-78, 85))   # joint limits less the firmware's 2 deg margin
HZ, RATE = 25, 50


def fk(sh, el):
    s, e = math.radians(sh), math.radians(sh + el)
    return L1 * math.sin(s) + L2 * math.sin(e), L1 * math.cos(s) + L2 * math.cos(e)


def ik(x, z, pitch):
    """(shoulder, elbow, head) degrees putting the head pivot at x (forward), z (up)."""
    c = (x * x + z * z - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c) > 1:
        raise ValueError(f'({x:.3f}, {z:.3f}) is out of reach')
    el = math.acos(c)
    sh = math.atan2(x, z) - math.atan2(L2 * math.sin(el), L1 + L2 * math.cos(el))
    sh, el = math.degrees(sh), math.degrees(el)
    return sh, el, pitch - sh - el


def path(points, speed, pause):
    """Minimum-jerk segments through the given (x, z) points at a peak tool speed (m/s)."""
    out = []
    for (x0, z0), (x1, z1) in zip(points, points[1:]):
        d = math.hypot(x1 - x0, z1 - z0)
        T = max(0.5, 1.875 * d / speed)
        for t in np.arange(0, T, 1 / RATE):
            s = t / T
            s = s ** 3 * (10 - 15 * s + 6 * s * s)
            out.append((x0 + (x1 - x0) * s, z0 + (z1 - z0) * s))
        out += [(x1, z1)] * int(pause * RATE)
    return np.array(out)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--speed', type=float, default=0.02, help='peak head-pivot speed in m/s (default 0.02)')
    p.add_argument('--size', type=float, default=0.09, help='line length in m (default 0.09)')
    p.add_argument('--shapes', nargs='+', default=['horizontal', 'vertical', 'square'],
                   choices=['horizontal', 'vertical', 'square'])
    p.add_argument('--line', type=float, nargs=4, metavar=('X0', 'Z0', 'X1', 'Z1'),
                   help='run this segment (cm from the shoulder pivot) instead of the built-in shapes')
    p.add_argument('--joint-speed', type=float, default=60,
                   help='cap on every joint in deg/s; 8.5 is about 25 motor RPM (1333 pulses/s at ~150 pulses/deg)')
    p.add_argument('--repeat', type=int, default=1, help='back-and-forth passes along the horizontal line')
    p.add_argument('--no-level', dest='level', action='store_false',
                   help='hold the head joint still instead of keeping the head level')
    p.add_argument('--execute', action='store_true')
    args = p.parse_args()

    x0, z0 = fk(HOME[0], HOME[1])
    pitch = sum(HOME)                      # head angle in the world, kept constant
    h, d = args.size / 2, 0.4 * args.size / 2   # home has the elbow at 63 deg: little room to go down
    w = 0.35 * args.size / 2                     # square half-width: reach and the shoulder's +10 cap
    shapes = {
        'horizontal': [(x0, z0)] + [(x0 - h, z0), (x0 + h, z0)] * args.repeat + [(x0, z0)],
        'vertical': [(x0, z0), (x0, z0 + h), (x0, z0 - d), (x0, z0)],
        'square': [(x0, z0), (x0 - w, z0), (x0 + w, z0), (x0 + w, z0 + 2 * w), (x0 - w, z0 + 2 * w), (x0 - w, z0), (x0, z0)],
    }
    if args.line:   # explicit segment in cm from the shoulder pivot (x forward, z up)
        a, b = (args.line[0] / 100, args.line[1] / 100), (args.line[2] / 100, args.line[3] / 100)
        shapes['line'] = [(x0, z0)] + [a, b] * args.repeat + [a, (x0, z0)]
        args.shapes = ['line']
    plans = {}
    for name, pts in ((n, shapes[n]) for n in args.shapes):
        speed = args.speed
        while True:   # slow the tool down until no joint exceeds --joint-speed
            xz = path(pts, speed, 0.4)
            q = np.array([ik(x, z, pitch) for x, z in xz])
            if not args.level:
                q[:, 2] = HOME[2]          # head held still; the pivot still follows the line
            peak = np.abs(np.gradient(q, axis=0)).max() * RATE
            if peak <= args.joint_speed:
                break
            speed *= 0.95 * args.joint_speed / peak
        for j, (lo, hi) in enumerate(SAFE):
            if q[:, j].min() < lo or q[:, j].max() > hi:
                raise SystemExit(f'{name}: joint {j} leaves {lo}..{hi} ({q[:, j].min():.1f}..{q[:, j].max():.1f}); use a smaller --size')
        vel = np.gradient(q, axis=0) * RATE
        plans[name] = (xz, q, vel)
        print(json.dumps({'shape': name, 'tool_speed_cm_s': round(speed * 100, 2), 'seconds': round(len(q) / RATE, 1), 'min': q.min(0).round(1).tolist(),
                          'max': q.max(0).round(1).tolist(), 'peak_joint_speed': np.abs(vel).max(0).round(1).tolist()}))
    if not args.execute:
        return

    horizon = None
    arm = ArmMotion(output=ROOT / f'recordings/ik-line-{time.strftime("%Y%m%d-%H%M%S")}.jsonl',
                    max_speed=60, max_accel=500, tuck=False)
    with arm:
        horizon = 0.3 if arm.transport.timeout > 0.3 else 0.12
        try:
            for name, (xz, q, vel) in plans.items():
                r = arm.move_to(list(HOME), speed=8)
                if not r['ok']:
                    raise ArmError(f'home: {r["reason"]}')
                begin, dev, worst = time.monotonic(), [], [0.0] * 3
                while (now := time.monotonic() - begin) <= len(q) / RATE:
                    k = min(len(q) - 1, int(round((now + horizon) * RATE)))
                    out = arm.stream_point(q[k].tolist(), np.clip(vel[k], -59, 59).tolist(), horizon=horizon)
                    a = out['angles']
                    if out['state'] not in ('streaming', 'holding', 'moving'):
                        raise ArmError(f'{name}: stream state {out["state"]}')
                    x, z = fk(a[0], a[1])
                    i = min(len(q) - 1, int(round(now * RATE)))
                    # distance from the ideal straight path, and head world-angle error
                    seg = xz[max(0, i - 3):i + 4]
                    dev.append(min(math.hypot(x - sx, z - sz) for sx, sz in seg) * 1000)
                    worst = [max(worst[j], abs(a[j] - q[i, j])) for j in range(3)]
                    time.sleep(max(0, 1 / HZ - (time.monotonic() - begin - now)))
                print(json.dumps({'shape': name, 'path_error_mm': {'median': round(float(np.median(dev)), 1),
                                  'max': round(float(np.max(dev)), 1)}, 'worst_joint_lag': [round(w, 2) for w in worst]}), flush=True)
            r = arm.move_to(list(HOME), speed=8)
            if r['ok']:
                arm.relax()
        except ArmError as e:
            print(json.dumps({'error': str(e), 'at': angles(arm.sample) if arm.sample else None}))


if __name__ == '__main__':
    main()
