#!/usr/bin/env python3
"""Play motion_lib emote clips on the real arm by streaming closed-loop setpoints.

    python3 scripts/emote.py nod_yes/medium perk_up/small --execute
    python3 scripts/emote.py shrug_fail/big --amplitude 1.6 --home -12 52 12 --execute
    python3 scripts/emote.py lamp_wake/arm lamp_happy/arm --gains 1 2 0.6 --execute
    python3 scripts/emote.py nod_yes/medium              # preview only: no connection

The sim's shoulder_pitch, elbow_pitch and wrist_pitch drive the arm's shoulder, elbow and
head (same sign, as in the app's 3D view); base yaw, wrist yaw and head roll are dropped.
Clip offsets are applied around --home. Each clip is slowed (time_scale) until every joint
stays under its speed cap and the acceleration stays under --accel, then streamed at 25 Hz
with a 0.12 s horizon (0.3 s over Tailscale). Every clip runs in its own session, so a stall ends only that clip.

Limits measured on 2026-10-11 (see CLOSED_LOOP.md): the shoulder stays in -30..+10 (it
cannot lift back from below -36 or from about +20); the head reverses through about 0.7 deg
of gear slack, so fast head reversals lag and the firmware's 5 deg / 300 ms rule can trip
above about 32 deg/s. The trace of every clip (command and measured angles) is saved for
sim/compare_real.py.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
# motion_lib sits next to this app on the Arduino Q (deploy.sh), at the repo root on the Mac.
sys.path.insert(0, str(ROOT / 'motion_lib' if (ROOT / 'motion_lib').exists() else ROOT.parents[1] / 'motion_lib'))
from arm_client import ArmError, angles  # noqa: E402
from arm_motion import ArmMotion  # noqa: E402
from player import load_clip, play  # noqa: E402

HOME = (-12.04, 63.46, 5.27)
SAFE = ((-30, 10), (-70, 77), (-68, 85))   # shoulder, elbow, head (head stays out of the -72 slow zone)
SHOULDER_STOP = -31.0
HZ = 25
HORIZON = {'usb': 0.12, 'tailscale': 0.3}   # Tailscale points can arrive 100-170 ms apart


class SimArm:
    """motion_lib's default robot, just enough for play()."""
    joint_names = ['base_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_yaw', 'head_roll']
    home = np.array([0, -10, 70, -60, 0, 0.])
    rate = 50


def trajectory(name, home, amplitude, speeds, accel, gains=(1, 1, 1), elbow_lift=None):
    """Time (s), pose (deg) and velocity (deg/s) arrays for one clip, plus its time_scale."""
    local = ROOT / 'clips' / f"{name.replace('/', '__')}.json"
    clip, sim = load_clip(str(local) if local.exists() else name), SimArm()
    def render(scale):
        return play(clip, sim, start=sim.home, amplitude=amplitude, time_scale=scale, enforce_ranges=False)
    tr = render(1.0)
    dq = np.gradient(tr.q[:, 1:4], axis=0) * np.array(gains)
    speed = (np.abs(dq).max(0) * sim.rate / np.array(speeds)).max()
    if elbow_lift:   # straightening the elbow lifts forearm and head: it stalls above ~15 deg/s
        speed = max(speed, -dq[:, 1].min() * sim.rate / elbow_lift)
    acc = np.abs(np.gradient(dq, axis=0)).max() * sim.rate ** 2
    scale = max(1.0, speed, (acc / accel) ** 0.5)
    if scale > 1.0:
        tr = render(scale)
    q = np.array(home) + (tr.q[:, 1:4] - sim.home[1:4]) * np.array(gains)
    for j, (lo, hi) in enumerate(SAFE):
        q[:, j] = np.clip(q[:, j], lo, hi)
    return tr.t, q, np.gradient(q, axis=0) * sim.rate, scale


def perform(name, home, args, output):
    t, q, vel, scale = trajectory(name, home, args.amplitude, args.speeds, args.accel, args.gains, args.elbow_lift)
    trace = {'t': t.tolist(), 'q': q.tolist(), 'time_scale': scale, 'home': list(home), 'real': []}
    result = {'clip': name, 'time_scale': round(scale, 2), 'seconds': round(float(t[-1]), 2),
              'range': [round(float(np.ptp(q[:, j])), 1) for j in range(3)], 'error': None}
    arm = ArmMotion(output=output, max_speed=60, max_accel=500, tuck=False)
    with arm:
        try:
            horizon = args.horizon or HORIZON['tailscale' if arm.transport.timeout > 0.3 else 'usb']
            for attempt in range(2):   # a network gap can lapse the 300 ms lease; that only halts
                r = arm.move_to(list(home), speed=8)
                if r['ok'] or r['reason'] != 'lease':
                    break
            if not r['ok']:
                raise ArmError(f'home before {name}: {r["reason"]}')
            begin, worst = time.monotonic(), [0.0] * 3
            while (now := time.monotonic() - begin) <= t[-1]:
                k = min(len(t) - 1, int(round((now + horizon) * SimArm.rate)))
                out = arm.stream_point(q[k].tolist(), np.clip(vel[k], -59, 59).tolist(), horizon=horizon)
                a, cur = out['angles'], q[min(len(t) - 1, int(round(now * SimArm.rate)))]
                trace['real'].append([time.monotonic() - begin, *a])
                worst = [max(worst[j], abs(a[j] - cur[j])) for j in range(3)]
                if out['state'] not in ('streaming', 'holding', 'moving'):
                    raise ArmError(f'stream state {out["state"]}')
                if a[0] < SHOULDER_STOP:
                    raise ArmError(f'shoulder at {a[0]:.2f}')
                time.sleep(max(0, 1 / HZ - (time.monotonic() - begin - now)))
            r = arm.move_to(list(home), speed=8)
            result.update(worst_lag=[round(w, 2) for w in worst], stalls=arm.servo_status()['stalls'], home_ok=r['ok'])
        except ArmError as e:
            result['error'] = str(e)
        finally:
            try:   # relax only at home; otherwise the firmware hold timeout lets go
                if all(abs(x - h) < 1.5 for x, h in zip(angles(arm.status()), home)):
                    arm.relax()
            except ArmError:
                pass
    if result['error']:
        result['recovered'] = return_home(home)
    return result, trace


def return_home(home):
    """A firmware rejection ends the gateway session with the drivers asleep wherever the
    arm stopped. Inside the shoulder's safe range a fresh session can simply move home."""
    with ArmMotion(output=ROOT / f'recordings/emote-recover-{time.strftime("%Y%m%d-%H%M%S")}.jsonl', tuck=False) as arm:
        where = angles(arm.status())
        if not SAFE[0][0] <= where[0] <= SAFE[0][1]:
            return f'shoulder at {where[0]:.1f}: recover by hand or fold the elbow first (CLOSED_LOOP.md)'
        r = arm.move_to(list(home), speed=8)
        if r['ok']:
            arm.relax()
        return r['ok']


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('clips', nargs='+', help='motion_lib clip names (nod_yes/medium) or clips/ in this app (lamp_wake/arm)')
    p.add_argument('--amplitude', type=float, default=1.0, help='motion_lib amplitude (1.6 = exaggerated)')
    p.add_argument('--home', type=float, nargs=3, default=HOME, metavar=('SH', 'EL', 'HD'),
                   help='pose the clip offsets are applied around (-12 52 12 gives the elbow room)')
    p.add_argument('--gains', type=float, nargs=3, default=(1, 1, 1), metavar=('SH', 'EL', 'HD'),
                   help='extra per-joint amplitude on top of --amplitude, e.g. 1 2 0.6 to exaggerate the elbow')
    p.add_argument('--speeds', type=float, nargs=3, default=(30, 25, 22), metavar=('SH', 'EL', 'HD'),
                   help='per-joint speed caps in deg/s (default 30 25 22)')
    p.add_argument('--elbow-lift', type=float, default=14,
                   help='speed cap in deg/s while the elbow straightens (lifts); folding uses --speeds')
    p.add_argument('--horizon', type=float, help='streaming look-ahead in s (default 0.12 on USB, 0.3 on Tailscale)')
    p.add_argument('--accel', type=float, default=300, help='acceleration cap in deg/s^2 (firmware: 500)')
    p.add_argument('--traces', default='recordings/emote-traces.json', help='command and measured angles per clip')
    p.add_argument('--execute', action='store_true', help='move the arm; without it, only print the plan')
    args = p.parse_args()
    home = tuple(args.home)
    for lo_hi, h, name in zip(SAFE, home, ('shoulder', 'elbow', 'head')):
        if not lo_hi[0] <= h <= lo_hi[1]:
            p.error(f'--home {name} {h} is outside {lo_hi}')
    if not args.execute:
        for name in args.clips:
            t, q, vel, scale = trajectory(name, home, args.amplitude, args.speeds, args.accel, args.gains, args.elbow_lift)
            print(json.dumps({'clip': name, 'time_scale': round(scale, 2), 'seconds': round(float(t[-1]), 2),
                              'min': q.min(0).round(1).tolist(), 'max': q.max(0).round(1).tolist(),
                              'peak_speed': np.abs(vel).max(0).round(1).tolist()}))
        return
    stamp = time.strftime('%Y%m%d-%H%M%S')
    traces = {}
    for i, name in enumerate(args.clips):
        result, traces[name] = perform(name, home, args, ROOT / f'recordings/emote-{stamp}-{i}.jsonl')
        print(json.dumps(result), flush=True)
    out = ROOT / args.traces
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(traces))
    print(f'traces: {out}')


if __name__ == '__main__':
    main()
