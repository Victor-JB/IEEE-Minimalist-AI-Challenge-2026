#!/usr/bin/env python3
"""Tune the stepper controllers of stepper_pid_sim.py on the simulated arm.

Grid-searches
  pid     gains, speed/acceleration limits, deadband (with braking and an I-zone)
  smooth  the planner's speed/acceleration limits and the sensor trim
scoring each setting on a big target jump, small corrections, a push on the head (and, for
smooth, the power-up test). Lost steps, collisions and peak torque above TORQUE_MARGIN are
(time spent above TORQUE_MARGIN) penalised hardest, then overshoot, hunting, end error and settling time.

Re-run it after you put measured numbers (gearbox, driver current, supply) into the CONFIG
block of stepper_pid_sim.py, then copy the winners into PID_TUNED / SMOOTH_SCALE there.

  python3 tune_stepper.py              both controllers (a few minutes on 8 cores)
  python3 tune_stepper.py --only pid
"""
import argparse
import itertools
import math
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import stepper_pid_sim as S

TORQUE_MARGIN = 0.9            # motor torque / pull-out torque; time spent above it is penalised

PID_GRID = dict(kp=[6.0, 10.0, 15.0, 25.0, 40.0],     # (deg/s) per deg
                ki=[0.0, 2.0, 5.0],                    # (deg/s) per (deg*s)
                deadband=[0.0, 0.1, 0.25],             # deg
                scale=[0.5, 0.7, 0.9])                 # x planner VMAX / AMAX per joint
SMOOTH_GRID = dict(scale=[0.6, 0.7, 0.8, 0.9, 1.0],   # x VMAX / AMAX per joint
                   trim_ki=[2.0, 4.0, 8.0],
                   deadband=[0.05, 0.1, 0.2])


def pid_settings(kp, ki, deadband, scale):
    return dict(kp=kp, ki=ki, kd=0.0, deadband=deadband, brake=True, i_zone=3.0, i_limit=2.0, resync=True,
                max_speed=[round(S.VMAX_DEG[j] * scale) for j in S.STEPPERS],
                max_accel=[round(S.AMAX_DEG[j] * scale) for j in S.STEPPERS])


def cost(m, name):
    """Lower is better. Returns (cost, summary dict)."""
    lost = sum(m['slips'])
    hits = [c for c in m['contacts']]
    tq = float(m['tq_peak'].max())
    if name == 'push':
        lost, hits = 0, []                     # the push itself overpowers the arm; judge the recovery
    c = (1000 if lost else 0) + lost + 1000 * len(hits) + 2000 * m['tq_time']
    info = dict(lost=lost, hits=len(hits), torque=tq)
    if 'jumps' in m:
        j = m['jumps']
        over = max(x['over'].max() for x in j)
        settle = max(x['settle'].max() for x in j)
        end = max(x['final'].max() for x in j)
        osc = max(x['osc'].max() for x in j)
        settle = settle if np.isfinite(settle) else 20.0
        c += 10 * over + 5 * settle + 20 * end + 50 * osc
        info.update(over=over, settle=settle, end=end, wobble=osc)
    else:
        err = float(m['final'].max())
        c += 20 * err
        info.update(err=err)
    return c, info


def evaluate(task):
    kind, params, scenarios = task
    if kind == 'pid':
        settings = pid_settings(**params)
        ctl, pid = 'pid', settings
    else:
        S.VMAX_DEG = {j: v * params['scale'] if j != 'head_tilt' else v for j, v in BASE_VMAX.items()}
        S.AMAX_DEG = {j: v * params['scale'] if j != 'head_tilt' else v for j, v in BASE_AMAX.items()}
        S.TRIM = dict(BASE_TRIM, ki=params['trim_ki'], deadband=params['deadband'])
        ctl, pid = 'smooth', None
    total, parts = 0.0, {}
    for scn in scenarios:
        sim = S.Sim(ctl, 'joint', scn, pid=pid).run()
        c, info = cost(S.metrics(sim), scn)
        total += c
        parts[scn] = info
    return total, kind, params, parts


BASE_VMAX, BASE_AMAX, BASE_TRIM = dict(S.VMAX_DEG), dict(S.AMAX_DEG), dict(S.TRIM)


def fmt(parts):
    out = []
    for scn, i in parts.items():
        if 'over' in i:
            out.append(f'{scn}: over {i["over"]:.2f} settle {i["settle"]:.2f} end {i["end"]:.2f} wobble {i["wobble"]:.2f} '
                       f'lost {i["lost"]} torque {100 * i["torque"]:.0f}%')
        else:
            out.append(f'{scn}: err {i["err"]:.2f} lost {i["lost"]} torque {100 * i["torque"]:.0f}%')
    return ' | '.join(out)


def search(kind, grid, scenarios, procs):
    keys = list(grid)
    tasks = [(kind, dict(zip(keys, vals)), scenarios) for vals in itertools.product(*grid.values())]
    t0 = time.time()
    with Pool(procs) as pool:
        res = pool.map(evaluate, tasks, chunksize=1)
    res.sort(key=lambda r: r[0])
    print(f'\n{kind}: {len(tasks)} settings x {len(scenarios)} tests in {time.time() - t0:.0f} s. Best:')
    for total, _, params, parts in res[:6]:
        print(f'  cost {total:8.1f}  {params}\n      {fmt(parts)}')
    return res[0]


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', choices=['pid', 'smooth'])
    ap.add_argument('--procs', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    a = ap.parse_args()
    print(f'Assumptions: {S.VBUS:g} V, driver current {S.CURRENT:g} x rated, {S.MICROSTEPS} microsteps, '
          f'joint AS5600s, gearbox stiffness {S.STIFFNESS}, backlash {S.BACKLASH_DEG}')
    if a.only in (None, 'pid'):
        best = search('pid', PID_GRID, ('step', 'small', 'push'), a.procs)
        print('PID_TUNED =', pid_settings(**best[2]))
    if a.only in (None, 'smooth'):
        best = search('smooth', SMOOTH_GRID, ('step', 'small', 'push', 'boot'), a.procs)
        print('SMOOTH_SCALE =', best[2]['scale'], ' TRIM ki / deadband =', best[2]['trim_ki'], best[2]['deadband'])
