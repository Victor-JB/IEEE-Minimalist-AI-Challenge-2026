#!/usr/bin/env python3
"""Cross-check python/kinematics.py against the CAD-exact MuJoCo model (dev branch).

    git archive origin/dev tools/arm_mujoco | tar -x -C /tmp/mj
    pip install mujoco numpy                      # in a venv; not a dependency of arm/
    python3 arm/scripts/check_kinematics_mujoco.py /tmp/mj/tools/arm_mujoco

Compares the lamp light (elegnt_demo.py's scene light inside the shade) and the head-tilt pivot
over random poses inside SAFE_RANGES / ROLL_RANGE. Last run (MuJoCo 3.15, dev c088a9d): worst
errors 3e-16 m, 7e-16, 2e-16 m: fk() is the MJCF's chain exactly.
"""
import math
import random
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
import kinematics as K                              # noqa: E402
from arm_config import HOME, ROLL_RANGE, SAFE_RANGES  # noqa: E402


def main(model_dir):
    sys.path.insert(0, model_dir)
    import elegnt_demo
    m = mujoco.MjModel.from_xml_string(elegnt_demo.scene_xml())
    d = mujoco.MjData(m)
    light, tilt = m.light('lamp_light').id, m.body('head_tilt').id
    rng = random.Random(0)
    poses = [(tuple(HOME), 0.0)] + [(tuple(rng.uniform(lo, hi) for lo, hi in SAFE_RANGES), rng.uniform(*ROLL_RANGE))
                                    for _ in range(500)]
    worst = np.zeros(3)
    for q, r in poses:
        mujoco.mj_resetData(m, d)
        for name, v in zip(('shoulder', 'elbow', 'wrist', 'head_tilt'), list(q) + [r]):
            d.qpos[m.joint(name).qposadr[0]] = math.radians(v)
        mujoco.mj_kinematics(m, d)
        mujoco.mj_camlight(m, d)
        h = K.fk(q, r)
        err = [np.linalg.norm(d.light_xpos[light] - h.light), np.linalg.norm(d.light_xdir[light] - h.axis),
               np.linalg.norm(d.xpos[tilt] - h.pivot)]
        worst = np.maximum(worst, err)
    print(f'{len(poses)} poses; worst light position {worst[0]:.2e} m, light direction {worst[1]:.2e}, '
          f'tilt pivot {worst[2]:.2e} m')
    return 0 if worst.max() < 1e-6 else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
