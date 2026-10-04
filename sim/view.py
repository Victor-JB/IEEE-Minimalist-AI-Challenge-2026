"""View and exercise the robot described in sim/robot.toml (regenerated on every run).

    python sim/view.py           # interactive: pose joints with the sliders in the Control panel
    python sim/view.py --demo    # play a test motion through the same easing the sketch uses
    python sim/view.py --torque  # holding torque each joint needs against gravity
"""

import argparse
import math
import time

import mujoco
import mujoco.viewer
import numpy as np

import configurator

SEND_HZ = 20  # how often the planner sends targets, like the app's main.py


class ServoModel:
    """Python copy of the easing loop in sketch.ino: speed-limited steps toward the
    target. Keep it in sync with the sketch so motion here matches the real robot."""

    def __init__(self, cfg):
        joints = cfg["joint"]
        self.lo = np.array([j["range"][0] for j in joints], float)
        self.hi = np.array([j["range"][1] for j in joints], float)
        self.current = np.array([j["home"] for j in joints], float)
        self.target = self.current.copy()
        self.period = 1 / cfg["motion"]["control_hz"]
        self.max_step = cfg["motion"]["max_speed_deg_s"] * self.period

    def set_targets(self, degrees):
        self.target = np.clip(degrees, self.lo, self.hi)

    def step(self):
        self.current += np.clip(self.target - self.current, -self.max_step, self.max_step)
        return self.current


def demo_targets(t, cfg):
    """Stand-in for the planner: each joint swings around home at its own pace."""
    targets = []
    for i, j in enumerate(cfg["joint"]):
        lo, hi = j["range"]
        amplitude = 0.4 * min(j["home"] - lo, hi - j["home"], (hi - lo) / 2)
        targets.append(j["home"] + amplitude * math.sin(2 * math.pi * t / (4 + i)))
    return targets


def run_demo(model, data, cfg):
    servo = ServoModel(cfg)
    next_command = next_servo = 0.0
    last_sync = 0.0
    start = time.perf_counter()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # The same two loops as on the robot: planner at SEND_HZ, servo easing at control_hz.
            if data.time >= next_command:
                servo.set_targets(demo_targets(data.time, cfg))
                next_command += 1 / SEND_HZ
            if data.time >= next_servo:
                data.ctrl[:] = np.radians(servo.step())
                next_servo += servo.period

            mujoco.mj_step(model, data)

            # Pace the sim to real time and redraw at ~60 fps.
            wall = time.perf_counter() - start
            if data.time > wall:
                time.sleep(data.time - wall)
            if wall - last_sync > 1 / 60:
                viewer.sync()
                last_sync = wall


def report_torque(model, cfg, samples=5000, seed=0):
    """Static gravity torque per joint at home and the worst over random poses in range."""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    dofs = model.jnt_dofadr
    at_home = np.abs(data.qfrc_bias[dofs])

    rng = np.random.default_rng(seed)
    lo, hi = model.jnt_range[:, 0], model.jnt_range[:, 1]
    worst = np.zeros(model.njnt)
    for _ in range(samples):
        data.qpos[:] = rng.uniform(lo, hi)
        data.qvel[:] = 0  # with zero velocity, qfrc_bias is gravity alone
        mujoco.mj_forward(model, data)
        worst = np.maximum(worst, np.abs(data.qfrc_bias[dofs]))

    kgcm = 1 / 0.0980665  # hobby servos are rated in kg*cm
    print(f"Holding torque against gravity ({samples} random poses within joint ranges):")
    print(f"  {'joint':<16}{'home N*m':>10}{'worst N*m':>11}{'worst kg*cm':>13}")
    for j, h, w in zip(cfg["joint"], at_home, worst):
        print(f"  {j['name']:<16}{h:>10.3f}{w:>11.3f}{w * kgcm:>13.1f}")
    print("Static only: leave at least 2x margin for acceleration, friction and payload.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--demo", action="store_true", help="play a test motion")
    mode.add_argument("--torque", action="store_true", help="print holding torques and exit")
    args = parser.parse_args()

    cfg, model_path = configurator.generate()
    model = mujoco.MjModel.from_xml_path(str(model_path))

    if args.torque:
        report_torque(model, cfg)
        return

    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)  # start at the home pose
    if args.demo:
        run_demo(model, data, cfg)
    else:
        # Control panel (right side) has one slider per joint, in radians.
        mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
