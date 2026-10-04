"""The robot as the motion library sees it, loaded from sim/robot.toml."""

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import configurator  # noqa: E402


class Robot:
    def __init__(self, config_path=configurator.CONFIG_PATH):
        self.cfg = cfg = configurator.load_config(config_path)
        joints = cfg["joint"]
        self.joint_names = [j["name"] for j in joints]
        self.n = len(joints)
        self.lo = np.array([j["range"][0] for j in joints], float)
        self.hi = np.array([j["range"][1] for j in joints], float)
        self.home = np.array([j["home"] for j in joints], float)
        self.max_torque = [j["max_torque"] for j in joints]

        motion = cfg["motion"]
        self.rate = motion["control_hz"]
        self.max_vel = motion["max_speed_deg_s"]
        self.max_acc = motion["max_accel_deg_s2"]
        self.max_jerk = motion["max_jerk_deg_s3"]
        self.margin = motion["limit_margin_deg"]

        self._models = {}

    def model(self, collisions=False):
        """MuJoCo model; collisions=True gives the contact-checking variant."""
        if collisions not in self._models:
            xml = configurator.build_mjcf(self.cfg, collisions=collisions)
            self._models[collisions] = mujoco.MjModel.from_xml_string(xml)
        return self._models[collisions]

    def index(self, name):
        return self.joint_names.index(name)

    def pose(self, base=None, **joints):
        """Pose vector from `base` (default home) with joints overridden by name."""
        q = np.array(self.home if base is None else base, float)
        for name, value in joints.items():
            q[self.index(name)] = value
        return q

    def safe_clip(self, q, extra=0.0):
        """Clip a pose into the joint ranges minus the validation margin."""
        return np.clip(q, self.lo + self.margin + extra, self.hi - self.margin - extra)
