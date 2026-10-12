"""Representative functional moves for the gamma comparison (sprint brief Part D test).

Used by `review.py --gamma` (plots) and `demo.py --gamma <scenario>` (sim playback).
"""

import numpy as np

from modifiers import Context, Segment, apply_rules, render

# Target poses (joint degrees). Head pitch in the world is shoulder + elbow + wrist pitch.
DESK_LEFT = {"base_yaw": 50, "shoulder_pitch": 25, "elbow_pitch": 35, "wrist_pitch": 10}
DESK_RIGHT = {"base_yaw": -45, "shoulder_pitch": 20, "elbow_pitch": 40, "wrist_pitch": 5}
AIM = {"base_yaw": 25, "shoulder_pitch": 5, "elbow_pitch": 60, "wrist_pitch": -25}
POINT = {"base_yaw": -30, "shoulder_pitch": 30, "elbow_pitch": 15, "wrist_pitch": -5}
FAR = {"base_yaw": 10, "shoulder_pitch": 110, "elbow_pitch": -10, "wrist_pitch": -40}  # shoulder can't reach 110
USER = {"base_yaw": 0, "shoulder_pitch": 0, "elbow_pitch": 55, "wrist_pitch": -70}
HOME = {"base_yaw": 0, "shoulder_pitch": -10, "elbow_pitch": 70, "wrist_pitch": -60, "wrist_yaw": 0, "head_roll": 0}

SCENARIOS = {
    "reach_desk_spot": {
        "description": "reach to a desk spot, finish the task, wait, return",
        "segments": [Segment("move", 1.6, DESK_LEFT, task_complete=True, looks_at="task"),
                     Segment("wait", 2.5), Segment("move", 1.6, HOME)],
    },
    "reaim_light": {
        "description": "re-aim the light, hold the angle steady (steady_hold), return",
        "segments": [Segment("move", 1.5, AIM, looks_at="task"), Segment("wait", 3.0, steady_hold=True),
                     Segment("move", 1.5, HOME)],
    },
    "point_at_object": {
        "description": "instruction received, point at an object, return",
        "segments": [Segment("move", 1.2, POINT, instruction=True), Segment("wait", 1.5),
                     Segment("move", 1.3, HOME)],
    },
    "return_home": {
        "description": "return home from the right desk area, then wait",
        "start": DESK_RIGHT,
        "segments": [Segment("move", 1.6, HOME, task_complete=True), Segment("wait", 2.5)],
    },
    "out_of_reach": {
        "description": "target beyond the shoulder's range",
        "segments": [Segment("move", 2.0, FAR, out_of_reach=True), Segment("wait", 1.0),
                     Segment("move", 1.8, HOME)],
    },
    "uncertain_move": {
        "description": "move the planner is unsure about, then wait",
        "segments": [Segment("move", 1.6, DESK_RIGHT, uncertain=True, task_complete=True),
                     Segment("wait", 2.0), Segment("move", 1.6, HOME)],
    },
    "social_greeting": {
        "description": "social task: turn to the user on request, stay engaged, return",
        "category": "social",
        "segments": [Segment("move", 1.4, USER, instruction=True, looks_at="user"), Segment("wait", 2.5),
                     Segment("move", 1.4, HOME)],
    },
}


def start_pose(scenario, robot):
    q = np.array(robot.home, float)
    for name, value in scenario.get("start", {}).items():
        q[robot.index(name)] = value
    return q


def run_scenario(scenario, robot, gamma=None, seed=0):
    """(trajectory, rule log). gamma None = the preset for the scenario's category."""
    ctx = Context(category=scenario.get("category", "function"))
    gamma = ctx.gamma if gamma is None else gamma
    start = start_pose(scenario, robot)
    segments, log = apply_rules(scenario["segments"], robot, ctx, gamma, np.random.default_rng(seed), start)
    return render(segments, robot, start), log
