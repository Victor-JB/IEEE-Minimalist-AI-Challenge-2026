"""Watch the motion clips and the expressive rules on the simulated robot.

    python motion_lib/demo.py                          # guided tour of 16 clips
    python motion_lib/demo.py --list                   # every clip with its tags
    python motion_lib/demo.py --clip nod_yes/big_eager # one clip (add --loop to repeat it)
    python motion_lib/demo.py --clip celebrate/medium --amplitude 0.7 --time-scale 1.5
    python motion_lib/demo.py --category emotion       # every clip in a category
    python motion_lib/demo.py --gamma reach_desk_spot  # one functional task at gamma 0 -> 0.3 -> 1.0
    python motion_lib/demo.py --raw                    # skip the servo model: the exact authored motion

Targets go through the same speed-limited easing as the sketch (ServoModel from
sim/view.py) at the control rate, so what you see is what the real servos would get.
The eyes glow with the clip's light events. Close the viewer window to quit.
"""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from functional import SCENARIOS, run_scenario, start_pose
from player import hold_pose, list_clips, load_clip, move, play, ref
from robot import Robot, configurator
from view import ServoModel  # sim/view.py (sim/ is on the path via robot.py)

TOUR = [
    "idle_breathe/calm", "perk_up/big", "curious_head_tilt/double", "nod_yes/big_eager", "shake_no/medium",
    "look_at_user_return/quick_left", "double_take/big", "excited_bounce/big_eager", "celebrate/medium",
    "sad_droop/deep", "startle_recoil/big", "relaxed_sit_down/calm", "dance_110/party",
    "stretch_reach_limit/short_try", "go_to_sleep/drowsy", "wake_up/slow_groggy",
]
PAUSE = 0.6  # seconds of stillness between clips


def transition(robot, start, target, label):
    distance = np.abs(np.asarray(target) - start).max()
    if distance < 0.5:
        return None
    duration = max(0.8, 1.875 * distance / (0.6 * robot.max_vel))
    return move(robot, start, target, duration, label=label)


def chain(robot, parts):
    traj = None
    for part in parts:
        if part is not None:
            traj = part if traj is None else traj.then(part)
    return traj


def clip_sequence(robot, refs, args, start=None):
    """Clips back to back, with transitions to entry poses and short pauses."""
    pose = np.array(robot.home if start is None else start, float)
    parts = []
    for name in refs:
        clip = load_clip(name)
        entry = clip.get("entry_pose")
        if clip["mode"] == "absolute" and entry:
            go = transition(robot, pose, [entry[n] for n in robot.joint_names], f"-> start pose of {name}")
        else:  # additive clips are validated from home; other poses may lack joint headroom
            go = transition(robot, pose, robot.home, f"-> home before {name}")
        if go:
            parts.append(go)
            pose = go.q[-1]
        repeat = 2 if clip["loopable"] else 1
        parts.append(play(clip, robot, start=pose, amplitude=args.amplitude, time_scale=args.time_scale,
                          repeat=repeat))
        pose = parts[-1].q[-1]
        parts.append(hold_pose(robot, pose, PAUSE, label=ref(clip)))
    return chain(robot, parts)


def gamma_sequence(robot, name):
    scenario = SCENARIOS[name]
    start, pose, parts = start_pose(scenario, robot), robot.home, []
    for gamma in (0.0, 0.3, 1.0):
        parts.append(transition(robot, pose, start, "reset"))
        parts.append(hold_pose(robot, start, 1.0, label=f"gamma {gamma:g}: {name}"))
        traj, log = run_scenario(scenario, robot, gamma)
        traj.marks = [(a, b, f"gamma {gamma:g}: {label}") for a, b, label in traj.marks]
        print(f"gamma {gamma:g}: " + (", ".join(e[2:] for e in log if e.startswith("+")) or "function only"))
        parts.append(traj)
        pose = traj.q[-1]
    return chain(robot, parts)


def label_at(traj, t):
    for a, b, label in traj.marks:
        if a <= t <= b:
            return label.replace("function:", "")
    return ""


def light_at(traj, t):
    level = 1.0
    for e in traj.events:
        if e["t"] <= t and "light" in e:
            level = e["light"]
    return level


def run_viewer(robot, traj, loop=False, raw=False):
    model = mujoco.MjModel.from_xml_string(configurator.build_mjcf(robot.cfg))
    data = mujoco.MjData(model)
    eye = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, "eye")
    glow = np.array([1.0, 0.85, 0.3])
    data.qpos[:] = np.radians(traj.q[0])
    data.ctrl[:] = data.qpos
    mujoco.mj_forward(model, data)

    servo = ServoModel(robot.cfg)
    servo.current, servo.target = traj.q[0].copy(), traj.q[0].copy()
    shown, last_sync, next_servo = None, 0.0, 0.0
    start_wall = time.perf_counter()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.06, 0, 0.24]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 0.95, 145, -15
        while viewer.is_running():
            t = data.time
            if t > traj.duration and loop:
                data.time, next_servo, start_wall, t = 0.0, 0.0, time.perf_counter(), 0.0
            q = traj.q[min(int(t / traj.dt), len(traj.q) - 1)]

            if raw:
                data.ctrl[:] = np.radians(q)
            elif t >= next_servo:
                servo.set_targets(q)
                data.ctrl[:] = np.radians(servo.step())
                next_servo += servo.period
            model.mat_rgba[eye, :3] = 0.05 + (glow - 0.05) * light_at(traj, t)

            mujoco.mj_step(model, data)

            label = label_at(traj, t) if t <= traj.duration else "done (close the window to quit)"
            if label != shown:
                print(f"{t:6.1f}s  {label}")
                shown = label
                try:
                    viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                      label, ""))
                except (AttributeError, TypeError):
                    pass

            wall = time.perf_counter() - start_wall
            if data.time > wall:
                time.sleep(data.time - wall)
            if wall - last_sync > 1 / 60:
                viewer.sync()
                last_sync = wall


def print_list():
    print(f"{'clip':38}{'mode':10}{'category':11}{'v':>6}{'a':>5}{'secs':>6}  status")
    for name in list_clips():
        c = load_clip(name)
        t = c["tags"]
        print(f"{name:38}{c['mode']:10}{t['category']:11}{t['valence']:>+6.1f}{t['arousal']:>5.1f}"
              f"{c['duration_nominal']:>6.1f}  {c.get('validation', {}).get('status', '-')}")
    print(f"\ngamma scenarios: {', '.join(SCENARIOS)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    what = parser.add_mutually_exclusive_group()
    what.add_argument("--list", action="store_true")
    what.add_argument("--clip", help="name/variant, e.g. nod_yes/medium")
    what.add_argument("--category", choices=["attitude", "attention", "intention", "emotion"])
    what.add_argument("--gamma", choices=list(SCENARIOS), metavar="SCENARIO")
    parser.add_argument("--amplitude", type=float, help="within the clip's validated range")
    parser.add_argument("--time-scale", type=float, help="2.0 = twice as slow")
    parser.add_argument("--loop", action="store_true", help="repeat until the window closes")
    parser.add_argument("--raw", action="store_true", help="no servo model: exact authored trajectory")
    args = parser.parse_args()

    if args.list:
        print_list()
        return
    robot = Robot()
    if args.gamma:
        traj = gamma_sequence(robot, args.gamma)
    elif args.clip:
        traj = clip_sequence(robot, [args.clip], args)
        if args.loop:  # come back to the start so every repeat begins where the first did
            traj = chain(robot, [traj, transition(robot, traj.q[-1], traj.q[0], "reset")])
    elif args.category:
        refs = [n for n in list_clips() if load_clip(n)["tags"]["category"] == args.category]
        traj = clip_sequence(robot, refs, args)
    else:
        traj = clip_sequence(robot, TOUR, args)
    print(f"playing {traj.duration:.0f}s of motion")
    run_viewer(robot, traj, loop=args.loop, raw=args.raw)


if __name__ == "__main__":
    main()
