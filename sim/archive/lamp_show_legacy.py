"""Play a lamp show on the real 3-joint arm model (sim/stepper_desk.toml), headless.

Clips play around a lamp posture ("@desk", "@user", "@sleep" in the show glide there) and go through apps/stepper_desk/scripts/emote.py's trajectory(), the same conversion the board
uses (speed caps 30/25/28 deg/s, elbow lift 14 deg/s, 300 deg/s^2, joint clamps). Reports peak
speed, sim tracking error and the joint torque each clip needs (inverse dynamics), against the
shoulder's measured 1.27 N m lifting limit; renders a video and a plot.

    python sim/lamp_show.py                                   # -> docs/presentation/
    python sim/lamp_show.py --clips @user,nod_yes/lamp,@desk,celebrate/lamp --amplitude 1.2
    mjpython sim/lamp_show.py --view                          # live in the MuJoCo viewer, loops
    mjpython sim/lamp_show.py --browse                        # one clip at a time: N/P, R, B, K keys
"""
import json, os, subprocess, sys
from pathlib import Path

import argparse
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--clips", help="comma-separated clip names (default: clips the real arm has played)")
ap.add_argument("--amplitude", type=float, default=1.0)
ap.add_argument("--out", help="output folder (default docs/presentation)")
ap.add_argument("--view", action="store_true", help="play live in the MuJoCo viewer instead (macOS: mjpython)")
ap.add_argument("--browse", action="store_true", help="step through the /lamp clips one at a time in the viewer (mjpython)")
args = ap.parse_args()
repo = Path(__file__).resolve().parent.parent
out = Path(args.out) if args.out else repo / "docs" / "presentation"
out.mkdir(parents=True, exist_ok=True)
os.environ["ROBOT_CONFIG"] = str(repo / "sim" / "stepper_desk.toml")
sys.path.insert(0, str(repo / "motion_lib"))
import mujoco, numpy as np
from robot import Robot
from player import play, load_clip, move, hold_pose

r = Robot()
HOME = np.array([-12.04, 63.46, 5.27])   # the arm's start pose
# Lamp postures, as in apps/lumen/lumen/config.py: "@desk" etc. in the show glide there; clips play
# as offsets around the current posture, the way Lumen plays them.
POSTURES = {"desk": (-12.0, 63.5, 58.0), "user": (-12.0, 63.5, 25.0), "sleep": (-20.0, 76.0, 80.0),
            "home": tuple(HOME)}
CAPS, ELBOW_LIFT, BASE_SPEED = (30, 25, 28), 14, 24
# Default: a lamp's day. Rests on the desk, notices you and lifts its head, interacts, goes back to
# work, and falls asleep.
SHOW = args.clips.split(",") if args.clips else [
    "@desk", "idle_breathe/lamp", "scan_room/lamp", "look_at_user_return/lamp",
    "@user", "notice/lamp", "curious_head_tilt/lamp", "nod_yes/lamp", "excited_bounce/lamp",
    "@desk", "point_with_head/lamp", "celebrate/lamp", "shake_no/lamp", "shrug_fail/lamp",
    "sad_droop/lamp", "stretch_reach_limit/lamp", "go_to_sleep/lamp", "@sleep", "idle_breathe/lamp"]
AMP = args.amplitude

sys.path.insert(0, str(repo / "apps/stepper_desk/scripts"))
import emote  # the exact clip -> real-arm conversion the board uses (speed caps, elbow lift, clamps)
from player import Trajectory


def glide(a, b):
    """Posture change at BASE_SPEED under each joint's cap (Lumen's MotionWorker._glide_to)."""
    caps = [min(BASE_SPEED, ELBOW_LIFT if j == 1 and b[j] < a[j] else CAPS[j]) for j in range(3)]
    return max(0.4, max(1.875 * abs(b[j] - a[j]) / caps[j] for j in range(3)))


m = r.model(); d = mujoco.MjData(m)
sub = int(round(1 / r.rate / m.opt.timestep))

if args.browse:   # one clip at a time in the viewer, keys to step through (macOS: mjpython)
    import glob, json, time
    import mujoco.viewer
    # Postures Lumen plays each clip from (apps/lumen/lumen/behavior/pools.py); the rest: DESK.
    ON_USER = {"notice", "curious_head_tilt", "lean_in_interest", "tail_wag", "excited_bounce", "nod_yes",
               "confident_snap", "shake_no", "shrug_fail", "hesitate"}
    clip_dir = repo / "apps/stepper_desk/clips"
    refs = sorted(Path(p).name[:-5].replace("__", "/") for p in glob.glob(str(clip_dir / "*__lamp.json")))
    if args.clips:
        refs = args.clips.split(",")
    state = {"i": 0, "base": None, "restart": True, "pause": False}
    cache = {}

    def clip_traj(ref, posture):
        if (ref, posture) not in cache:
            t, q, _, _ = emote.trajectory(ref, np.array(POSTURES[posture]), AMP, CAPS, 300, (1, 1, 1),
                                          elbow_lift=ELBOW_LIFT)
            tt = np.arange(0, t[-1] + 1e-9, 1 / r.rate)
            cache[ref, posture] = (tt, np.array([np.interp(tt, t, q[:, j]) for j in range(3)]).T)
        return cache[ref, posture]

    def default_base(ref):
        return "user" if ref.split("/")[0] in ON_USER else "desk"

    def key(code):
        if code in (262, 78):      # right arrow / N: next clip
            state["i"] = (state["i"] + 1) % len(refs); state["base"] = None; state["restart"] = True
        elif code in (263, 80):    # left arrow / P: previous clip
            state["i"] = (state["i"] - 1) % len(refs); state["base"] = None; state["restart"] = True
        elif code == 82:           # R: replay
            state["restart"] = True
        elif code == 66:           # B: play it from the other posture
            ref = refs[state["i"]]
            cur = state["base"] or default_base(ref)
            state["base"] = "desk" if cur == "user" else "user"; state["restart"] = True
        elif code == 75:           # K: pause / resume looping
            state["pause"] = not state["pause"]

    def describe(ref):
        p = clip_dir / (ref.replace("/", "__") + ".json")
        return json.loads(p.read_text()).get("description", "") if p.exists() else ""

    print("browse: N / right = next, P / left = previous, R = replay, B = other posture, K = pause looping")
    d.qpos[:] = np.radians(POSTURES["desk"]); mujoco.mj_forward(m, d)
    with mujoco.viewer.launch_passive(m, d, key_callback=key) as v:
        v.cam.lookat[:] = [0.1, 0, 0.22]; v.cam.distance = 0.9; v.cam.azimuth = 90; v.cam.elevation = -5
        while v.is_running():
            state["restart"] = False
            ref = refs[state["i"]]
            posture = state["base"] or default_base(ref)
            tt, qq = clip_traj(ref, posture)
            now = np.degrees(d.qpos.copy())
            lead = move(r, now, qq[0], glide(now, qq[0])).q if np.abs(qq[0] - now).max() > 0.5 else qq[:1]
            seq = np.vstack([lead, qq, np.repeat(qq[-1:], int(1.2 * r.rate), 0)])
            title = f"{state['i'] + 1}/{len(refs)}  {ref}   (from {posture.upper()})"
            info = f"{describe(ref)}\n{tt[-1]:.1f} s    N/P next/prev  R replay  B other posture  K pause"
            t0 = time.perf_counter()
            for k, target in enumerate(seq):
                if state["restart"] or not v.is_running():
                    break
                v.set_texts([(None, mujoco.mjtGridPos.mjGRID_TOPLEFT, title, ""),
                             (None, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, info, "")])
                d.ctrl[:] = np.radians(target)
                for _ in range(sub):
                    mujoco.mj_step(m, d)
                v.sync()
                time.sleep(max(0.0, t0 + (k + 1) / r.rate - time.perf_counter()))
            while state["pause"] and not state["restart"] and v.is_running():
                d.ctrl[:] = np.radians(seq[-1]); mujoco.mj_step(m, d); v.sync(); time.sleep(1 / r.rate)
    sys.exit()

base = np.array(POSTURES["home"])
traj = hold_pose(r, base, 0.6)
for ref in SHOW:
    if ref.startswith("@"):
        base = np.array(POSTURES[ref[1:]])
        traj = traj.then(move(r, traj.q[-1], base, glide(traj.q[-1], base), label=ref))
        traj = traj.then(hold_pose(r, base, 0.5))
        continue
    t, q, _, scale = emote.trajectory(ref, base, AMP, CAPS, 300, (1, 1, 1), elbow_lift=ELBOW_LIFT)
    tt = np.arange(0, t[-1] + 1e-9, 1 / r.rate)
    qq = np.array([np.interp(tt, t, q[:, j]) for j in range(3)]).T
    if np.abs(qq[0] - traj.q[-1]).max() > 0.5:
        traj = traj.then(move(r, traj.q[-1], qq[0], glide(traj.q[-1], qq[0])))
    traj = traj.then(Trajectory(tt, qq, r.joint_names, [], [(0.0, float(tt[-1]), ref)]))
    traj = traj.then(hold_pose(r, traj.q[-1], 0.4))
    print(f"  {ref:26} {tt[-1]:5.1f} s  time_scale {scale:.2f}")


if args.view:   # interactive viewer, real time, loops the show (macOS: run with mjpython)
    import time
    import mujoco.viewer
    labels = [(a, b, lab) for a, b, lab in traj.marks if lab not in ("hold", "move")]
    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.lookat[:] = [0.05, 0, 0.22]; v.cam.distance = 0.95; v.cam.azimuth = 120; v.cam.elevation = -12
        while v.is_running():
            d.qpos[:] = np.radians(traj.q[0]); d.qvel[:] = 0; mujoco.mj_forward(m, d)
            t0 = time.perf_counter()
            for i, target in enumerate(traj.q):
                if not v.is_running():
                    break
                now = traj.t[i]
                clip_now = next((lab for a, b, lab in labels if a <= now <= b), "")
                v.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, clip_now or " ", f"{now:5.1f} / {traj.duration:.0f} s"))
                d.ctrl[:] = np.radians(target)
                for _ in range(sub):
                    mujoco.mj_step(m, d)
                v.sync()
                time.sleep(max(0.0, t0 + traj.t[i] + traj.dt - time.perf_counter()))
    sys.exit()
d.qpos[:] = np.radians(traj.q[0]); mujoco.mj_forward(m, d)
renderer = mujoco.Renderer(m, 480, 640)
cam = mujoco.MjvCamera(); cam.lookat[:] = [0.05, 0, 0.22]; cam.distance = 0.95; cam.azimuth = 120; cam.elevation = -12
fps = 25; every = r.rate // fps
ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                       "-s", "640x480", "-r", str(fps), "-i", "-", "-pix_fmt", "yuv420p",
                       str(out / "stepper_desk_show.mp4")], stdin=subprocess.PIPE)
sim_q, torque = [], []
for i, target in enumerate(traj.q):
    d.ctrl[:] = np.radians(target)
    tq = []
    for _ in range(sub):
        mujoco.mj_step(m, d); tq.append(np.abs(d.actuator_force.copy()))
    torque.append(np.max(tq, 0)); sim_q.append(np.degrees(d.qpos.copy()))
    if i % every == 0:
        renderer.update_scene(d, cam); ff.stdin.write(renderer.render().tobytes())
ff.stdin.close(); ff.wait()

sim_q = np.array(sim_q)
# Required joint torque from inverse dynamics on the command (gravity + inertia + armature),
# independent of how stiff the simulated position controller is.
qr = np.radians(traj.q); qd = np.gradient(qr, traj.dt, axis=0); qdd = np.gradient(qd, traj.dt, axis=0)
di = mujoco.MjData(m); torque = []
for a, b, c in zip(qr, qd, qdd):
    di.qpos[:], di.qvel[:], di.qacc[:] = a, b, c
    mujoco.mj_inverse(m, di); torque.append(np.abs(di.qfrc_inverse.copy()))
torque = np.array(torque)
vel = np.abs(np.gradient(traj.q, traj.dt, axis=0))
lift = {"shoulder_pitch": 1.27}
print(f"show: {len(SHOW)} clips, {traj.duration:.1f} s")
print(f"{'joint':15} {'range used':>16} {'peak deg/s':>10} {'track err':>9} {'peak N*m':>8} {'hold lim':>8} {'lift lim':>8}")
for j, n in enumerate(r.joint_names):
    print(f"{n:15} {traj.q[:, j].min():7.1f}..{traj.q[:, j].max():6.1f} {vel[:, j].max():10.1f} "
          f"{np.abs(sim_q[:, j] - traj.q[:, j]).max():8.2f}° {torque[:, j].max():8.2f} "
          f"{r.max_torque[j]:8.2f} {lift.get(n, float('nan')):8.2f}")
over = torque[:, 0] > lift["shoulder_pitch"]
print(f"shoulder above the 1.27 N*m measured lifting limit: {over.mean() * 100:.1f}% of the show")
print(f"firmware speed cap {r.max_vel} deg/s exceeded: {[n for j, n in enumerate(r.joint_names) if vel[:, j].max() > r.max_vel]}")

print("per-clip peak shoulder torque (lift limit 1.27 N*m):")
for a, b, lab in traj.marks:
    if lab in ("hold", "move"): continue
    k = (traj.t >= a) & (traj.t <= b)
    print(f"  {lab:26} {torque[k, 0].max():.2f} N*m  shoulder {traj.q[k,0].min():6.1f}..{traj.q[k,0].max():5.1f}  sim track err {np.abs(sim_q[k] - traj.q[k]).max():.2f} deg")
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
for j, n in enumerate(r.joint_names):
    ax[0].plot(traj.t, traj.q[:, j], label=n); ax[1].plot(traj.t, torque[:, j], label=n)
ax[1].axhline(1.27, ls="--", c="k", lw=0.8, label="shoulder lift limit (measured)")
for a, b, lab in traj.marks:
    if lab not in ("hold", "move"): ax[0].axvspan(a, b, alpha=0.05); ax[0].text(a, ax[0].get_ylim()[1], lab.replace("/", "\n"), fontsize=6, va="top")
ax[0].set_ylabel("joint angle (deg)"); ax[1].set_ylabel("required torque (N·m)"); ax[1].set_xlabel("s")
ax[0].legend(fontsize=7, loc="lower right"); ax[1].legend(fontsize=7, loc="upper right")
fig.tight_layout(); fig.savefig(out / "stepper_desk_show.png", dpi=130)
