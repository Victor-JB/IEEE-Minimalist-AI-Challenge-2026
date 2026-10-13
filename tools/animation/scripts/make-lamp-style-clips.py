#!/usr/bin/env python3
"""Lamp-native emote clips for this 3-joint arm (shoulder, elbow, head; all pitch).

motion_lib's clips were written for the old 6-joint robot: most of their character is base yaw,
wrist yaw and head roll, which this arm doesn't have (shake_no, scan_room, tail_wag and the head
tilts shrink to 0-6 deg here), and they were timed for 120 deg/s servos, so emote.py slows them
5-8x. These clips are written for this arm instead, like a desk lamp moves:

- The shade is the face. Clips play around the pose luxoiii holds: DESK (rest, shade looking at the
  desk) or USER (shade lifted to the person). +head looks down, -head looks up.
- The head leads and the arm follows: shoulder and elbow keyframes lag the head by LAG s.
- Squash and stretch with the elbow: +elbow folds (crouch, gravity helps, fast), -elbow
  straightens (lifts the forearm and head: capped at 14 deg/s, so lifts are slow and deliberate).
- Lean back with the shoulder, never far forward: +8 deg forward already needs 1.39 N m static,
  over the 1.27 N m the shoulder can lift.
- Hold key poses long enough to read, add anticipation before big moves, small overshoots on
  arrival.
- No yaw: "no" is a recoil and a look away, scanning sweeps the desk near to far.

Each segment is timed from this arm's caps (minimum-jerk peak speed is 1.875x the average), so
emote.py plays them at time_scale ~1 instead of slowing them down.

    python3 scripts/make-lamp-style-clips.py      # writes clips/<name>__lamp.json
"""
import json
import math
from pathlib import Path

OUT = Path(__file__).resolve().parents[1] / 'clips'
JOINTS = ('shoulder_pitch', 'elbow_pitch', 'wrist_pitch')
CAPS = (30.0, 25.0, 28.0)     # deg/s shoulder, elbow (folding), head; luxoiii's SPEED_CAPS
ELBOW_LIFT = 14.0             # deg/s straightening the elbow (CLOSED_LOOP.md)
ACCEL = 300.0                 # deg/s^2
LAG = 0.15                    # s the body trails the head
MIN_SEG = 0.25


def seg_time(a, b, pace):
    t = MIN_SEG
    for j in range(3):
        d = abs(b[j] - a[j])
        cap = ELBOW_LIFT if (j == 1 and b[j] < a[j]) else CAPS[j]
        t = max(t, 1.875 * d / cap * 1.04, math.sqrt(5.78 * d / ACCEL) * 1.04)
    return t * pace


def build(steps, lag=LAG):
    """steps: [(pose (sh, el, hd) offsets, hold s, pace >= 1)]; starts and ends at (0, 0, 0)."""
    t, prev = 0.0, (0.0, 0.0, 0.0)
    head, body = [(0.0, prev)], [(0.0, prev)]
    for pose, hold, pace in steps + [((0.0, 0.0, 0.0), 0.0, 1.0)]:
        t += seg_time(prev, pose, pace)
        head.append((t, pose))
        body.append((t + lag, pose))
        if hold:
            t += hold
            head.append((t, pose))
            body.append((t + lag, pose))
        prev = pose
    keys = {}
    for tk, pose in head:
        keys.setdefault(round(tk, 3), {})['wrist_pitch'] = float(pose[2])
    for tk, pose in body:
        k = keys.setdefault(round(tk, 3), {})
        k['shoulder_pitch'], k['elbow_pitch'] = float(pose[0]), float(pose[1])
    frames = [{'t': tk, 'joints': j, 'ease': 'ease_in_out'} for tk, j in sorted(keys.items())]
    return frames, round(t + lag, 3)


# name -> (description, valence, arousal, loopable, steps). Offsets (shoulder, elbow, head) in deg.
CLIPS = {
    'idle_breathe': ('Slow breath: sinks a little and rises, the shade keeps looking.', 0.1, 0.1, True,
                     [((0, 5, -2), 0.5, 1.6), ((0, -1, 1), 0.3, 1.6)]),
    'scan_room': ('Sweeps the desk with the shade, near to far, then back to the middle.', 0.0, 0.4, False,
                  [((0, 3, 22), 0.6, 1.0), ((-3, -4, -12), 0.7, 1.0), ((0, 2, 8), 0.4, 1.0)]),
    'sad_droop': ('A sigh: breathes in, then sags and lets the shade hang.', -0.6, 0.2, False,
                  [((0, -4, -6), 0.3, 1.2), ((-4, 12, 26), 1.4, 1.3), ((-2, 8, 18), 0.6, 1.5)]),
    'relaxed_sit_down': ('Settles in: sinks into a crouch with a little wiggle, then comes back up.', 0.3, 0.1, False,
                         [((-6, 12, 4), 0.2, 1.1), ((-6, 9, 1), 0.0, 1.0), ((-7, 13, 5), 1.5, 1.0)]),
    'stretch_reach_limit': ('Crouches, stretches up tall and looks up, trembles at the top, relaxes.', 0.3, 0.4, False,
                            [((-2, 8, 10), 0.3, 1.0), ((2, -28, -34), 0.6, 1.0), ((2, -28, -38), 0.0, 1.0),
                             ((2, -28, -33), 0.0, 1.0), ((2, -28, -37), 0.4, 1.0)]),
    'excited_bounce': ('Two springy bobs with the shade held on you, head bobbing on top.', 0.8, 0.8, False,
                       [((-3, 9, -10), 0.0, 1.0), ((0, 0, 6), 0.0, 1.0), ((-3, 9, -10), 0.0, 1.0), ((0, 0, 4), 0.0, 1.0)]),
    'tail_wag': ('Happy little head bobs, like a wag.', 0.6, 0.6, True,
                 [((0, 3, -8), 0.0, 1.0), ((0, 0, 6), 0.0, 1.0), ((0, 3, -8), 0.0, 1.0), ((0, 0, 4), 0.0, 1.0)]),
    'double_take': ('Glances, looks away, then snaps back with a startled lift and leans back.', 0.2, 0.7, False,
                    [((0, 0, -10), 0.15, 1.0), ((0, 0, 4), 0.35, 1.0), ((-4, -6, -24), 0.0, 1.0),
                     ((-4, -6, -20), 0.7, 1.0)]),
    'notice': ('Played around USER after the head lifts to a person: a quick glance back down, then a '
               'startled little lift and lean back.', 0.4, 0.6, False,
               [((0, 0, 10), 0.15, 1.0), ((-3, -6, -8), 0.6, 1.0)]),
    'perk_up': ('Dips, then rises and lifts the shade, a little overshoot, holds bright-eyed.', 0.6, 0.7, False,
                [((0, 5, 6), 0.1, 1.0), ((-2, -10, -20), 0.0, 1.0), ((-2, -10, -16), 0.8, 1.0)]),
    'curious_head_tilt': ('Lowers in for a closer look, cocks the shade up, holds, cocks again.', 0.3, 0.4, False,
                          [((0, 8, -4), 0.3, 1.0), ((0, 8, -16), 0.5, 1.0), ((0, 8, -8), 0.2, 1.0), ((0, 8, -18), 0.6, 1.0)]),
    'lean_in_interest': ('Lowers the shade closer to what it watches and holds there.', 0.4, 0.3, False,
                         [((0, 12, -9), 1.4, 1.2)]),
    'nod_yes': ('Two clear nods down toward the desk.', 0.6, 0.5, False,
                [((0, 0, -4), 0.0, 1.0), ((1, 2, 20), 0.0, 1.0), ((0, 0, -3), 0.0, 1.0), ((1, 2, 16), 0.0, 1.0)]),
    'confident_snap': ('One firm nod with a wind-up.', 0.6, 0.7, False,
                       [((0, 0, -7), 0.1, 1.0), ((1, 3, 18), 0.4, 1.0)]),
    'point_with_head': ('Points the shade at a spot on the desk, taps twice, looks back.', 0.3, 0.5, False,
                        [((0, 6, 26), 0.5, 1.0), ((0, 6, 20), 0.0, 1.0), ((0, 6, 26), 0.0, 1.0), ((0, 6, 20), 0.0, 1.0),
                         ((0, 6, 26), 0.6, 1.0)]),
    'hesitate': ('Starts to go, stops, pulls back a little, thinks, gives up.', -0.1, 0.3, False,
                 [((0, 3, 12), 0.3, 1.0), ((-2, 0, 5), 0.8, 1.2)]),
    'celebrate': ('Crouches, springs the shade up, bobs twice at the top.', 0.9, 0.9, False,
                  [((-2, 10, 10), 0.2, 1.0), ((-3, -10, -30), 0.0, 1.0), ((-3, -10, -18), 0.0, 1.0),
                   ((-3, -10, -30), 0.0, 1.0), ((-3, -10, -20), 0.5, 1.0)]),
    'shake_no': ('Refuses: recoils back, looks away and down, shudders, comes back slowly.', -0.4, 0.5, False,
                 [((-6, -4, -10), 0.1, 1.0), ((-6, 2, 22), 0.2, 1.0), ((-6, 2, 17), 0.0, 1.0), ((-6, 2, 22), 0.6, 1.4)]),
    'shrug_fail': ('Lifts up hopefully, then drops: oh well.', -0.3, 0.3, False,
                   [((-2, -6, -12), 0.4, 1.0), ((-3, 10, 16), 0.8, 1.0)]),
    'go_to_sleep': ('Drowsy: the shade droops, jerks awake once, then sinks low.', -0.1, 0.0, False,
                    [((0, 4, 14), 0.4, 1.8), ((0, 0, 2), 0.2, 1.0), ((-6, 12, 24), 1.5, 2.0)]),
    'wake_up': ('Stirs: little shakes of the shade, a stretch up, then settles.', 0.3, 0.4, False,
                [((0, 0, -8), 0.0, 1.0), ((0, 0, 2), 0.0, 1.0), ((0, -12, -20), 0.6, 1.2), ((0, -6, -10), 0.2, 1.0)]),
    'look_at_user_return': ('Glances up from the desk at you, holds, goes back to work.', 0.4, 0.3, False,
                            [((-2, -4, -34), 1.2, 1.0)]),
    'dance_90': ('Grooves at 90 bpm: elbow bobs on the beat, shade swings on the off-beat.', 0.8, 0.8, True,
                 [((0, 6, -10), 0.0, 1.0), ((0, 0, 10), 0.0, 1.0)] * 3),
}


def clip(name, description, valence, arousal, loopable, steps):
    frames, duration = build(steps)
    return {
        'name': name, 'variant': 'lamp', 'description': description,
        'tags': {'category': 'emotion', 'family': 'lamp_style', 'valence': valence, 'arousal': arousal,
                 'source': 'tools/animation/scripts/make-lamp-style-clips.py'},
        'mode': 'additive', 'joint_mask': list(JOINTS),
        'keyframes': frames, 'events': [],
        'params': {'amplitude': {'default': 1.0, 'range': [0.5, 1.5]}, 'time_scale': {'default': 1.0, 'range': [1.0, 3.0]},
                   'hold': {'default': 0.0, 'range': [0.0, 1.0]}},
        'returns_to_start': True, 'loopable': loopable, 'duration_nominal': duration,
    }


if __name__ == '__main__':
    OUT.mkdir(exist_ok=True)
    for name, spec in CLIPS.items():
        path = OUT / f'{name}__lamp.json'
        path.write_text(json.dumps(clip(name, *spec), indent=1) + '\n')
        print(path.relative_to(OUT.parent))
