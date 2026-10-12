#!/usr/bin/env python3
"""Port the expressive-lamp show to motion_lib clips for this 3-joint arm.

Source: arm_mujoco/lamp_demo.py (POSE and SHOW) on branch full-model-animations@ef1b0d2.
Its shoulder, elbow and wrist lean toward the front for positive angles, like this arm, so
each pose becomes an additive offset from the lamp's neutral 'look' pose (15, 45, 55).
Base yaw and head tilt are dropped (this arm has neither), so look_left/right and the
yaw-only head shake are left out. Segment times follow the lamp's minimum-jerk profile at
its 2-3 rad/s joint limits; scripts/emote.py slows each clip to this arm's speed caps.

    python3 scripts/make-lamp-clips.py      # writes arm/clips/lamp/lamp_*.json
"""
import json
from pathlib import Path

OUT = Path(__file__).resolve().parents[3] / 'arm' / 'clips' / 'lamp'
# (shoulder, elbow, wrist) from lamp_demo.POSE, yaw and head_tilt dropped.
POSE = {
    'sleep': (10, 90, 80), 'tall': (0, 0, 0), 'yawn': (-8, -10, -25), 'look': (15, 45, 55),
    'curious': (20, 50, 45), 'crouch': (-15, 80, 40), 'peek': (50, 15, 60),
    'nod_down': (15, 45, 72), 'bounce_dn': (8, 62, 50),
}
NEUTRAL = POSE['look']
VMAX = 120.0   # deg/s; minimum-jerk peak speed is 1.875 x the average

# clip name -> [(pose, hold after arriving s, speed factor <= 1)], each from and back to 'look'.
CLIPS = {
    'lamp_wake': ('Wakes from sleep: crouch, stretch up tall, yawn, then looks.', 0.3, 0.7,
                  [('sleep', 0.8, 1.0), ('crouch', 0.15, 0.6), ('tall', 0.3, 1.0), ('yawn', 0.6, 0.5), ('look', 0.0, 1.0)]),
    'lamp_curious': ('Cocks toward something, then crouches and peeks in.', 0.5, 0.5,
                     [('curious', 0.8, 0.7), ('look', 0.2, 1.0), ('crouch', 0.1, 1.0), ('peek', 0.7, 1.0), ('look', 0.0, 1.0)]),
    'lamp_yes': ('Two firm nods.', 0.6, 0.5,
                 [('nod_down', 0.0, 1.0), ('look', 0.0, 1.0), ('nod_down', 0.0, 1.0), ('look', 0.0, 1.0)]),
    'lamp_happy': ('Two happy bounces.', 0.8, 0.8,
                   [('bounce_dn', 0.0, 1.0), ('look', 0.0, 1.0), ('bounce_dn', 0.0, 1.0), ('look', 0.0, 1.0)]),
    'lamp_sleep': ('Stretches tall, then sinks slowly back to sleep, and wakes again.', -0.3, 0.2,
                   [('tall', 0.3, 0.7), ('sleep', 1.5, 0.45), ('look', 0.0, 0.6)]),
}


def offsets(pose):
    return {name: float(POSE[pose][i] - NEUTRAL[i])
            for i, name in enumerate(('shoulder_pitch', 'elbow_pitch', 'wrist_pitch'))}


def clip(name, description, valence, arousal, steps):
    t, prev = 0.0, NEUTRAL
    keys = [{'t': 0.0, 'joints': offsets('look'), 'ease': 'ease_in_out'}]
    for pose, hold, factor in steps:
        distance = max(abs(a - b) for a, b in zip(POSE[pose], prev))
        t += max(0.35, 1.875 * distance / (VMAX * factor))
        keys.append({'t': round(t, 3), 'joints': offsets(pose), 'ease': 'ease_in_out'})
        if hold > 0:
            t += hold
            keys.append({'t': round(t, 3), 'joints': offsets(pose), 'ease': 'linear'})
        prev = POSE[pose]
    return {
        'name': name, 'variant': 'arm', 'description': description,
        'tags': {'category': 'emotion', 'family': 'lamp_demo', 'valence': valence, 'arousal': arousal,
                 'source': 'full-model-animations@ef1b0d2 arm_mujoco/lamp_demo.py'},
        'mode': 'additive', 'joint_mask': ['shoulder_pitch', 'elbow_pitch', 'wrist_pitch'],
        'keyframes': keys, 'events': [],
        'params': {'amplitude': {'default': 1.0, 'range': [0.3, 2.5]}, 'time_scale': {'default': 1.0, 'range': [1.0, 6.0]},
                   'hold': {'default': 0.0, 'range': [0.0, 1.0]}},
        'returns_to_start': True, 'loopable': False, 'duration_nominal': round(t, 3),
    }


if __name__ == '__main__':
    OUT.mkdir(exist_ok=True)
    for name, (description, valence, arousal, steps) in CLIPS.items():
        (OUT / f'{name}__arm.json').write_text(json.dumps(clip(name, description, valence, arousal, steps), indent=1) + '\n')
        print(OUT / f'{name}__arm.json')
