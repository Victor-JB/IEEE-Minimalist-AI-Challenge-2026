# motion_lib: expressive motion library

The Sprint 1 brief (`adding_emotion.txt`): a keyframed clip library, a player, sim validation,
and expressive modifier rules. Everything runs against the robot in `sim/robot.toml`.

## Watch it
```
python motion_lib/demo.py                          # guided tour of 16 clips in the MuJoCo viewer
python motion_lib/demo.py --clip nod_yes/big_eager --loop
python motion_lib/demo.py --category emotion
python motion_lib/demo.py --gamma reach_desk_spot  # the same task at gamma 0 -> 0.3 -> 1.0
python motion_lib/demo.py --list
```
Review images (trajectory plots plus a filmstrip of rendered frames, instead of GIFs):
`review/clips/*.png` (one per clip, linked from [CATALOG.md](CATALOG.md)) and
`gamma_comparison/*.png`.

## Pipeline
```
python motion_lib/author.py          # clips are authored in code -> clips/*.json, poses.json
python motion_lib/validate.py        # Part C checks; shrinks ranges in the JSON -> validation_report.md
python motion_lib/review.py          # plots + filmstrips -> review/clips/, CATALOG.md
python motion_lib/review.py --gamma  # Part D comparison -> gamma_comparison/
python -m unittest discover motion_lib/tests
```
Re-run from `author.py` after changing a clip or `sim/robot.toml` (joint limits, speed, accel, jerk).

## Files
| file | role |
|---|---|
| `robot.py` | joints, limits and MuJoCo models from `sim/robot.toml` |
| `player.py` | the clip format (see its docstring) and `play()` / `move()` -> dense trajectory at the control rate |
| `author.py` | every clip, written with helpers for anticipation, overshoot/settle, follow-through (`lag`), oscillation |
| `checks.py` | joint limits + margin, velocity / acceleration / jerk, collisions, end pose, torque |
| `validate.py` | amplitude x time_scale grid, 10 sampled start poses, range shrinking with reasons |
| `modifiers/` | `Segment` functional trajectories, the rule engine, and the 8 rules |
| `functional.py` | the 7 representative functional moves used for the gamma comparison |
| `demo.py`, `review.py` | sim playback; plots |

## Conventions
- Angles are joint degrees (0 = the straight pose in `robot.toml`). `+wrist_pitch` = look down,
  `+wrist_yaw` / `+base_yaw` = turn left, head pitch in the world = shoulder + elbow + wrist pitch.
- `amplitude` scales motion around the straight start-to-end line, so entry and exit poses
  never move. `time_scale` 2.0 means twice as slow. `hold` adds time at the keyframe marked `peak`.
- Additive clips are validated from home and from sampled poses. A `WARN` means some poses lack
  joint headroom, so the next sprint's selector must check before playing.

## Known limits / next steps
- **The 120 deg/s speed limit is the binding constraint.** Most high-arousal clips have no
  amplitude or speed headroom left. Measure the real servos, update `robot.toml`, and re-run the pipeline.
- **The collision model is crude** (capsules plus a box, parts within 2 links ignored). Use the
  CAD meshes once the arm exists.
- **Whether a motion reads as alive can't be checked in sim.** Watch `demo.py` and prune or
  retune in `author.py`.
- **App send rate:** the app's `main.py` sends targets at 20 Hz. Send clip trajectories at the
  control rate (50 Hz), or short gestures lose detail.
- **Expressiveness costs time:** at gamma 1 a functional task takes about 1.5-2x as long.
  That's the paper's trade-off, and the reason function tasks use gamma ~0.25.
