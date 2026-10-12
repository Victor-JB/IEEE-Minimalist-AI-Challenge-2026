# sim: robot descriptions and the MuJoCo viewer

Two robot descriptions share one format and one generator (`configurator.py`):

| File | Robot | Used by |
| --- | --- | --- |
| `robot.toml` (default) | The original 6-joint expressive robot (base yaw, shoulder, elbow, wrist pitch/yaw, head roll; servo speeds) | `motion_lib` authoring/validation (the concept robot the clips are authored on) |
| `stepper_desk.toml` | The real 3-joint stepper arm in `apps/stepper_desk`: shoulder, elbow, head | `motion_lib` playback with `ROBOT_CONFIG`, `compare_real.py` |

```sh
python tools/sim/view.py                                   # robot.toml
python tools/sim/view.py --config tools/sim/stepper_desk.toml    # the real arm, with its CAD parts
python tools/sim/view.py --config tools/sim/stepper_desk.toml --torque
ROBOT_CONFIG=tools/sim/stepper_desk.toml python motion_lib/demo.py --clip nod_yes/medium
python tools/sim/compare_real.py apps/stepper_desk/recordings/emote-traces.json
python tools/sim/longest_line.py --plot      # longest straight line for the head pivot
```

Needs `pip install mujoco numpy` (and `matplotlib` for `compare_real.py` plots). Every config writes `tools/sim/build/<name>.xml`.

## The real-arm model (`stepper_desk.toml`)

- **Geometry**: the app's 3D view (`apps/stepper_desk/mac/ArmCAD.swift`): 18.5 cm shoulder→elbow and elbow→head pivots, 4.5 cm head. The `[cad]` section loads the same ARM v29 meshes and part transforms (`apps/stepper_desk/mac/Resources/ArmCAD`, imported from `full-model-animations@ef1b0d2`). All three joints turn the same way, positive toward the front. Joint names keep motion_lib's (`wrist_pitch` is the head) so clips play; clip joints the arm lacks are dropped.
- **Masses**: the CAD branch's budget (`arm_mujoco/build_mjcf.py`, `expected_masses.json`): shoulder link 0.855 kg, elbow link 0.409 kg, head 0.345 kg, including the cycloid cams and plates the app does not draw. Part densities only spread each body's mass over its visible parts, using enclosed STL volumes (MuJoCo's own estimate would weigh the hollow shade as a solid bowl).
- **Limits**: firmware joint limits (shoulder −45…38°), 60 °/s and 500 °/s² from the speed sweep, stiff position actuators (kp 1000, CAD drive damping 0.65) that hold within about 0.1° like the real loop.
- **Torque**: `max_torque` is the holding limit fitted to what the arm held (shoulder 2.3, elbow 2.0, head 1.0 N·m). `[measured]` records what the kinematic sim does not reproduce yet: the shoulder's lifting limit (about 1.27 N·m), position-dependent friction near the ends of travel, the head's slow zone, learned pulses/degree and streaming lag. Details and evidence: `apps/stepper_desk/CLOSED_LOOP.md`.

`compare_real.py` drives the sim with the exact command the arm streamed and reports RMS error for both, plus the real arm's streaming delay. To make the sim match more closely, the next additions are a ~50 ms command delay, per-joint reversal slack and 0.088° sensor quantization.
