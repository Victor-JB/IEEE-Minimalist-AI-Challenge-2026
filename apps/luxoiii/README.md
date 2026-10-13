# luxoiii

Camera → local perception → behavior → expressive motion and light. The shared arm module is
[`arm/`](../../arm/README.md); the behavior specification is [`docs/STATE_MACHINE.md`](../../docs/STATE_MACHINE.md).

## Preview on the Mac

From the repository root:

```sh
python3 sim/lamp_show.py
```

Open `http://127.0.0.1:8082/`. This preview includes the complete ARM v29 base and internal cams,
with X-ray, solid, and hidden-case views. Without `--observer-url` it uses no camera, board connection or inference runtime.

## Run the app

From `apps/luxoiii`, with its Python dependencies installed:

```sh
python -m luxoiii --fake-arm --stream 8080   # camera/AI, simulated motion, no physical outputs
python -m luxoiii.io.camera --probe         # list cameras
python -m luxoiii.desk --port 8081          # standalone manual desk (stop other hardware owners first)
```

On the board, the main app is `python -m luxoiii --headless --stream 8080`. The camera/HUD is at `/`,
manual controls at `/desk`, and the read-only model animation at `/preview`. The manual page pauses
behavior while connected and releases ownership back to the app on disconnect.

The deployment commands below are run from the repository root. Review [migration notes](../../docs/NAMING.md)
before replacing a running board installation:

```sh
LUXOIII_NO_START=1 bash apps/luxoiii/scripts/deploy.sh  # install without switching running services
bash apps/luxoiii/scripts/deploy.sh                    # install and start luxoiii.service
bash apps/luxoiii/scripts/run.sh                       # on the board: foreground development
```

App deployment does not reflash the MCU. The firmware comes from `arm/`. The lamp is verified on D7;
the new service sets `LUXOIII_LIGHT=firmware`. Otherwise the default light backend is `sim`.
Use `LUXOIII_HOST` and `LUXOIII_LINK` to select the board/connection.

## Implementation status

Implemented: quantized face detection, palm/hand landmarks and tracking, gesture rules, motion gating,
state-dependent inference scheduling, expressive behavior, light ramps, firmware light control,
performance logging, a shared manual desk, and the complete browser CAD renderer.

The separate hybrid demo in `tools/spatial` now connects hand/face inference to the full simulated
arm and independently to real-pivot subject tracking. It selects desk spots using an estimated finger ray.
Book detection remains unimplemented. The regular arm mode still expects working joints;
`--fake-arm` skips all physical arm outputs. The supplied board source and simulated tests do not by
themselves establish live demo reliability.

Keep the real light off during subject tracking. Camera and pivot are team-verified. Avoid full-arm hardware
commands while the steppers are broken. Do not interpret an animated preview as measured arm motion.

## Development and measurement

From the repository root:

```sh
python3 -m unittest discover -s apps/luxoiii/tests
bash arm/scripts/test.sh
python3 apps/luxoiii/tools/simulate.py
```

`PERF=1` enables pipeline timing; `PERF_CSV=/path/to/run.csv` saves samples. Transition logs are in
`apps/luxoiii/logs/`. `tools/scorecard.py` summarizes logged results. Synthetic simulation/power data is
for testing the tooling, not competition efficiency claims.

`luxoiii/paths.py` centralizes repository paths. Arm configuration remains in `arm/python/arm_config.py`.

## Live spatial preview

See [the spatial observer guide](../../tools/spatial/README.md) to show estimated face locations
relative to the physical head camera beside the full X-ray arm. Inference runs on the UNO Q;
the Mac receives camera/encoder/hand snapshots. The dedicated hybrid controls optionally activate
only the pivot for subject tracking; D7 stays off. View calibration assumptions are adjustable; board targeting uses the
documented default calibration. Read the guide before changing calibration.
