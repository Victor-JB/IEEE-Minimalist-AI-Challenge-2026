# luxoiii

An expressive, local edge-AI desk lamp for the ACM/IEEE SEC 2026 Minimalist Edge AI Challenge.
The Arduino UNO Q runs camera perception and behavior on Linux, with light and motion control on its STM32.

## Start here

- **Main app:** `bash apps/luxoiii/scripts/run.sh`, then open `http://127.0.0.1:8082/`.
  Live camera, two-hand gestures, expressive full X-ray model, subject tracking and digital D7 On/Off.
- **Board connection:** [tools/spatial/README.md](tools/spatial/README.md) covers the observer and SSH tunnel.
- **Offline model:** `PYTHONPATH=apps/luxoiii python3 -m luxoiii --offline`.
- **App and board commands:** [apps/luxoiii/README.md](apps/luxoiii/README.md).
- **Hardware/API:** [arm/README.md](arm/README.md).
- **Measured efficiency and accuracy:** [UNO Q scorecard](reports/efficiency/README.md).
- **Naming and deployment migration:** [docs/NAMING.md](docs/NAMING.md).

## Layout

| Directory | Purpose |
|---|---|
| `apps/luxoiii/luxoiii/` | Python app, perception, behavior, light, desk UI, and model preview |
| `apps/luxoiii/tests/` | Behavior, perception, integration and layout tests |
| `apps/luxoiii/tools/` | Headless behavior simulation and efficiency scorecards |
| `arm/` | STM32 firmware, shared Python API, calibration, clips and full CAD model |
| `tools/animation/` | Lamp clip authoring tools and reference clips |
| `sim/` | Full-model preview entry point and historical simulation material |
| `scripts/` | Board setup and reproducible full-model import |
| `docs/` | Behavior spec, migration notes and presentation material |

## Current hardware and demo scope

As confirmed by the team on October 13: the pivot, camera and lamp on **D7** work; the arm steppers are broken.
The D7 lamp configuration uses D12 for the head stepper DIR signal. Do not revert that pin assignment.
Keep the physical arm supported. The hybrid demo in `tools/spatial` uses the physical pivot for subject tracking and the simulated arm for expressive motion and gesture lighting. D7 can turn on only with fresh face-free observations and the entire estimated physical beam below horizontal; it uses an 800 ms firmware expiry.
The main app automatically enables subject tracking with fresh data; its switch saves an explicit off preference, and Stop remains latched. Pointing selects an estimated desk
spot in the simulated scene; the real light has a separate face/physical-orientation gate.
The older controller is now explicitly `python -m luxoiii.legacy`; its `--fake-arm` skips the hardware connection entirely. Book detection remains
unimplemented; this demo does not claim to detect a book.

The app and arm source include the newer board-side behavior and hand tracking work recovered before
this refactor. The running board has not been renamed, redeployed or reflashed by the refactor.

## Verify

```sh
python3 -m unittest discover -s apps/luxoiii/tests
bash arm/scripts/test.sh
node --check apps/luxoiii/luxoiii/desk/cad_view.js
```

These checks use simulated hardware. Live camera/model tests require the board and installed inference runtime.
