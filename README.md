# luxoiii

An expressive, local edge-AI desk lamp for the ACM/IEEE SEC 2026 Minimalist Edge AI Challenge.
The Arduino UNO Q runs camera perception and behavior on Linux, with light and motion control on its STM32.

## Start here

- **Full model / X-ray preview:** `python3 sim/lamp_show.py`, then open `http://127.0.0.1:8082/`.
  Includes the complete base, 112 CAD parts and mechanically coupled internal cams from `full-model-animations`.
  The preview is a browser animation; it sends no hardware commands and is not yet driven by live AI events.
- **Live 3D people view:** [tools/spatial/README.md](tools/spatial/README.md), with read-only camera and encoder setup.
- **App and board commands:** [apps/luxoiii/README.md](apps/luxoiii/README.md).
- **Hardware/API:** [arm/README.md](arm/README.md).
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
Keep the physical arm supported and the lamp aimed at the desk. The intended hybrid demo uses physical
light/pivot outputs with simulated failed joints. That hybrid control mode is still to be integrated;
`--fake-arm` currently skips the hardware connection entirely. Book detection remains unimplemented.

The app and arm source include the newer board-side behavior and hand tracking work recovered before
this refactor. The running board has not been renamed, redeployed or reflashed by the refactor.

## Verify

```sh
python3 -m unittest discover -s apps/luxoiii/tests
bash arm/scripts/test.sh
node --check apps/luxoiii/luxoiii/desk/cad_view.js
```

These checks use simulated hardware. Live camera/model tests require the board and installed inference runtime.
