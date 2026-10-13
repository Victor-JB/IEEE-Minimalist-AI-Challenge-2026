# luxoiii naming and migration

The product name, Python package and service prefix are **luxoiii** (lowercase).

| Before | Now |
|---|---|
| `apps/lumen/` | `apps/luxoiii/` |
| `python -m lumen` | `python -m luxoiii` |
| `lumen.service`, `lumen-desk.service` | `luxoiii.service`, `luxoiii-desk.service` |
| `LUMEN_LIGHT`, `LUMEN_LIGHT_*` | `LUXOIII_LIGHT`, `LUXOIII_LIGHT_*` |
| `STEPPER_HOST`, `STEPPER_LINK`, `STEPPER_VENV`, `STEPPER_BUILD_DIR` | `LUXOIII_HOST`, `LUXOIII_LINK`, `LUXOIII_VENV`, `LUXOIII_BUILD_DIR` |
| `LUMEN_NO_START` | `LUXOIII_NO_START` |
| `apps/stepper_desk/` | `tools/animation/` |
| `scripts/luxoIII-board-setup.sh` | `scripts/luxoiii-board-setup.sh` |

Old environment names are accepted as fallbacks; the luxoiii name takes precedence. Old Python package
paths are not entry points. Dated records retain the names and paths used when those records were made.

The existing GitHub repository URL, local checkout directory, board checkout directory
(`/home/arduino/IEEE-Minimalist-AI-Challenge-2026`) and the board's existing `luxo3` network alias are
infrastructure identifiers, not product branding. They were not renamed, so existing access continues to work.
The deployment default is the known board address, `100.78.199.105`, with `LUXOIII_HOST` to override it.

## Safe deployment transition

The repo contains the board's newer source plus the Mac's desk light controls. The standalone desk's
0–1 brightness UI is converted to the board API's 0–1000 per-mille duty units.

The running board was left unchanged. Before deploying, preserve any additional board edits made since
this snapshot. `LUXOIII_NO_START=1 bash apps/luxoiii/scripts/deploy.sh` installs the new app and units
without stopping or enabling services. A normal deployment stops/disables legacy units and starts
`luxoiii.service`. The new units also conflict with old units to prevent competing arm/camera owners.
Application deployment does not flash firmware. `arm/scripts/deploy.sh` does flash unless
`ARM_NO_FLASH=1` is specified; avoid reflashing the verified hardware just for naming changes.

## Full model

`python3 scripts/import_full_model.py` imports the pinned `full-model-animations` model, every STL instance,
and its exact body hierarchy and gear constraints into `arm/cad/full-model.json`. The original XML is
retained as `arm/cad/source-xray.xml`. The browser shares one renderer between the desk and the standalone
preview. The old reduced `model.json` remains for compatibility but is no longer used by the browser.

X-ray is the default, with solid and hidden-case options. The preview animates base yaw, shoulder, elbow,
head pitch and head pivot. Internal drive motion is derived kinematically; it is not measured telemetry.
The physical pivot, camera and D7 lamp are team-verified. The `tools/spatial` hybrid demo connects live hand/face events to this simulation and optional
physical subject tracking; see its README for the separate, non-service deployment.
