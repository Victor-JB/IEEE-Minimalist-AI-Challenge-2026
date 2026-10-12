# arm: the shared arm module

Everything about the physical arm lives here, so Lumen (`apps/lumen`) and every manual or test
tool use the same firmware, protocol, parameters and clips.

```
arm/
├── sketch/          STM32 firmware: 1 kHz closed loop on the AS5600 joint encoders, joint limits,
│                    300 ms motion lease, driver sleep (folder name must match sketch.ino)
├── python/          flat modules, imported by plain name once arm/python is on sys.path
│   ├── arm_config.py   ★ the one place for HOME, SAFE_RANGES, speed/accel caps, stream rate
│   ├── arm_client.py · arm_motion.py   Python API (ArmMotion.move_to / stream_point / relax)
│   ├── gateway.py · arm_gateway.py · script_gateway.py · scanner.py   board-side router bridge
│   ├── player.py       clips → dense joint trajectories
│   └── trajectory.py   clip → real-arm trajectory under the caps in arm_config
├── calibration/     arm-calibration-2026-10-11.json (zero offsets, sensor directions, microsteps)
├── clips/           authored/ (from tools/motion_lib/author.py) · lamp/ (lamp_*/arm) · poses.json
├── scripts/         deploy.sh (Mac → board: copy, venv, compile + flash) · flash-local.sh · test.sh
└── tests/           firmware logic (C++ against the sketch headers) + Python API + shared-config tests
```

```sh
bash arm/scripts/test.sh          # no board needed
bash arm/scripts/deploy.sh        # once, and whenever sketch/ or python/ changes (flashes the MCU)
```

Use it from any script:

```python
sys.path.insert(0, "<repo>/arm/python")
import arm_config as C
from arm_motion import ArmMotion
from trajectory import trajectory
```

Changing a cap or range in `arm_config.py` changes it for the app and the manual tools together.
The Mac app (dev branch, `apps/stepper_desk/mac`) keeps its own Swift copy of the joint limits
(`ArmModel.swift`): update both if the firmware limits change.
