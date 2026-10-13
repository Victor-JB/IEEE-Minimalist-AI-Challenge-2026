# arm: the shared arm module

Everything about the physical arm lives here, so Lumen (`apps/lumen`) and every manual or test
tool use the same firmware, protocol, parameters and clips.

```
arm/
├── sketch/          STM32 firmware: 1 kHz closed loop on the AS5600 joint encoders, joint limits,
│                    300 ms motion lease, driver sleep (folder name must match sketch.ino)
│                    + head-tilt servo (roll.hpp). Pins: STEP D3/D5/D9, DIR D6/D4/D7 (elbow,
│                    shoulder, head), SLEEP D8, roll servo D2 (TIM2, 50 Hz), encoders I2C D20/D21
│                    + lamp LED (light.hpp), off by default: LIGHT_ENABLED 1 = LED on D7, head DIR on D12
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

## Lamp LED (not wired yet, not flashed)

`arm_light` RPC, gateway `LIGHT <0-1000> [fade ms]` / `LIGHTSTAT`, `ArmMotion.light()`. The level
is a duty in per mille, already gamma-corrected by Lumen (`apps/lumen/lumen/light/light.py`); the MCU
fades linearly to it over `fade ms` and drives a 20 kHz PWM. No session owner needed, and the level
stays when the session ends. Firmware built with `LIGHT_ENABLED 0` (the default) or older
firmware answers `supported: False` and the light is a no-op, so nothing else breaks.

**Pin: D7 (PB2, TIM8 CH4N).** TIM8 drives no other header pin, so the LED gets its own timer.
Every other PWM pin shares a timer that is taken: TIM3 (D3, D6, D8), TIM1 (D5, D11-D13) and TIM4
(D9, D10) run the STEP pulse trains, whose period and mode change constantly; TIM2 (D2, D20, D21)
is the roll servo's fixed 50 Hz (visible flicker, bands on camera) and the encoder I2C. D7 is the
head DIR today, so turning the light on means moving that wire:

1. Head driver DIR: **D7 -> D12**. D12 only runs as a plain GPIO, the same way D6 already does.
2. Set `#define LIGHT_ENABLED 1` at the top of `sketch/sketch.ino` (or pass it as a build flag),
   run `bash arm/scripts/test.sh`, then `bash arm/scripts/deploy.sh`.
3. LED driver, low side, with a logic-level N-MOSFET that is fully on at a 3.3 V gate (AO3400,
   IRLB8721, IRLZ44N; not IRF540 or IRFZ44N):
   - D7 -> 100 ohm -> gate; 10 kohm gate -> GND (keeps the LED off while the MCU boots).
   - Source -> GND, tied to the UNO Q GND. Drain -> LED cathode (-).
   - LED anode (+) -> LED supply +. A 12 V LED strip already has its resistors. A bare 1 W white
     LED (Vf about 3.2 V, 300 mA) on 5 V needs (5 - 3.2) / 0.3 = 6 ohm: use 6.8 ohm, 1 W. A
     constant-current LED driver with a PWM/DIM input is better, wired the same way.
   - Anything over about 0.5 A gets its own supply (or the servo's buck), not the board's 5 V.

The other option is Linux-side PWM/GPIO on the QRB2210: see `apps/lumen/lumen/light/light.py`.
