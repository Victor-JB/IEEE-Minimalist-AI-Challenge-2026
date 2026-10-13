# arm: the shared arm module

Everything about the physical arm lives here, so luxoiii (`apps/luxoiii`) and every manual or test
tool use the same firmware, protocol, parameters and clips.

```
arm/
├── sketch/          STM32 firmware: 1 kHz closed loop on the AS5600 joint encoders, joint limits,
│                    300 ms motion lease, driver sleep (folder name must match sketch.ino)
│                    + head-tilt servo (roll.hpp). Pins: STEP D3/D5/D9, DIR D6/D4/D12 (elbow,
│                    shoulder, head), SLEEP D8, roll servo D2 (TIM2, 50 Hz), encoders I2C D20/D21
│                    + lamp LED (light.hpp), LIGHT_ENABLED 1 (default) = LED on D7, head DIR on D12
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
Historical Mac tools on the `dev` branch used their own Swift limits; current app and desk use the shared Python configuration.

## Lamp LED (wired and flashed 2026-10-13: LED on D7, head DIR on D12)

`arm_light` RPC, gateway `LIGHT <0-1000> [fade ms]` / `LIGHTSTAT`, `ArmMotion.light()`.
**D7 is a plain digital on/off output: HIGH on, LOW off. No PWM or fades.**
The existing request shape remains compatible: any positive level turns the lamp fully on,
zero or negative turns it off, and the response reports 1000 or 0. Fade values are validated
but ignored. `LIGHT_ENABLED 0` disables the lamp for the old wiring.

Automatic lighting uses `arm_light([level, fade_ms, lease_ms])`, with a 1–2000 ms lease.
The independent MCU tick forces D7 LOW when the lease expires, even if the Linux observer
stops. Read requests do not renew it. Omitted/zero lease retains manual on/off behavior.
The hybrid observer uses an 800 ms lease and checks fresh face results plus the physical
beam's horizon clearance before every renewal. Older firmware that rejects the lease
cannot activate automatic lighting.

**Pin: D7 (PB2), with head DIR on D12.** D7 no longer uses TIM8.
The following wiring is already verified on the current board:

1. Head driver DIR: **D7 -> D12**. D12 only runs as a plain GPIO, the same way D6 already does.
2. Build with `LIGHT_ENABLED 1` (the default in `sketch/sketch.ino`; 0 only for the old wiring),
   run `bash arm/scripts/test.sh`, then `bash arm/scripts/deploy.sh`.
3. LED driver, low side, with a logic-level N-MOSFET that is fully on at a 3.3 V gate (AO3400,
   IRLB8721, IRLZ44N; not IRF540 or IRFZ44N):
   - D7 -> 100 ohm -> gate; 10 kohm gate -> GND (keeps the LED off while the MCU boots).
   - Source -> GND, tied to the UNO Q GND. Drain -> LED cathode (-).
   - LED anode (+) -> LED supply +. A 12 V LED strip already has its resistors. A bare 1 W white
     LED (Vf about 3.2 V, 300 mA) on 5 V needs (5 - 3.2) / 0.3 = 6 ohm: use 6.8 ohm, 1 W. A
     constant-current LED driver with a PWM/DIM input is better, wired the same way.
   - Anything over about 0.5 A gets its own supply (or the servo's buck), not the board's 5 V.

The other option is Linux-side PWM/GPIO on the QRB2210: see `apps/luxoiii/luxoiii/io/light.py`.
