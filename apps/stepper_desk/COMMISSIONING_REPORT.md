# Arduino Q robot arm commissioning record

Updated 11 October 2026. Project: `apps/stepper_desk` in the IEEE Minimalist AI Challenge 2026 repository. This is the consolidated handover record for the hardware, Mac controls, calibration experiments, accepted tolerances, firmware, recordings, and next development stages.

## Current result

All three joints move, report angle feedback, enforce configured directional limits, and sleep when idle. The user accepted the observed local endpoint prediction errors, and the working acceptance tolerance is **1.5°**. Six subsequent single-joint sensor-target checks passed: the largest powered or released endpoint error was **1.062°**.

This is **local, approximate endpoint acceptance**, not a claim that the entire arm is fully calibrated across every joint limit. Independent physical angle accuracy, actual hardware microstep settings, globally valid pulse-to-angle conversions, coordinated path accuracy, and autonomous stall recovery remain unverified. No learned model is deployed. The existing GUI's global ratio verification gate remains intact.

| Item | Current status |
| --- | --- |
| Sensor/motor mapping, saved zero offsets and direction signs | Recorded; motion direction checked |
| Magnet mounting | User confirmed centered/coaxial and rigidly attached to output joints |
| Physical joint angle against an independent reference | Not measured in this commissioning record |
| Shared D8 sleep and motion guards | Implemented, tested, deployed |
| Actual STEP pad edge counts | 100 observed for 100 counted on all axes at three tested rates, drivers asleep |
| Local model endpoint error | Accepted retrospectively at 1.5°; old 0.5° failures retained |
| Local sensor-target stopping | Six real moves passed, both directions per joint |
| Full-range and coordinated calibration | Incomplete; local evidence must not be extrapolated |
| Driver current/torque and temperature | Physical settings; no software measurement or control |
| Latest software checks | 101 Python tests passed; integrated C++/Swift checks and Mac build also passed earlier in this commissioning run |

Authoritative current summaries: [progress](diagnostics/full-arm-calibration-progress.json), [local profile](calibration/local-endpoint-profile-v1.json), [target verification](diagnostics/local-target-verification-2026-10-11.json), and [retrospective acceptance](diagnostics/local-endpoint-acceptance-v2-2026-10-11.json). Older dated README checks describe their individual test runs, not the current overall state.

## System and project history

The original prototype used Zephyr on an nRF9151 Connect Kit, with user-supplied DIR P30 and STEP P31, a DRV8825, and e-ink status. Its application path is `/Users/tongtong/Nextcloud/Projects/tongtong-suite/NCS-Project/nrf9151-connectkit/applications/stepper_test`. Arduino Q development is separate; this project uses the Mac for status and does not drive the e-ink displays.

During early driver troubleshooting the user reported 20 V VMOT, shared grounds, a 1000 µF capacitor across VMOT/GND, RESET/SLEEP around 4.53 V, ENABLE 0 V, and an HW216-marked carrier. VREF was initially 1.17 V, then set to 0.5 V, and later adjusted until movement worked. Those are historical observations, not current measured settings. In-circuit multimeter readings of 12 Ω and 9 Ω beside the driver did not establish the sense-resistor markings. The present current limit must not be inferred from those readings. A reported failure near “2800” lacks a verified unit/test definition here and is not the software speed limit.

The Mac application evolved from a simple stepper test into arm visualization, per-motor ramp editing, Arm plus Hold Forward/Hold Reverse controls, sensor calibration, limit guards, sleep control, and a separate Poses & recordings workspace. The base swivel was removed. The three motorized joints are shoulder, elbow, and head/wrist. A separate CAD head-tilt servo is fixed in the visualization and is not a fourth controlled motor.

The chosen longer-term architecture is a conventional planner with a small learned correction around it, while leaving compute available on the Qualcomm processor for later vision and pose tracking. That direction is a plan, not an implemented learned controller. See [FOUNDATION.md](FOUNDATION.md).

## Hardware and wiring

Joint order in public scripts/UI is **shoulder, elbow, head**. Motor-array order in firmware/recordings is **elbow, shoulder, head**. Confusing these orders produces incorrect analysis.

| Joint | Connection | Motor / zero-based axis | STEP | DIR | Timer | Sensor / mux channel | Configured microsteps | Forward angle change | Hard limits |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Shoulder | Base to first segment | Motor 2 / axis 1 | D5 / PA11 | D4 | TIM1 CH4 | Sensor 1 / CH0 | 16 | Positive | −45° to +38° |
| Elbow | First to second segment | Motor 1 / axis 0 | D3 / PB0 | D2 | TIM3 CH3 | Sensor 2 / CH1 | 2 | Negative | −72° to +79° |
| Head | Second segment to head | Motor 3 / axis 2 | D9 / PB8 | D7 | TIM4 CH3 | Sensor 3 / CH2 | 16 | Negative | −80° to +87° |

All three DRV8825 SLEEP inputs share **D8 / PB4**. SLEEP is disconnected from 5 V and RESET. RESET stays HIGH separately; ENABLE stays LOW. Grounds are common. VMOT is supplied externally at the user's reported 20 V; D8 controls sleep and does not switch VMOT or the 5 V header supply. Stopping removes coil holding torque. The user reports the gearboxes resist back-driving, but load holding and cooling rate have not been characterized.

The physical multiplexer is **PCA9548A**, powered at 3.3 V, RESET HIGH, address 0x70 with A0–A2 LOW. Its SDA and SCL connect to the Arduino Q header labeled **SDA** and **SCL**: firmware aliases D20/PB11 and D21/PB10, on I2C2. Use the printed SDA/SCL labels if D20/D21 are not visible. Sensor 1 connects to SD0/SC0, sensor 2 to SD1/SC1, sensor 3 to SD2/SC2. Each AS5600 is at address 0x36; the mux selects one channel at a time. Logic and pull-ups on the Q side are 3.3 V.

Sensors are sampled at approximately 50 Hz through a 100 kHz I²C bus. Weak-field readings remain usable if magnet detection is present, as requested. Missing/strong-field/error/stale feedback stops protected motion. A valid angle is not a torque measurement.

The saved settings assume 200 full steps/revolution and half-step for Motor 1, 1/16 for Motors 2 and 3. These are **configuration values**, not independently confirmed MODE wiring or motor specifications. The CAD models 16:1 reductions; the user was unsure of the physical reductions. Measured pulses per reported joint degree therefore remain more useful than an assumed gearbox ratio.

## Connection and deployment

```text
Mac app or Python script
  → SSH through ADB USB forwarding, localhost:2222
  → Arduino Q Linux, arduino@daboard
  → Python bridge client
  → /var/run/arduino-router.sock
  → arduino-router, /dev/ttyHS1 at 115200 baud
  → Arduino_RouterBridge on STM32U585
  → timers and driver pins
```

The Qualcomm Linux processor handles transport and future high-level inference; the STM32 owns pulse timing and motion guards. The router is the UART owner. Do not open `/dev/ttyHS1` from a second program. The USB serial Monitor is not the Mac application's motor-command transport.

User-reported networking history: the board joined BroncoFi at 172.31.96.45 after MAC registration, but the Mac could not reach SSH over campus Wi-Fi. USB forwarding and passwordless SSH worked. That DHCP address is historical and may change; USB is the established control path. No private key or password belongs in the documentation.

```sh
# Mac connection setup; does not request motor motion.
adb -s 2709356903 forward tcp:2222 tcp:22
ssh -p 2222 arduino@localhost
```

Tailscale (added 2026-10-11): the board is on the tailnet as `daboard`. Tailscale 1.104.1 runs without root from `/home/arduino/tailscale/` in userspace-networking mode, as a systemd user service with linger enabled, so it survives reboots. The Python client, `scripts/deploy.sh` and the Mac app use USB when the board is plugged in and fall back to `ssh arduino@daboard` otherwise; `STEPPER_LINK=usb|tailscale` forces one and `STEPPER_HOST` overrides the host. Measured STATUS round trips: USB about 18 ms; Tailscale about 30 ms median with spikes to about 120-250 ms, so Tailscale requests get a 0.6 s deadline (USB 0.25 s). The MCU's 300 ms motion lease is unchanged, so a long spike halts and holds the arm rather than letting it run on.

Mac ADB path used: `/opt/homebrew/bin/adb`. Remote project: `/home/arduino/IEEE-Minimalist-AI-Challenge-2026/apps/stepper_desk`; Python environment: `.venv/bin/python`. Board serial: `2709356903`. USB forwarding must be restored after unplugging the board or restarting ADB.

Pinned versions in [sketch.yaml](sketch/sketch.yaml): Arduino Zephyr core 1.0.0, RouterBridge 0.4.3, RPClite 0.3.1, MsgPack 0.4.2, ArxContainer 0.7.0, ArxTypeTraits 0.3.2, DebugLog 0.8.4. Python dependencies: arduino-router-bridge 0.5.1 and msgpack 1.2.3.

The latest deployed preparation/settling build used `build-counted`, with 126,168 bytes flash and 60,572 bytes global RAM reported by compilation. `scripts/flash-local.sh` accepts `STEPPER_BUILD_DIR`; explicitly select `build-counted` when flashing that artifact. The normal deployment script compiles into `build` and flashes that freshly compiled directory. Flashing replaces the active MCU sketch.

```sh
# From apps/stepper_desk; deployment compiles and flashes.
bash scripts/deploy.sh
zsh mac/build.sh
bash scripts/test.sh
```

Latest app: [Stepper Desk Q.app](mac/Stepper%20Desk%20Q.app). No autonomous boot-time motor start is configured. Git baseline during documentation was `e99d1be`; subsequent commissioning changes and this report are working-tree changes, not a new commit/push claim.

## Saved angle calibration

The snapshot [arm-calibration-2026-10-11.json](calibration/arm-calibration-2026-10-11.json) preserves the user's saved GUI calibration, not a live pose. [joint-limits-2026-10-11.json](calibration/joint-limits-2026-10-11.json) preserves the requested bounds and confirmed direction signs.

| Joint | Zero offset | Sensor reversed |
| --- | --- | --- |
| Shoulder | 22.5° | No |
| Elbow | 53.349609375° | Yes |
| Head | −30.322265625° | Yes |

Conversion: `raw_degrees = raw_count × 360 / 4096`; calibrated angle is the wrapped result of `(raw_degrees − zero) × sign`, with sign −1 for reversed sensors. Zero pose is shoulder upright, with elbow and head continuing straight from the preceding link. Original physical angle entries used while calibrating were not retained by the app.

Fingerprint used by the recorded calibration dataset and accepted local profile:

```text
0:22.5:false:true:16:200|1:53.349609375:true:true:2:200|2:-30.322265625:true:true:16:200
```

The user confirmed concentric output-shaft magnet mounting. A requested independent shoulder tilt measurement and a check of elbow M0/M1/M2 wiring were not supplied. Acceptance of local endpoint error does not turn those into measured facts. Changing sensor zeros, direction wiring, microsteps, gearing, load, or mounting can invalidate the recorded response.

## Controls and safeguards

Manual GUI use: connect, select motor/settings, Arm, then hold Forward or Reverse. Release, drag away, Escape, focus loss, disconnect, or a guard fault stops motion. Default ramp reaches 150 RPM in 0.06 s and then approaches the target at 3 s, subject to the selected target and pulse ceiling. RPM is calculated from configured steps/microsteps and requested pulse frequency, not measured motor shaft speed.

`pulses_per_second = RPM × full_steps_per_revolution × microsteps / 60`.
At configured 200 full steps and half-step, 200 RPM is approximately 1333 pulses/s and 300 RPM is 2000 pulses/s. At 1/16, 1333 pulses/s is about 25 RPM and the 2400 pulses/s ceiling is 45 RPM. Earlier low-rate and manually recorded sessions must not be relabeled with incorrect RPM conversions.

| Guard or behavior | Implemented bound |
| --- | --- |
| Motion HOLD lease | 300 ms; actual client HOLD required |
| Exclusive arm ownership | 2 s reservation; STATUS renews ownership, not motion |
| Motion service timing | More than 50 ms gap aborts |
| Feedback age for motion | At most 100 ms; all three sensors required for group motion |
| Driver wake gate | At least 3 ms after D8 HIGH |
| Default preparation | 80 ms |
| Optional hardware-jog preparation | 80–500 ms; powered calibration uses 400 ms |
| Hardware counted jog | 20–2400 pulses/s; 3600 pulses, 12° travel, 60 s firmware caps |
| General client jog/trial | At most 10° and 60 s; shorter limits selected per experiment |
| Client response checks | Wrong direction >0.3°, untouched joint >0.75°, or no measurable progress for 2 s aborts |
| Powered PAUSE | STEP stops; fixed maximum 500 ms, HOLD and feedback guards continue |
| Calibration settlement | Approximately 350 ms powered pause; stable late baseline/end samples within 0.176001° spread |
| Ordinary idle | All drivers asleep; sensor polling continues |
| Explicit lock | Stops and prevents automatic wake until enabled again |

Limit guards block outward motion at a boundary and permit inward recovery. These are software limits with finite sensing/stopping delay, not physical limit switches. During the explicit bounded calibration PAUSE, drivers remain awake without pulses so powered and released endpoints can be measured separately. Outside that exception, stopping the final active motor sleeps all three. There is no measured temperature control or automatic current adjustment.

Three motion paths remain distinct:

1. **Manual GUI:** hardware PWM with user-held RPM/ramp commands and direction/limit guards.
2. **Counted jog/calibration:** timer one-pulse PWM, count/restart by interrupt, verified STEP-pad counts. A delayed interrupt cannot silently allow repeated timer rollovers. STOP reconciles a pending/already-risen edge before forcing LOW. Counts are issued edges, not proof of successful motor steps.
3. **Original coordinated pose backend:** minimum-jerk planning, planned peak at most 150 pulses/s and actual cap 200 pulses/s/joint; forced-active/inactive pulses at least 4 µs; moves at most 5°/joint and at least 5 s; target 2° inside hard limits. It still requires all three verified constant conversions. Following error >2° for 150 ms, timing faults, and final error >1° after settlement stop the move. Its limits have not been relaxed to 1.5°.

The new local helper uses sensor-guided single-joint stopping. It does **not** install global ratios, issue coordinated moves, or use the model's predicted pulse count as a motion command.

## Experiments and findings

All dates below refer to this commissioning record on 11 October 2026. File prefixes identify raw JSONL recordings; failed/inconsistent attempts are retained.

| Stage | Evidence and result |
| --- | --- |
| I²C bring-up | Early mux/scanner tests returned no responding addresses. Later live motion recordings contain fresh readings on all three channels. The early scanner failure is historical; the precise hardware change resolving it is not established here. |
| Slow elbow probes | 25-pulse and later 41-pulse tests had no resolved angle change. Too little motion for a ratio; not proof of motor failure. |
| User manual session | `manual-feedback-20261011-083230.jsonl` captured elbow movement around 75.3° → 20.1° → 72.6°. User reported half-step/300 RPM; passive logging did not capture authoritative manual PWM edge counts. |
| Manual PWM comparison | At nominal configured 200 RPM, elbow moved roughly −1.67°/+2.46° in 0.4 s and −5.274°/+5.449° in 0.8 s. The low-rate comparison had negligible response. |
| Counted STEP backend | Asleep pad test matched 100 observed edges to 100 issued on all axes at 100, 1333, and 2400 pulses/s. This does not establish pulse width on an oscilloscope, torque, or successful rotor motion. |
| Short counted probes | All three joints moved in the expected direction. Short startup-dominated ratios were not accepted. |
| Elbow 6° trials | Six endpoint ratios approximately 141.8–150.8 pulses/degree; spread about 6.1%, above the old 5% constant-ratio criterion. |
| Elbow 10° trials | Approximately 135.4–140.2 pulses/degree; a narrower passing spread did not erase disagreement with the shorter strokes. |
| Initial shoulder trial | About 0.8° post-stop relaxation invalidated a sleep-endpoint ratio. |
| Powered settlement | Added PAUSE so counted travel is measured while coils are still energized, followed by a separate sleep-drift measurement. |
| Wake settlement | Movement before any counted STEP motivated 400 ms preparation and a stable late zero-count baseline. |
| Reversal/continuation | Head used more pulses after reversal than for same-direction continuation. A constant conversion would mix lost motion with movement scale. |
| Position dependence | Shoulder and elbow responded differently across adjacent angle windows. Repeated data supported local position dependence; cause was not identified. |
| Frozen local model holdouts | Six shorter strokes each for shoulder and elbow yielded worst prediction errors 1.414° and 1.164°. Both failed the original 0.5° criterion. |
| Revised acceptance | User said the observed error was acceptable. A new retrospective 1.5° acceptance artifact preserves the original failures and source hashes. |
| Actual target stopping | Six additional single-joint target checks passed the accepted tolerance, including powered and released endpoints. |

Sources: [PWM tests](diagnostics/elbow-pwm-tests-2026-10-11.json), [pad verification](diagnostics/counted-pad-verification-2026-10-11.json), [model evaluation](diagnostics/calibration-model-evaluation-2026-10-11.json), [frozen local models](diagnostics/local-piecewise-model-frozen-2026-10-11.json), [held-out evaluation](diagnostics/local-heldout-evaluation-2026-10-11.json), and [target verification](diagnostics/local-target-verification-2026-10-11.json).

### Local endpoint models

Head: `issued_pulses ≈ 143.1 × abs(angle_change) + 260 × reversal`. The reversal term corresponds to roughly 1.82° of additional issued travel, not a directly measured mechanical backlash angle. Four independent approximately 6° strokes were predicted within 0.240°; maximum error over the recorded training and held-out sets was 0.349°.

Shoulder and elbow use `N ≈ abs(F(final) − F(initial)) + B × reversal`, where `F(q) = (q − knot) × gain_below_or_above`.

| Joint | Knot | Gain below | Gain above | Reversal term | Worst recorded endpoint prediction error |
| --- | --- | --- | --- | --- | --- |
| Shoulder | −7° | 119.367390 pulses/° | 172.576578 pulses/° | 0 pulses in frozen fit | 1.414° |
| Elbow | 61° | 148.518211 pulses/° | 131.586583 pulses/° | 49.159499 pulses | 1.164° |

The shoulder's zero fitted reversal term is not evidence that the mechanism has no backlash. The fit is approximate and retains a corrected history-label issue described below. These are models of reported sensor angles under the tested conditions. Position-dependent response may involve drivetrain behavior, load, compliance, sensor effects, or missed motion; the recordings do not isolate a cause.

Accepted local operating regions are deliberately narrower than the mechanical hard limits:

| Joint | Accepted local region |
| --- | --- |
| Shoulder | −17.227° to +2.285° |
| Elbow | +51.680° to +65.127° |
| Head | −4.395° to +8.613° |

These bounds support bounded single-joint testing; they do not certify every pose or every load in the interval. The first elbow held-out trial started slightly beyond its training envelope and remains explicitly flagged/excluded when defining operational scope. The 1.5° criterion was accepted **after** observing the model errors; it is not a successful predeclared 0.5° test. There is only about 0.086° margin between the worst recorded shoulder model error and the new threshold.

### Actual target checks

Errors below are measured endpoint minus requested angle. “Released” is after STOP and driver sleep. Both head tests used 1000 pulses/s; the four shoulder/elbow tests used 1333 pulses/s. All used 400 ms preparation.

| Joint | Target | Issued pulses | Powered error | Released error | Result |
| --- | --- | --- | --- | --- | --- |
| Shoulder | −9.5° | 324 | +1.062° | +1.062° | Pass |
| Shoulder | −11.7° | 445 | −0.429° | −0.253° | Pass |
| Elbow | +60.5° | 474 | −0.734° | −0.998° | Pass |
| Elbow | +62.7° | 543 | +0.669° | +0.581° | Pass |
| Head | +1.5° | 343 | −0.357° | −0.357° | Pass |
| Head | +3.8° | 644 | +0.507° | +0.507° | Pass |

These checks stop according to measured sensor travel. The model's predicted counts were logged only for comparison and often differed from actual counts. Thus the six passes validate this stopping procedure locally; they do not validate feedforward positioning or a coordinated trajectory. Every recording ended stopped and asleep.

A later passive snapshot read shoulder −12.041°, elbow +63.281°, head +4.307°, owner 0, run 0, awake 0, and no sensor errors. This is the last documented pose, not a new zero. Small differences from immediately released endpoints can occur during sleep.

## Recording integrity and analysis corrections

Raw recordings are in [recordings/](recordings/), ignored by Git. The [recording manifest](diagnostics/recording-manifest-2026-10-11.json) indexes all JSONL files present at this documentation snapshot, including hashes, sizes, metadata, command requests, and sample counts. Preserve the raw files separately when backing up or sharing the experiment: a hash cannot recover a missing recording.

Version-1 events use relative Mac monotonic seconds and MCU milliseconds. Arrays use motor order; angle/reference/target are millidegrees. Requested commands are separate from accepted telemetry, so a command alone is not proof that the motor ran. Counts indicate issued STEP edges. Manual passive feedback recordings are not interchangeable with authoritative counted-jog telemetry.

New Python recordings also include UTC `started_at` and a UUID `session_id`. Older files lack those fields. The [motion-history audit](diagnostics/motion-history-audit-2026-10-11.json) used recorded accepted motion and MCU times, with filesystem creation timestamps helping across boot epochs; this does not prove uninterrupted hardware history. Manual intervention or an unrecorded session can invalidate an assumed reversal history.

A filename-based assumption incorrectly labeled `calquiet-shoulder-forward-01` as a reversal. It followed an earlier forward shoulder move and was a **continuation**. The audit records the correction. Frozen v1 remains unchanged; its shoulder reversal coefficient was zero, so correcting that label does not change those stored numerical predictions. It can matter to any future refit. Head training/held-out history labels remained supported. No inconsistent or failed trial was deleted to manufacture acceptance.

## Scripting and reproducibility

The full API and command syntax are in [scripts/SCRIPTING.md](scripts/SCRIPTING.md). Disconnect the GUI's motion controls before taking an exclusive scripted session. Do not run multiple motion scripts concurrently.

```sh
# Passive status; no motion and no ownership claim.
python3 scripts/armctl.py status

# Explicit global stop and sleep lock.
python3 scripts/armctl.py stop

# Offline analysis of an existing recording; never replays commands.
python3 scripts/analyze-recording.py recordings/local-target-head-02.jsonl

# Offline preview only: no connection or motor movement without --execute.
python3 scripts/validate-local-pose.py \
  --profile calibration/local-endpoint-profile-v1.json \
  --joint head --target 1.5 \
  --output recordings/new-local-head-check.jsonl
```

Adding `--execute` to the last example requests physical movement and requires a fresh start within the selected local bounds, a 1.5–5° displacement, a new output filename, healthy feedback, and enabled idle motion. Targets stay at least 2° inside hard limits. The helper bounds the move to 6 s, preserves existing guards, checks the refreshed/powered start and powered/released endpoint scope, requires real pulses/progress and “travel reached,” and stops on failure without automatic retry. Host-side scope checks are not an atomic MCU interlock; hard joint limits remain MCU-enforced.

`armctl.py trial ... --powered-settle` records powered and released endpoints. General `jog`, `trial`, `enable`, and `pose` require `--execute`. The normal `pose` command and GUI still require verified constant-ratio data. The accepted local profile is intentionally a different schema and must not be fed into the legacy ratio loader or used to set global verification flags.

Useful offline tools:

| File | Purpose |
| --- | --- |
| `scripts/analyze-recording.py` | Acceptance, settlement, drift, interior regression, and candidate-trial checks |
| `scripts/evaluate-calibration-model.py` | Frozen head model and original model-data diagnostic |
| `scripts/fit-local-calibration-model.py` | Frozen shoulder/elbow training fit; existing frozen output is protected against overwrite |
| `scripts/audit-motion-history.py` | Recorded prior directions and ordering limitations |
| `scripts/evaluate-local-heldouts.py` | Independent strokes against unchanged frozen models |
| `scripts/build-local-acceptance-v2.py` | Retrospective acceptance artifact generation/validation; the runtime profile is a separately saved artifact |
| `scripts/install-gui-ratios.py` | Validated import for legacy verified constant ratios; not for the local profile |
| `scripts/verify-counted-jog.py` | Asleep STEP pad diagnostic; locks movement and does not automatically re-enable it |

Do not overwrite frozen reports when changing a criterion or model. Write a new version, retain all source paths/hashes, and reserve new moves for independent validation.

## Implementation map and protocol

| Area | Main files |
| --- | --- |
| Manual control, PWM/ramp state | `sketch/motion.hpp`, `sketch/sketch.ino`, `python/gateway.py` |
| Sensor decode and directional limits | `sketch/sensors.hpp`, `sketch/limits.hpp` |
| Shared sleep and power generation | `sketch/power.hpp` |
| Counted timer edges and ISR accounting | `sketch/counted_jog.hpp` |
| Group state machine and slow pose planner | `sketch/trajectory.hpp`, `sketch/arm_control.hpp` |
| Group RPC/text/JSON-lines transport | `python/arm_gateway.py`, `python/script_gateway.py` |
| Mac-side bounded scripting | `python/arm_client.py`, `scripts/armctl.py` |
| Local profile target validation | `scripts/validate-local-pose.py` |
| GUI hold/ramp/transport | `mac/Controller.swift`, `mac/SSHLink.swift`, `mac/StepperDesk.swift` |
| Arm visualization/calibration | `mac/ArmModel.swift`, `mac/ArmView.swift`, `mac/ArmCAD.swift` |
| GUI poses/recording/import | `mac/PoseController.swift`, `mac/PoseView.swift` |
| Tests | `tests/`, native `mac/*Tests.swift`, `scripts/test.sh` |

`arm_command` operations: 0 claim; 1 configure; 2 pose; 3 jog; 4 status; 5 hold; 6 stop; 7 release; 8 power; 9 bounded powered pause. `arm_observe` is passive; `arm_lock` globally stops/locks without ownership. The legacy per-motor `stepper_command` path remains separate.

The version-1 arm snapshot is 38 integers: 14 header values `(version, error, owner, state, motion_id, MCU_ms, run, awake, enabled, power_generation, configured_mask, verified_mask, duration, elapsed)`, followed by eight values per physical motor `(angle, reference, target, count, direction, sensor_age, magnet_status, sensor_error)`. Zero-based directions are 0 forward, 1 reverse.

| State | Meaning |
| --- | --- |
| 0 / IDLE | Idle |
| 1 / PREPARING | Direction/wake preparation, no STEP |
| 2 / MOVING | Coordinated pose motion |
| 3 / MEASURING | Counted jog |
| 4 / COMPLETE | Pose completed |
| 5 / RELEASED | Stopped/released |
| 6 / LEASE_LOST | HOLD expired |
| 7 / SENSOR_FAULT | Feedback invalid/stale |
| 8 / LIMIT | Directional joint limit |
| 9 / TRACKING | Pose tracking/final endpoint failure |
| 10 / TIMING | Timing/service fault |
| 11 / TRAVEL | Bounded jog travel/count/time completion |
| 12 / SETTLING | STEP stopped, temporarily awake under guards |

In state 12, `run=1` means guarded powered settlement, not ongoing STEP output. Repeating PAUSE cannot extend the fixed 500 ms deadline or restart movement.

## Verification record

The latest complete Python discovery run passed **101 tests**. During this commissioning run, `scripts/test.sh` also passed portable motion, sensor, limit, power, coordinated motion, counted-timer, native UI, CAD mapping, and pose/recording tests; the Mac app rebuilt successfully. New local-profile/target tests specifically cover fingerprint and provenance mismatch, local bounds, invalid targets, movement/result checks, endpoint scope, preview without connecting, and failed-context cleanup. Unit tests do not replace physical measurements.

Live evidence includes verified firmware programming, static STEP pad masks, asleep counted-edge agreement, actual movement/direction checks, sensor quality and settlement checks, and the six local target tests above. No loaded coordinated path, full-range sweep, deliberate stall recovery, independent optical angle measurement, torque measurement, thermal measurement, or vision-load benchmark has been completed in this record.

## Next development stages

1. Extend commissioning beyond the narrow local regions with bounded, recorded moves and separate validation trials. Keep accepted error, load, direction history, sleep behavior, and microstep configuration explicit.
2. Use a representation that handles position dependence and reversal loss, or a bounded feedback correction, instead of forcing all data into one gear-ratio number. Validate the chosen mechanism on actual target moves and coordinated paths. The current local helper is a single-joint test tool.
3. Add supervised stall handling around the conventional planner: detect persistent tracking loss, stop and remeasure, then permit a bounded user-held retry. Limit reversal, displacement, elapsed time, and retries. No repeated automatic pushing into an obstruction.
4. Gather complete-session data across operating positions and payloads. Split training/evaluation by session, pose region, and load rather than neighboring samples. Preserve failed trials and prediction uncertainty.
5. Start learned correction with near-term tracking-error/stall-risk prediction and a bounded shared speed reduction. Keep limits, sensor validity, timing, sleep, ownership, and pulse generation authoritative on the MCU.
6. Benchmark Qualcomm CPU/memory and transport timing alongside the intended vision models before choosing inference rate and model size. Train initially on the Mac; run shadow inference before giving predictions control authority.
7. Put emotive behavior above motion as attention/gesture intent and approved pose sequences. Collision/workspace checks, camera-to-arm calibration, gesture coverage, and motion reliability remain prerequisites for autonomous expressive movement.

Reference inputs mentioned during planning include `/Users/tongtong/Downloads/2501.12493v1.pdf` and whiteboard image `/Users/tongtong/Downloads/IMG_7387.HEIC`. Their contents are not claimed as experimentally verified findings in this record. CAD visualization source is recorded in [Resources/ArmCAD/SOURCE.txt](mac/Resources/ArmCAD/SOURCE.txt).
