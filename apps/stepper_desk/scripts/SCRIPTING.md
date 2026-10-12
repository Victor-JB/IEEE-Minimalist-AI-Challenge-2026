# Scripted arm commissioning

The scripting interface collects motion data without operating the Mac GUI or using GUI automation tools. The Mac side uses Python’s standard library, ADB, and SSH. It uses the same STM32 controller, angle calibration, joint limits, sleep control, and motion watchdog as the pose workspace. This is commissioning for a conventional planner; a learned correction model has not been trained.

## Connection and ownership

The Mac connects to the Arduino Q over SSH. With USB forwarding configured, the board is `arduino@localhost` on port `2222`:

```sh
adb forward tcp:2222 tcp:22
ssh -p 2222 arduino@localhost
```

On the board, `script_gateway` connects to the Arduino router socket. The router owns the UART to the STM32. Scripts must not open `/dev/ttyHS1` directly. There is no HTTP control service.

The `status` command and Python `observe()` helper read telemetry without claiming or renewing control. `ArmSession` methods, including `record()`, require an exclusive session. Disconnect the Mac app’s motor or pose controls before opening one; a script cannot steal an occupied session.

The CLI `stop` command is a deliberate exception: it globally stops all motors and locks the drivers asleep without claiming ownership. It also stops a move started by the GUI. `enable --execute` requires exclusive ownership, clears the lock, and leaves idle drivers asleep. It does not restart a cancelled move.

## Joint names, directions, and calibration

The public interface uses **shoulder, elbow, head** order. Internal recording arrays retain physical motor order for compatibility with the Mac app.

| Joint | Physical motor | Sensor channel | Forward changes calibrated angle | Limits |
|---|---|---|---|---|
| Shoulder | Motor 2 | CH0 | Increases | −45° to +38° |
| Elbow | Motor 1 | CH1 | Decreases | −72° to +79° |
| Head | Motor 3 | CH2 | Decreases | −80° to +87° |

`--calibration` selects a saved angle-calibration file. The default is the repository snapshot at `calibration/arm-calibration-2026-10-11.json`. That file contains sensor offsets and configured motor settings. It does **not** establish the physical gearbox reductions or pulses per joint degree.

A separate ratios file is optional for observation and ratio trials, but required for coordinated poses. Ratios must come from at least three eligible trials per joint, including both directions, with no more than 5% spread. Each trial needs at least 50 issued pulses, at least 5° of measured travel, valid feedback, and a settled endpoint. Trials must match the active calibration and physical microstep settings. Changing either invalidates the old conversion.

## Python interface

Import `ArmSession` from `python/arm_client.py`; put the project’s `python/` directory on `PYTHONPATH`. Its constructor accepts `calibration=`, `ratios=`, and `output=` paths. Defaults use the saved calibration, no ratios, and a new recording file. Use it as a context manager so an owned session is stopped and released on normal completion or an exception.

| Method | Purpose |
|---|---|
| `status()` | Read feedback within an owned session; use `observe()` for a passive read. |
| `stop()` | Stop the owned session’s current motion without applying the global sleep lock. |
| `record(seconds)` | Collect telemetry for a bounded interval without requesting motion. |
| `jog(joint, direction, seconds, max_degrees, rate_hz=100)` | Run one joint for a bounded time and maximum measured travel. |
| `trial(joint, direction, degrees, timeout, rate_hz=100, powered_settle=False)` | Collect one bounded pulse-to-angle trial. |
| `move_pose([shoulder, elbow, head], timeout)` | Execute a small coordinated move using verified ratios. |
| `power(enabled)` | Enable movement or stop and sleep-lock the drivers. Enabling while idle does not wake them. |

Motion methods renew the controller's hold lease only while that bounded method is active. They stop on completion, timeout, invalid feedback, a guard fault, cancellation, or an exception. A context manager does not provide an unlimited background hold. `Ctrl-C` runs cleanup to send STOP and release ownership; the firmware's expiring lease also stops motion if communication disappears.

Low-level `_request()` is private implementation detail. Public methods enforce the bounds and recording needed for commissioning. `jog()` defaults to 0.5 seconds and a 2° travel cap; `trial()` defaults to 6° and a 30-second timeout. Both start at **100 pulses per second** by default; `rate_hz` in Python or `--rate-hz` on the CLI selects an integer from 20 to 2400 with the hardware-counted jog firmware. Timer periods are quantized, and interrupt latency can reduce achieved rate; recorded edge counts remain authoritative. Jogs and trials are limited to 60 seconds and 10° by the client. The client stops for an opposite-direction change over 0.3°, another joint moving over 0.75°, or no measurable progress for 2 seconds. These measured-response checks complement the firmware guards; they are not a trained stall classifier.

Python example, run from `apps/stepper_desk`, records without asking for motion:

```sh
PYTHONPATH=python python3 - <<'PYTHON'
from arm_client import ArmSession, observe

print(observe())  # Passive: does not claim or renew a session.
with ArmSession(output="recordings/idle-python.jsonl") as arm:
    arm.record(5)  # Requires exclusive ownership, but sends no motion start.
PYTHON
```

Every output path must be new; existing files are never replaced.

## Closed-loop motion

`python/arm_motion.py` is the interface intended for behavior code. The STM32 closes a position loop on the AS5600 angles (`sketch/servo.hpp`). Each millisecond it sets every joint's step rate from the trajectory velocity plus a correction proportional to the measured error, so a wrong pulses/degree value, reversal backlash or a lost step costs time instead of endpoint accuracy. Pulses/degree is a feedforward hint only and is refined online per direction. The legacy POSE/JOG paths and their verified-ratio gate are unchanged and cannot run while a closed-loop move is active.

```python
from arm_motion import ArmMotion
with ArmMotion() as arm:                       # joint order: shoulder, elbow, head
    r = arm.move_to([-5, 60, 10], speed=8)     # or duration=1.5; returns once holding
    arm.stream_point([-4, 61, 12], [1, 1, 2], horizon=0.1)  # behavior model, >= 4 Hz
    arm.scale(0.5)                             # live shared slowdown of the running move
    arm.halt(); arm.relax()                    # hold where it is / let drivers sleep
```

- Moves are synchronized minimum-jerk profiles: all joints arrive together. The default limits are 12°/s and 40°/s² per joint (`configure()`); a requested duration that is too short is stretched, never sped up. The closed-loop path caps STEP at 12,000 pulses/s (about 84°/s at about 142 pulses/degree); `configure()` accepts up to 60°/s and 500°/s². The counted jog, pose and manual paths keep their 2400 pulses/s cap.
- Targets must be at least 2° inside the hard limits. The MCU keeps the limit guard, fresh-sensor requirement, ownership and the 300 ms HOLD lease. If the lease lapses mid-move, the arm stops and holds where it is.
- After a move the drivers stay powered and the loop holds the pose (no sag) until `relax()`, STOP, the hold timeout (default 60 s), or the session ends. SLEEP is shared, so holding keeps all three drivers powered. Driver heating while holding is unmeasured.
- **Stalls:** a joint that is commanded more than 80 pulses in a window but moves less than 30% of the expected angle for two consecutive 150 ms windows (after 400 pulses since its last reversal, to allow for backlash), or whose following error is over 5° for 300 ms, has stalled. During `move_to` the firmware stops all joints, holds, backs the stalled joint off 1°, and retries at half speed. For the shoulder it first folds the elbow to a limit (`tuck='auto'` folds it opposite the shoulder's lean; pass an elbow angle once the right side is confirmed, or `tuck=False`), lifts the shoulder, then unfolds. It retries at most twice; a second stall within 2° of the first is reported as `obstruction` and the arm holds instead of pushing again. During streaming a stall stops and reports without retrying.
- `move_to` returns a dict with `ok`, `state`, `reason`, `final_angles`, `error_deg`, `retries`, `stalls`, `stalled_joint` and `stall_angle`. Faults (sensor, limit, timing) raise `ArmError` and leave the drivers asleep.
- Every command, 25 ms status sample and 100 ms loop telemetry record (`SERVOSTAT`: reference, estimate, commanded velocity, pulse rate, learned pulses/degree) goes into the session JSONL as training data for a later learned correction layer.

Command line: `python3 scripts/armctl.py move --angles S E H [--speed V | --duration T] [--hold SECONDS] [--tuck auto|off|ANGLE] --execute`.

Validated on the arm on 2026-10-11 (bench, speed ladder to 60 °/s, full-range gravity test, streamed emotes): results, the shoulder's stuck zones, the −34° shoulder floor and the recovery procedure are in [CLOSED_LOOP.md](../CLOSED_LOOP.md). The shoulder `tuck='auto'` fold did not free a stuck shoulder there; prefer `tuck=False` and the manual fold-then-fast-lift recovery described in that file.

Emotes: `python3 scripts/emote.py CLIP [CLIP...] [--amplitude A] [--home S E H] --execute` streams `motion_lib` clips (preview without `--execute`). It saves command and measured traces for `sim/compare_real.py`.

Gear-slack compensation: `Settings::slackDeg` (motor order elbow, shoulder, head) issues an uncounted burst of steps after each direction reversal. It defaults to 0 on all joints; see CLOSED_LOOP.md before enabling it.

Wire commands (motor order elbow, shoulder, head; millidegrees): `MOVE id e s h duration_ms speed_mdeg_s generation`, `STREAM id e s h ve vs vh horizon_ms generation`, `HALT`, `SERVO axis ppd_milli vmax_mdeg_s amax_mdeg_s2`, `SERVOTIME hold_ms tuck_mode(0 off,1 auto,2 fixed) tuck_elbow_mdeg`, `SCALE milli`, `SERVOSTAT`. Arm snapshot states 13-18 are moving, holding, recovering, stalled, streaming and fault.

## Counted pulses and powered measurement

Commissioning jogs use one-pulse PWM2 on each joint's dedicated STM32 timer. Hardware emits one STEP rising edge, returns LOW, and stops the timer. The update interrupt counts that pulse and may start the next one. A delayed interrupt slows the rate rather than allowing uncounted pulses. STOP also accounts for an already-emitted edge whose interrupt has not run, forces STEP LOW, and prevents restart. The firmware caps a jog at 3600 pulses, 12° measured travel, or 60 seconds; the client's stricter travel bounds still apply. The conventional pose scheduler remains the separate, slower counted backend.

The asleep diagnostic independently samples the STEP input pads; it does not infer edges from elapsed time. On 2026-10-11, all three pads matched 100 issued pulses to 100 observed rising edges at each of 100, 1333, and 2400 pulses/sec. Results are in `diagnostics/counted-pad-verification-2026-10-11.json`. This verifies output counting with SLEEP LOW; it does not verify motor steps, gearbox conversion, torque, or pulse widths with an oscilloscope.

For a ratio trial, `powered_settle=True` (CLI `--powered-settle`) separates wake alignment, driven travel, and movement after sleep:

1. The driver wakes with a 400 ms guarded preparation period and no STEP pulses. The client requires fresh, stable zero-count readings from the latter part of this period for the powered starting angle.
2. The bounded jog runs at the requested pulse rate.
3. `PAUSE` stops STEP immediately and preserves the count while keeping the drivers awake. Firmware state **12 (`SETTLING`)** lasts at most 500 ms from the first PAUSE. The client samples for about 350 ms and requires a stable powered endpoint with an unchanged count.
4. STOP puts the drivers to sleep, and the client separately records the released angle. Results include `powered_initial_angles`, `powered_final_angles`, and `sleep_delta_deg` in shoulder/elbow/head order.

Preparation and powered settling both require the normal 300 ms HOLD lease. Sensors and control timing remain guarded. Repeated PAUSE cannot extend its deadline or resume motion; STOP, lock, or a fault release power. State 12 reports `run=1` to retain guarded coil power, even though no steps are emitted. Ordinary jogs retain the 80 ms preparation and automatic sleep on stop. Powered measurement is an explicit calibration option, not continuous idle holding.

Powered trials estimate pulses per degree between the stable powered endpoints. Released-position drift is reported separately and must still be considered when assessing pose accuracy with automatic sleep. These endpoints do not remove backlash, missed steps, sensor mounting errors, or position-dependent transmission effects.

### Low-level commissioning protocol

Public methods are preferred. The optional text form is `JOG id axis direction rate_hz prepare_ms generation`; the corresponding Bridge payload is `[3, owner, id, axis, direction, rate_hz, prepare_ms, generation]`. `prepare_ms` accepts 80–500 only for a hardware jog. Omitting it retains 80 ms; the older form without a rate retains the default rate. `PAUSE id` maps to `[9, owner, id]` and is accepted only for that active counted jog. Internal axis order is physical motor order.

For a firmware change, `arm_pulse_selftest([owner, rate_hz, pulses])` requires exclusive ownership, movement locked off, SLEEP physically LOW, and no running motion. It returns `[1, error, rate_hz, axis0_issued, axis0_observed, axis0_error, ...]` for all three axes. It accepts 20–2400 pulses/sec and 1–200 pulses. Any count disagreement must be investigated before moving the arm.

## Command-line interface

Run these commands from `apps/stepper_desk`. No app window needs to be open.

```sh
# Passive observation; safe to use while another session owns the arm.
python3 scripts/armctl.py status

# Requires the app to disconnect; records without asking for motion.
python3 scripts/armctl.py record --seconds 5 --output recordings/idle.jsonl

# Global stop and sleep lock, even if a different session owns motion.
python3 scripts/armctl.py stop

# Exclusive session required; clears the lock but leaves the drivers asleep.
python3 scripts/armctl.py enable --execute
```

The following commands **move the arm**. They are examples for deliberate commissioning after the motor-power and response check, not a batch to run unattended:

```sh
# Short elbow-forward probe: forward reduces the elbow angle.
python3 scripts/armctl.py jog elbow --direction forward --seconds 0.5 --max-degrees 2 --rate-hz 100 --output recordings/elbow-probe.jsonl --execute

# One 6° powered-endpoint measurement; repeats must use new filenames.
python3 scripts/armctl.py trial elbow --direction forward --degrees 6 --timeout 30 --rate-hz 1333 --powered-settle --output recordings/elbow-forward-1.jsonl --execute
```

`jog`, `trial`, `pose`, and `enable` reject the invocation unless `--execute` is present. Omitting it does not provide a simulated move. Get all supported options with, for example, `python3 scripts/armctl.py trial --help`.

After collecting eligible trials for **all three joints**, build a ratio file offline:

```sh
python3 scripts/armctl.py verify-ratios recordings/trial-*.jsonl --output recordings/verified-ratios.json
```

Use the same `--calibration path/to/calibration.json` on collection, verification, and motion commands when replacing the default calibration. Verification checks recording fingerprints, acceptance, endpoints, direction, pulse counts, travel, and trial repeatability before writing a new file. It preserves trial evidence and source paths. Duplicate owner/move IDs are counted only once. The loader recomputes pulses per degree from the stored trials and ignores any supplied `pulses_per_degree` summary; editing that number cannot change the conversion. Keep original recordings so the trial evidence remains auditable.

A pose command accepts **absolute** shoulder, elbow, and head angles in degrees. It requires all three verified conversions, a start inside the limits, targets at least 2° inside the limits, and no more than 5° change per joint. Substitute a measured nearby target for the placeholders below; this template is not directly runnable:

```text
python3 scripts/armctl.py pose --angles SHOULDER ELBOW HEAD --ratios recordings/verified-ratios.json --timeout 120 --output recordings/pose-1.jsonl --execute
```

## Local single-joint target checks

`scripts/validate-local-pose.py` checks one absolute sensor-angle target against the accepted local endpoint profile. It uses **sensor feedback to stop a bounded jog**; the model's predicted pulse counts are reported only and never command the move. Passing this check is not feedforward-model verification, coordinated-planner verification, global ratio calibration, or independent physical-angle metrology.

```sh
# Offline preview: validates the profile and target, without connecting.
python3 scripts/validate-local-pose.py --profile calibration/local-endpoint-profile-v1.json --joint shoulder --target -9.5 --output recordings/local-shoulder-check.jsonl

# Append --execute only when deliberately running that one bounded check.
```

Execution requires the profile's calibration fingerprint and evidence hashes to match. The selected joint must start within its local bounds, the requested displacement must be 1.5–5°, and the target must also lie at least 2° inside the hard limits. The helper uses existing guarded powered-settle jogging, a six-second timeout, and 1333 pulses/sec for shoulder/elbow or 1000 for head. It does not retry automatically or load global ratios.

A pass requires reaching the requested travel, positive issued pulses, measurable progress in the requested direction, and both powered and released sensor endpoints within the profile's accepted tolerance (at most 1.5°). The jog's refreshed starting angle and powered baseline must still satisfy the local bounds and displacement range; both endpoints must remain within the local bounds. Other-joint drift is rejected. The refreshed-start checks are postconditions: the local envelope is not an atomic firmware interlock, while existing firmware joint limits remain active throughout. Output includes separate powered/released target errors and a model count range with reversal history marked unknown.

## Recording and analysis

Script sessions write JSONL using the Mac app's recording schema, under the ignored `recordings/` directory by default. Keep the metadata line: it identifies calibration and conversion assumptions. Commands are logged separately from telemetry because a requested move may be rejected. A valid angle reading and issued STEP count do not measure torque or prove every pulse produced motion.

New Python recordings retain schema version 1 and add `started_at` (an absolute UTC timestamp ending in `Z`) and a unique `session_id` UUID to the metadata line. Event `time` remains elapsed monotonic seconds from session start, including when a test injects its own clock; it is not replaced by wall-clock time. These fields help identify files and order sessions, while older recordings without them remain readable. A timestamp or UUID does not prove an uninterrupted hardware history: record intervening manual movement, power cycles, driver-setting changes, and sessions from other controllers separately when reconstructing direction history.

Analyze a specific file without opening a board connection:

```sh
python3 scripts/analyze-recording.py recordings/session.jsonl
```

For powered trials, the analyzer validates the zero-count powered baseline and the paused endpoint, requires an unchanged pulse count and clean release, and reports released-position drift separately. It labels these measurements `powered_settle`; ordinary trials are `sleep_settle`.

The analyzer reports each JOG or POSE request and whether telemetry demonstrates acceptance. Accepted moves include initial and settled final angles, signed travel, pulse counts, duration, sensor freshness, untouched-joint drift, and stop/fault reason. With enough distinct samples and travel, it also fits interior angle versus issued pulses to help separate endpoint effects from the moving section.

The analyzer never executes recorded commands or saves calibration. `consistency_criteria_met` indicates that the available trials pass the numeric repeatability checks; the analyzer’s `verified` field remains false. The separate `verify-ratios` command validates the evidence and writes a conversion file for the client. Inspect drift, faults, direction, and load conditions before accepting measurements. Visual replay in the Mac app is also read-only.

Record the payload, arm support, initial pose, microstep settings, driver current setting, and any observed slip alongside each session. Current is adjusted physically on these DRV8825 boards; software cannot set or measure torque. Compare whole sessions rather than treating nearby samples as independent experiments.

## Current commissioning evidence

Calibration is **not complete**. There is no accepted all-joint ratio set, and coordinated pose validation remains gated on verified conversions. Direction and position dependence, wake alignment, and released-position drift are under investigation. The saved sensor offsets are distinct from the still-unverified physical motor-to-joint conversions.

Early 60/100-pulse-per-second elbow trials produced little or no resolved motion. Later manual PWM tests at the configured half-step 200 RPM (about 1333 pulses/sec) produced clear elbow movement in both directions. The new counted backend passed the asleep pad tests above, and bounded physical probes moved all three joints. These checks establish usable motion and reliable issued-edge accounting; they do not establish a repeatable gearbox ratio.

Powered settling was added after observing movement during wake with zero STEP pulses and drift after releasing coil current. Use the raw recordings and analyzer results to distinguish those effects from driven motion. Do not loosen repeatability thresholds or mark conversions verified merely because the arm moves. No learned correction model has been trained.
