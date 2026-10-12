# Lumen: current work (2026-10-12)

Status notes to sync the team. Repo: `IEEE-Minimalist-AI-Challenge-2026`.

## TL;DR

- The "FAULT" the lamp kept hitting was the **arm's head joint stalling** while it tracked a
  face, not the camera.
- Two fixes are committed on local `main` (**not pushed yet**): Python-side stall recovery and
  firmware-side "grind through" recovery.
- The firmware change is **not compiled, tested or flashed yet**.
- The **head-tilt servo (HPS-2027, J5 `head_roll`)** is written: firmware, Python and Lumen
  (servo on D2, elbow DIR moved to D6). It's **flashed and working on the arm**, but not
  committed yet (see section 4).

## Where the code is

| Location | Branch | Commit | Notes |
|---|---|---|---|
| `~/IEEE-tongtong-worktree` | `main` | `b8e1407` | **Work here.** 2 commits ahead of `origin/main`, not pushed |
| `~/IEEE-Minimalist-AI-Challenge-2026` | `lumen-graceful-stalls` | `b8e1407` | Same commits; that branch is merged into `main` |

Commits on top of `origin/main` (`8e10ad0`):

1. `bd93b4e` **Lumen: survive arm stalls and faults, center the face first**
2. `b8e1407` **Arm firmware: grind through streaming stalls on the spot**

To publish: `git push origin main` from the worktree (not done yet).

## 1. Diagnosis: why it kept faulting

Sources: `apps/lumen/logs/transitions-20261012.csv` and `apps/lumen/logs/arm-*.jsonl`.

- Both faults at 04:03 happened in **STANDBY**, 2–15 s after a face was confirmed, i.e. while the
  look-at logic was steering the head.
- In each of those sessions the firmware reported the head stalling 10 times
  (`stalled_joint: head`, at about +11 to +14°) before the session shut down and Lumen went to FAULT.
- Typical stall: the head was at 13.97° and told to go to 12.04°. That ~2° reversal is the same size
  as the gear backlash (~2–2.5°), and the head has a known tight spot near +14° (noted in `servo.hpp`).
- Look-at sends small, slow, back-and-forth corrections. The firmware's stall detector reads the
  backlash take-up as "joint isn't following". The old streaming behavior was to stop and report,
  and enough stops ended in FAULT. Hold drift was already disabled for the same reason
  (`config.py`, `HOLD_DRIFT_GAIN`).
- The 04:17 run used the new Python worker (commit 1) and had **124 stall reports in about 8 min
  with no FAULT**. The recovery works, but the arm still stalls constantly. Commit 2 targets that.

## 2. Commit `bd93b4e`: Lumen stall and fault handling (Python)

16 files under `apps/lumen` and `arm/python`.

- A stalled STREAM is no longer fatal. The gateway returns it (`state: stalled`) instead of ending
  the session and putting the drivers to sleep (`script_gateway.py`, `arm_motion.stream_point`).
- **Motion worker** (`apps/lumen/lumen/motion/worker.py`) recovers in place:
  - Sends HALT, holds powered, then pushes again more slowly (10, 6, then 4 deg/s).
  - A clip resumes from where it stalled and plays the rest more slowly.
  - FAULT only if the arm is jammed: 6 abandoned commands within 60 s.
- The worker thread survives errors, logs the traceback, and reconnects when BOOT asks. Before
  this, the thread died and the app was stuck in BOOT forever.
- A BOOT timeout (20 s) now leads to FAULT, and the fault reason is printed. FAULT auto-retries
  once after 10 s. Faults more than 2 min apart don't latch.
- **Look-at** (`behavior/look_at.py`) integrates the error and centers the face vertically before
  STANDBY clips start. The offset is kept across NOTICE, STANDBY and LOST, slewed, and applied
  during clips.
- Hold drift was rewritten as smooth waypoints and is disabled for now because it tripped stalls.
- New tests: `tests/test_look_at.py`, `tests/test_worker.py`. Python tests pass.

## 3. Commit `b8e1407`: firmware grinds through stalls (C++)

Files: `arm/sketch/servo.hpp`, `arm/python/arm_motion.py` (docstring), `arm/tests/servo_test.cpp`.

- Before: a stall while streaming made the joint stop, report STALLED, and wait for the host to HALT.
- Now the stalled joint **stays powered and keeps pushing toward the streamed target at a low step
  rate**: 3 deg/s on the first grind, then 1.5, then 1. A stepper has the most torque at low speed,
  which helps it through tight spots and heavy poses. Each grind lasts 1.2 s.
- If the joint moves at least 0.5° during a grind (or catches up), the grind counter resets.
- If 3 grinds in a row make no progress, it's treated as a real obstruction: STALLED, as before,
  and the Python recovery above takes over.
- New settings in `servo::Settings`: `grindSpeed=3`, `grindProgressDeg=0.5`, `grindMs=1200`,
  `maxGrinds=3`.
- New simulator test `streamingGrindsThroughTightSpot`: a head tight spot at 11–15° that is only
  passable below about 4 deg/s. The existing "won't move at all" test now expects 3 grinds before
  STALLED.

**Not verified yet:**

- The firmware tests need a host `c++`, which the board doesn't have. Run them on the Mac:
  `bash arm/scripts/test.sh`
- Not compiled for the MCU and not flashed. Flash with `bash arm/scripts/deploy.sh` (from the Mac)
  after the tests pass.

## 4. Head-tilt servo (J5 `head_roll`): working on the arm, uncommitted

**Bring-up 2026-10-12:** the firmware is flashed from `/private/tmp/claude-501/-Users-tongtong-Nextcloud-Projects-IEEE-Minimalist-AI-Challenge-2026/5d8bd2b6-2f0a-4366-a83a-6cb32a8078d4/scratchpad/recover/home/IEEE-tongtong-worktree`, and the servo is
on its own 7.4 V supply. A ±20° wiggle and a return to center worked. It's a **270° servo**
(500–2500 µs) and **1500 µs = level shade**; both are set in `arm_config.py`.
Still to check: `ROLL_SIGN` (does +head_roll tilt the way the clips intend?) and how far
`ROLL_RANGE` (now ±45°) can widen.

The flashed firmware also contains the untested stall-grind change from section 3. Run
`bash arm/scripts/test.sh` on the Mac before relying on it.


**Wiring (done):** the servo signal is on **D2** and the elbow DIR moved **D2 → D6**. D2 is on
TIM2, the only timer that no STEP pin uses, so the servo gets a clean hardware 50 Hz signal.
(D10 and D11 share a timer with the head and shoulder STEP pins. The step code changes those
timers' frequency constantly, which would scramble a servo signal.)

**Power (still open):** don't use the board's 5 V. The HPS-2027 wants about 6–7.4 V and can pull
2 A or more, which can brown out and reset the board. Use a buck converter off the stepper supply
(6.0–7.4 V, at least 3 A), tie its ground to the board's GND, and put 470–1000 µF across the
servo near it.

What was added (in `/private/tmp/claude-501/-Users-tongtong-Nextcloud-Projects-IEEE-Minimalist-AI-Challenge-2026/5d8bd2b6-2f0a-4366-a83a-6cb32a8078d4/scratchpad/recover/home/IEEE-tongtong-worktree`, **not committed yet**):

- **Firmware**
  - `arm/sketch/roll.hpp` (new, portable): clamps the angle, slews the command, and maps
    degrees to a 500–2500 µs pulse (270° servo). Out-of-range angles are clamped instead of
    refused.
  - `sketch.ino`: drives the D2 output from the 1 kHz control loop and has `DIR_PINS = {6, 4, 7}`.
  - `arm_control.hpp`: new `arm_command` ops 16 `ROLL` (angle in mdeg, speed in mdeg/s),
    17 `ROLLCONFIG`, 18 `ROLLOFF` (limp), and a new `arm_roll_status` RPC.
  - The servo goes limp when the session is released, when the 2 s owner timeout hits, and on
    LOCK. `ROLL` is refused while movement is locked.
  - Compiles for the MCU with `arduino-cli` (18% of flash). Not flashed.
- **Python (`arm/python`)**
  - The gateway has `ROLL`, `ROLLCONFIG`, `ROLLOFF` and `ROLLSTAT`.
  - `ArmMotion(roll=...)` adds `configure_roll()`, `roll()`, `roll_off()` and `roll_status()`.
  - **On firmware without the servo, `roll()` is a no-op**, so an unflashed board still runs.
- **Config**
  - `arm_config.py` has `ROLL_ENABLED`, `ROLL_CENTER_US=1500`, `ROLL_SIGN=1`,
    `ROLL_RANGE=(-45, 45)`, `ROLL_SPEED_CAP=180`, `ROLL_HOME=0`. **None of these are calibrated
    yet.**
- **Clips**
  - `trajectory.trajectory_with_roll()` also returns the clip's `head_roll` track on the same time
    base. `trajectory()` is unchanged. 62 clips use `head_roll`; for example `curious_head_tilt`
    goes to about 19°.
- **Lumen**
  - `motion/worker.py` plays each clip's `head_roll` on the servo. Holds and moves ease it back to
    `ROLL_HOME` at 60°/s. Changes under 0.3° aren't sent.
- **Tests**
  - `arm/tests/roll_test.cpp` is new and added to `test.sh`.
  - New gateway, `ArmMotion` and worker roll tests.
  - Python suites pass: arm 91 tests, Lumen 25. The C++ tests pass a syntax check only; run them
    on the Mac.

### Bring-up checklist

1. Power the servo from its own supply (see above).
2. On the Mac: `bash arm/scripts/test.sh`, then `bash arm/scripts/deploy.sh` to flash.
3. Small angles first: `ArmMotion(roll=arm_config.ROLL)` then `arm.roll(10)` / `arm.roll(-10)`.
4. Set `ROLL_CENTER_US` so 0 is a level shade, and `ROLL_SIGN` so `curious_head_tilt` tilts the
   intended way. Widen `ROLL_RANGE` only as far as the shade and cable clear.
5. Run Lumen. The connect line says `(head tilt on)`.

## 5. Lumen Desk: browser control page (new, uncommitted)

`apps/lumen/lumen/desk/` replaces the Stepper Desk Mac app with a page served from the board, so
it works from any browser and needs no Swift build.

Run it with `cd apps/lumen && ../../.venv/bin/python -m lumen.desk`, then open
`http://daboard:8081/`. Stop Lumen first: only one program can own the arm and camera.

What's on the page:

- **Camera** from the head (MJPEG, about 15 fps).
- **Arm view:** a side view drawn from the measured angles, with the commanded pose dashed and
  HOME faint, plus a front view of the head tilt.
- **Joint cards:** angle, commanded value, safe and hard limits, magnet health, sensor age, and
  hold-to-jog −/+ buttons (closed loop; it stops 0.3 s after you let go).
- **Header:** Connect, Enable movement / Sleep & lock, Relax, Arm, and STOP (also on Escape).
  The page disarms when you switch tabs.
- **Move to pose:** joint and head-tilt targets, "use current" and HOME buttons, and a speed
  setting. Targets are clamped to the safe ranges.
- **Head tilt:** slider, presets, wiggle, a ±135° wide mode, and Release.
- **Clips:** every clip, with an amplitude setting, played around the current pose (head tilt
  included).
- **Taught poses:** saved to `arm/clips/taught_poses.json`, with Go, Load and Delete.
- **Feedback plot:** measured vs commanded over the last 30 s, plus tracking error.
- **Recordings:** every desk and Lumen session, with download and replay on the arm view.

Left out from the Mac app, because closed-loop firmware made them obsolete: open-loop RPM jogs,
startup ramp curves, microstepping and VREF calculators, and ratio-measurement trials.

**Tests:** `tests/test_desk.py` (15 tests, fake arm) passes. On the board, connect, telemetry,
camera (14.6 fps), a move to HOME and a head-tilt round trip all worked.



```sh
cd ~/IEEE-tongtong-worktree
python3 -m unittest discover -s apps/lumen/tests   # Lumen logic (no board needed)
bash arm/scripts/test.sh                           # firmware sim + arm API (needs c++, so run on the Mac)
bash apps/lumen/scripts/run.sh                     # run Lumen in the foreground on the board, HUD :8080
```
