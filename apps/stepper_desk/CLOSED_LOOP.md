# Closed-loop motion: results and limits (2026-10-11)

The STM32 closes a position loop on the AS5600 angles (`sketch/servo.hpp`): each millisecond every joint's step rate is the trajectory velocity plus a correction proportional to the measured error, so ratio errors, gear slack and lost steps cost time rather than endpoint accuracy. Behavior code uses `python/arm_motion.py` (API in [scripts/SCRIPTING.md](scripts/SCRIPTING.md#closed-loop-motion)). This page records what the loop achieved on the real arm, the limits found, and how to recover when the arm gets stuck. Raw recordings are in the ignored `recordings/` folder under the names given below.

Joint order everywhere is shoulder, elbow, head. Start/home pose: shoulder −12.04°, elbow 63.46°, head 5.27°.

## Accuracy at speed

| Test | Result |
| --- | --- |
| Bench, one joint at a time: +3, back, −3, back, +5, back at 3, 8, 12 °/s, others holding (`closedloop-{head,elbow,shoulder}-01`) | 54/54 moves complete, 0 retries, 0 stalls. Worst endpoint error: head 0.175°, elbow 0.099°, shoulder 0.095°. Holding joints moved ≤ 0.09°. |
| Speed ladder, ±15° moves, 2 each way per level, speed/accel from (12 °/s, 40 °/s²) to (60 °/s, 500 °/s²) (`closedloop-limits-*-01`) | Every joint clean at every level: worst endpoint error 0.30°, no stalls or retries. 15° in 0.43–0.47 s. Peak commanded joint speed 67–73 °/s. The shoulder reaches the 12,000 pulses/s cap at 60 °/s and lags the planned path by up to 2.8° mid-move (stall threshold 5°). |

The closed-loop STEP cap is 12,000 pulses/s (about 84 °/s); jog, pose and manual paths keep 2,400 pulses/s. `configure()` accepts up to 60 °/s and 500 °/s². The real ceiling is above that and has not been measured.

Learned pulses/degree after the sweep (forward / reverse): shoulder 141.6 / 166.6, elbow 155.3 / 158.5, head 150.6 / 169.1. The direction asymmetry is gear slack and gravity; the loop absorbs it.

## Gravity and range: where the arm gets stuck

Full-range test, 25/50/75/100 % toward 2° inside each limit, 3 s holds (`closedloop-range-*`):

- **Head** holds anywhere in −78…+85° (holding error ≤ 0.08°, drift ≤ 0.09°). Lifting out of the bottom stalls near −72° above about 3 °/s.
- **Elbow** is clean over its whole range, −70…+77°.
- **Shoulder** lowers cleanly to −43°, but **cannot lift back** from below about −36° (3 or 8 °/s, any elbow fold: −40, +63, +70, +77°). Leaning forward, it also could not lift back from +21° (elbow 51°), while it lifts cleanly from −27° and +3°.

Consequences already in place:
- The shoulder's hard minimum is now **−34°** (`sketch/limits.hpp`, `servo.hpp`, `trajectory.hpp`, `python/arm_client.py`, `mac/ArmModel.swift`), so commanded targets stop at −32°. `calibration/joint-limits-2026-10-11.json` still records the original −45° request.
- Emote playback (`scripts/emote.py`) keeps the shoulder in −30…+10°.

With the CAD masses (see below), the forward stalls fit a lifting torque of about 1.27 N·m, while the shoulder still held 1.96 N·m: stepper running torque is well below holding torque. The back-side stall needed only 0.4–0.7 N·m (once gravity was even helping), and the head's −72° stall 0.13 N·m, so those are position-dependent friction near the ends of travel, probably mechanical (cable tension, a tight spot in a cycloid drive, rubbing).

**Head tight spot (found in the evening, unresolved):** moving toward positive angles the head now stalls at about +13.7 to +14.7° (obstruction even at 8 °/s with slack compensation off; 20 °/s stalls with it on). The afternoon sweep passed through the same angles at 60 °/s, so something changed mechanically. Check the head around +14° for a wire pulling, the shade rubbing the second link, or a loose horn/screw. Until then, avoid fast head moves through +13…+15°.

## Recovering a stuck arm

Slow lifts stall more. What worked, from shoulder +22° (forward lean):
1. Fold the elbow so the weight sits over or behind the shoulder pivot: elbow −50° for a forward lean (to its positive limit for a backward lean).
2. Lift the shoulder **fast**, about 45 °/s, to −12°.
3. Unfold the elbow (0°, then home at 30 °/s).

From the back side (below −36°) no motor fold worked; it took a manual recovery in the app. The −34° floor now prevents getting there. `scripts/emote.py` returns home automatically after a stall when the shoulder is still inside −30…+10°.

## Streaming and emotes

`scripts/emote.py` plays `motion_lib` clips: the sim's shoulder_pitch, elbow_pitch and wrist_pitch drive shoulder, elbow and head (same sign, as in the app's 3D view), streamed at 25 Hz with a 0.12 s horizon. Each clip is slowed until every joint is under its speed cap (default 40/45/32 °/s) and acceleration is under 300 °/s².

- 7 clips at amplitude 1 (`emote-05-compare-*`): 0 stalls, real RMS error 0.4–1.1° against the command (0.27–0.57° with the ~50 ms streaming delay removed); worst instantaneous 1.4–3.8°.
- 7 exaggerated clips at amplitude 1.6 around home −12 / 52 / 12 (`emote-06-big-*`): 0 stalls; ranges up to 17° / 33° / 35°. The head lags up to 5.6–6.4° in the big nod and bounce, at the edge of the firmware's 5° / 300 ms stall rule.
- Head reversals are limited by gear slack (median about 0.7–0.8° over 124 reversals; elbow 0.5°, shoulder 0.4°): after a reversal the head lags for about 0.3 s. A 40 °/s nod tripped the stall rule this way. Keep the head at ≤ 32 °/s for now.

Head slack compensation exists in the firmware (`Settings::slackDeg`, an uncounted burst of steps on each reversal; in the simulator it cuts a 30 °/s nod's lag from 0.76° to 0.49°) but ships **off** on all joints until the head tight spot is fixed and an A/B test on the arm shows it helps.

### Lamp clips, elbow exaggeration and the evening slowdown

`scripts/make-lamp-clips.py` ports the lamp show from `arm_mujoco/lamp_demo.py` (full-model-animations) to `clips/lamp_*__arm.json`; `emote.py` plays them by name (`lamp_wake/arm`). `--gains SH EL HD` scales each joint on top of `--amplitude` (e.g. `0.7 2 0.6` doubles the elbow). Shoulder lifting torque was checked in the sim first: `lamp_wake` needs 1.46-1.87 N·m to stand up from its crouch (the shoulder lifts about 1.27), so it is not played on the arm.

As the evening went on the elbow could no longer straighten (lift the forearm and head) quickly: emotes that ran at 45 °/s in the afternoon stalled at it, and at 25 °/s 12 of 14 exaggerated clips played (lamp yes, happy, curious, sleep; double_take, hesitate, nod, shrug, lean_in, sad_droop, startle, celebrate). Straightening above about 15 °/s still stalled, so `--elbow-lift` (default 14 °/s) caps it separately and the defaults are now 30/25/22 °/s. Finally the elbow would not lift from 63° at all, even at 8 °/s, while it had done so all day. Together with the head's worsening tight spot this points to heat (motors or DRV8825s after hours powered) or a mechanical/driver fault; let the arm cool and check driver current before tuning further.

## Real arm vs simulation

`sim/stepper_desk.toml` models this arm (CAD geometry and masses, fitted torque limits; see [sim/README.md](../../sim/README.md)). `sim/compare_real.py` replays the traces `emote.py` saves. The sim tracks the command within 0.07–0.18° RMS, the real arm within 0.4–1.7°: the sim has no streaming delay, gear slack or 0.088° sensor quantization yet.

## Connection

**On the board (recommended for emotes):** `scripts/deploy.sh` also installs `emote.py`, `armctl.py`, the calibration, the arm clips, and `motion_lib`'s player and clips in `/home/arduino/stepper-desk-q`, plus numpy in its venv. On the Arduino Q the client sees the router socket and starts the gateway as a local process (no SSH hop): STATUS round trips 16 ms median, 33 ms worst. Run it there:

```sh
ssh arduino@daboard
cd stepper-desk-q && .venv/bin/python scripts/emote.py lamp_happy/arm --execute
```

**From the Mac:** 
USB (ADB forward) when the board is plugged in, otherwise Tailscale to `daboard`; see [COMMISSIONING_REPORT.md](COMMISSIONING_REPORT.md#connection-and-deployment). STATUS round trips: USB about 18 ms, Tailscale about 30 ms with spikes to 120–250 ms (Tailscale requests get a 0.6 s deadline; the MCU's 300 ms motion lease is unchanged).
