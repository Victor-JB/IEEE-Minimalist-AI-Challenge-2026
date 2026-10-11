# Learned correction around the arm planner

## Current milestone

The physical system is a fixed-base three-joint arm. The first foundation is explicit commissioning, a conventional bounded planner, measured feedback, and reproducible recording. Shoulder uses Motor 2 / sensor CH0; elbow Motor 1 / CH1; head Motor 3 / CH2. All drivers sleep when idle through D8. Current/torque remains a manual driver setting.

Implemented: measured pulses-per-degree trials, repeatability acceptance, taught pose capture, shared minimum-jerk trajectories, all-axis aborts on tracking/sensor/timing/lease faults, command and feedback JSONL, and visual-only replay. Commissioning jogs are bounded and now support hardware-counted rates up to 2400 pulses/sec; the coordinated pose scheduler remains deliberately slow. The Mac supervises a held move; the STM32 owns output timing and guards. This is not yet autonomous stall recovery or a learned controller.

## Counted commissioning and measurement state

The jog backend uses one-pulse PWM2: the timer produces one STEP rising edge and stops at rollover; the interrupt counts it and conditionally starts another pulse. Delayed interrupts cannot hide additional edges. STOP reconciles pending or in-flight pulses and forces the output LOW. Limits, fresh sensor feedback, the 300 ms HOLD lease, 50 ms control-service guard, pulse budget, and automatic D8 sleep remain authoritative.

Independent asleep pad sampling matched 100 issued pulses to 100 observed edges on all three outputs at 100, 1333, and 2400 pulses/sec. See `diagnostics/counted-pad-verification-2026-10-11.json`. Bounded physical probes also moved all three joints. These results validate output counting and basic response, not motor step fidelity or a mechanical conversion.

Optional powered-endpoint calibration uses 400 ms of guarded zero-pulse preparation to measure wake alignment, followed by a bounded jog. PAUSE then stops pulses while preserving their count and enters state 12 (`SETTLING`) for at most 500 ms. The client samples a stable powered endpoint over about 350 ms, issues STOP, and records the released angle separately. The same HOLD lease and sensor/timing guards apply during preparation and settling; repeated PAUSE cannot extend the deadline or restart motion. Ordinary idle behavior still sleeps automatically.

No all-joint ratio set is verified yet. Direction/position effects, backlash, wake alignment, and sleep drift are still being measured. Powered endpoints help separate these effects; they do not justify accepting inconsistent conversions. Script options and protocol details are documented in `scripts/SCRIPTING.md`.

## Local commissioning result on 11 October 2026

The user accepted a 1.5° local endpoint tolerance after the frozen shoulder/elbow models missed the original 0.5° criterion. Six subsequent single-joint sensor-target tests passed, with a maximum powered/released error of 1.062°. The accepted profile is limited to measured local regions and does not unlock globally verified constant ratios or coordinated motion. Original failures, reversal-history corrections, and out-of-range trials remain recorded.

[COMMISSIONING_REPORT.md](COMMISSIONING_REPORT.md) consolidates the hardware, model parameters, all experiment stages, exact target results, scope limits, file map, and reproducible commands. The next baseline work is broader-range and coordinated validation, not another claim that motor response is still entirely unmeasured.

## Next: establish a trustworthy baseline

1. Verify all three conversions, backlash and direction under light load. Record both directions at several positions without deliberate hard stalls. Check sensor mounting, mechanical zero, idle drift and weak-field performance.
2. Record small coordinated moves with different safe displacements and payloads. Inspect achieved versus planned timing, pulse backlog, encoder lag, completion error and false stops. Use timestamps; repeated command responses are not independent sensor samples.
3. Measure Linux CPU use and serial latency with the intended vision models running. Set a motion inference budget from these measurements. Extend and validate the coordinated pose scheduler before raising its rates; the new hardware-counted high-rate backend currently serves commissioning jogs only. Retain lease, all-axis stop, output wake gating and fixed joint limits.
4. Add a supervised recovery state machine: coordinated slowdown on rising error; stop and remeasure on persistent failure; explicit user-held retry from the measured pose at a lower speed. Keep retry count, elapsed time, displacement and reversal bounded. No automatic repeated pushing into an obstruction.

## Learned residual, after the baseline works

Keep the conventional planner and MCU guards authoritative. A small Linux-side predictor sees recent measured/reference joint positions, velocities, issued pulses, tracking errors, direction, sensor validity/age and the upcoming trajectory. Begin by predicting near-future tracking error or stall risk, then selecting a bounded *shared speed reduction* that preserves coordination. Do not let the model command raw STEP, alter joint limits, bypass stale-sensor handling, or increase driver current.

Store motion/session identifiers, the verified calibration fingerprint, payload/test conditions, and successful versus interrupted outcomes with each dataset. Add higher-rate MCU timestamps/batched telemetry if the 10 Hz initial logs cannot resolve relevant dynamics. Split evaluation by whole session, pose region and payload rather than adjacent samples. Compare to the conventional planner on completion rate, max error, false stops, time and compute cost. Use shadow inference before allowing the predictor to affect motion; reject stale/nonfinite/out-of-range model outputs and fall back to the baseline.

No model is trained from synthetic placeholders. Offline training can happen on the Mac. The first deployed predictor should be small enough to coexist with vision on the Qualcomm CPU, with its rate/thread limits selected from actual benchmarks. STM32 continues motion and fault protection if Linux inference stalls.

## Emotive behavior above motion

Treat emotion/vision as intent: attention target, desired gesture, energy and timing. Resolve that intent into approved poses and trajectories, then pass it through the same planner and guard layer. Pose/style generation must obey workspace and mechanical restrictions. Begin with a small library of measured safe gestures; add learned style only after motion reliability is measured. Collision checks, camera/arm calibration and user-space boundaries remain separate work before autonomous large gestures.

## What still needs physical validation

Repeatable pulse-to-angle ratios, backlash under load, pulse-width/timing accuracy, sensor delay, following-error thresholds, coordinated stopping distance and passive load holding have not been established by unit tests or the asleep pad diagnostic. The pad test verifies edge-count agreement, not these mechanical and timing properties. The commissioning UI exists to gather that evidence without guessing the reductions. Unknown ratios keep pose moves locked, but recording and guided ratio jogging remain available.
