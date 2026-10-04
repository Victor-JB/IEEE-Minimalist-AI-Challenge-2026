# Performance & energy

Judges want before/after numbers (accuracy, latency, power/energy) and an architecture whose
savings make intuitive sense when they watch the live demo. This folder holds the measuring
tools; log every result in [RESULTS.md](RESULTS.md).

## What to report

| Metric | How | Tool |
|---|---|---|
| Latency (p50 / p90 ms), per stage | timing around preprocess / invoke / postprocess | `bench_detector.py`, `perfstats` |
| Throughput (inferences/s, loop fps) | count over a fixed duration | both |
| Energy per inference (mJ) | `P_avg / inferences_per_s`; above idle: `(P_avg - P_idle) / inferences_per_s` | `--watts`, `--idle-watts` |
| Average system power in a realistic session (W) | power meter over a scripted 5-min demo | meter + `perfstats` |
| Accuracy | recall @ IoU 0.5 and false positives/frame on a fixed eval set | TODO: `eval_detector.py` |
| Model size, peak RAM, CPU %, temperature / clock | file size, `/proc` samples | both |

Report energy **above idle** as well as whole-board energy. The above-idle number is the
model's own cost. The whole-board number is what the battery actually sees.

## Measuring power on the UNO Q

- **Ground truth: an inline USB-C power meter** (e.g. FNIRSI FNB58, which logs to a PC) between
  the charger and the board/hub. It measures board + hub + camera. Say so, or measure the hub +
  camera alone once and subtract.
- **Idle baseline first:** board booted, camera plugged in, nothing running, average over 30 s.
- **Steady state:** run the benchmark for 30 s or more so the meter's average settles, then
  pass the reading in with `--watts`.
- **On-board sensors:** `perfstats` automatically reads any `/sys/class/hwmon/*/power*_input`.
  Check with `ls /sys/class/hwmon/*/`. The board probably exposes only temperatures, so treat
  the meter as the source of truth.
- **Long runs catch thermal throttling.** `perfstats` logs CPU clock and temperature, so you can
  see whether the latency drifts as the chip heats up.

## Tools

**1. Sandboxed benchmark.** Inference is the only thing running. Use it for clean,
comparable model numbers.
```
python perf/bench_detector.py --threads 1 2 4 --duration 30            # latency/throughput sweep
python perf/bench_detector.py --threads 4 --watts 3.4 --idle-watts 2.1  # + energy/inference
python perf/bench_detector.py --model path/to/other.tflite              # before/after a model change
python perf/bench_detector.py --camera 0                                # real frame instead of synthetic
```
It prints a markdown row; paste it into RESULTS.md.

**2. In-situ instrumentation (`perfstats.py`).** Numbers from the full running system. Add
`@timed("name")` to a function or `with timed("name"):` around a block, and `count("name")`
for events. It is **off unless `PERF=1`**, and `@timed` costs nothing when it's off, so leave
it in the code.
```
PERF=1 python inference/main.py --headless
PERF=1 PERF_WATTS=3.6 PERF_IDLE_WATTS=2.1 PERF_CSV=run.csv python inference/main.py --headless
```
A report prints on exit (Ctrl+C): per-timer calls/s, mean, p50/p90/p99, counts, CPU %, clock,
temperature, RAM, and approximate energy per call. `PERF_CSV` saves every sample for plots.
It is already wired into `detector.py` (preprocess / invoke / postprocess / total) and
`inference/main.py` (camera.read, display, frame and face counts).
For the App Lab app, copy `perfstats.py` next to the detector in `apps/expressive_robot/python/`.

## Optimization plan (in order)

0. **Baseline everything before changing anything.** Benchmark + in-situ + idle/active power
   + accuracy. Every later claim is "X vs this".
1. **Find where the time goes.** Look at the `perfstats` breakdown. Your board log shows face
   centers at x ≈ 700, so the camera is delivering frames wider than 640. Every frame is decoded
   at the higher resolution and then shrunk. Ask the camera for a native 640x480 mode (check the
   resolution `Camera` prints at startup).
2. **Runtime settings (cheap, quick numbers).**
   - Thread count 1/2/4: fewer threads are slower but can cost fewer mJ per inference.
   - CPU governor and maximum clock (`/sys/devices/system/cpu/cpufreq/policy0/`): compare
     finishing fast then idling against running slower at a lower clock, in J/inference.
3. **System level: the biggest energy wins and the easiest story for judges.**
   - **Sensor gating:** a cheap motion check (frame difference on an 80x60 downscale, <1 ms)
     decides whether the detector runs at all. Nothing moving → no inference.
   - **Detect, then track:** run full detection at 2–5 Hz and predict the face position in
     between (constant velocity or optical flow on a small region). The planner already smooths
     at 20 Hz, so the motion won't look any different.
   - **Behavior-linked compute:** tie the inference rate to the planner state (sleepy 1 Hz,
     curious 5 Hz, attentive 10 Hz). "The robot's attention is its compute budget" is easy for
     judges to grasp.
   - **Deep idle:** if nobody is seen for N minutes, lower the camera fps and the CPU clock.
     Wake on motion.
4. **Model level.**
   - **Quantization before/after:** the model is already int8 (w8a8). Benchmark the float32
     version of the same model (Qualcomm AI Hub publishes the variants) for a clean size /
     latency / energy / accuracy comparison.
   - **Input resolution (probably the biggest model-level win):** the model is fixed at 640x480,
     and its cost scales with pixel count. Re-export it at 320x240 from the PyTorch source
     (qai-hub-models) and re-quantize: about 4x fewer operations. Faces 0.5–2 m from the robot
     are large, so accuracy should hold. Verify that it does.
   - **Accelerators:** try the TFLite GPU delegate on the Adreno GPU, and check whether a
     Qualcomm QNN delegate supports the QRB2210. Compare mJ, not just ms: a GPU can be faster
     without using less energy.
   - **Structured pruning / distillation:** needs a training setup and a dataset (WIDER FACE).
     Only structured (channel) pruning speeds up CPU inference. Do this only if time allows.
5. **Accuracy check after every change.** Record about 200 frames at the demo spot. Label them
   with a stronger detector on a PC and spot-check by hand. Re-run recall and false positives
   after each optimization, so no gain secretly costs accuracy.

## Demo station

- Keep the power meter visible, and show a heads-up display with the mode
  (idle / gated / tracking / detecting), the inference rate, and live W.
- Walk up to the robot and the power rises; walk away and it drops. That shows the gating
  without any explanation.
- Have the before/after table from RESULTS.md printed or on screen.
