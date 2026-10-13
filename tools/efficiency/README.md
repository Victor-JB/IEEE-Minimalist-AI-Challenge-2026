# On-board efficiency and accuracy evaluation

This directory measures section 3 of the Hardware Deployment rubric: the actual
baseline, a justified optimization, quantified gains, and retained accuracy.
The baseline is commit `7119cf6`, the pushed version of the live 8082 app.

## Public data

- [WIDER FACE](https://huggingface.co/datasets/CUHK-CSE/wider_face): 240 validation
  images selected by a fixed SHA256 ordering, without inspecting predictions.
  Download `data/WIDER_val.zip` and `data/wider_face_split.zip` to a local cache.
  The dataset is CC BY-NC-ND 4.0. Images are not committed or redistributed here.
- [HaGRIDv2](https://github.com/hukenovs/hagrid): 30 test images in each of palm,
  like, dislike, one, point, fist and no_gesture, one image per subject within
  each category. Selection uses the same fixed SHA256 ordering. Public source
  archives are fetched with HTTP ranges rather than downloading tens of GB.
  See the project's variant of CC BY-SA 4.0 in its `license/en_us.pdf`.
  Demographic metadata and annotated landmarks are not used for this evaluation.

Preparation uses only labels and IDs, never model confidence or outcomes:

```sh
python3 tools/efficiency/prepare_public.py --root /tmp/luxoiii-validation --wider 240 --hagrid 30
```

The manifests record image IDs, ground-truth boxes, labels and image SHA256.
Data files stay outside Git. `evaluate.py` verifies every image hash before
inference and resets hand tracking between unrelated images.

## Target-device measurements

Use the board's installed LiteRT/OpenCV environment. Make separate baseline
and candidate directories; copy the baseline with `git archive 7119cf6`.
Include `apps/luxoiii` and `arm/python` in each. The runner and
`tools/spatial/inference.py` can be shared, but `PYTHONPATH` selects the code
being measured. Both controlled benchmark variants set OpenCV to one thread; the old live
observer used OpenCV's four-thread default. Stop competing inference during each measured run, then resume
it; the live camera service can remain running. Do not run the benchmark while
copying the dataset to the board.

```sh
PYTHONPATH=/path/to/baseline/apps/luxoiii python3 tools/efficiency/evaluate.py \
  --data /path/to/data --output baseline-results.json --label baseline-7119cf6 --threads 2
PYTHONPATH=/path/to/candidate/apps/luxoiii python3 tools/efficiency/evaluate.py \
  --data /path/to/data --output optimized-results.json --label final-serial-4-conservative --threads 4 --discovery-interval 1
```

`profile_runtime.py` runs the 1/2/4-thread sweep on reproducible random pixels.
This is a latency experiment, never an accuracy result. Its peak RSS is cumulative
across loaded thread configurations; use the separate evaluation-process RSS
for before/after memory comparisons.

The final default keeps the conservative second-hand discovery schedule. The
experimental `--discovery-interval 3` reduces palm calls but lost ground-truth
matches in the synthetic sequence audit, so it is not enabled in the live app.
The recorded concurrent ablation used `--parallel --threads 2 --discovery-interval 3`;
its ROI policy is experimental, not the final default. Four
sequential threads were chosen for the faster face-only workload and comparable
hand acquisition latency. `--only-temporal` repeats just the motion audit.

## What the metrics mean

- Latency includes image-to-tensor preprocessing, model invocation, decoding and
  NMS. The gesture workload runs face detection as well as hand detection, as
  the real app does. File reading/JPEG decoding and hardware RPC are excluded.
- Report p50, p95 and mean, process CPU milliseconds, and reciprocal mean latency
  as *model-workload throughput*. This is not camera FPS or full-system latency.
- WIDER results use score-ordered, one-to-one IoU >= 0.5 matches against all valid
  boxes, ignoring detections matching invalid boxes. These subset precision /
  recall results are not the official WIDER easy/medium/hard average precision.
  Also report recall for faces at least 32 pixels on both sides after resizing.
- Gesture hand boxes are the bounding extent of the inferred 21 landmarks.
  Match at IoU >= 0.5. End-to-end gesture recall includes missed hands;
  conditional classification accuracy excludes misses and is labelled as such.
  `one` and `point` both map to Point; `like` to Far / thumbs-up; `dislike` to
  Close / thumbs-down; `palm` to the greeting. Other labels map to no command.
- The synthetic sequence test uses two fixed images per active category,
  translated by at most six horizontal and three vertical pixels over twelve
  frames. It measures temporal workload and consistency, not real video
  accuracy, second-hand arrival latency, motion blur, or long-term tracking.
- Unchanged model weights mean no claim of new quantization, pruning, training,
  distillation, smaller model size or NPU acceleration. Quantization was already
  provided by the Qualcomm models. CPU time is not electrical energy; actual
  power/energy savings require an external meter.

## Live CPU, RAM and processing rates

Run the read-only sampler **on the UNO Q**, while the normal services and viewing
browser remain running:

```sh
python3 tools/efficiency/measure_live_resources.py --duration 60 --output reports/efficiency/live-resources.json
```

This samples both user services and the whole Linux system about once per second.
CPU percentages use all logical CPU capacity: on the four-core board, one fully
occupied core is 25%. The full-window average uses CPU counter differences and
actual elapsed time; the maximum is the highest sampled interval. Process CPU
includes all its threads. These service processes have no worker subprocesses.

RAM is in MiB. RSS measures each process's resident memory; PSS apportions shared
pages, allowing the observer and web service's PSS to be added without counting
shared pages twice. System memory in use is `MemTotal - MemAvailable`, including
other services. Memory peaks are sampled, not guaranteed instantaneous maxima.
The browser's rendering memory/CPU belongs to the viewing computer and is excluded
from these board measurements. Sampler overhead is included in system usage.

Face passes, hand-pipeline updates and palm detector calls are reported separately.
They overlap and cannot be added into a total neural-inference count. Counter rates
use the observer's own uptime between cached observation snapshots; their window
differs slightly from the resource-sampling window. Camera images and landmarks
are discarded. Scene counts and modes describe an uncontrolled live workload,
not a repeat of the fixed-image benchmark.

**Energy remains unmeasured.** The board exposes no usable electrical power or
energy reading, and no external input-power measurement was available. We cannot
calculate numerical inferences per joule from CPU usage, latency or charger ratings.
A future measurement must define its workload and supply boundary, measure joules
over the same window, then divide completed workloads by those joules. With average
watts and a matching measured throughput, `workloads/J = workloads/s ÷ watts`.
Document whether the camera, servo and separately powered lamp are included.

## References

Shuo Yang, Ping Luo, Chen Change Loy and Xiaoou Tang. *WIDER FACE: A Face
Detection Benchmark*, CVPR 2016. [Dataset](https://shuoyang1213.me/WIDERFACE/).

Anton Nuzhdin, Alexander Nagaev, Alexander Sautin, Alexander Kapitanov and Karina
Kvanchiani. *HaGRIDv2: 1M Images for Static and Dynamic Hand Gesture Recognition*,
2024. [Source and license](https://github.com/hukenovs/hagrid).

The use of hand landmarks to update the next ROI follows the established
[MediaPipe Hand Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker)
approach. The experimental interval extends this to one hand in two-hand mode; the
accuracy audit is why that extension stays disabled by default.
