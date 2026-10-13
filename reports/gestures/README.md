# Gesture accuracy without losing the inference speedup

The live luxoiii app now uses a 21,168-byte landmark classifier instead of the
original wrist-distance rules. It learns foreshortened pointing, open palm,
thumbs up/down, and non-command hand shapes from public HaGRID training data.
The image models, detection thresholds, crop geometry, four-thread sequential
schedule, face guard, physical tracking, and pointing projection are unchanged.

## Results measured on actual UNO Q hand-model outputs

The primary metric is unchanged: a prediction must first match a human-annotated
hand box at IoU ≥ 0.5, then have the correct gesture. Missed hands and negative
hands remain in the denominator. Prediction boxes are the tight extent of the
21 landmarks, without padding. Both recognizers receive identical cached model
outputs, preventing timing/noise differences from changing their input.

| Image sample | Original rules | New classifier | Correct pointing, including misses |
|---|---:|---:|---:|
| Original 210-image regression sample | 125/258 = 48.4% | 174/258 = 67.4% | 21/60 → 35/60 |
| 140-image official validation sample | 80/168 = 47.6% | 114/168 = 67.9% | 14/40 → 25/40 |
| **Fresh 210-image test sample** | **119/259 = 45.9%** | **174/259 = 67.2%** | **18/60 → 30/60** |

The [before/after chart](gesture-comparison.svg) shows our fresh-test results on
a full 0–100% scale: overall correct recall improved by 21.2 percentage points,
and pointing improved by 20.0 percentage points. Both measures include missed
hands. We tested 210 held-out still photographs from 189 subjects; these results
do not measure live camera accuracy. A [PNG copy](gesture-comparison.png) is
available for slides. The plot reads counts directly from the two
`test-*-score.json` files and verifies that their manifest and landmark-cache
hashes match each other and the committed source files. Rebuild the exports
and the embedded website chart with Matplotlib:
`python3 tools/efficiency/plot_gesture_comparison.py --embed-preview`.

On the fresh test sample, classification accuracy conditional on successful
localization rose from 119/177 (67.2%) to 174/177 (98.3%). All 30 localized
pointing hands were recognized, including 16 forward-pointing hands versus
4/16 with the old rule. The 30 other pointing hands failed localization and
still count as misses. The 100% conditional pointing result is a small-sample
observation, not a claim of perfect camera-to-gesture recognition.

False commands on the 72 localized negative hands fell from 35 to 2. Including
unmatched detections, strict command precision rose from 52.6% to 70.3%; an
unmatched command can be a real hand with insufficient box overlap, so this is
not a direct estimate of accidental commands per minute in live use.

## Speed

The UNO Q used four LiteRT CPU threads per image model, sequential inference,
and one OpenCV thread. The board's normal camera service remained running;
the live observer was not competing with the benchmark. There were no hardware
commands in the benchmark.

The first five fresh-test images per category were selected without consulting
their outcomes. Each image ran old/new/new/old (ABBA), resetting hand tracks
between runs: 35 images, 70 timings per recognizer. Timing includes face and
hand inference and gesture classification; it excludes image decoding,
camera capture, networking, browser rendering, and hardware RPC.

| Workload | Original rules | New classifier |
|---|---:|---:|
| Mean face + hands + classification | 183.53 ms | 183.18 ms |
| p95 face + hands + classification | 195.00 ms | 194.89 ms |
| Mean classification per detected hand (765 calls) | 0.023 ms | 1.242 ms |

Total measured latency is effectively unchanged; the small difference is
measurement noise, not a new speedup. The additional classifier costs about
1.2 ms per hand and adds no image-model invocation. This preserves the prior
optimization's scheduling and detector improvements. The earlier 205.68 ms
score used a different image workload and should not be compared directly to
183.18 ms as another improvement. No new power-saving claim is made.

## Training, selection, and independent testing

- Source: [official HaGRIDv2](https://github.com/hukenovs/hagrid), using its
  subject-separated train, validation, and test annotations. Dataset and
  derived landmark fixtures/weights are attributed under CC BY-SA 4.0.
- Training: 28,819 auto-annotated 2D landmark hands from 13,428 training
  subjects. Human gesture labels supervise a 106 → 48 → 5 NumPy network.
  Classes include `one`, `point`, `palm`, `like`, `dislike`, `fist`,
  `no_gesture`, `peace`, and `ok`, including secondary hands.
- Selection: 4,447 landmark hands from 1,551 official validation subjects;
  epoch 100, seed 103. The 140 photo validation images also use the validation
  split and overlap this selection population. They are a tuning check,
  **not a second independent holdout**.
- Fresh test: 210 images from 189 test subjects, 30 images per each of the
  original seven categories. Every subject from the old regression sample was
  excluded. There is zero overlap with training or validation subjects.
  The classifier, artifact hash, and thresholds were frozen before fresh-test
  inference; no test-based tuning followed.
- Thresholds: landmark presence ≥ 0.65, command probability ≥ 0.5, palm ≥ 0.5.
  No threshold or crop changes were needed.

The original regression set was already inspected and is not an untouched
holdout. The dataset's supplied MediaPipe landmarks support training only;
all reported photo results use landmarks actually produced by the deployed
quantized models on the UNO Q, not the supplied annotation landmarks.

The [validation learning curve](training-accuracy.svg) plots every recorded
epoch in `training-report.json`: 82.75% after epoch 1 to 98.00% after epoch 100.
It uses supplied landmarks, not live camera images. No training-set accuracy
history was recorded. A [PNG copy](training-accuracy.png) is available for slides.
Rebuild both figures and the embedded website chart with Matplotlib:
`python3 tools/efficiency/plot_gesture_training.py --embed-preview`.

Fresh photos were resized with OpenCV to fit 640×480, preserving aspect ratio,
and saved losslessly as PNG before transfer. PNG roundtrips were checked for
exact pixel equality. Both recognizers see the same decoded frames. Manifests
record original-image hashes, derived-frame hashes, and the OpenCV version.
The original regression cache reproduces all old confusion counts exactly.

## Remaining limitations

Hand detection/localization is now the main bottleneck: the fresh test has
82/259 unmatched target hands. Tight landmark boxes often sit inside the larger
annotated full-hand box; this protocol intentionally keeps the old IoU rule
so the improvement cannot come from changing the metric. Some hands are also
genuinely missed or poorly landmarked. The classifier cannot recover an
undetected hand. The balanced public still-image sample does not measure live
gesture stability, motion blur, interaction latency, or desk-target accuracy.
No private camera frames were recorded for this evaluation.

## Evidence and reproduction

- `frozen-candidate.json`: runtime/model hashes before fresh testing.
- `training-report.json`, `landmarks-manifest.json`, `split-integrity.json`:
  training history, inputs, and subject-separation audit.
- `*-manifest.json`: fixed photo selections and hashes.
- `*-landmarks.json.gz`: actual board outputs, including low-presence proposals;
  presence filtering remains at 0.65. Cache hashes in score files refer to the
  decompressed JSON bytes. These contain public hand landmarks, not photos.
- `*-original-score.json`, `*-new-score.json`: full confusion counts and
  per-image predictions with identical localization matching.
- `runtime.json`: all paired latency observations.
- [Model card](../../apps/luxoiii/models/gesture_landmarks_v1.md): architecture,
  training recipe, attribution, and model SHA-256.

Re-score the committed cache without needing the board or photographs:

```sh
PYTHONPATH=apps/luxoiii python3 tools/efficiency/score_gestures.py \
  --cache reports/gestures/test-landmarks.json.gz \
  --manifest reports/gestures/test-manifest.json \
  --classifier current --output /tmp/gesture-test-score.json
```

Use `prepare_gesture_data.py` to reproduce the deterministic train/validation/
test selections, `prepare_model_frames.py` for lossless frame staging, and
`collect_gesture_landmarks.py` on the board for fresh image-model outputs.
`benchmark_gesture_runtime.py` reproduces the paired speed measurement.
The scorer rejects incomplete/duplicate caches and refuses to score a missing
trained model as if the legacy fallback were the new classifier.

Runtime validation includes the app regression suite, classifier rotation/
mirror/aspect tests, invalid-input and artifact checks, and a live app smoke
check. Hardware safeguards and gesture hold behavior remain unchanged.
The final suite ran 317 tests: 313 passed and four runtime-dependent tests were
skipped on the Mac. The two browser-logic checks passed. All ten live health
samples were fresh, reported `landmark-mlp-v1`, and showed subject tracking
connected with steppers disabled; no camera frames were saved.
