# Model selection, quantization and the compute budget

Lumen runs everything on the Arduino UNO Q's Linux side: a Qualcomm QRB2210 with 4× Cortex-A53,
CPU-only TFLite, no GPU or NPU delegate. Every model was chosen against that budget, and every
model in the app is **int8 (w8a8)**: 8-bit weights and activations, the smallest and fastest form
the A53 runs well. This page explains why each model was picked, how the book model was benchmarked
and quantized, and how much of the board the live loop uses.

Where each model's settings live: `apps/lumen/lumen/ai/<model>/config.py` (see the app README).

| Job | Model | Size | Input | Licence | Runs when |
|---|---|---|---|---|---|
| Is anything moving? | motion gate: a frame difference, no neural net | – | 80×60 gray | – | IDLE, SLEEP, HOLD |
| Face | Qualcomm Lightweight-Face-Detection (`face_det_lite`), w8a8 | 1.0 MB | 640×480 gray | not recorded in the repo: check its Qualcomm AI Hub page | IDLE (1 Hz, only after motion), NOTICE, STANDBY, LOST, CHECK-IN, GOODBYE |
| Hands | MediaPipe palm detector + hand landmarks (Qualcomm AI Hub export), w8a8 | 2.2 + 3.3 MB | 256² / 224² RGB | not recorded in the repo (upstream MediaPipe is Apache-2.0); check the Qualcomm AI Hub page | STANDBY, SEARCH (STOP), ILLUMINATE, FAULT (reset) |
| Book | Qualcomm AI Hub Yolo-X (YOLOX-S), w8a8, COCO class 73 "book" | 9.4 MB | 640² BGR | Apache-2.0 | SEARCH pauses, AIM, HOLD (0.5 Hz check), REACQUIRE |

The design idea is a cascade: the free motion gate decides whether anything runs at all, the
state machine decides which model runs and how often (`state/machine.py` `STATES`), and the
expensive book model only runs at the moments it can matter.

---

## Book detector

The book model was the one real choice: face and hands have one obvious small int8 model each,
while "find a book on a desk" has many candidate detectors.

### Candidates and why some were never tried

All candidates are COCO-trained detectors (COCO's 80 classes include "book", class 73), so no
training was needed to get a working system.

| Candidate | Source | Licence | Outcome |
|---|---|---|---|
| **YOLOX-S 640 w8a8** | Qualcomm AI Hub Yolo-X v0.64.0 (public, no login) | Apache-2.0 | **Shipped** |
| YOLOX-nano 320 / 416 / 256, fp32 + int8 | exported by us from Megvii YOLOX 0.3.0 (`bench/export_yolox.py`) | Apache-2.0 | fp32 works but weaker; our int8 export is broken (see below) |
| EfficientDet-Lite0 320, Lite2 448, fp32 + int8 | Google MediaPipe object-detector models | Apache-2.0 | Lite0 int8 is the fallback if YOLOX-S is too slow on the board |
| YOLOv5 / v8 / v10 / v11, Ultralytics | Qualcomm AI Hub / Ultralytics | **AGPL-3.0** | Not used: copyleft would cover the app |
| YOLOv6 / v7 | Qualcomm AI Hub | **GPL-3.0** | Not used, same reason |
| SSD-MobileNetV2, EfficientDet, YOLO-NAS (Qualcomm repos) | Hugging Face | – | Not used: download needs a login |

### How it was measured

- **Eval set (stand-in until the lamp-camera set exists):** COCO val2017, built by
  `bench/coco_bookset.py`. 112 images with books (211 books) and 150 images with no book. Books
  covering too little of the image (a spine on a far shelf) are "ignore" regions: missing them
  isn't a miss and detecting them isn't a false positive, because a book under the lamp is big.
- **Metrics** (`bench/eval_detector.py`, per score threshold):
  - **recall @ IoU 0.5**: share of labelled books found;
  - **false positives / frame**: wrong detections per image;
  - **book-free images that fire**: what would cause a false BOOK_FOUND;
  - **latency** p50 / p90 of the whole `detect()` (preprocess + invoke + postprocess).
- **Quantization:** TFLite full-integer post-training quantization (per-channel int8 weights, int8
  activations, uint8 input/output), calibrated on 200 COCO train2017 images (half with books),
  letterboxed exactly the way the app feeds frames, and disjoint from the eval set.
  Qualcomm's and Google's int8 models come quantized by their publishers; the YOLOX-nano ones are
  ours (`bench/export_yolox.py`). Our export also keeps only the book row of the 80-class head,
  so the other 79 class maps are never computed.

### Results (COCO stand-in set, Mac, 10 threads)

| Model | Threshold | Recall @ IoU 0.5 | FP / frame | Book-free images that fire | Mac p50 |
|---|---|---|---|---|---|
| **YOLOX-S 640 w8a8 (shipped)** | **0.3** | **0.445** | 0.20 | 0.7 % | ~28 ms |
| YOLOX-S 640 w8a8 | 0.5 | 0.246 | 0.03 | 0 % | ~28 ms |
| EfficientDet-Lite0 320 int8 | 0.3 | 0.237 | 0.24 | 2 % | ~9 ms |
| YOLOX-nano 320 fp32 | 0.2 | 0.204 | 0.12 | 0.7 % | ~13 ms |
| YOLOX-nano 320 int8 (our export) | any | **0.000 (broken)** | – | – | ~7 ms |

### Decisions

- **YOLOX-S w8a8** has almost twice the recall of anything smaller, with very few false alarms,
  under a permissive licence. Its cost (the largest of the four models) is contained by running it
  only at sweep pauses and 0.5 Hz checks; see the compute budget below.
- **Score threshold 0.3, not 0.5.** At 0.5 recall halves (0.25). The false alarms at 0.3 are
  absorbed by the state machine: BOOK_FOUND needs 2 hits in a row (`ai/book/config.py`), and only
  0.7 % of book-free images fire even once.
- **BGR input.** Qualcomm documents RGB, but BGR scored better (book recall 0.445 vs 0.384,
  persons 0.957 vs 0.904), and OpenCV frames are BGR anyway, so no conversion is spent.
- **Why the YOLOX-nano int8 export failed:** the calibrated activation ranges came out about 15×
  wider than the real values, so the int8 outputs bear no relation to the float model's. A fix would
  be per-layer range clamping or quantization-aware training; there wasn't time. This is the
  general lesson: quantize, then always re-evaluate accuracy, because int8 can silently break a
  model that looks fine in fp32.
- **COCO is a hard stand-in** (small books on shelves); a book under a desk lamp fills 30-70 % of the
  frame. Expect real numbers to be better, but measure them:
  `bench/capture_desk_set.py` records ~150 lamp-camera frames (≈100 with books, ≈50 traps such as a
  laptop, phone, papers), pre-labels them, you correct them by hand, then
  `bench/eval_detector.py --data bench/data/desk --sweep 0.3 0.4 0.5`.

## Face and hands

- **Face, `face_det_lite` w8a8 (1 MB):** the smallest maintained face detector in Qualcomm AI Hub.
  It takes grayscale 640×480, so there's no color conversion, and its heatmap output decodes in a
  few lines (`ai/face/detector.py`). Measured on the board on 2026-10-03: about 70 ms per detection.
- **Hands, MediaPipe palm + landmarks w8a8:** the standard two-stage hand pipeline. The palm
  detector (256²) is the expensive part. **Tracking** skips it: once a hand is found, the next crop
  comes from the last landmarks, so only the 224² landmark model runs. On the Mac a detection costs
  ~15 ms with the palm detector and ~3 ms tracked. Gestures come from rules on the 21 landmarks
  (`ai/hands/gestures.py`), with no extra model; the canned MediaPipe gesture classifier is wrapped
  for comparison (`python -m lumen.ai.hands.compare_gestures DIR`).

---

## Compute budget on the UNO Q

Not yet measured live end to end; this is an estimate, so treat it as a plan for what to measure.

**Method.** Every piece of the live loop was timed on a Mac (Apple Silicon, TFLite with 4 threads,
median of repeated calls). The face model's time on the board is known (~70 ms), so its Mac time
(3.4 ms) gives a **Mac → board factor of ~20×**. That number is from an older run whose camera
frames were wider than 640, so the real factor may be lower; the table shows **×10 (optimistic) to
×20 (pessimistic)**.

| Piece | Mac | UNO Q estimate | How often |
|---|---|---|---|
| Face model | 3.4 ms | 35-70 ms | 1-10 Hz depending on state |
| Hands: palm + landmarks (no hand tracked) | 11.8 ms | 120-240 ms | each hand inference without a tracked hand |
| Hands: tracked (landmarks only) | ~3 ms | 30-60 ms | each hand inference with a tracked hand |
| Book YOLOX-S 640 | 31 ms | 310-620 ms | SEARCH pauses, 0.5 Hz in HOLD, 3 Hz in REACQUIRE |
| Motion gate | 0.24 ms | 2-5 ms | up to 10 Hz |
| State machine + director + perception bookkeeping | < 0.01 ms | < 0.2 ms | every frame |
| Arm kinematics (IK for AIM / ADJUST) | 0.007 ms | < 0.2 ms | once per AIM / ADJUST |
| SEARCH sweep planning (FK + coverage) | 30 ms | 0.3-0.6 s | once per run (cached after the first SEARCH) |
| Clip trajectory | 0.25 ms | 3-5 ms | once per clip |
| Arm stream tick (25 Hz, math only) | < 0.01 ms | < 0.2 ms | 25 Hz, own thread |
| HUD draw + JPEG for the :8080 view | 1.2 ms | 12-24 ms | every frame, **only while a browser is watching** |

**The arm keeps streaming during inference.** TFLite releases Python's GIL while it runs, so the
25 Hz arm thread is not blocked: with the book model running back to back for 3 s, the 25 Hz thread's
worst gap was 46 ms (target 40 ms; the firmware only halts after 250 ms without a setpoint).
The kinematics, planning and "thinking" layers cost almost nothing; the neural nets are the budget.

**Per state** (the main loop is one thread; at 15 fps it has ~66 ms per frame; pessimistic numbers):

| State | Models | Load | Verdict |
|---|---|---|---|
| IDLE | gate 10 Hz, face 1 Hz after motion | ~12 % | fine (measured 21 fps on the board) |
| NOTICE | face 10 Hz | ~70 % | OK, it lasts ~1.5 s |
| STANDBY, hand in view (tracked) | face 5 Hz + hand 10 Hz | ~95 % | borderline |
| STANDBY, no hand in view | face 5 Hz + palm detector 10 Hz | **~270 %** | over budget: the loop drops to ~3-4 fps |
| SEARCH | book ×2 per pause | 2 × 0.6 s per pause vs a 0.6 s dwell | the 2nd inference may land after the arm moves on |
| HOLD | hand 8 Hz + book 0.5 Hz | ~80 % | OK, but each book check freezes frames for ~0.6 s |
| REACQUIRE / a missed book check | book 3 Hz | **~190 %** | saturated: book runs back to back (~1.5 fps) |

**What to measure first on the board** (`PERF=1 bash apps/lumen/scripts/run.sh`, the timing report
prints on Ctrl+C), and the config levers if the numbers confirm the table (no code changes needed):

1. **STANDBY with no hand:** lower the hand rate in `state/machine.py` `STATES["STANDBY"]`
   (10 → 4 Hz).
2. **Book latency:** if YOLOX-S is well over ~300 ms, lower `BOOK_CONFIRM_HZ` (`ai/book/config.py`,
   3 → 1) and REACQUIRE's book rate in `STATES` (3 → 1 Hz), and raise `SWEEP_DWELL_S`
   (`movement/config.py`) to cover `BOOK_PER_POSE` inferences, or set `BOOK_PER_POSE = 1` and
   `BOOK_FOUND_CONSECUTIVE = 1` for the sweep. If it's still too slow, switch to
   EfficientDet-Lite0 320 int8 (needs its decoder moved from `bench/book_candidates.py` into the app).
3. **Close the :8080 tab** when measuring power; the JPEG encode is pure overhead.
4. Then log the per-state numbers with `tools/scorecard.py` (see `docs/DEMO_MEASUREMENT.md`).
