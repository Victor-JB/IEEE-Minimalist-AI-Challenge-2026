# Gesture landmark classifier v1

`gesture_landmarks_v1.npz` is a small luxoiii-trained classifier that turns the
existing hand model's 21 landmarks into `point`, `open_palm`, `thumbs_up`,
`thumbs_down`, or no command. It improves recognition of an index finger aimed
toward the camera, whose tip can appear closer to the wrist in the image.
It adds no image model or additional camera pass.

## Architecture and artifact

- 106 geometric features → 48 ReLU units → 5 softmax outputs.
- 5,381 float32 parameters; 21,168 bytes on disk.
- SHA-256: `e15bfec6814e1127431304881eed29c7ffa1160232339eedbdd295876d8d787f`.
- Feature format version: 1. NumPy inference; no training-library dependency.
- Default command confidence threshold: 0.5; uncertain results return no command.
- Features remove translation and independent image-axis scaling. Palm-basis
  coordinates and joint features supplement normalized landmark positions.
- Inference uses x/y only. Existing world landmarks and physical pointing
  geometry remain separate from this classifier.

## Data and training

Source: the [official HaGRID repository](https://github.com/hukenovs/hagrid),
using HaGRIDv2's official `annotations_with_landmarks/annotations.zip` train and
validation splits. Human gesture labels supervise the classifier; the 2D
landmarks are MediaPipe auto-annotations. This is gesture classification from
landmarks, not a retrained hand detector or a measurement of landmark accuracy.

Training uses 28,819 hands from 13,428 unique training subjects. Selection uses
4,447 hands from 1,551 validation subjects, with zero subject overlap. Source
categories are `one`, `point`, `palm`, `like`, `dislike`, `fist`, `no_gesture`,
`peace`, and `ok`. Both `one` and `point` map to pointing; fist, peace, OK, and
other non-command hands map to no command. This includes secondary hands in
the selected images. No test examples train the model or select its epoch.

Training seed is 103. The trainer uses balanced sampling, Adam, 100 epochs,
batch size 256, a decaying learning rate starting at 0.0015, and weight decay
0.0001. Half of the sampled training hands receive rotation, horizontal
reflection, aspect, and small coordinate-noise augmentation. Thumb gestures
receive only small rotations to preserve the meaning of up and down. Selection
maximizes macro recall on the unaugmented validation annotations; epoch 100
was selected. These validation labels therefore are not an untouched test set.

Reproduce from the cached official training/validation annotations:

```sh
python3 tools/efficiency/train_gesture_classifier.py \
  --data /tmp/luxoiii-gesture-data/landmarks \
  --output /tmp/gesture_landmarks_v1.npz \
  --report /tmp/gesture_training.json --seed 103 --epochs 100 --hidden 48
```

The generated training report records the SHA-256 of every input landmark
archive, the selection history, confusion matrix, and class recalls. Preparing
the data is handled by `tools/efficiency/prepare_gesture_data.py`; its manifest
records source archive URLs, split selection, and subject-disjointness checks.

## Validation scope

On the 4,447 auto-annotated validation hands, the selected model has 98.00%
argmax accuracy; 98.02% with the 0.5 confidence rejection threshold. These
numbers assume usable landmarks are already supplied. They are **not** the
camera-to-gesture accuracy of the robot. Public-photo results must include
missed hands and use actual UNO Q hand-model outputs; those results belong in
the gesture evaluation report.

The runtime tests contain normalized, translated, rounded landmark poses from
the official training split. They check command mapping, rejection, rotation,
reflection, input validation, and aspect invariance. They are not independent
accuracy evidence. Raw photos and private camera frames are not included.

## Attribution and license

HaGRID is provided by the [HaGRID authors and maintainers](https://github.com/hukenovs/hagrid).
The source dataset and derived landmark fixtures are under
[CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/). This trained
weight artifact is also provided under CC BY-SA 4.0 with that attribution.
The surrounding source code retains the repository's license.

The model is intended for this interactive lamp demo. It has not been
validated for safety-critical decisions, other gesture vocabularies, or all
camera, lighting, occlusion, and motion-blur conditions.
