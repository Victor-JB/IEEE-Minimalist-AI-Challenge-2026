# Lumen: the behavior app

Camera → perception (face / hand / book, each only when the state asks) → state machine →
expressive motion and light. The behavior spec is [docs/STATE_MACHINE.md](../../docs/STATE_MACHINE.md);
this folder implements it.

Lumen is Python on the Linux side only. Everything about the arm (firmware, API, parameters,
clips) comes from the shared [arm/](../../arm/README.md) module, and Lumen starts at boot as a
systemd service. [apps/README.md](../README.md) explains why it is not
an App Lab app and how boot is decided.

## Run it

```sh
# Laptop with a webcam, no board: the full behavior with a simulated arm
cd apps/lumen && python3 -m pip install -r requirements.txt
python3 -m lumen --fake-arm --stream 8080        # HUD at http://localhost:8080/
python3 -m lumen.io.camera --probe               # list cameras

# Tests (pure logic + layout guards; no camera or board needed)
python3 -m unittest discover -s apps/lumen/tests

# Board, first time
bash arm/scripts/deploy.sh                       # flash the arm firmware + arm venv (skip if done)
bash apps/lumen/scripts/deploy.sh                # copy apps/lumen + arm/, venv, boot service, start
#   then in App Lab: make sure no app shows the DEFAULT badge

# Board, every change
bash apps/lumen/scripts/deploy.sh
ssh arduino@daboard journalctl --user -u lumen -f                                       # live logs
ssh arduino@daboard bash IEEE-Minimalist-AI-Challenge-2026/apps/lumen/scripts/run.sh   # foreground
```

HUD stream: `http://daboard:8080/` (state, active models, light, energy, detections).
Transition log: `apps/lumen/logs/transitions-<date>.csv` on the board. Every state change with
the models running and the cumulative inference counts, which is your scorecard data.

## Layout

```
apps/lumen/
├── README.md · requirements.txt · lumen.service
├── scripts/      deploy.sh (Mac → board: apps/lumen + arm/) · run.sh (on the board, foreground)
├── tests/        test_machine.py · test_expressive.py · test_layout.py
├── models/       face_det_lite-tflite-w8a8/ · mediapipe_hand_gesture-tflite-w8a8/
├── bench/        (dev branch) bench_detector.py · what to measure · RESULTS.md
└── lumen/
    ├── __main__.py · app.py        entry point · main loop
    ├── config.py                   behavior numbers (rates, debounce, timers); arm numbers come from arm_config
    ├── machine.py                  ★ STATES and TRANSITIONS, as data
    ├── events.py                   event names + debouncers
    ├── paths.py                    models/logs paths; puts arm/python on sys.path (the only place)
    ├── perfstats.py                timers + system stats (no-op unless PERF=1)
    ├── translog.py · hud.py        transitions CSV · demo overlay
    ├── perception/                 Perception.step, scheduler, motion_gate, face_detector + face,
    │                               hands (TODO), gestures, book (TODO)
    ├── behavior/                   pools, expressive (noise + energy), director, look_at
    ├── motion/worker.py            thread streaming to the arm at 25 Hz (or fake), via arm/python
    └── io/                         camera, stream (MJPEG), light (TODO)
```

From `arm/` Lumen uses `arm_config` (home, caps, safe ranges), `arm_motion.ArmMotion`,
`trajectory` (clip → real-arm trajectory) and the clips. `tests/test_layout.py` fails if Lumen
ever defines its own copy of an arm parameter.

## Where to start writing

Working today: the state machine (tested), the expressive layer, the face path, the motion gate,
gesture rules, the director, and the fake-arm loop (the full happy path runs end to end).
In order of what unblocks the demo:

1. **Run it on the board with the face only.** `deploy.sh`, then walk up: IDLE → NOTICE → STANDBY.
   Tune `config.py` and the look-at gain in `behavior/look_at.py`.
2. **`perception/hands.py`.** Load the MediaPipe Hand models from `models/mediapipe_hand_gesture-tflite-w8a8/`;
   return 21 landmarks per hand. That folder also has a canned gesture classifier: compare it with
   the rules in `gestures.py` on real hands (print `perception.gesture` on the HUD).
3. **`perception/book.py`.** Load the book detector; return boxes.
4. **`behavior/director.py`:** `_sweep_poses()` and the AIM pose (book position → arm pose).
   Both are marked TODO. A real left-right sweep needs the 4th (yaw) joint.
5. **`io/light.py`.** Drive the LED once it's wired.
6. Mirror mode in STANDBY (TODO in `director.tick`), then the optimization rungs.

To change behavior, edit `machine.py`'s two tables and keep `docs/STATE_MACHINE.md` in sync.
Add a test in `tests/test_machine.py` for any new transition.
