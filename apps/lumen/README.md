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

Manual control: `python -m lumen.desk` (stop Lumen first; one arm owner), then
`http://daboard:8081/`. It has the camera, live joints and sensors, enable/lock, arm/disarm,
hold-to-jog, pose targets, taught poses, clips, head tilt, the measured-vs-commanded plot, and
recordings with replay. It's the Stepper Desk Mac app's job, on the closed-loop firmware.

HUD stream: `http://daboard:8080/` (state, active models, light, energy, detections).
Transition log: `apps/lumen/logs/transitions-<date>.csv` on the board. Every state change with
the models running and the cumulative inference counts, which is your scorecard data. Transitions
into FAULT also have a `reason` (`STALL`, `SENSOR_STALE`, `LINK_LOST`, `OVERTEMP`, `ARM`, `ERROR`,
then the detail), which the HUD shows while in FAULT.

## Layout

Every folder has one `config.py` with all of its tunable numbers. To change how something
behaves, edit that file and restart Lumen; nothing else needs touching.

```
apps/lumen/
├── README.md · requirements.txt · lumen.service
├── scripts/      deploy.sh (Mac → board: apps/lumen + arm/) · run.sh (on the board, foreground)
├── tests/        python3 -m unittest discover -s apps/lumen/tests
├── tools/        simulate.py (whole demo on a fake clock) · scorecard.py (logs → RESULTS rows)
├── models/       the .tflite files (paths are set in each model's config)
└── lumen/
    ├── __main__.py · app.py        entry point · main loop (camera → AI → state → movement + light)
    ├── movement/                   ★ everything that moves the arm
    │   ├── config.py               stall handling, face centering, AIM / SEARCH / ADJUST, per-state
    │   │                           moves (sleep, wake, check-in, reacquire, mirror), noise + energy
    │   ├── worker.py               the thread streaming to the arm at 25 Hz (or fake)
    │   ├── aim.py · look_at.py     book → arm pose, sweep · face centering
    │   ├── modifiers.py            glance / windup / breathe / hesitate plans
    │   ├── expressive.py · pools.py  clip choice + noise · which clips each state plays
    ├── ai/                         ★ perception: frames in, events out
    │   ├── config.py               event rules (face seen/confirmed/lost, presence, crops)
    │   ├── face/       config.py · detector.py · tracker.py    model path, thresholds, which face
    │   ├── hands/      config.py · landmarker.py · gestures.py model, thresholds, tracking, gesture
    │   │                                                       rules, hold times, wave
    │   ├── book/       config.py · detector.py                 found/lost rules (model: in progress)
    │   └── motion_gate/ config.py · gate.py                    the free frame-difference gate
    ├── state/                      ★ the behavior state machine
    │   ├── config.py               state timers (boot, fault cooldown, notice, lost, check-in, reacquire)
    │   ├── machine.py              STATES (models + rates + pool per state) and TRANSITIONS, as data
    │   └── director.py             turns states into movement and light
    ├── light/      config.py · light.py      brightness ramps, gamma, coupling, LED backend
    ├── io/         config.py · camera.py · power.py · thermal.py   camera, SLEEP power, overtemp
    ├── ui/         hud.py · thinking.py · stream.py   the :8080 web view (state, next, arm, AI boxes)
    ├── desk/                       Lumen Desk: browser control page (python -m lumen.desk, :8081)
    └── events.py · paths.py · translog.py · perfstats.py
```

The arm's hardware limits (HOME, speed/accel caps, joint safe ranges) live in
`arm/python/arm_config.py`, shared with the firmware tools; `movement/config.py` re-exports them.
`tests/test_layout.py` fails if Lumen ever defines its own copy of an arm parameter.

## Debugging: what to edit

| Symptom | Edit |
|---|---|
| Face not detected / false faces | `ai/face/config.py` `SCORE_THRESHOLD` |
| NOTICE → STANDBY too slow / too eager | `ai/config.py` `FACE_CONFIRM_K/N`, `FACE_LOST_S` |
| Head overshoots or crawls when centering a face | `movement/config.py` `RATE_DEG_S`, `DEADBAND` |
| Gesture not recognized | `ai/hands/config.py` gesture rules + `GESTURE_HOLD_S` / `PALM_HOLD_S`; check with `python -m lumen.ai.hands.compare_gestures DIR` |
| Hands not found | `ai/hands/config.py` `PALM_THRESHOLD`, `LANDMARK_THRESHOLD`, `TRACKING` |
| Book found/lost flicker | `ai/book/config.py` |
| Arm stalls / gives up / FAULTs | `movement/config.py` stall section; caps in `arm/python/arm_config.py` |
| Aims at the wrong spot | `movement/config.py` camera + desk geometry (`CAMERA_HFOV_DEG`, `DESK_AREA`) |
| CHECK-IN too long | `movement/config.py` `CHECKIN_MAX_UP` |
| A state times out too soon / late | `state/config.py` |
| Which models run in a state, which clips it plays | `state/machine.py` `STATES` · `movement/pools.py` |
| Light too bright / ramps | `light/config.py` |
| Idle never sleeps, or wakes too easily | `ai/config.py` `NO_MOTION_SLEEP_S` · `ai/motion_gate/config.py` `MOTION_THRESHOLD` |

To change behavior, edit `state/machine.py`'s two tables and keep `docs/STATE_MACHINE.md` in sync
(`tests/test_state_diagram.py` checks it). Add a test for any new transition.
