# Lumen: the behavior app

Camera → perception (face / hand / book, each only when the state asks) → state machine →
expressive motion and light. The behavior spec is [docs/STATE_MACHINE.md](../../docs/STATE_MACHINE.md);
this folder implements it.

Lumen is Python on the Linux side only. The arm firmware comes from `apps/stepper_desk`, and
Lumen starts at boot as a systemd service. [apps/README.md](../README.md) explains why it is not
an App Lab app and how boot is decided.

## Run it

```sh
# Laptop with a webcam, no board: the full behavior with a simulated arm
cd apps/lumen && python3 -m pip install numpy opencv-python ai-edge-litert
python3 -m lumen --fake-arm --stream 8080        # HUD at http://localhost:8080/

# Tests (pure logic, no camera or board needed)
python3 -m unittest discover -s apps/lumen/tests

# Board, first time
bash apps/stepper_desk/scripts/deploy.sh         # flash the arm firmware (skip if already done)
bash apps/lumen/scripts/deploy.sh                # install venv + boot service, start it
#   then in App Lab: make sure no app shows the DEFAULT badge

# Board, every change
bash apps/lumen/scripts/deploy.sh                # copy, reinstall deps if needed, restart
ssh arduino@daboard journalctl --user -u lumen -f      # live logs
ssh arduino@daboard bash IEEE-Minimalist-AI-Challenge-2026/apps/lumen/scripts/run.sh   # foreground
```

HUD stream: `http://daboard:8080/` (state, active models, light, energy, detections).
Transition log: `apps/lumen/logs/transitions-<date>.csv` on the board. Every state change with
the models running and the cumulative inference counts, which is your scorecard data.

## Layout

```
apps/lumen/
├── README.md
├── requirements.txt            venv deps on the board
├── lumen.service               systemd user unit (boot)
├── scripts/
│   ├── deploy.sh               Mac → board: copy, venv, enable + restart the service
│   └── run.sh                  on the board: stop the service, run in the foreground
├── tests/
│   ├── test_machine.py         every transition path, with a fake clock
│   └── test_expressive.py      no repeats, energy decay, pool sanity
└── lumen/
    ├── __main__.py             `python -m lumen` entry point + flags
    ├── app.py                  the main loop (wires everything below)
    ├── config.py               every tunable number (rates, debounce times, timers, caps)
    ├── paths.py                imports shared code from inference/, perf/, motion_lib/, stepper_desk/
    ├── events.py               event names + debouncers (KofN, HeldFor, AbsentFor)
    ├── machine.py              ★ STATES (models + pool per state) and TRANSITIONS, as data
    ├── translog.py             transitions CSV
    ├── hud.py                  overlay for the demo stream
    ├── perception/
    │   ├── __init__.py         Perception.step(): runs due models → debounced events
    │   ├── scheduler.py        per-state model rates (the compute budget)
    │   ├── motion_gate.py      80×60 frame difference: no motion → no neural net
    │   ├── face.py             wraps inference/detector.py + largest-face target with hysteresis
    │   ├── hands.py            TODO: MediaPipe Hand (AI Hub, int8)
    │   ├── gestures.py         rules on 21 landmarks: point, thumbs up/down, open palm
    │   └── book.py             TODO: YOLO-family "book" detector (AI Hub, int8)
    ├── behavior/
    │   ├── pools.py            animation pools → motion_lib clip names (all verified to exist)
    │   ├── expressive.py       noise (amplitude/speed jitter, OU drift) + energy mood + no-repeat
    │   ├── director.py         state → what the arm and light do; loops pools; look-at
    │   └── look_at.py          face position → joint offsets
    └── io/
        ├── arm.py              MotionWorker thread: streams to ArmMotion at 25 Hz (or fake)
        └── light.py            TODO: LED driver (stub keeps the level for HUD/logs)
```

Shared code is imported, not copied: the camera and face detector from `inference/`, timers
from `perf/`, clips from `motion_lib/`, the arm API from `apps/stepper_desk/python`, and clip →
real-arm trajectories from `apps/stepper_desk/scripts/emote.py`.

## Where to start writing

Working today: the state machine (tested), the expressive layer, the face path, the motion gate,
gesture rules, the director, and the fake-arm loop (the full happy path runs end to end).
In order of what unblocks the demo:

1. **Run it on the board with the face only.** `deploy.sh`, then walk up: IDLE → NOTICE → STANDBY.
   Tune `config.py` and the look-at gain in `behavior/look_at.py`.
2. **`perception/hands.py`.** Load the MediaPipe Hand models; return 21 landmarks per hand. Then
   check `gestures.py` thresholds on real hands (print `perception.gesture` on the HUD).
3. **`perception/book.py`.** Load the book detector; return boxes.
4. **`behavior/director.py`:** `_sweep_poses()` and the AIM pose (book position → arm pose).
   Both are marked TODO. A real left-right sweep needs the 4th (yaw) joint.
5. **`io/light.py`.** Drive the LED once it's wired.
6. Mirror mode in STANDBY (TODO in `director.tick`), then the optimization rungs.

To change behavior, edit `machine.py`'s two tables and keep `docs/STATE_MACHINE.md` in sync.
Add a test in `tests/test_machine.py` for any new transition.
