# Lumen: an expressive edge-AI desk lamp

Team repo for the IEEE SEC 2026 Minimalist Edge AI Challenge, hardware track (Arduino UNO Q).

```
apps/lumen/   THE app: camera → AI → state machine → expressive motion. Runs at boot on the board.
              Start here: apps/lumen/README.md
arm/          The shared arm module: firmware, Python API, arm_config (one set of parameters),
              calibration, clip player and clips. Used by the app and by every arm tool.
docs/         STATE_MACHINE.md: the behavior spec the app implements
```

Bring-up on the board:

```sh
bash arm/scripts/deploy.sh          # flash the arm firmware + install the arm venv (once)
bash apps/lumen/scripts/deploy.sh   # install Lumen and its boot service
```

How boot works and why Lumen isn't an App Lab app: [apps/README.md](apps/README.md).

**Dev branch (`dev`).** The same code plus the test bench: `apps/stepper_desk` (Mac app and
scripts for driving, calibrating and testing the arm by hand, commissioning records),
`tools/` (MuJoCo sims, clip authoring, CAD arm model) and `apps/lumen/bench` (benchmarks).

```sh
python3 -m unittest discover -s apps/lumen/tests   # app logic + layout guards
bash arm/scripts/test.sh                           # firmware logic + arm API
```
