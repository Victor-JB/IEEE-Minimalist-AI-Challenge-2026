# luxoiii

The **main app is the live web experience at http://127.0.0.1:8082/**: Big Caslon interface,
full X-ray model, camera, two-hand gestures, digital personality, real-head subject tracking,
and automatic physical lamp On/Off.

## Run the main app

From the repository root:

```sh
bash apps/luxoiii/scripts/run.sh
# Equivalent:
PYTHONPATH=apps/luxoiii python3 -m luxoiii
```

On the Mac it uses the existing observer tunnel at `http://127.0.0.1:18083`. On the board it
uses `http://127.0.0.1:8083`. Override with `--observer-url URL` or `LUXOIII_OBSERVER_URL`.
The web server binds only to loopback. See [observer setup](../../tools/spatial/README.md)
for the camera/AI process and the SSH tunnel.

```sh
PYTHONPATH=apps/luxoiii python3 -m luxoiii --offline  # model only; no physical outputs
```

The earlier `python -m luxoiii.preview` and `python3 sim/lamp_show.py` commands remain
compatible offline viewers. Supplying `--observer-url` connects them to live data.

## Physical behavior

D7 is **digital On/Off**, with no PWM or dimming. It switches fully on after one clear second
without detected faces, only while the entire estimated physical beam points below horizontal.
A face, unsuitable angle, stale camera or encoder reading, Stop, or expired connection switches
it off. The board enforces an 800 ms light lease independently of the web app.

The physical swivel tracks subjects within ±45°. The failed stepper joints stay asleep;
the full simulated arm performs greetings, curiosity, idle loops and pointing responses.
Tracking and gestures start by default with fresh data, unless explicitly stopped or disabled.
The simulated desk light is independent of D7 and retains its visual brightness and spotlight.

## Board deployment

The web app uses two supporting services: `luxoiii-desk` supplies the camera stream on 8081,
and `luxoiii-observer` runs camera AI plus the working swivel and lamp on 8083.
`luxoiii.service` starts both dependencies and serves the main app on loopback port 8082.
The manual desk is a separate diagnostic tool; do not enable full-arm movement while the
steppers are broken.

```sh
LUXOIII_NO_START=1 bash apps/luxoiii/scripts/deploy.sh  # install without changing running services
bash apps/luxoiii/scripts/deploy.sh                    # install and start the service chain
```

Before migrating a board running a standalone observer (including one from `/tmp`), stop that
observer and preserve local board edits; only one process may listen on 8083. App deployment
copies the app, shared arm code and observer, but does not flash the MCU. Keep the verified D7
firmware; its source and flashing instructions are in [arm/](../../arm/README.md).
The current Mac app can continue using an existing legacy board observer during migration.

## Older full-arm controller

The earlier camera/HUD and book-search state machine is available explicitly as:

```sh
PYTHONPATH=apps/luxoiii python3 -m luxoiii.legacy --fake-arm --stream 8080
```

It is not the main app or the default boot service. Its physical mode requires working joints;
book detection remains unimplemented. The live 8082 app selects virtual desk spots instead.

## Verify

```sh
python3 -m unittest discover -s apps/luxoiii/tests
bash arm/scripts/test.sh
node apps/luxoiii/tests/test_emotion_view.cjs
node apps/luxoiii/tests/test_desk_light.cjs
```

`luxoiii/paths.py` centralizes repository paths. Arm geometry and calibration come from `arm/`.
Tests use simulated hardware; real inference checks require the board and its installed models.

Measured model optimization and public dataset validation: [efficiency scorecard](../../reports/efficiency/README.md).
