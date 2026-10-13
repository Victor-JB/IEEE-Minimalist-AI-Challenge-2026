# Demo measurement: the 5-minute scripted run

How to turn one scripted demo into the numbers judges want: time per state, inferences per minute
per model per state, average power per state, and energy per inference above idle. Everything
below runs from `apps/lumen/` on the board unless noted. Results go into `apps/lumen/bench/RESULTS.md`
(dev branch).

Tools: `tools/scorecard.py` (reads the logs, prints markdown), `tools/simulate.py` (the same demo
headless with a fake clock, for trying the pipeline without the board), `lumen/perfstats.py`
(`PERF=1`), and an inline USB-C power meter that logs to a PC (e.g. FNIRSI FNB58).

## Setup (once per session)

1. Meter inline between the charger and the board/hub. It measures board + hub + camera (+ the
   lamp LED if it shares the supply): say so in the notes column.
2. Sync clocks: run `date` on the board and check the meter PC's clock. If they differ, note the
   difference and pass it as `--power-offset` (seconds to add to the meter's timestamps).
3. Start a fresh transitions file, so the scorecard only sees this run:
   `mv logs/transitions-$(date +%Y%m%d).csv logs/before-run-$(date +%H%M).csv 2>/dev/null`
4. Stop the boot service so only the measured run owns the camera and arm:
   `systemctl --user stop lumen`
5. Desk: book off the desk, within reach. Seat about 0.6–1 m in front of the lamp.

## Idle baseline (60 s)

Board booted, camera plugged in, Lumen stopped, nobody moving in front of it. Log the meter for
60 s and take the average: that is `--idle-watts` (and the `idle W` column). Repeat it at the end
of the session; if the two differ by more than ~0.1 W (thermal drift), use the mean and say so.

## The run (5:00)

Start the meter log, then start Lumen with perfstats on:

```sh
PERF=1 PERF_CSV=logs/perf-run.csv PERF_IDLE_WATTS=<idle W> python -m lumen --headless
```

Stay out of view while it boots. The operator calls the times; the person does this:

| time | person does | expected path |
|---|---|---|
| 0:00 | out of view | BOOT → IDLE (gated: face model runs only when something moves) |
| 0:30 | walk up, sit, look at the lamp | IDLE → NOTICE → STANDBY |
| 0:50 | point (index finger, near the face) for 1 s; no book on the desk | ACK → SEARCH → NOT FOUND → STANDBY |
| 1:10 | put the book on the desk, in the sweep area; point again | ACK → SEARCH → AIM → ILLUMINATE/HOLD |
| 1:30 | thumbs up over the book for about 0.5 s | HOLD → ADJUST → HOLD (closer) |
| 1:45 | thumbs down for about 0.5 s | HOLD → ADJUST → HOLD (further) |
| 1:50 | read, head down, hands still | HOLD, CHECK-IN every 8–15 s |
| 3:00 | cover the book with a hand for about 4 s (until the lamp peeks around), then uncover | HOLD → REACQUIRE → HOLD |
| 3:10 | read | HOLD / CHECK-IN |
| 4:00 | open palm over the book, held 1.5 s | GOODBYE → STANDBY |
| 4:20 | stand up and walk out of view | STANDBY → LOST (4 s) → IDLE (10 s total) |
| 5:00 | operator: Ctrl+C (perfstats prints its report and saves the CSV); stop the meter log | |

Keep thumbs-up / thumbs-down short: they repeat every second while held. Optional deep-idle
extension: stay away 5 more minutes (IDLE → SLEEP at 5 min with no motion), then wave once
(SLEEP → IDLE) and stop. Faults are not part of the measured run; demo them separately.

Rehearse with no meter first, and check the console's transition lines match the table. The
same script runs headless in `tools/simulate.py` (`SCENARIO`), so a changed protocol should be
changed there too.

## From logs to RESULTS rows

Copy `logs/transitions-<date>.csv`, `logs/perf-run.csv` and the meter export to the Mac. The
meter CSV needs a time column (epoch, ISO date-time or HH:MM:SS, local time) and a watts column,
or volts and amps; rename headers if the meter's software uses other names.

```sh
python3 apps/lumen/tools/scorecard.py transitions-20261012.csv \
    --perf perf-run.csv --power meter.csv --idle-watts 2.1 \
    --change "baseline: behavior-linked rates + motion gate" > card.md
```

| RESULTS.md section | where the numbers come from |
|---|---|
| Sandboxed benchmark | `python bench/bench_detector.py --threads 4 --watts <W> --idle-watts <idle>` (prints its own row) |
| Full system | the scorecard's "Row for RESULTS.md → Full system" (loop fps and camera.read from `--perf`, detect ms = `detector.total`, CPU %, meter average W, idle W). Also printed by perfstats at Ctrl+C. |
| Behavior-linked compute (new section) | paste the scorecard's two tables: time and inferences/min per state, and W / mJ per inference per state |
| Accuracy | not from this run (fixed eval set, `eval_detector.py`, TODO) |

Before/after: run the same protocol with the change and with its baseline (e.g. a branch where
every state runs face 10 Hz + hand 10 Hz + book 3 Hz, gate off), and add one scorecard per run.
Never overwrite a row.

## Reading the numbers honestly

- Rates are measured, not configured: the configured Hz × 60 is shown in brackets. The model
  scheduler can't run faster than every other frame for rates near the camera fps (10 Hz at
  15 fps gives 7.5 Hz), so measured is lower than configured; that is real.
- mJ / inference per state charges all of the state's power above idle (camera, motion, light)
  to its inferences: an upper bound. The clean per-model figure comes from `bench_detector.py`.
- Short states (NOTICE, ADJUST: under a second) may get no meter samples at 1 Hz. Log the meter
  at 10 Hz or more if you want them.
- Known issue: the logged face count in IDLE includes frames the motion gate skipped (the
  scheduler counts before the gate check in `perception/__init__.py`; see the expected failure in
  `tests/test_simulate.py`). Until that is fixed, IDLE face inferences/min are overstated and the
  gating savings don't show; the simulator prints the real detector calls next to the logged ones.
- The state that is open when the log ends (IDLE after walking away) is not counted, nor is BOOT
  before the first transition. Pass `--end-t <t>` to count the last state's time.

## Try the pipeline without the board

```sh
python3 apps/lumen/tools/simulate.py --csv /tmp/sim.csv --power-csv /tmp/sim_power.csv
python3 apps/lumen/tools/scorecard.py /tmp/sim.csv --power /tmp/sim_power.csv --idle-watts 2.0 --end-t 520
```

The simulated power trace is a toy model: never put its numbers in RESULTS.md.
