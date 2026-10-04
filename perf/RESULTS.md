# Results log

Board: Arduino UNO Q (QRB2210, 4x Cortex-A53). Add a row for every change; never overwrite one.
Power is measured with an inline USB-C meter (board + hub + camera) unless noted.

## Sandboxed benchmark (`perf/bench_detector.py`)

| date | change | model | threads | pre ms | invoke p50 / p90 ms | post ms | total p50 / p90 ms | inf/s | W | mJ/inf | mJ/inf above idle |
|---|---|---|---|---|---|---|---|---|---|---|---|
| | baseline | face_det_lite (w8a8, 640x480) | 4 | | | | | | | | |

## Full system (`PERF=1 inference/main.py --headless`)

| date | change | loop fps | detect ms (mean) | camera.read ms | CPU % | avg W | idle W | notes |
|---|---|---|---|---|---|---|---|---|
| 2026-10-03 | baseline (pre-perfstats log) | 13.3 | ~70 | | | | | `infer_out_on_arduino.log`; camera frames wider than 640 |

## Accuracy (fixed eval set)

| date | change | recall @ IoU 0.5 | false positives / frame | notes |
|---|---|---|---|---|
| | baseline | | | |
