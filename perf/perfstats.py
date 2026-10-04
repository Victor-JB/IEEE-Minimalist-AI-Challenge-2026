"""Lightweight timing and system stats to sprinkle into any code.

Off by default, and free when off: @timed returns the original function unchanged.
Turn it on with an environment variable; a report prints at exit (Ctrl+C included):

    PERF=1 python inference/main.py --headless

Usage:
    from perfstats import timed, count

    @timed("detector.preprocess")   # time every call of a function
    def _preprocess(self, frame): ...

    with timed("camera.read"):      # time a block
        frame = cam.read()

    count("frames.gated")           # count events, e.g. frames skipped by gating

Environment variables (all optional):
    PERF=1               enable
    PERF_CSV=run.csv     also save every timing and system sample (for plots / before-after)
    PERF_WATTS=3.1       average board power during the run, read from a USB power meter
    PERF_IDLE_WATTS=2.2  board power at idle, to also report energy above idle

On Linux a background thread samples CPU load, CPU clock, temperature, memory and
any on-board power sensor (hwmon) every 0.5 s. Elsewhere only timings are collected.
"""

import atexit
import csv
import functools
import glob
import os
import sys
import threading
import time
from collections import defaultdict, deque

ENABLED = os.environ.get("PERF", "0") not in ("", "0")
MAX_SAMPLES = 100_000  # per timer; oldest samples drop off after this

_start = time.perf_counter()
_timings = defaultdict(lambda: deque(maxlen=MAX_SAMPLES))  # name -> (t, seconds)
_counts = defaultdict(int)


class timed:
    """Decorator or context manager that records how long the wrapped code takes."""

    def __init__(self, name):
        self.name = name

    def __call__(self, func):
        if not ENABLED:
            return func
        record = _timings[self.name].append

        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            t0 = time.perf_counter()
            try:
                return func(*args, **kwargs)
            finally:
                t1 = time.perf_counter()
                record((t1 - _start, t1 - t0))

        return wrapper

    def __enter__(self):
        if ENABLED:
            self._t0 = time.perf_counter()
        return self

    def __exit__(self, *exc):
        if ENABLED:
            t1 = time.perf_counter()
            _timings[self.name].append((t1 - _start, t1 - self._t0))


def count(name, n=1):
    if ENABLED:
        _counts[name] += n


# --- system sampling (Linux) ---------------------------------------------------


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def _cpu_times():
    fields = [int(v) for v in _read("/proc/stat").splitlines()[0].split()[1:9]]
    idle = fields[3] + fields[4]  # idle + iowait
    return sum(fields), idle


def _mean(values):
    values = [v for v in values if v is not None]
    return sum(values) / len(values) if values else None


def _cpu_mhz():
    return _mean(int(_read(p)) / 1000 if _read(p) else None
                 for p in glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq"))


def _temp_c():
    temps = [int(_read(p)) / 1000 for p in glob.glob("/sys/class/thermal/thermal_zone*/temp") if _read(p)]
    return max(temps) if temps else None


def _rss_mb():
    for line in (_read("/proc/self/status") or "").splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1]) / 1024
    return None


POWER_SENSORS = glob.glob("/sys/class/hwmon/hwmon*/power*_input")  # microwatts, if the board has any


def _power_w():
    readings = [int(_read(p)) / 1e6 for p in POWER_SENSORS if _read(p)]
    return sum(readings) if readings else None


class SystemMonitor(threading.Thread):
    def __init__(self, period=0.5):
        super().__init__(daemon=True)
        self.period = period
        self.samples = []  # dicts: t, cpu_pct, cpu_mhz, temp_c, rss_mb, power_w

    def run(self):
        prev_total, prev_idle = _cpu_times()
        while True:
            time.sleep(self.period)
            total, idle = _cpu_times()
            busy = 1 - (idle - prev_idle) / max(total - prev_total, 1)
            prev_total, prev_idle = total, idle
            self.samples.append({
                "t": time.perf_counter() - _start,
                "cpu_pct": 100 * busy,
                "cpu_mhz": _cpu_mhz(),
                "temp_c": _temp_c(),
                "rss_mb": _rss_mb(),
                "power_w": _power_w(),
            })


monitor = None
if ENABLED and sys.platform.startswith("linux"):
    monitor = SystemMonitor()
    monitor.start()


# --- reporting ------------------------------------------------------------------


def _pct(sorted_values, p):
    return sorted_values[min(len(sorted_values) - 1, int(p / 100 * len(sorted_values)))]


def report(watts=None, idle_watts=None):
    wall = time.perf_counter() - _start
    lines = [f"\n=== perfstats: {wall:.1f} s ==="]

    lines.append(f"{'timer':<24}{'calls':>7}{'/s':>7}{'mean ms':>9}{'p50':>8}{'p90':>8}{'p99':>8}{'max':>8}")
    for name in sorted(_timings):
        ms = sorted(d * 1000 for _, d in _timings[name])
        if not ms:
            continue
        lines.append(f"{name:<24}{len(ms):>7}{len(ms) / wall:>7.1f}{sum(ms) / len(ms):>9.2f}"
                     f"{_pct(ms, 50):>8.2f}{_pct(ms, 90):>8.2f}{_pct(ms, 99):>8.2f}{ms[-1]:>8.2f}")
    for name in sorted(_counts):
        lines.append(f"count {name:<18}{_counts[name]:>7}{_counts[name] / wall:>7.1f}")

    measured_power = None
    if monitor and monitor.samples:
        s = monitor.samples

        def col(key, fn=_mean):
            values = [x[key] for x in s if x[key] is not None]
            return fn(values) if values else None

        parts = [
            f"cpu {col('cpu_pct'):.0f}% avg / {col('cpu_pct', max):.0f}% max",
            f"clock {col('cpu_mhz'):.0f} MHz avg" if col("cpu_mhz") else None,
            f"temp {col('temp_c', max):.1f} C max" if col("temp_c") else None,
            f"mem {col('rss_mb', max):.0f} MB max" if col("rss_mb") else None,
        ]
        measured_power = col("power_w")
        if measured_power:
            parts.append(f"on-board power {measured_power:.2f} W avg")
        lines.append("system: " + ", ".join(p for p in parts if p))

    watts = watts or measured_power
    if watts:
        lines.append(f"energy at {watts:.2f} W board power"
                     + (f" ({idle_watts:.2f} W idle)" if idle_watts else "") + ":")
        for name in sorted(n for n in _timings if _timings[n]):
            mean_s = sum(d for _, d in _timings[name]) / len(_timings[name])
            line = f"  {name:<22}{watts * mean_s * 1000:>9.1f} mJ/call"
            if idle_watts:
                line += f"   {(watts - idle_watts) * mean_s * 1000:>8.1f} mJ/call above idle"
            lines.append(line)
        lines.append("  (approximate: assumes average power during every stage; "
                     "perf/bench_detector.py gives a clean J/inference)")
    return "\n".join(lines)


def save_csv(path):
    """Long format: kind, name, t_s, value. One row per timing (ms) or system sample."""
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["kind", "name", "t_s", "value"])
        for name, samples in _timings.items():
            w.writerows(("timing_ms", name, f"{t:.4f}", f"{d * 1000:.3f}") for t, d in samples)
        for sample in monitor.samples if monitor else []:
            for key, value in sample.items():
                if key != "t" and value is not None:
                    w.writerow(("system", key, f"{sample['t']:.2f}", f"{value:.3f}"))


def _env_float(name):
    value = os.environ.get(name)
    return float(value) if value else None


def _at_exit():
    print(report(_env_float("PERF_WATTS"), _env_float("PERF_IDLE_WATTS")))
    if os.environ.get("PERF_CSV"):
        save_csv(os.environ["PERF_CSV"])
        print(f"saved {os.environ['PERF_CSV']}")


if ENABLED:
    atexit.register(_at_exit)
