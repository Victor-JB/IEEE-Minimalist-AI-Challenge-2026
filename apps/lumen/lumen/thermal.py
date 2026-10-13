"""OVERTEMP: board temperature with hysteresis, sampled at about 1 Hz by the motion worker.

Sensors are the Linux thermal zones (/sys/class/thermal/thermal_zone*/temp) and hwmon temperature
inputs (/sys/class/hwmon/hwmon*/temp*_input), both in millidegrees C; the hottest one counts.
On a Mac there are none, so the monitor is disabled and never trips. The stepper drivers are plain
STEP/DIR drivers: the firmware has no temperature reading for them, so this is SoC/board only.

Trips at C.OVERTEMP_C; stays hot until every sensor is below C.OVERTEMP_CLEAR_C, so FAULT's
cooldown waits for the board to really cool down before BOOT.
"""
import glob
import time
from pathlib import Path

from . import config as C


class Overtemp(RuntimeError):
    """The board is too hot to keep the motors powered."""


def _read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def _millideg(path):
    def read():
        raw = _read(path)
        if raw is None or not raw.lstrip("-").isdigit():
            return None
        t = int(raw) / 1000
        return t if -40 < t < 200 else None           # unplugged / bogus sensors report nonsense
    return read


def discover(root="/sys/class"):
    """{sensor name: callable -> deg C or None}. Empty where sysfs has none (macOS)."""
    sensors = {}

    def add(name, read):
        key, n = name, 2
        while key in sensors:
            key, n = f"{name}#{n}", n + 1
        sensors[key] = read

    for path in sorted(glob.glob(f"{root}/thermal/thermal_zone*/temp")):
        zone = Path(path).parent
        add(_read(zone / "type") or zone.name, _millideg(path))
    for path in sorted(glob.glob(f"{root}/hwmon/hwmon*/temp*_input")):
        dev = Path(path).parent
        label = _read(path[:-len("_input")] + "_label")
        add((_read(dev / "name") or dev.name) + (f"/{label}" if label else ""), _millideg(path))
    return sensors


class ThermalMonitor:
    def __init__(self, sensors=None, high=None, low=None, period=None, clock=time.monotonic):
        self.sensors = discover() if sensors is None else dict(sensors)
        self.high = C.OVERTEMP_C if high is None else high
        self.low = C.OVERTEMP_CLEAR_C if low is None else low
        self.period = C.OVERTEMP_CHECK_S if period is None else period
        self.clock = clock
        self.hot, self.reading, self.reason, self._last = False, None, None, None

    @property
    def enabled(self):
        return bool(self.sensors)

    def poll(self):
        """Sample at most every `period` s. Returns True on the sample that trips (once)."""
        if not self.sensors:
            return False
        now = self.clock()
        if self._last is not None and now - self._last < self.period:
            return False
        self._last = now
        temps = {}
        for name, read in self.sensors.items():
            try:
                t = read()
            except Exception:                     # a sensor that vanished is not an overtemp
                t = None
            if t is not None:
                temps[name] = t
        if not temps:
            return False
        name, t = max(temps.items(), key=lambda kv: kv[1])
        self.reading = (name, t)
        if not self.hot and t >= self.high:
            self.hot = True
            self.reason = f"{name} {t:.1f} C >= {self.high:.0f} C (cools below {self.low:.0f} C)"
            return True
        if self.hot and t < self.low:
            self.hot = False
        return False
