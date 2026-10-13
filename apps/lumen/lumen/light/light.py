"""Lamp LED: eased, time-based ramps and a perceptual brightness scale, behind one backend interface.

    light = Light(make_backend(light_cfg.LIGHT_BACKEND, worker))
    light.set(1.0)              # ramp to full over LIGHT_RAMP_S (ease in/out)
    light.step(-0.15)           # ADJUST: one brightness step
    light.couple(0.7)           # LightCoupling: dim to 70 % of the set level (CHECK-IN), 1.0 restores
    light.update(now)           # every main-loop frame: advances the ramp, writes the backend

Levels (`level`, `target`) are perceptual, 0-1: what the HUD shows and what states ask for. The
backend gets a duty cycle, duty = level ** LIGHT_GAMMA, because perceived brightness is roughly
the duty to the power 1/2.2: a linear duty ramp would jump at the bottom and crawl at the top.

Backends (env LUMEN_LIGHT, see light/config.py). Each has write(duty 0-1, final) and close():

- "sim" (default)  SimBackend: remembers the duty and prints it when a ramp ends. No hardware.
- "firmware"       FirmwareBackend: `LIGHT <0-1000> <fade ms>` to the STM32, which drives a MOSFET
                   on D7 (arm/README.md "Lamp LED" for the pin choice and wiring; the firmware
                   must be built with LIGHT_ENABLED 1). The motion worker thread owns the arm
                   session, so this only hands the worker a per-mille target; the worker sends it
                   at most every LIGHT_FIRMWARE_EVERY_S, faded over that interval on the MCU.
                   On firmware without the lamp it is a no-op.
- "linux-pwm"      LinuxPwmBackend: sysfs PWM on the QRB2210 (Linux) side,
                   /sys/class/pwm/pwmchip<LUMEN_LIGHT_PWMCHIP>/pwm<LUMEN_LIGHT_PWM_CHANNEL>.
- "gpio"           GpioBackend: on/off only, via libgpiod (python3-gpiod), line
                   LUMEN_LIGHT_GPIO_LINE on LUMEN_LIGHT_GPIOCHIP. Use it if no PWM is reachable.

Finding a Linux PWM/GPIO on the UNO Q (not checked on hardware: the board is down). The UNO-style
header pins D0-D21/A0-A5 belong to the STM32, not to Linux, so the Linux options need a QRB2210
pin, which on the UNO Q means the bottom high-density connectors (JMISC / JMEDIA; check the UNO Q
pinout/schematic for which pins are GPIO or PWM-capable and their voltage: 1.8 V logic is common
on the QRB side, so the MOSFET must switch fully at that gate voltage or need a level shifter).
On the board:
    ls /sys/class/pwm/                      # pwmchipN present? cat pwmchipN/npwm = channels
    sudo cat /sys/kernel/debug/pwm          # which chip/channel is which device-tree node
    gpiodetect; gpioinfo                    # GPIO chips and line names/users (libgpiod tools)
If no pwmchip shows up, the PWM block isn't enabled in the device tree (needs an overlay), so use
"gpio" (on/off) or the firmware backend. sysfs needs write access: run as root or add a udev rule.
If any backend fails to open, Lumen prints why and falls back to "sim", so the lamp app still runs.
"""
import time
from pathlib import Path

from . import config as light_cfg


def ease(s):
    """Smoothstep ease in/out on 0-1: zero slope at both ends, so ramps start and land softly."""
    s = max(0.0, min(1.0, s))
    return s * s * (3.0 - 2.0 * s)


def to_duty(level, gamma=light_cfg.LIGHT_GAMMA):
    """Perceptual level (0-1) -> PWM duty (0-1)."""
    return max(0.0, min(1.0, level)) ** gamma


def clamp01(x):
    return max(0.0, min(1.0, float(x)))


class Light:
    def __init__(self, backend=None, clock=time.monotonic, gamma=light_cfg.LIGHT_GAMMA):
        self.backend = backend if backend is not None else SimBackend()
        self.clock, self.gamma = clock, gamma
        self.level = 0.0                 # perceptual level right now (HUD)
        self.target = 0.0                # what the state / ADJUST asked for
        self.scale = 1.0                 # LightCoupling multiplier (CHECK-IN dims to 0.7)
        self._from, self._t0, self._dur = 0.0, 0.0, 0.0
        self._written = None

    @property
    def goal(self):
        """Where the ramp ends: the set level times the coupling scale."""
        return self.target * self.scale

    @property
    def duty(self):
        return to_duty(self.level, self.gamma)

    @property
    def ramping(self):
        return self.level != self.goal

    def set(self, level, ramp_s=light_cfg.LIGHT_RAMP_S, now=None):
        now = self._advance(now)
        self.target = clamp01(level)
        self._retarget(ramp_s, now)

    def step(self, delta, ramp_s=light_cfg.LIGHT_STEP_RAMP_S, now=None):
        """ADJUST: brighter (+) / dimmer (-), from the set level (not mid-ramp), clamped."""
        self.set(self.target + delta, ramp_s, now)

    def couple(self, scale, ramp_s=light_cfg.LIGHT_COUPLING_RAMP_S, now=None):
        """LightCoupling: scale the set level (0.7 = dim to 70 %), 1.0 restores it."""
        now = self._advance(now)
        self.scale = clamp01(scale)
        self._retarget(ramp_s, now)

    def _advance(self, now):
        """Bring the old ramp up to now, so a new one starts from the real level: no jump."""
        now = self.clock() if now is None else now
        self.update(now)
        return now

    def _retarget(self, ramp_s, now):
        self._from, self._t0, self._dur = self.level, now, max(0.0, ramp_s)
        if self._dur == 0.0:
            self.level = self.goal
        self._write(final=not self.ramping)

    def update(self, now=None):
        """Advance the ramp to `now` and write the backend if the duty changed. Returns the level."""
        now = self.clock() if now is None else now
        if self.ramping:
            s = 1.0 if self._dur <= 0 else (now - self._t0) / self._dur
            if s >= 1.0:
                self.level = self.goal
            else:
                self.level = self._from + (self.goal - self._from) * ease(s)
        self._write(final=not self.ramping)
        return self.level

    def _write(self, final):
        duty = self.duty
        if duty != self._written:
            self._written = duty
            self.backend.write(duty, final)

    def close(self):
        self.backend.close()


# --- backends -----------------------------------------------------------------------------------
class SimBackend:
    """No hardware: remembers the duty, prints it when a ramp lands (not every frame)."""
    name = "sim"

    def __init__(self, log=print):
        self.duty, self.log = 0.0, log

    def write(self, duty, final=False):
        self.duty = duty
        if final and self.log:
            self.log(f"Light (sim): duty {duty:.1%} (level {duty ** (1 / light_cfg.LIGHT_GAMMA):.0%})")

    def close(self):
        pass


class FirmwareBackend:
    """STM32 PWM via the motion worker (it owns the arm session; see MotionWorker.set_light)."""
    name = "firmware"

    def __init__(self, worker):
        self.worker = worker
        if getattr(worker, "fake", False):
            print("Light: firmware backend with a fake arm: nothing reaches a lamp")

    @staticmethod
    def permille(duty):
        return max(0, min(1000, round(duty * 1000)))

    def write(self, duty, final=False):
        self.worker.set_light(self.permille(duty))

    def close(self):
        self.worker.set_light(0)


class LinuxPwmBackend:
    """sysfs PWM: /sys/class/pwm/pwmchipN/pwmM/{period,duty_cycle,enable} (nanoseconds)."""
    name = "linux-pwm"

    def __init__(self, chip=light_cfg.LIGHT_PWM_CHIP, channel=light_cfg.LIGHT_PWM_CHANNEL, hz=light_cfg.LIGHT_PWM_HZ,
                 root="/sys/class/pwm", export_wait_s=1.0):
        base = Path(root) / f"pwmchip{chip}"
        if not base.is_dir():
            raise FileNotFoundError(f"{base} not found (ls {root}; see light/light.py)")
        self.dir = base / f"pwm{channel}"
        if not self.dir.is_dir():
            (base / "export").write_text(str(channel))
            end = time.monotonic() + export_wait_s      # udev creates the files asynchronously
            while not self.dir.is_dir() and time.monotonic() < end:
                time.sleep(0.02)
            if not self.dir.is_dir():
                raise FileNotFoundError(f"export of channel {channel} on {base} made no {self.dir.name}")
        if not 1 <= hz <= 1_000_000:
            raise ValueError("PWM frequency out of range")
        self.period = round(1e9 / hz)
        self._ns = None
        self._put("duty_cycle", 0)                  # duty <= period at every step
        self._put("period", self.period)
        self.write(0.0)
        self._put("enable", 1)

    def _put(self, name, value):
        (self.dir / name).write_text(str(value))

    def write(self, duty, final=False):
        ns = round(self.period * max(0.0, min(1.0, duty)))
        if ns != self._ns:
            self._put("duty_cycle", ns)
            self._ns = ns

    def close(self):
        try:
            self.write(0.0)
            self._put("enable", 0)
        except OSError:
            pass


class GpioBackend:
    """On/off through libgpiod (v2 API, v1 fallback): on at duty >= threshold."""
    name = "gpio"

    def __init__(self, chip=light_cfg.LIGHT_GPIO_CHIP, line=light_cfg.LIGHT_GPIO_LINE, threshold=0.05):
        import gpiod                                  # python3-gpiod on the board
        self.threshold, self.line, self.on = threshold, line, None
        if hasattr(gpiod, "request_lines"):           # libgpiod >= 2
            from gpiod.line import Direction, Value
            self._values = (Value.INACTIVE, Value.ACTIVE)
            self._req = gpiod.request_lines(chip, consumer="lumen-light", config={
                line: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)})
            self._set = lambda on: self._req.set_value(line, self._values[on])
        else:                                         # libgpiod 1.x
            self._req = gpiod.Chip(chip).get_line(line)
            self._req.request(consumer="lumen-light", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
            self._set = lambda on: self._req.set_value(int(on))

    def write(self, duty, final=False):
        on = duty >= self.threshold
        if on != self.on:
            self._set(on)
            self.on = on

    def close(self):
        try:
            self.write(0.0)
            self._req.release()
        except Exception:
            pass


def make_backend(name=None, worker=None, log=print):
    """Backend by name ("sim", "firmware", "linux-pwm", "gpio"); falls back to sim on any error."""
    name = (name or light_cfg.LIGHT_BACKEND).strip().lower()
    try:
        if name == "sim":
            return SimBackend(log)
        if name == "firmware":
            if worker is None:
                raise ValueError("the firmware backend needs the motion worker")
            return FirmwareBackend(worker)
        if name in ("linux-pwm", "pwm", "sysfs"):
            return LinuxPwmBackend()
        if name in ("gpio", "gpiod"):
            return GpioBackend()
        raise ValueError(f"unknown light backend {name!r} (sim, firmware, linux-pwm, gpio)")
    except Exception as e:
        log(f"Light: {name} backend unavailable ({e}); using sim")
        return SimBackend(log)
