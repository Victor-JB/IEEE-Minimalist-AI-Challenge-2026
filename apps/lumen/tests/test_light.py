"""Lamp light: eased ramps, gamma, LightCoupling in CHECK-IN, backend selection, the firmware path
through the motion worker, and the sysfs PWM backend on a fake sysfs tree. No hardware."""
import os
import random
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen.light import config as light_cfg
from lumen.movement import config as move_cfg
from lumen import events as E  # noqa: E402
from lumen.state.director import Director  # noqa: E402
from lumen.movement.expressive import Expressive  # noqa: E402
from lumen.light import light as L  # noqa: E402
from lumen.state.machine import Machine  # noqa: E402
from lumen.movement import worker as W  # noqa: E402


class Recorder:
    """Backend that keeps every write."""
    def __init__(self):
        self.writes, self.closed = [], False

    def write(self, duty, final=False):
        self.writes.append((duty, final))

    def close(self):
        self.closed = True


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make_light():
    clock, rec = Clock(), Recorder()
    return L.Light(rec, clock=clock), clock, rec


class Ramps(unittest.TestCase):
    def test_ramp_takes_the_ramp_time_and_eases_in_and_out(self):
        light, clock, rec = make_light()
        light.set(1.0, ramp_s=0.5)
        self.assertEqual(light.level, 0.0)                  # no jump
        levels = []
        for k in range(1, 11):
            clock.t = 100.0 + 0.05 * k
            levels.append(light.update())
        self.assertAlmostEqual(levels[4], 0.5)              # halfway at half time (symmetric ease)
        self.assertLess(levels[0], 0.05)                    # eases in: slow start...
        self.assertGreater(levels[8], 0.97)                 # ...and eases out: slow landing
        steps = [b - a for a, b in zip([0.0] + levels, levels)]
        self.assertLess(steps[0], steps[4])
        self.assertLess(steps[-1], steps[4])
        self.assertTrue(all(s >= 0 for s in steps))         # monotonic, no overshoot
        self.assertEqual(light.level, 1.0)
        self.assertFalse(light.ramping)
        self.assertEqual(rec.writes[-1], (1.0, True))       # final write flagged
        self.assertTrue(all(not f for _, f in rec.writes[1:-1]))

    def test_not_done_before_the_ramp_time(self):
        light, clock, _ = make_light()
        light.set(1.0, ramp_s=0.5)
        clock.t += 0.49
        self.assertLess(light.update(), 1.0)
        clock.t += 0.02
        self.assertEqual(light.update(), 1.0)

    def test_retarget_mid_ramp_starts_from_the_current_level(self):
        light, clock, _ = make_light()
        light.set(1.0, ramp_s=0.5)
        clock.t += 0.25
        mid = light.update()
        light.set(0.0, ramp_s=0.5)
        self.assertAlmostEqual(light.level, mid)            # continuous
        clock.t += 0.5
        self.assertEqual(light.update(), 0.0)

    def test_zero_ramp_jumps_and_clamps(self):
        light, _, _ = make_light()
        light.set(3.0, ramp_s=0)
        self.assertEqual((light.level, light.target), (1.0, 1.0))
        light.set(-1, ramp_s=0)
        self.assertEqual(light.level, 0.0)

    def test_step_clamps_and_uses_the_set_level(self):
        light, clock, _ = make_light()
        light.set(0.5, ramp_s=0)
        light.step(0.15)
        self.assertAlmostEqual(light.target, 0.65)
        clock.t += light_cfg.LIGHT_STEP_RAMP_S
        self.assertAlmostEqual(light.update(), 0.65)
        for _ in range(5):
            light.step(0.15)
        self.assertEqual(light.target, 1.0)
        for _ in range(10):
            light.step(-0.15)
        self.assertEqual(light.target, 0.0)

    def test_update_writes_only_on_change(self):
        light, clock, rec = make_light()
        light.update()
        n = len(rec.writes)
        for _ in range(5):
            clock.t += 0.1
            light.update()
        self.assertEqual(len(rec.writes), n)


class Gamma(unittest.TestCase):
    def test_perceptual_mapping(self):
        self.assertEqual(L.to_duty(0.0), 0.0)
        self.assertEqual(L.to_duty(1.0), 1.0)
        self.assertAlmostEqual(L.to_duty(0.5), 0.5 ** 2.2)  # half brightness is ~22 % duty
        self.assertLess(L.to_duty(0.5), 0.25)
        self.assertEqual(L.to_duty(2.0), 1.0)
        self.assertEqual(L.to_duty(-1.0), 0.0)
        duties = [L.to_duty(k / 20) for k in range(21)]
        self.assertEqual(duties, sorted(duties))

    def test_backend_gets_duty_hud_gets_level(self):
        light, _, rec = make_light()
        light.set(0.7, ramp_s=0)
        self.assertAlmostEqual(light.level, 0.7)
        self.assertAlmostEqual(rec.writes[-1][0], 0.7 ** light_cfg.LIGHT_GAMMA)


class FakeWorker:
    """Director's view of MotionWorker: every command is a no-op."""
    fake = True

    def __init__(self):
        self.current = list(move_cfg.HOME)          # data, not commands: AIM/ADJUST plan from these
        self.base_roll = 0.0

    def __getattr__(self, name):
        return lambda *a, **k: None

    def poll_events(self):
        return []


class Coupling(unittest.TestCase):
    def setUp(self):
        self.light, self.clock, _ = make_light()
        self.d = Director(FakeWorker(), self.light, Expressive(random.Random(1)), rng=random.Random(2))
        self.m = Machine(on_enter=self.d.on_enter, on_exit=self.d.on_exit, now=0.0, rng=random.Random(3))
        for ev in (E.READY, E.FACE_SEEN, E.FACE_CONFIRMED, E.POINT, E.CLIP_DONE, E.BOOK_FOUND, E.AT_POSE):
            self.m.handle(ev, 0.0)
        self.assertEqual(self.m.state, "HOLD")

    def settle(self):
        self.clock.t += 1.0
        return self.light.update()

    def test_lit_in_illuminate(self):
        self.assertEqual(self.light.target, 1.0)
        self.assertEqual(self.settle(), 1.0)

    def test_checkin_dims_to_70_percent_and_restores(self):
        self.settle()
        self.m.handle(E.CHECKIN_DUE, 1.0)
        self.assertEqual(self.m.state, "CHECKIN")
        self.assertAlmostEqual(self.settle(), light_cfg.LIGHT_CHECKIN_SCALE)
        self.assertEqual(self.light.target, 1.0)                 # the set level is untouched
        self.m.handle(E.CLIP_DONE, 2.0)
        self.assertEqual(self.m.state, "HOLD")
        self.assertEqual(self.settle(), 1.0)

    def test_adjust_during_and_after_coupling(self):
        self.m.handle(E.FURTHER, 1.0)
        self.assertEqual(self.m.state, "ADJUST")
        self.assertAlmostEqual(self.light.target, 0.85)
        self.m.handle(E.MOVE_DONE, 1.5)
        self.m.handle(E.CHECKIN_DUE, 2.0)
        self.assertAlmostEqual(self.settle(), 0.85 * light_cfg.LIGHT_CHECKIN_SCALE)
        self.m.handle(E.CLIP_DONE, 3.0)
        self.assertAlmostEqual(self.settle(), 0.85)

    def test_fault_during_checkin_dims_without_staying_coupled(self):
        self.m.handle(E.CHECKIN_DUE, 1.0)
        self.m.handle(E.FAULT, 1.1)
        self.assertEqual(self.m.state, "FAULT")
        self.assertEqual(self.light.scale, 1.0)
        self.assertAlmostEqual(self.settle(), 0.2)

    def test_goodbye_ramps_off(self):
        self.settle()
        self.m.handle(E.PALM, 1.0)
        self.assertEqual(self.m.state, "GOODBYE")
        self.assertEqual(self.settle(), 0.0)


class Selection(unittest.TestCase):
    def quiet(self):
        self.logged = []
        return self.logged.append

    def test_default_is_sim(self):
        if "LUMEN_LIGHT" not in os.environ:
            self.assertEqual(light_cfg.LIGHT_BACKEND, "sim")
        self.assertIsInstance(L.make_backend("sim", log=self.quiet()), L.SimBackend)
        self.assertIsInstance(L.Light().backend, L.SimBackend)

    def test_firmware_needs_a_worker(self):
        w = W.MotionWorker(Expressive(random.Random(1)), fake=True)
        self.assertIsInstance(L.make_backend("firmware", w, log=self.quiet()), L.FirmwareBackend)
        self.assertIsInstance(L.make_backend("firmware", None, log=self.quiet()), L.SimBackend)

    def test_unknown_or_unavailable_falls_back_to_sim(self):
        self.assertIsInstance(L.make_backend("laser", log=self.quiet()), L.SimBackend)
        self.assertIn("laser", self.logged[-1])
        # No /sys/class/pwm/pwmchip0 on a Mac (or no permission): falls back, does not crash.
        if not Path("/sys/class/pwm/pwmchip0").exists():
            self.assertIsInstance(L.make_backend("linux-pwm", log=self.quiet()), L.SimBackend)

    def test_sim_logs_only_when_a_ramp_lands(self):
        logs = []
        light, clock, _ = make_light()
        light.update()                                  # the start-up "off" write
        light.backend = L.SimBackend(log=logs.append)
        light.set(1.0, ramp_s=0.5)
        for _ in range(10):
            clock.t += 0.06
            light.update()
        self.assertEqual(len(logs), 1)
        self.assertIn("100", logs[0])


class FirmwarePath(unittest.TestCase):
    class Arm:
        def __init__(self):
            self.sent = []

        def light(self, level, fade_ms=0):
            self.sent.append(f"LIGHT {level} {fade_ms}")

        def stream_point(self, pose, vel=None, horizon=0.1):
            return {"state": "streaming"}

    def setUp(self):
        self.w = W.MotionWorker(Expressive(random.Random(1)), fake=True)
        self.w.arm = self.Arm()

    def test_backend_formats_per_mille(self):
        self.assertEqual(L.FirmwareBackend.permille(0.0), 0)
        self.assertEqual(L.FirmwareBackend.permille(0.5 ** 2.2), 218)
        self.assertEqual(L.FirmwareBackend.permille(1.0), 1000)
        self.assertEqual(L.FirmwareBackend.permille(1.2), 1000)
        b = L.FirmwareBackend(self.w)
        b.write(0.4567)
        self.assertEqual(self.w.light_cmd, 457)
        b.close()
        self.assertEqual(self.w.light_cmd, 0)

    def test_worker_sends_changes_rate_limited_with_a_fade(self):
        fade = round(light_cfg.LIGHT_FIRMWARE_EVERY_S * 1000)
        self.w.set_light(300)
        self.w._send([0.0, 60.0, 5.0])
        self.w.set_light(400)
        self.w._send([0.0, 60.0, 5.0])                 # too soon: held back
        self.assertEqual(self.w.arm.sent, [f"LIGHT 300 {fade}"])
        self.w.light_at -= light_cfg.LIGHT_FIRMWARE_EVERY_S
        self.w._send([0.0, 60.0, 5.0])
        self.w.light_at -= light_cfg.LIGHT_FIRMWARE_EVERY_S
        self.w._send([0.0, 60.0, 5.0])                 # unchanged: nothing sent
        self.assertEqual(self.w.arm.sent, [f"LIGHT 300 {fade}", f"LIGHT 400 {fade}"])

    def test_resent_after_reconnect_and_skipped_without_an_arm(self):
        self.w.set_light(250)
        self.w._light_tick()
        self.w.light_sent, self.w.light_at = None, -1e9        # what _connect does
        self.w._light_tick()
        self.assertEqual(len(self.w.arm.sent), 2)
        self.w.arm = None
        self.w.set_light(900)
        self.w._light_tick()                                   # fake arm / disconnected: no error

    def test_arm_without_light_support_is_fine(self):
        class Old:
            def stream_point(self, pose, vel=None, horizon=0.1):
                return {"state": "streaming"}
        self.w.arm = Old()
        self.w.set_light(500)
        self.w._send([0.0, 60.0, 5.0])


class LinuxPwm(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        chip = self.root / "pwmchip2"
        (chip / "pwm1").mkdir(parents=True)
        for name in ("period", "duty_cycle", "enable"):
            (chip / "pwm1" / name).write_text("0")
        (chip / "export").write_text("")

    def tearDown(self):
        self.tmp.cleanup()

    def read(self, name):
        return (self.root / "pwmchip2" / "pwm1" / name).read_text()

    def test_writes_period_duty_enable_in_nanoseconds(self):
        b = L.LinuxPwmBackend(chip=2, channel=1, hz=20000, root=self.root)
        self.assertEqual((self.read("period"), self.read("duty_cycle"), self.read("enable")),
                         ("50000", "0", "1"))
        b.write(0.25)
        self.assertEqual(self.read("duty_cycle"), "12500")
        b.write(2.0)
        self.assertEqual(self.read("duty_cycle"), "50000")
        b.close()
        self.assertEqual((self.read("duty_cycle"), self.read("enable")), ("0", "0"))

    def test_missing_chip_or_channel_raises(self):
        with self.assertRaises(FileNotFoundError):
            L.LinuxPwmBackend(chip=7, channel=0, root=self.root)
        with self.assertRaises(FileNotFoundError):
            L.LinuxPwmBackend(chip=2, channel=3, root=self.root, export_wait_s=0.05)
        self.assertEqual((self.root / "pwmchip2" / "export").read_text(), "3")   # it tried to export


if __name__ == "__main__":
    unittest.main()
