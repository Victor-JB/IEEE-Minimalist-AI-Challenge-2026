"""The FAULT sources from docs/STATE_MACHINE.md with fakes (no board, no cv2): each one reaches the
director as a FAULT whose reason starts with its code (STALL, SENSOR_STALE, LINK_LOST, OVERTEMP)."""
import random
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen import config as C                      # noqa: E402
from lumen import events as E                      # noqa: E402
from lumen import thermal                          # noqa: E402
from lumen.thermal import ThermalMonitor           # noqa: E402
from lumen.behavior.director import Director       # noqa: E402
from lumen.behavior.expressive import Expressive   # noqa: E402
from lumen.motion import worker as W               # noqa: E402
import arm_client                                  # noqa: E402  (arm/python, on the path via lumen.paths)


class FakeArm:
    """ArmMotion stand-in: streams until point `fail_at`, then raises `error` from stream_point."""
    def __init__(self, fail_at=None, error=None):
        self.fail_at, self.error, self.points, self.statuses, self.closed = fail_at, error, 0, 0, False

    def stream_point(self, pose, vel=None, horizon=0.1):
        self.points += 1
        if self.fail_at is not None and self.points >= self.fail_at:
            raise self.error
        return {"state": "streaming", "angles": pose}

    def status(self):
        self.statuses += 1
        return {}

    def halt(self):
        pass

    def roll(self, angle, speed=None):
        pass

    def angles(self):
        return [0.0, 60.0, 5.0]

    def servo_status(self):
        return {"stalled_joint": "head", "stall_angle": 5.1}

    def relax(self):
        pass

    def close(self):
        self.closed = True


def worker(arm, **kwargs):
    w = W.MotionWorker(Expressive(random.Random(1)), **kwargs)
    w.fake = False
    w._connect = lambda: (setattr(w, "arm", arm), True)[1]
    return w


def first_fault(w, timeout=3.0):
    """Run the worker thread until it reports a FAULT; (reason, seconds) or (None, timeout)."""
    t0 = time.monotonic()
    w.start()
    try:
        while time.monotonic() - t0 < timeout:
            for event, _, info in w.poll_events():
                if event == E.FAULT:
                    return info, time.monotonic() - t0
            time.sleep(0.01)
        return None, timeout
    finally:
        w.stop()
        w.join(1)


class SensorStale(unittest.TestCase):
    def test_stale_encoder_is_its_own_fault_reason(self):
        arm = FakeArm(fail_at=5, error=arm_client.SensorStale("Encoder feedback stale: head (age 250 ms)"))
        reason, took = first_fault(worker(arm))
        self.assertTrue(reason.startswith("SENSOR_STALE: "), reason)
        self.assertIn("head (age 250 ms)", reason)
        self.assertLess(took, 1.0)
        self.assertTrue(arm.closed)                  # session closed: drivers asleep

    def test_firmware_sensor_fault_is_sensor_stale(self):
        arm = FakeArm(fail_at=1, error=arm_client.SensorStale("Closed-loop fault: sensor"))
        reason, _ = first_fault(worker(arm))
        self.assertTrue(reason.startswith("SENSOR_STALE: "), reason)

    def test_reasons_are_distinct(self):
        self.assertTrue(W.fault_reason(W.Jammed("arm jammed")).startswith("STALL: "))
        self.assertTrue(W.fault_reason(arm_client.ArmError("Closed-loop fault: limit")).startswith("ARM: "))
        self.assertTrue(W.fault_reason(KeyError("x")).startswith("ERROR: KeyError"))
        self.assertEqual(W.fault_reason(arm_client.SensorStale("x"), "connect failed: "),
                         "SENSOR_STALE: connect failed: x")


class FailingStatusArm(FakeArm):
    def status(self):
        super().status()
        if self.statuses >= 3:
            raise arm_client.LinkLost("No reply from the arm gateway in 0.25 s")
        return {}


class LinkLost(unittest.TestCase):
    def test_dead_gateway_while_streaming_is_link_lost_fast(self):
        arm = FakeArm(fail_at=10, error=arm_client.LinkLost("Arm transport closed: router socket gone"))
        reason, took = first_fault(worker(arm))
        self.assertTrue(reason.startswith("LINK_LOST: "), reason)
        self.assertLess(took, 1.5)                   # 10 points at 25 Hz + the raise
        self.assertTrue(arm.closed)

    def test_lost_ownership_and_rpc_timeouts_are_link_lost(self):
        for e in (ConnectionError("Arm ownership lost; reconnect"), TimeoutError("rpc"), BrokenPipeError()):
            self.assertTrue(W.fault_reason(e).startswith("LINK_LOST: "), e)

    def test_relaxed_worker_keeps_the_link_alive_and_notices_it_dying(self):
        arm = FailingStatusArm()
        w = worker(arm)
        w.relaxed = True                             # FAULT/relax: nothing streams
        reason, took = first_fault(w)
        self.assertGreaterEqual(arm.statuses, 3)     # STATUS every KEEPALIVE_S keeps MCU ownership
        self.assertEqual(arm.points, 0)
        self.assertTrue(reason.startswith("LINK_LOST: "), reason)
        self.assertLess(took, 3 * W.KEEPALIVE_S + 1.0)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class Sensor:
    def __init__(self, t):
        self.t = t

    def __call__(self):
        return self.t


class Overtemp(unittest.TestCase):
    def monitor(self, *temps):
        self.clock, self.sensors = Clock(), [Sensor(t) for t in temps]
        return ThermalMonitor({f"zone{i}": s for i, s in enumerate(self.sensors)},
                              high=85, low=75, period=1.0, clock=self.clock)

    def test_no_sensors_means_disabled(self):
        m = ThermalMonitor({})
        self.assertFalse(m.enabled)
        self.assertFalse(m.poll() or m.hot)

    def test_hysteresis_and_rate(self):
        m = self.monitor(60.0, 70.0)
        self.assertFalse(m.poll())
        self.sensors[1].t = 86.0
        self.clock.now = 0.5
        self.assertFalse(m.poll() or m.hot)                 # sampled at most once per period
        self.clock.now = 1.0
        self.assertTrue(m.poll())                           # trips once...
        self.assertIn("zone1 86.0 C", m.reason)
        self.clock.now = 2.0
        self.sensors[1].t = 80.0
        self.assertFalse(m.poll())                          # ...and stays hot between the thresholds
        self.assertTrue(m.hot)
        self.clock.now = 3.0
        self.sensors[1].t = 74.0
        m.poll()
        self.assertFalse(m.hot)

    def test_vanished_sensor_is_not_overtemp(self):
        m = self.monitor(60.0)
        self.sensors[0].t = None
        self.assertFalse(m.poll() or m.hot)

    def test_discovers_thermal_zones_and_hwmon(self):
        with tempfile.TemporaryDirectory() as root:
            zone = Path(root, "thermal", "thermal_zone0")
            hw = Path(root, "hwmon", "hwmon1")
            zone.mkdir(parents=True)
            hw.mkdir(parents=True)
            (zone / "type").write_text("cpu-thermal\n")
            (zone / "temp").write_text("51234\n")
            (hw / "name").write_text("pmic\n")
            (hw / "temp1_input").write_text("42000\n")
            (hw / "temp1_label").write_text("die\n")
            sensors = thermal.discover(root)
            self.assertEqual({k: f() for k, f in sensors.items()}, {"cpu-thermal": 51.234, "pmic/die": 42.0})

    def test_overtemp_faults_the_worker_and_blocks_recovery_until_cool(self):
        m = self.monitor(60.0)
        arm = FakeArm()
        w = worker(arm, thermal=m)
        w._connect = lambda: (setattr(w, "arm", arm), True)[1]
        w.start()
        try:
            time.sleep(0.2)
            m.sensors["zone0"].t = 90.0
            self.clock.now = 5.0
            reason = self._fault(w)
            self.assertTrue(reason.startswith("OVERTEMP: zone0 90.0 C >= 85 C"), reason)
            self.assertTrue(arm.closed and w.hot)
            # FAULT's cooldown: no RECOVERED while hot, even long after the 10 s cooldown.
            d = Director(w, FakeLight(), Expressive(random.Random(1)))
            d.faults, fault = 1, SimpleNamespace(state="FAULT", entered_at=0.0)
            self.assertNotIn(E.RECOVERED, d.poll_events(C.FAULT_COOLDOWN_S + 60, fault))
            # A RESET reboots early: BOOT's connect is refused while still hot.
            arm.closed = False
            self.sensors[0].t = 80.0
            self.clock.now = 6.0
            w.connect()
            self.assertTrue(self._fault(w).startswith("OVERTEMP: still hot: "))
            self.assertFalse(arm.closed)              # never reconnected
            # Cooled below the lower threshold: recovery and reconnect go ahead.
            self.sensors[0].t = 70.0
            self.clock.now = 7.0
            deadline = time.monotonic() + 2
            while w.hot and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertFalse(w.hot)
            self.assertIn(E.RECOVERED, d.poll_events(C.FAULT_COOLDOWN_S + 60, fault))
        finally:
            w.stop()
            w.join(1)

    def _fault(self, w, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for event, _, info in w.poll_events():
                if event == E.FAULT:
                    return info
            time.sleep(0.01)
        self.fail("no FAULT")


class ReasonSurfaced(unittest.TestCase):
    def test_transitions_log_has_the_fault_reason(self):
        import csv
        from lumen.machine import Transition
        from lumen.translog import TransitionLog
        with tempfile.TemporaryDirectory() as d:
            log = TransitionLog(Path(d))
            log.write(Transition(1.0, "STANDBY", "FAULT", E.FAULT), {}, 0.3, {}, "LINK_LOST: no reply")
            log.write(Transition(2.0, "FAULT", "BOOT", E.RESET), {}, 0.3, {})
            log.f.close()
            rows = list(csv.DictReader(log.path.open()))
            self.assertEqual([r["reason"] for r in rows], ["LINK_LOST: no reply", ""])
            # A file from before the reason column is not appended to with a different layout.
            log.path.write_text("wall_time,t,from,to,event,models,energy,inferences_so_far\n")
            again = TransitionLog(Path(d))
            again.f.close()
            self.assertNotEqual(again.path, log.path)


class FakeLight:
    def set(self, level):
        pass


if __name__ == "__main__":
    unittest.main()
