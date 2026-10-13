"""The FAULT sources from docs/STATE_MACHINE.md with fakes (no board, no cv2): each one reaches the
director as a FAULT whose reason starts with its code (STALL, SENSOR_STALE, LINK_LOST, OVERTEMP)."""
import random
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen import events as E                      # noqa: E402
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


if __name__ == "__main__":
    unittest.main()
