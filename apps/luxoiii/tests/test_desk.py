"""luxoiii Desk with a fake arm: gating (armed, healthy, enabled), hold-to-run jogs, moves, clips,
stalls, head tilt, and the HTTP API. No board or camera needed."""
import json
import sys
import threading
import unittest
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from luxoiii import config as C                      # noqa: E402
from luxoiii.desk import arm as D                    # noqa: E402

try:
    import numpy  # noqa: F401
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class FakeArm:
    """ArmMotion stand-in: joints in motor order (elbow, shoulder, head), millidegrees."""
    def __init__(self, pose=(-12.0, 63.5, 5.3), enabled=1, magnet=0x20, stall_after=None):
        self.pose, self.calls, self.roll_supported = list(pose), [], True
        self.enabled, self.magnet, self.stall_after, self.points = enabled, magnet, stall_after, 0
        self.closed, self.state = False, 0
        self.sample = self._sample()

    def _sample(self):
        mm = [self.pose[1], self.pose[0], self.pose[2]]
        return dict(kind="arm", error=0, state=self.state, awake=int(self.state != 0), enabled=self.enabled,
                    generation=1, configured=7,
                    axes=[dict(angle=round(a * 1000), age=5, magnet=self.magnet, sensor_error=0) for a in mm])

    def status(self):
        self.calls.append("status"); self.sample = self._sample(); return self.sample

    def stream_point(self, pose, vel=None, horizon=0.1):
        self.calls.append(("stream", [round(p, 3) for p in pose]))
        self.points += 1
        if self.stall_after and self.points >= self.stall_after:
            self.state = 16; self.sample = self._sample()
            return {"state": "stalled", "angles": self.pose}
        self.pose, self.state = list(pose), 17
        self.sample = self._sample()
        return {"state": "streaming", "angles": pose}

    def servo_status(self):
        return dict(state="holding", reason="complete", stalls=0, stalled_joint="head", stall_angle=14.1)

    def halt(self):
        self.calls.append("halt"); self.state = 14

    def roll(self, angle, speed=None):
        self.calls.append(("roll", round(angle, 2), speed))

    def roll_off(self):
        self.calls.append("roll_off")

    def light(self, level):
        self.calls.append(("light", float(level)))

    def roll_status(self):
        return dict(supported=True, enabled=1, target=0.0, current=0.0, pulse_us=1500, fault=0)

    def power(self, enabled):
        self.calls.append(("power", enabled)); self.enabled = int(enabled)

    def relax(self):
        self.calls.append("relax"); self.state = 0

    def close(self):
        self.closed = True

    def streamed(self):
        return [c[1] for c in self.calls if isinstance(c, tuple) and c[0] == "stream"]


class DeskTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.fake = Clock(), FakeArm()
        self.desk = D.DeskArm(factory=lambda out: self.fake, clock=self.clock)
        D.paths.LOGS.mkdir(exist_ok=True)
        self.desk.send("connect")
        self.tick()

    def tick(self, n=1):
        for _ in range(n):
            self.desk._drain()
            if self.desk.arm:
                self.desk._step(self.clock())
            self.clock.t += D.DT

    def test_connect_reads_joints_in_joint_order(self):
        s = self.desk.snapshot()
        self.assertTrue(s["connected"] and s["healthy"])
        self.assertEqual([j["angle"] for j in s["joints"]], [-12.0, 63.5, 5.3])
        self.assertEqual(s["joints"][0]["magnet_text"], "ok")

    def test_motion_refused_when_locked_or_unhealthy(self):
        self.fake.enabled = 0; self.fake.status(); self.desk.sample = self.fake.sample
        self.desk.send("jog", 0, 1, 5); self.tick(3)
        self.assertEqual(self.fake.streamed(), [])
        self.assertIn("locked", self.desk.snapshot()["notice"])
        self.fake.enabled, self.fake.magnet = 1, 0x10; self.fake.status(); self.desk.sample = self.fake.sample
        self.desk.send("goto", [0.0, 60.0, 5.0], 10); self.tick(3)
        self.assertEqual(self.fake.streamed(), [])
        self.assertIn("magnet", self.desk.snapshot()["notice"])

    def test_drivers_wake_for_a_move_and_sleep_after(self):
        self.assertNotIn("relax", self.fake.calls)
        self.desk.send("goto", [-11.0, 63.0, 5.0], 10); self.tick(3)
        self.assertTrue(self.fake.sample["awake"])                  # streaming woke them
        self.tick(100)                                               # move done + SETTLE_S
        self.assertEqual(self.fake.calls.count("relax"), 1)
        self.assertFalse(self.fake.sample["awake"])
        self.desk.send("jog", 2, 1, 5); self.tick(2)                 # a jog wakes them again...
        self.tick(int((D.JOG_DEADMAN_S + D.SETTLE_S) / D.DT) + 3)    # ...and they sleep after it
        self.assertEqual(self.fake.calls.count("relax"), 2)

    def test_jog_moves_one_joint_and_stops_without_renewal(self):
        self.desk.send("jog", 1, -1, 10); self.tick(5)
        pts = self.fake.streamed()
        self.assertGreaterEqual(len(pts), 5)
        self.assertLess(pts[-1][1], 63.5)                          # elbow went down
        self.assertEqual((pts[-1][0], pts[-1][2]), (-12.0, 5.3))   # others untouched
        self.tick(int(D.JOG_DEADMAN_S / D.DT) + 3)                 # no renewal: hold-to-run ends
        n = len(self.fake.streamed()); self.tick(5)
        self.assertEqual(len(self.fake.streamed()), n)
        self.assertIsNone(self.desk.snapshot()["busy"])

    def test_releasing_the_jog_button_puts_the_drivers_to_sleep(self):
        self.desk.send("jog", 2, 1, 5); self.tick(3)
        self.desk.send("jog", 0, 0, 0); self.tick()                 # the page's button-up message
        self.assertIsNone(self.desk.snapshot()["busy"])
        self.tick(int(D.SETTLE_S / D.DT) + 2)
        self.assertEqual(self.fake.calls.count("relax"), 1)
        self.desk.send("jog", 0, 0, 0); self.tick(20)              # a stray button-up while idle: nothing
        self.assertEqual(self.fake.calls.count("relax"), 1)

    def test_awake_with_nothing_to_do_falls_asleep(self):
        self.fake.state = 14; self.fake.status()                    # e.g. woken by another path
        self.tick(int(D.SETTLE_S / D.DT) + 4)
        self.assertEqual(self.fake.calls.count("relax"), 1)

    def test_jog_stays_in_the_safe_range(self):
        self.fake.pose = [C.SAFE_RANGES[0][1] - 0.1, 63.5, 5.3]; self.fake.status(); self.desk.sample = self.fake.sample
        for _ in range(20):
            self.desk.send("jog", 0, 1, 20); self.tick()
        self.assertLessEqual(max(p[0] for p in self.fake.streamed()), C.SAFE_RANGES[0][1])

    def test_goto_reaches_the_clamped_target_with_head_tilt(self):
        self.desk.send("goto", [-5.0, 60.0, 500.0], 15, 12); self.tick(400)
        self.assertEqual(self.fake.streamed()[-1], [-5.0, 60.0, C.SAFE_RANGES[2][1]])
        self.assertIn(("roll", 12.0, None), self.fake.calls)
        self.assertIn("done", self.desk.snapshot()["notice"])

    def test_stall_halts_and_reports(self):
        self.fake.stall_after = 3
        self.desk.send("goto", [0.0, 50.0, 5.0], 10); self.tick(10)
        self.assertIn("halt", self.fake.calls)
        self.assertEqual(len(self.fake.streamed()), 3)
        self.assertIn("Stalled: head at 14.1", self.desk.snapshot()["notice"])

    def test_stop_halts_a_running_move_then_sleeps(self):
        self.desk.send("goto", [0.0, 50.0, 5.0], 5); self.tick(3)
        self.desk.send("stop"); self.tick()
        self.assertIn("halt", self.fake.calls)
        self.assertIsNone(self.desk.snapshot()["busy"])
        self.tick(int(D.SETTLE_S / D.DT) + 2)
        self.assertIn("relax", self.fake.calls)                      # stopped: drivers sleep

    def test_roll_is_clamped_unless_wide(self):
        self.desk.send("roll", 100, None, False); self.desk.send("roll", 100, None, True); self.tick()
        rolls = [c for c in self.fake.calls if isinstance(c, tuple) and c[0] == "roll"]
        self.assertEqual([r[1] for r in rolls], [C.SAFE_RANGES and D.A.ROLL_RANGE[1], 100.0])

    def test_head_tilt_goes_limp_when_idle(self):
        self.desk.send("roll", 30, 60, False); self.tick()
        self.assertNotIn("roll_off", self.fake.calls)
        self.tick(int((30 / 60 + D.SETTLE_S) / D.DT) + 3)          # tilt time + settle
        self.assertEqual(self.fake.calls.count("roll_off"), 1)
        self.desk.send("roll", 0, None, False); self.tick(2)          # the next input wakes it again
        self.assertIn(("roll", 0.0, None), self.fake.calls)

    @unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
    def test_clip_streams_and_tilts_the_head(self):
        self.desk.send("clip", "curious_head_tilt/single", 1.0); self.tick(400)
        rolls = [c[1] for c in self.fake.calls if isinstance(c, tuple) and c[0] == "roll"]
        self.assertGreater(max(rolls), 15)
        self.assertEqual(rolls[-1], D.A.ROLL_HOME)
        self.assertIn("done", self.desk.snapshot()["notice"])

    def test_light_switches_and_needs_a_connection(self):
        self.assertIn(("light", 0.0), self.fake.calls)                # connect matches the page (off)
        self.desk.send("light", 0.4); self.tick()
        self.assertIn(("light", 400.0), self.fake.calls)
        self.assertEqual(self.desk.snapshot()["light"], 0.4)
        self.desk.send("light", 7); self.tick()                        # clamped to full
        self.assertIn(("light", 1000.0), self.fake.calls)
        self.assertFalse(any(c == "power" or (isinstance(c, tuple) and c[0] == "power")
                             for c in self.fake.calls))                # no driver wake for the light
        self.desk.send("disconnect"); self.tick()
        self.desk.send("light", 0); self.tick()
        self.assertEqual(self.desk.snapshot()["light"], 1.0)          # unchanged: not connected
        self.assertIn("Connect first", self.desk.snapshot()["notice"])

    def test_power_and_disconnect(self):
        self.desk.send("power", False); self.tick()
        self.assertIn(("power", False), self.fake.calls)
        self.desk.send("disconnect"); self.tick()
        self.assertTrue(self.fake.closed)
        self.assertFalse(self.desk.snapshot()["connected"])

    def test_link_loss_closes_the_session(self):
        def boom(*a, **k):
            raise RuntimeError("link lost")
        self.fake.status = boom
        self.clock.t += 1
        try:
            self.desk._step(self.clock())
        except RuntimeError as e:
            self.desk._lost(e)
        self.assertFalse(self.desk.snapshot()["connected"])
        self.assertIn("Lost the arm", self.desk.snapshot()["notice"])


class ServerTests(unittest.TestCase):
    def setUp(self):
        from luxoiii.desk import server as S
        from luxoiii.desk.video import VideoFeed
        self.S = S
        self.desk = D.DeskArm(factory=lambda out: FakeArm())
        self.desk.start()
        self.server = S.serve(self.desk, VideoFeed(), 0)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.shutdown)
        self.addCleanup(self.desk.stop)

    def get(self, path):
        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}{path}", timeout=3) as r:
            return r.read()

    def post(self, path, body):
        req = urllib.request.Request(f"http://127.0.0.1:{self.port}{path}", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=3) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def test_page_state_and_commands(self):
        self.assertIn(b"luxoiii Desk", self.get("/"))
        self.assertEqual(self.post("/cmd", {"cmd": "connect"})[0], 200)
        import time
        for _ in range(50):
            if json.loads(self.get("/state"))["connected"]:
                break
            time.sleep(0.05)
        s = json.loads(self.get("/state"))
        self.assertTrue(s["connected"])
        self.assertIn("video", s)
        self.assertEqual(self.post("/cmd", {"cmd": "rm -rf"})[0], 400)
        self.assertIsInstance(json.loads(self.get("/history?since=0")), list)

    def test_recording_names_cannot_escape(self):
        self.assertIsNone(self.S.recording_path("../../etc/passwd"))
        self.assertIsNone(self.S.recording_path("x.jsonl/../../y.jsonl"))
        with self.assertRaises(urllib.error.HTTPError):
            self.get("/recordings/..%2F..%2Fapp.py")

    def test_replay_decodes_joint_order(self):
        import tempfile
        with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as f:
            s = FakeArm(pose=(-10.0, 60.0, 4.0))._sample()
            f.write(json.dumps({"type": "sample", "time": 1.5, "sample": s}) + "\n")
            f.write(json.dumps({"type": "command", "time": 1.6, "text": "STATUS"}) + "\n")
        out = self.S.replay(Path(f.name))
        Path(f.name).unlink()
        self.assertEqual(out, [[1.5, [-10.0, 60.0, 4.0]]])


if __name__ == "__main__":
    unittest.main()
