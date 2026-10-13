"""The desk on luxoiii's stream port: Connect pauses luxoiii and moves the arm to the desk, Disconnect
hands it back; the HUD and the desk share one HTTP server. Fake worker and arm, no board."""
import json
import socket
import sys
import time
import unittest
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from luxoiii.desk import arm as D                    # noqa: E402
from luxoiii.desk.shared import Handoff              # noqa: E402
from luxoiii.io.stream import FrameStreamer          # noqa: E402
from test_desk import FakeArm                      # noqa: E402


class FakeWorker:
    def __init__(self):
        self.arm, self.calls = object(), []

    def release(self):
        self.calls.append("release")
        self.arm = None

    def connect(self):
        self.calls.append("connect")


def wait(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while not cond() and time.monotonic() < end:
        time.sleep(0.02)
    return cond()


class HandoffTest(unittest.TestCase):
    def setUp(self):
        self.worker = FakeWorker()
        self.arm = FakeArm()
        self.h = Handoff(self.worker, desk=D.DeskArm(factory=lambda out: self.arm))

    def tearDown(self):
        self.h.desk.stop()

    def test_connect_pauses_luxoiii_and_disconnect_resumes(self):
        self.assertFalse(self.h.manual)
        self.assertIn("luxoiii is driving", self.h.snapshot()["notice"])
        self.h.send("connect")
        self.assertTrue(wait(lambda: self.h.desk.arm is not None))
        self.assertTrue(self.h.manual)
        self.assertEqual(self.worker.calls, ["release"])
        self.assertEqual(self.h.snapshot()["luxoiii"], "paused")
        self.h.send("disconnect")
        self.assertTrue(wait(lambda: not self.h.manual))
        self.assertIsNone(self.h.desk.arm)
        self.assertTrue(self.arm.closed)
        self.assertTrue(self.h.resume)               # main loop re-enters BOOT, which reconnects

    def test_commands_ignored_while_luxoiii_drives(self):
        self.h.send("power", True)
        time.sleep(0.1)
        self.assertTrue(self.h.desk.cmds.empty())

    def test_failed_connect_gives_the_arm_back(self):
        def busy(out):
            raise RuntimeError("arm busy")
        h = Handoff(FakeWorker(), desk=D.DeskArm(factory=busy))
        try:
            h.send("connect")
            self.assertTrue(wait(lambda: h.resume, timeout=10))
            self.assertFalse(h.manual)
            self.assertIn("luxoiii resumed", h.snapshot()["notice"])
        finally:
            h.desk.stop()


class MergedServerTest(unittest.TestCase):
    def setUp(self):
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            self.port = s.getsockname()[1]
        self.h = Handoff(FakeWorker(), desk=D.DeskArm(factory=lambda out: FakeArm()))
        self.srv = FrameStreamer(self.port, desk=self.h, video=self.h.feed)

    def tearDown(self):
        self.srv.close()
        self.h.desk.stop()

    def get(self, path):
        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}{path}", timeout=3) as r:
            return r.read()

    def test_hud_desk_and_api_on_one_port(self):
        self.assertIn(b'src="/stream"', self.get("/"))
        self.assertIn(b'href="/desk"', self.get("/"))
        self.assertIn(b"luxoiii Desk", self.get("/desk"))
        self.assertEqual(json.loads(self.get("/state"))["luxoiii"], "running")
        req = urllib.request.Request(f"http://127.0.0.1:{self.port}/cmd", method="POST",
                                     data=json.dumps({"cmd": "connect"}).encode(),
                                     headers={"Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=3).read()
        self.assertTrue(wait(lambda: json.loads(self.get("/state"))["connected"]))


if __name__ == "__main__":
    unittest.main()
