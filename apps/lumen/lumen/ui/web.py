"""The one web page (default :8080): Lumen's HUD feed, the desk controls, and an Auto/Manual switch.

Auto: Lumen runs the state machine and owns the arm; the page shows the HUD (state, where it can
go next, what the arm plays, the AI boxes) and the desk controls are greyed out.
Manual: the machine sits in MANUAL, Lumen has released the arm, and the desk controls (jog, poses,
clips, head tilt, light) drive it. One process owns the camera and the arm, so nothing conflicts.

Handover, so the two never hold the arm at once:
  to Manual: machine -> MANUAL (director: worker.release()), then once released, desk connects
  to Auto:   desk disconnects, then machine -> BOOT (worker reconnects and homes)
"""
import threading

from .. import events as E
from . import config as ui_cfg
from . import thinking
from .stream import HudFeed

MODES = ("auto", "manual")


class WebPage:
    def __init__(self, port, fake_arm=False, start_mode=ui_cfg.START_MODE):
        from ..desk.arm import DeskArm
        from ..desk.server import serve
        self.fake = fake_arm
        self.requested = start_mode if start_mode in MODES else "manual"
        self.mode, self.status = "auto", {}
        self._desk_asked = False
        self.desk = DeskArm()
        self.desk.start()
        self.feed = HudFeed(ui_cfg.VIDEO_QUALITY)
        self.server = serve(self.desk, self.feed, port, app=self)
        threading.Thread(target=self.server.serve_forever, daemon=True, name="web").start()

    # -- from the page (server thread) ---------------------------------------------------
    def request(self, mode):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        self.requested = mode

    # -- from the main loop ------------------------------------------------------------
    def events(self, machine, worker):
        """Machine events for the mode switch this frame; also hands the arm to and from the desk."""
        if self.requested == "manual":
            if machine.state != "MANUAL":
                return [E.MANUAL]
            if worker.released and not self._desk_asked and not self.fake:
                self.desk.send("connect")
                self._desk_asked = True
            return []
        if machine.state == "MANUAL":
            if self._desk_asked:
                self.desk.send("disconnect")
                self._desk_asked = False
            if not self.desk.snapshot()["connected"]:
                return [E.AUTO]
        return []

    def update(self, machine, now, doing):
        self.mode = "manual" if machine.state == "MANUAL" else "auto"
        self.status = dict(state=machine.path, lines=thinking.lines(machine, now, doing),
                           requested=self.requested, fake_arm=self.fake)

    def close(self):
        self.desk.stop()
        self.server.shutdown()
