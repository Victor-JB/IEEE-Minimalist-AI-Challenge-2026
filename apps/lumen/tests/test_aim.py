"""Camera geometry and the AIM / SEARCH / ADJUST behavior, with a fake worker. No board, no cv2."""
import math
import random
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen import config as C                      # noqa: E402
from lumen import events as E                      # noqa: E402
from lumen.behavior import aim as G                # noqa: E402
from lumen.behavior.director import Director       # noqa: E402
from lumen.behavior.expressive import Expressive   # noqa: E402
from lumen.motion import worker as W               # noqa: E402
import kinematics as K                             # noqa: E402  (arm/python, via lumen.paths)

FRAME = (640, 480)


def inside(q):
    return all(lo <= v <= hi for v, (lo, hi) in zip(q, C.SAFE_RANGES))


class Camera(unittest.TestCase):
    def setUp(self):
        self.view = K.aim((0.40, 0.0, 0.0), ref=C.HOME)       # a pose looking down at the desk

    def test_image_centre_is_where_the_beam_lands(self):
        q, r = self.view.q, self.view.roll
        p = G.desk_point(q, r, 320, 240, *FRAME, plane_z=0.0)
        self.assertTrue(all(abs(a - b) < 0.02 for a, b in zip(p, (0.40, 0.0, 0.0))), p)   # camera 0 cm off axis

    def test_pixel_to_desk_round_trips(self):
        q, r = self.view.q, -15.0
        for u, v in ((0, 0), (639, 0), (320, 240), (100, 400), (600, 479)):
            p = G.desk_point(q, r, u, v, *FRAME, plane_z=0.0)
            self.assertIsNotNone(p)
            back = G.project(q, r, p, *FRAME)
            self.assertAlmostEqual(back[0], u, places=6)
            self.assertAlmostEqual(back[1], v, places=6)

    def test_image_directions(self):
        q = self.view.q
        top, bottom = G.desk_point(q, 0, 320, 0, *FRAME), G.desk_point(q, 0, 320, 479, *FRAME)
        left, right = G.desk_point(q, 0, 0, 240, *FRAME), G.desk_point(q, 0, 639, 240, *FRAME)
        self.assertGreater(top[0], bottom[0])          # image top = the far side of the desk
        self.assertGreater(left[1], right[1])          # image left = the lamp's left (+y)

    def test_fov_sets_the_footprint(self):
        h = K.fk((0.0, 0.0, 180.0))                    # straight up, then the head straight down
        d = G.desk_point((0.0, 0.0, 180.0), 0, 639.999, 240, *FRAME, plane_z=0.0)
        self.assertAlmostEqual(math.degrees(math.atan2(-d[1] - h.camera[1], h.camera[2])),
                               C.CAMERA_HFOV_DEG / 2, places=3)

    def test_horizon_returns_none(self):
        self.assertIsNone(G.desk_point(C.HOME, 0, 320, 240, *FRAME))   # HOME looks at the user

    def test_book_box_to_aim(self):
        q, r = self.view.q, self.view.roll
        book = (400, 100, 120, 160, 0.9)               # right of centre, towards the far side
        p = G.book_target(q, r, book, FRAME)
        self.assertGreater(p[0], 0.40)
        self.assertLess(p[1], 0.0)
        a = G.aim_at_book(q, r, book, FRAME, ref=q)
        self.assertTrue(inside(a.q))
        hit = K.beam_hit(a.q, a.roll, C.BOOK_TOP_Z)
        self.assertTrue(all(abs(x - y) < 1e-6 for x, y in zip(hit, p)))

    def test_move_time_respects_caps(self):
        a, b = (0, 60, 0), (10, 40, 40)
        t = G.move_time(a, b)
        for j, cap in enumerate((C.SPEED_CAPS[0], C.ELBOW_LIFT_CAP, C.SPEED_CAPS[2])):
            self.assertLessEqual(1.875 * abs(b[j] - a[j]) / t, cap + 1e-9)


class Sweep(unittest.TestCase):
    def test_pauses_cover_the_desk(self):
        plan = G.sweep_poses()
        self.assertTrue(3 <= len(plan) <= 5)
        for q, r in plan:
            self.assertTrue(inside(q), q)
            self.assertTrue(K.ROLL_RANGE[0] <= r <= K.ROLL_RANGE[1], r)
        self.assertGreaterEqual(G.coverage(plan), 0.95)

    def test_near_to_far(self):
        hits = [K.beam_hit(q, r)[0] for q, r in G.sweep_poses()]
        self.assertEqual(hits, sorted(hits))

    def test_roll_zigzag_is_what_widens_it(self):
        pitch_only = G.coverage(G.sweep_poses(rolls=()))
        self.assertLess(pitch_only, 0.8)               # without J5 the frame is too narrow near the lamp
        self.assertGreater(G.coverage(G.sweep_poses()), pitch_only + 0.2)


class FakeWorker:
    def __init__(self):
        self.calls, self.current, self.base_roll, self.base = [], list(C.HOME), 0.0, list(C.HOME)
        self.offset, self.queued = None, []

    def __getattr__(self, name):          # connect, relax, play...
        return lambda *a, **k: self.calls.append((name, a, k))

    def goto(self, pose, duration, token, done=E.MOVE_DONE):
        self.calls.append(("goto", pose, duration, token, done))

    def path(self, segments, token, done=E.MOVE_DONE):
        self.calls.append(("path", segments, token, done))

    def sweep(self, poses, dwell, token):
        self.calls.append(("sweep", poses, dwell, token))

    def set_base(self, pose):
        self.base = list(pose)

    def set_offset(self, offset):
        self.offset = offset

    def poll_events(self):
        out, self.queued = self.queued, []
        return out


class FakeLight:
    def __init__(self):
        self.level = 0.0

    def set(self, v):
        self.level = v

    def step(self, d):
        self.level += d


class DirectorGeometry(unittest.TestCase):
    def setUp(self):
        self.w = FakeWorker()
        self.d = Director(self.w, FakeLight(), Expressive(random.Random(1)), rng=random.Random(1))
        self.m = SimpleNamespace(state="SEARCH", ctx={}, entered_at=0.0, spec=SimpleNamespace(pool="searching"))

    def enter(self, state):
        self.m.state = state
        self.d.on_enter(state, self.m)

    def test_search_sweeps_the_planned_pauses_with_roll(self):
        self.enter("SEARCH")
        _, poses, dwell, tok = self.w.calls[-1]
        self.assertTrue(3 <= len(poses) <= 5)
        self.assertTrue(all(len(p) == 4 for p in poses))
        self.assertTrue(any(abs(p[3]) > 5 for p in poses))

    def test_book_model_runs_only_after_settled_pause_reports(self):
        self.enter("SEARCH")
        self.assertFalse(self.m.ctx.get("at_pause_pose"))
        self.d.poll_events(0.0, self.m)
        self.assertFalse(self.m.ctx.get("at_pause_pose"))          # moving: no inference
        self.w.queued = [(E.AT_POSE, self.d.token, 1)]             # the worker settled at pause 1
        for _ in range(2):
            self.m.ctx["at_pause_pose"] = False                    # app.py clears it every frame
            self.d.poll_events(0.0, self.m)
            self.assertTrue(self.m.ctx["at_pause_pose"])
        self.m.ctx["at_pause_pose"] = False
        self.d.poll_events(0.0, self.m)
        self.assertFalse(self.m.ctx["at_pause_pose"])              # BOOK_PER_POSE frames, then off
        self.assertEqual(self.d.view, self.d.sweep_plan[1])

    def _found_at(self, pause, box):
        self.enter("SEARCH")
        self.w.queued = [(E.AT_POSE, self.d.token, pause)]
        self.d.poll_events(0.0, self.m)
        self.d.perception, self.d.frame_wh = SimpleNamespace(last_book=box), FRAME
        self.w.current = list(self.d.sweep_plan[pause][0])
        self.enter("AIM")
        return self.w.calls[-1]

    def test_aim_lights_the_book_the_camera_saw(self):
        box = (300, 300, 100, 120, 0.8)
        name, pose, dur, tok, done = self._found_at(2, box)
        view_q, view_r = self.d.sweep_plan[2]
        self.assertEqual((name, done), ("goto", E.AT_POSE))
        self.assertTrue(inside(pose[:3]))
        book = G.book_target(view_q, view_r, box, FRAME)
        hit = K.beam_hit(pose[:3], pose[3], C.BOOK_TOP_Z)
        self.assertTrue(all(abs(a - b) < 1e-6 for a, b in zip(hit, book)), (hit, book))
        self.assertGreaterEqual(dur, G.move_time(self.w.current, pose))
        self.enter("HOLD")
        self.assertEqual(self.w.base, pose)                       # the hold keeps the roll too

    def test_aim_without_a_box_lights_where_the_camera_looked(self):
        name, pose, dur, tok, done = self._found_at(1, None)
        q, r = self.d.sweep_plan[1]
        centre = G.desk_point(q, r, 320, 240, *FRAME)
        hit = K.beam_hit(pose[:3], pose[3], centre[2])
        self.assertTrue(all(abs(a - b) < 1e-6 for a, b in zip(hit, centre)))

    def test_adjust_moves_along_the_beam_with_anticipation(self):
        self._found_at(2, (300, 300, 100, 120, 0.8))
        target, before = self.d.aim_target, list(self.d.aim_pose)
        s0 = K.standoff_of(before[:3], before[3], target)
        self.m.ctx["adjust"] = E.CLOSER
        self.enter("ADJUST")
        name, segments, tok, done = self.w.calls[-1]
        self.assertEqual((name, done), ("path", E.MOVE_DONE))
        self.assertEqual(len(segments), 2)
        wind, move = segments[0][0], segments[1][0]
        self.assertGreater(K.standoff_of(wind[:3], wind[3], target), s0)          # back a little first
        self.assertAlmostEqual(K.standoff_of(move[:3], move[3], target), s0 - C.ADJUST_STEP_M, places=6)
        hit = K.beam_hit(move[:3], move[3], target[2])
        self.assertTrue(all(abs(a - b) < 1e-6 for a, b in zip(hit, target)))     # still on the book
        self.m.ctx["adjust"] = E.FURTHER
        self.enter("ADJUST")
        back = self.w.calls[-1][1][-1][0]
        self.assertAlmostEqual(K.standoff_of(back[:3], back[3], target), s0, places=6)

    def test_adjust_stops_at_reach(self):
        self._found_at(2, (300, 300, 100, 120, 0.8))
        self.m.ctx["adjust"] = E.CLOSER
        for _ in range(30):
            self.enter("ADJUST")
            self.assertTrue(inside(self.d.aim_pose[:3]))
        segments = self.w.calls[-1][1]
        self.assertEqual(len(segments), 1)                       # nothing left to move: no wind-up


class FakeArm:
    """Measured angles lag the setpoint by `lag` deg on the head until `settle_after` points."""
    def __init__(self, settle_after):
        self.points, self.settle_after = 0, settle_after

    def stream_point(self, pose, vel=None, horizon=0.1):
        self.points += 1
        lag = 0.0 if self.points >= self.settle_after else 5.0
        return {"state": "streaming", "angles": [pose[0], pose[1], pose[2] - lag]}

    def roll(self, angle, speed=None):
        pass


class WorkerSettle(unittest.TestCase):
    def test_at_pose_waits_for_measured_angles(self):
        w = W.MotionWorker(Expressive(random.Random(1)), fake=True)
        pose = [-12.0, 63.0, 10.0, 15.0]
        w.current = pose[:3]
        moves = int(W.SWEEP_MIN_MOVE_S / W.DT)
        w.arm = FakeArm(settle_after=moves + int(0.5 / W.DT))      # still 5 deg off 0.5 s after arriving
        t0 = time.monotonic()
        self.assertTrue(w._sweep([pose], 0.0, 1))
        events = w.poll_events()
        self.assertEqual(events[0], (E.AT_POSE, 1, 0))
        self.assertGreaterEqual(time.monotonic() - t0, W.SWEEP_MIN_MOVE_S + 0.45)
        self.assertEqual(w.roll, 15.0)                             # the pause's J5 look
        self.assertEqual(w.base, list(C.HOME))                      # restored after the sweep

    def test_settled_rules(self):
        w = W.MotionWorker(Expressive(random.Random(1)), fake=True)
        w.measured = [0.0, 60.0, 5.0]
        self.assertFalse(w.settled([0.0, 60.0, 5.0], 0.0))                   # too soon
        self.assertTrue(w.settled([0.0, 60.0, 5.0], C.SETTLE_MIN_S))
        self.assertFalse(w.settled([0.0, 60.0, 9.0], C.SETTLE_MIN_S))        # still 4 deg away
        self.assertTrue(w.settled([0.0, 60.0, 9.0], C.SETTLE_MAX_S))         # backlash: give up waiting
        w.measured = None
        self.assertTrue(w.settled([0.0, 60.0, 9.0], C.SETTLE_MIN_S))         # fake arm: time only

    def test_path_runs_segments_then_reports_once(self):
        w = W.MotionWorker(Expressive(random.Random(1)), fake=True)
        w.cmds.put(("path", [([-12.0, 63.0, 6.0, 5.0], 0.1), ([-11.0, 62.0, 7.0, 5.0], 0.1)], 4, E.MOVE_DONE))
        w.start()
        time.sleep(0.5)
        w.stop()
        w.join(1)
        self.assertEqual([e for e in w.poll_events() if e[0] == E.MOVE_DONE], [(E.MOVE_DONE, 4, None)])
        self.assertEqual(w.base, [-11.0, 62.0, 7.0])
        self.assertEqual(w.base_roll, 5.0)


if __name__ == "__main__":
    unittest.main()
