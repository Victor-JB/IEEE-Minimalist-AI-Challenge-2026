"""Main loop: camera -> perception -> events -> state machine -> director -> arm + light.

    camera frame ─► Perception.step(rates of the current state) ─► events
                                                                    │
    MotionWorker (thread, 25 Hz) ◄─ Director ◄─ Machine.handle(event) ◄┘
"""
import time

from . import paths  # noqa: F401  (sys.path for inference/, perf/, motion_lib/, stepper_desk/)
from . import config as C
from .behavior.director import Director
from .behavior.expressive import Expressive
from .io.arm import MotionWorker
from .io.light import Light
from .machine import Machine
from .perception import Perception
from .perception.book import BookDetector
from .perception.hands import HandLandmarker
from .perception.motion_gate import MotionGate
from .translog import TransitionLog


def run(args):
    from camera import Camera, FpsCounter            # inference/camera.py
    from perfstats import count, timed               # perf/perfstats.py (no-op unless PERF=1)
    from . import hud

    expressive = Expressive()
    worker = MotionWorker(expressive, fake=args.fake_arm)
    light = Light()
    director = Director(worker, light, expressive)
    face = None
    if not args.no_face:
        from .perception.face import FaceTracker
        face = FaceTracker()
    perception = Perception(face=face, hands=HandLandmarker(), book=BookDetector(),
                            gate=MotionGate(C.MOTION_THRESHOLD))
    log = TransitionLog()
    streamer = None
    if args.stream:
        from stream import FrameStreamer             # inference/stream.py
        streamer = FrameStreamer(args.stream)

    def entered(state, m):
        director.on_enter(state, m)
        perception.on_state_change()

    worker.start()
    machine = Machine(on_enter=entered, on_exit=director.on_exit, now=time.monotonic())
    print(f"Lumen running: state {machine.path}" + (" (fake arm)" if args.fake_arm else ""))

    with Camera(args.camera) as cam:
        fps = FpsCounter()
        last_print = 0.0
        while True:
            now = time.monotonic()
            with timed("camera.read"):
                frame = cam.read()
            if frame is None:
                print("Camera frame grab failed")
                break
            count("frames")

            with timed("perception"):
                events = perception.step(frame, now, machine.models, machine.ctx)
            machine.ctx["at_pause_pose"] = False
            events += director.poll_events(now, machine, perception)
            director.note_events(events, now)

            for ev in events:
                tr = machine.handle(ev, now)
                if tr:
                    log.write(tr, machine.models, expressive.energy, perception.sched.counts)
                    print(f"{tr.src:>10} --{tr.event}--> {machine.path}")
            tr = machine.tick(now)
            if tr:
                log.write(tr, machine.models, expressive.energy, perception.sched.counts)
                print(f"{tr.src:>10} --{tr.event}--> {machine.path}")

            director.tick(now, machine, perception, frame.shape)
            fps.tick()

            if streamer:
                hud.draw(frame, machine, perception, light, expressive.energy, fps.fps)
                streamer.update(frame)
            if args.headless and now - last_print > 5:
                last_print = now
                print(f"[{machine.path}] {fps.fps:.1f} fps, inferences {perception.sched.counts}")
