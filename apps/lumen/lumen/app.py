"""Main loop: camera -> perception -> events -> state machine -> director -> arm + light.

    camera frame ─► Perception.step(rates of the current state) ─► events
                                                                    │
    MotionWorker (thread, 25 Hz) ◄─ Director ◄─ Machine.handle(event) ◄┘
"""
import time

from . import config as C
from .behavior.director import Director
from .behavior.expressive import Expressive
from .io.light import Light, make_backend
from .io.power import Power
from .motion.worker import MotionWorker
from .machine import Machine
from .perception import Perception
from .perception.book import BookDetector
from .perception.hands import HandLandmarker
from .perception.motion_gate import MotionGate
from .translog import TransitionLog


def run(args):
    from . import hud
    from .io.camera import Camera, FpsCounter
    from .perfstats import count, timed              # no-op unless PERF=1

    expressive = Expressive()
    worker = MotionWorker(expressive, fake=args.fake_arm)
    light = Light(make_backend(C.LIGHT_BACKEND, worker))   # env LUMEN_LIGHT; sim by default
    power = Power()
    director = Director(worker, light, expressive, power=power)
    face = None
    if not args.no_face:
        from .perception.face import FaceTracker
        face = FaceTracker()
    perception = Perception(face=face, hands=HandLandmarker(), book=BookDetector(),
                            gate=MotionGate(C.MOTION_THRESHOLD))
    log = TransitionLog()
    streamer = None
    if args.stream:
        from .io.stream import FrameStreamer
        streamer = FrameStreamer(args.stream)

    def entered(state, m):
        director.on_enter(state, m)
        perception.on_state_change(m.ctx)

    worker.start()
    machine = Machine(on_enter=entered, on_exit=director.on_exit, now=time.monotonic())
    print(f"Lumen running: state {machine.path}" + (" (fake arm)" if args.fake_arm else ""))

    with Camera(args.camera) as cam:
        power.attach(cam)
        fps = FpsCounter()
        last_print = 0.0
        while True:
            with timed("camera.read"):
                frame = cam.read()                   # paced in SLEEP (io/power.py)
            now = time.monotonic()                   # after the read: the frame's own time
            if frame is None:
                print("Camera frame grab failed")
                break
            count("frames")

            with timed("perception"):
                events = perception.step(frame, now, machine.models, machine.ctx)
            machine.ctx["at_pause_pose"] = False
            events += director.poll_events(now, machine, perception)
            director.note_events(events, now)

            for tr in [machine.handle(ev, now) for ev in events] + [machine.tick(now)]:
                if tr:
                    log.write(tr, machine.models, expressive.energy, perception.sched.counts)
                    print(f"{tr.src:>10} --{tr.event}--> {machine.path}")
                    if tr.dst == "FAULT":
                        latched = "" if director.faults <= 1 else " (latched: hold an open palm 3 s, or restart Lumen)"
                        why = director.fault_reason if tr.event == "FAULT" else f"{tr.src} {tr.event}"
                        print(f"FAULT #{director.faults}: {why}{latched}")

            director.tick(now, machine, perception, frame.shape)
            light.update(now)                                # advance the light's ramp, write the LED
            fps.tick()

            if streamer:
                hud.draw(frame, machine, perception, light, expressive.energy, fps.fps)
                streamer.update(frame)
            if args.headless and now - last_print > 5:
                last_print = now
                print(f"[{machine.path}] {fps.fps:.1f} fps, inferences {perception.sched.counts}")
