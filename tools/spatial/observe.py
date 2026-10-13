#!/usr/bin/env python3
"""UNO Q face/hand observer and optional hybrid demo. Owns the USB camera directly.

The observation path only reads: arm_roll_status and stepper_command op 4, without HELLO or
claim. Unclaimed status returns EACCES plus raw encoder samples; apply saved
calibration here without changing firmware configuration. Keep HTTP on loopback. With --hybrid, explicit UI activation also claims the
quiet arm for subject-tracking pivot control and face/downward-beam-gated D7; no stepper commands.
"""
import argparse
import base64
import importlib
import json
from pathlib import Path
import signal
import sys
import threading
import time

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def capture_camera(video, stop, publish, clock=time.monotonic):
    """Publish actual captured frames; cached sequences and timeouts fail closed.

    VideoFeed increments its sequence only after a successful camera read and
    JPEG encode, so a stationary scene with identical image bytes is still fresh.
    Its own capture worker retries a failed/unplugged camera automatically.
    """
    try:
        with video.watch() as feed:
            sequence = 0
            while not stop.is_set():
                new_sequence, jpeg = feed.next_frame(sequence, timeout=.5)
                if stop.is_set():
                    break
                if jpeg is not None and new_sequence != sequence:
                    sequence = new_sequence
                    publish(jpeg, clock())
                else:
                    publish(None, 0.)
                    # next_frame returns immediately while a camera error is
                    # present; let its retry worker recover without busy-polling.
                    if feed.error:
                        stop.wait(.1)
    finally:
        publish(None, 0.)


def encoder(raw, joint, calibration):
    if len(raw) != 31 or raw[0] != 7 or raw[1] != joint or raw[3] not in (0, -13, -16):
        raise ValueError('Unsupported raw encoder status')
    count, magnet, error, age = raw[14:18]
    good = 0 <= count < 4096 and magnet & 0x28 == 0x20 and error == 0 and 0 <= age <= 100
    angle = ((count*360/4096-calibration['zero'])*(-1 if calibration['reversed'] else 1)+180)%360-180
    return dict(angle=angle if good else None, healthy=good, age_ms=age,
                source='raw encoder + saved calibration')


def main():
    from behavior import Demo
    from tracking import SubjectTracker
    from inference import Inference
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo', type=Path, required=True)
    p.add_argument('--package', default='luxoiii', choices=['luxoiii','lumen'])
    p.add_argument('--port', type=int, default=8083)
    p.add_argument('--camera',default='auto',help='USB camera: auto, an index, or a /dev/video path')
    p.add_argument('--hybrid',action='store_true',help='Allow UI activation of subject tracking and face/downward-beam-gated light')
    scheduling=p.add_mutually_exclusive_group()
    scheduling.add_argument('--parallel-inference',dest='parallel_inference',action='store_true',help='Experimental concurrent models; measured with two threads each')
    scheduling.add_argument('--serial-inference',dest='parallel_inference',action='store_false')
    p.set_defaults(parallel_inference=False)
    p.add_argument('--inference-threads',type=int,choices=[1,2,3,4],default=4)
    args=p.parse_args()
    sys.path.insert(0,str(args.repo/'apps'/args.package))
    FaceDetector=importlib.import_module(args.package+'.perception.face_detector').FaceDetector
    hands_module=importlib.import_module(args.package+'.perception.hands')
    gestures=importlib.import_module(args.package+'.perception.gestures')
    MotionGate=importlib.import_module(args.package+'.perception.motion_gate').MotionGate
    VideoFeed=importlib.import_module(args.package+'.desk.video').VideoFeed
    from pointing import target_from_hand, simulated_aim
    from hybrid import Outputs
    from arduino.router_bridge import Bridge
    import cv2
    import numpy as np
    cal=json.loads((args.repo/'arm/calibration/arm-calibration-2026-10-11.json').read_text())['saved_calibration']['joints']
    if len(cal)!=3 or any(c.get('calibrated') is not True for c in cal):
        raise ValueError('Saved joint calibration required')
    def physical_pose(bridge):
        joints=[encoder(bridge.call('stepper_command',[4,j,2147483647],timeout=.25),j,cal[j]) for j in range(3)]
        r=bridge.call('arm_roll_status',timeout=.25)
        if len(r)!=6 or r[0]!=3:raise ValueError('Unsupported head pivot status')
        return dict(joints=joints,pivot=dict(enabled=bool(r[1]),target=r[2]/1000,current=r[3]/1000,fault=r[5]))
    # The measured default uses four threads per model, sequentially. OpenCV
    # preprocessing stays single-threaded to avoid another competing worker pool.
    cv2.setNumThreads(1)
    detector=FaceDetector(num_threads=args.inference_threads)
    hands=hands_module.HandLandmarker(num_threads=args.inference_threads,max_hands=2,landmark_threshold=.65)
    gesture_model='landmark-mlp-v1' if getattr(gestures,'_classifier',lambda:None)() is not None else 'legacy-rules'
    inference=Inference(detector,hands,parallel=args.parallel_inference)
    tracker=SubjectTracker();demo=Demo(simulated_aim);gate=MotionGate(4.0)
    output=Outputs(Bridge(),read_pose=physical_pose) if args.hybrid else None
    output_lock=threading.Lock();demo_lock=threading.Lock()
    metrics=dict(face_calls=0,hand_calls=0,palm_calls=0,gated_frames=0)
    started_at=time.monotonic()
    lock=threading.Lock()
    current={'data':{'error':'Waiting for first camera frame'},'time':0}
    latest={'jpeg':None,'time':0}
    stopped=threading.Event()
    video=VideoFeed(args.camera)
    def publish_camera(jpeg,stamp):
        with lock:latest.update(jpeg=jpeg,time=stamp)
    def capture():
        while not stopped.is_set():
            try:
                capture_camera(video,stopped,publish_camera)
            except Exception:
                publish_camera(None,0)
                stopped.wait(1)
    def detect():
        bridge=Bridge()
        connected=False
        last_infer=-1e9; active_until=0; last_stamp=0
        while not stopped.is_set():
            started=time.monotonic()
            try:
                with lock: jpeg,stamp=latest['jpeg'],latest['time']
                if jpeg is None or started-stamp>2: raise ConnectionError('Camera unavailable')
                frame=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
                if frame is None: raise ValueError('Invalid camera frame')
                if not connected: connected=bridge.connect(timeout=3)
                joints=[];pivot={}; telemetry_error=None
                try:
                    if not connected: raise ConnectionError('Encoder bridge unavailable')
                    pose=physical_pose(bridge);joints=pose['joints'];pivot=pose['pivot']
                except Exception as exc:
                    joints=[];telemetry_error=str(exc)
                    bridge.disconnect();connected=False
                if stamp==last_stamp:
                    stopped.wait(.03);continue
                last_stamp=stamp
                moving=gate.update(frame)
                if moving:active_until=started+2
                run_models=started<active_until or started-last_infer>=3
                faces=[];hand_data=[]
                model_start=time.monotonic()
                run_faces=run_models or args.hybrid
                # Empty results from a skipped model are not evidence of no faces.
                # Automatic physical lighting always has a fresh face check.
                palms_before=hands.palm_calls
                faces=inference.run(frame,run_faces,run_models)
                if run_faces:metrics['face_calls']+=1
                if run_models:
                    metrics['hand_calls']+=int(hands.available)
                    metrics['palm_calls']+=hands.palm_calls-palms_before
                    for h in hands.last:
                        hand_data.append(dict(points=h.points,score=h.score,gesture=gestures.classify(h.points),tracked=h.tracked,right=h.right))
                    last_infer=started
                else:metrics['gated_frames']+=1
                if faces or hand_data:active_until=started+2
                with demo_lock:
                    candidate=demo.select_pointing_hand(started,hand_data)
                    prefer_virtual=demo.point_following and started-demo.point_at<=.9 and demo.target_mode=='virtual_pointer'
                    target=target_from_hand(candidate,joints,pivot,frame.shape[1],frame.shape[0],prefer_virtual=prefer_virtual) if candidate else None
                    demo_state=demo.update(started,faces,hand_data,frame.shape[1],target)
                model_ms=round((time.monotonic()-model_start)*1000,1)
                # Tracking uses a fresh command estimate after inference, compensating
                # for swivel travel since the camera frame. Projection keeps frame-time pose.
                control_pivot=pivot
                if pivot.get('enabled'):
                    r=bridge.call('arm_roll_status',timeout=.25)
                    if len(r)!=6 or r[0]!=3:raise ValueError('Unsupported head pivot status')
                    control_pivot=dict(enabled=bool(r[1]),current=r[3]/1000,fault=r[5])
                data=dict(size=[frame.shape[1],frame.shape[0]],
                    detections=[dict(x=d.x,y=d.y,w=d.w,h=d.h,score=d.score) for d in faces],
                    face_guard=dict(checked=bool(run_faces),frame_at=stamp if run_faces else None),
                    frame_jpeg=base64.b64encode(jpeg).decode(),joints=joints,pivot=pivot,
                    hands=hand_data,hands_available=hands.available,max_hands=2,demo=demo_state,
                    tracking=tracker.update(time.monotonic(),faces,hand_data,control_pivot,frame.shape[1],observed_pivot=pivot),
                    pipeline=dict(**metrics,**inference.timings,parallel=args.parallel_inference,threads_per_model=args.inference_threads,gesture_classifier=gesture_model,mode='hand ROI tracking' if hand_data and hand_data[0]['tracked'] else 'face + palm detection' if run_models else 'face guard · hands sleeping' if run_faces else 'motion gate · models sleeping',model_ms=model_ms,uptime_s=round(started-started_at,1)),
                    inference_ms=round((time.monotonic()-started)*1000,1),
                    telemetry_error=telemetry_error,source='UNO Q · face + hand w8a8',error=None)
                with lock: current.update(data=data,time=stamp)
            except Exception as exc:
                with lock: current.update(data=dict(error=str(exc)),time=0)
            stopped.wait(max(.02,.25-(time.monotonic()-started)))
        bridge.disconnect()
    def control():
        while not stopped.is_set():
            with lock:data,stamp=current['data'],current['time']
            now=time.monotonic()
            with demo_lock:data={**data,'demo':demo.snapshot(now)}
            with output_lock:
                if output:output.step(data,stamp,now)
            stopped.wait(.1)
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            if self.path!='/control':return self.send_error(404)
            if self.headers.get('Content-Type')!='application/json':return self.send_error(415)
            origin=self.headers.get('Origin')
            if origin and origin!='http://'+self.headers.get('Host',''):return self.send_error(403)
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=256:raise ValueError('Invalid command length')
                request=json.loads(self.rfile.read(length));action=request.get('action')
                if action not in ('enable','disable','heartbeat','stop','resume'):raise ValueError('Unknown control')
                now=time.monotonic()
                if action in ('stop','resume'):
                    with demo_lock:
                        if action=='stop':demo.stop(now)
                        else:demo.resume(now)
                with output_lock:
                    if action=='enable':
                        if not output:raise ValueError('Hybrid outputs are not configured')
                        output.requested=True;output.heartbeat=now
                    elif action=='heartbeat' and output:output.heartbeat=now
                    elif action in ('disable','stop') and output:
                        output.disable(force_light=action=='stop');output.reason='Physical outputs disabled'
                body=json.dumps(dict(ok=True)).encode()
                self.send_response(200)
            except Exception as exc:
                body=json.dumps(dict(ok=False,error=str(exc))).encode();self.send_response(400)
            self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)))
            self.end_headers();self.wfile.write(body)
        def do_GET(self):
            if self.path!='/spatial': return self.send_error(404)
            with lock:
                data=dict(current['data'],age_s=round(time.monotonic()-current['time'],3))
            with demo_lock:data['demo']=demo.snapshot(time.monotonic())
            with output_lock:data['outputs']=output.snapshot() if output else dict(enabled=False,connected=False,reason='Hybrid outputs not configured')
            body=json.dumps(data,allow_nan=False).encode()
            self.send_response(200)
            self.send_header('Content-Type','application/json')
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError): pass
        def log_message(self,*args): pass
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    workers=[threading.Thread(target=fn,daemon=True) for fn in (control,capture,detect)]
    def terminate(*_):
        stopped.set()
        # shutdown must run outside the serve_forever thread.
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,terminate)
    try:
        for worker in workers:worker.start()
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        with output_lock:
            if output:
                try:output.disable()
                except Exception:pass
        for worker in workers:worker.join(timeout=4)
        inference.close()
        server.server_close()


if __name__=='__main__': main()
