#!/usr/bin/env python3
"""UNO Q read-only face/encoder observer. Reuses the desk camera stream; never owns motors.

Only firmware reads: arm_roll_status and stepper_command op 4, without HELLO or
claim. Unclaimed status returns EACCES plus raw encoder samples; apply saved
calibration here without changing firmware configuration. Keep HTTP on loopback.
"""
import argparse
import base64
import importlib
import json
from pathlib import Path
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import urlopen


def encoder(raw, joint, calibration):
    if len(raw) != 31 or raw[0] != 7 or raw[1] != joint or raw[3] not in (0, -13, -16):
        raise ValueError('Unsupported raw encoder status')
    count, magnet, error, age = raw[14:18]
    good = 0 <= count < 4096 and magnet & 0x28 == 0x20 and error == 0 and 0 <= age <= 100
    angle = ((count*360/4096-calibration['zero'])*(-1 if calibration['reversed'] else 1)+180)%360-180
    return dict(angle=angle if good else None, healthy=good, age_ms=age,
                source='raw encoder + saved calibration')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo', type=Path, required=True)
    p.add_argument('--package', default='luxoiii', choices=['luxoiii','lumen'])
    p.add_argument('--port', type=int, default=8083)
    args=p.parse_args()
    sys.path.insert(0,str(args.repo/'apps'/args.package))
    FaceDetector=importlib.import_module(args.package+'.perception.face_detector').FaceDetector
    from arduino.router_bridge import Bridge
    import cv2
    import numpy as np
    cal=json.loads((args.repo/'arm/calibration/arm-calibration-2026-10-11.json').read_text())['saved_calibration']['joints']
    if len(cal)!=3 or any(c.get('calibrated') is not True for c in cal):
        raise ValueError('Saved joint calibration required')
    detector=FaceDetector(num_threads=2)
    lock=threading.Lock()
    current={'data':{'error':'Waiting for first camera frame'},'time':0}
    latest={'jpeg':None,'time':0}
    def capture():
        while True:
            try:
                with urlopen('http://127.0.0.1:8081/video.mjpg',timeout=5) as stream:
                    buf=b''
                    while True:
                        chunk=stream.read(4096)
                        if not chunk: raise ConnectionError('Camera stream ended')
                        buf+=chunk
                        while True:
                            start=buf.find(b'\xff\xd8'); end=buf.find(b'\xff\xd9',max(start,0))
                            if start<0 or end<0: break
                            frame=buf[start:end+2];buf=buf[end+2:]
                            with lock: latest.update(jpeg=frame,time=time.monotonic())
                        if len(buf)>2_000_000: buf=b''
            except Exception:
                with lock: latest.update(jpeg=None,time=0)
                time.sleep(1)
    def detect():
        bridge=Bridge()
        connected=False
        while True:
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
                    for j in range(3):
                        raw=bridge.call('stepper_command',[4,j,2147483647],timeout=.25)
                        joints.append(encoder(raw,j,cal[j]))
                    r=bridge.call('arm_roll_status',timeout=.25)
                    if len(r)!=6 or r[0]!=3: raise ValueError('Unsupported head pivot status')
                    pivot=dict(enabled=bool(r[1]),current=r[3]/1000,fault=r[5])
                except Exception as exc:
                    joints=[];telemetry_error=str(exc)
                    bridge.disconnect();connected=False
                faces=detector.detect(frame)
                data=dict(size=[frame.shape[1],frame.shape[0]],
                    detections=[dict(x=d.x,y=d.y,w=d.w,h=d.h,score=d.score) for d in faces],
                    frame_jpeg=base64.b64encode(jpeg).decode(),joints=joints,pivot=pivot,
                    inference_ms=round((time.monotonic()-started)*1000,1),
                    telemetry_error=telemetry_error,source='UNO Q · face_det_lite w8a8',error=None)
                with lock: current.update(data=data,time=stamp)
            except Exception as exc:
                with lock: current.update(data=dict(error=str(exc)),time=0)
            time.sleep(max(.02,.5-(time.monotonic()-started)))
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path!='/spatial': return self.send_error(404)
            with lock:
                data=dict(current['data'],age_s=round(time.monotonic()-current['time'],3))
            body=json.dumps(data,allow_nan=False).encode()
            self.send_response(200)
            self.send_header('Content-Type','application/json')
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError): pass
        def log_message(self,*args): pass
    threading.Thread(target=capture,daemon=True).start()
    threading.Thread(target=detect,daemon=True).start()
    ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever()


if __name__=='__main__': main()
