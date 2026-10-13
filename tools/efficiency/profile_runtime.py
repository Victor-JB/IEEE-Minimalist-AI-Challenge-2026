#!/usr/bin/env python3
"""CPU/runtime sweep; random input is latency-only, never an accuracy claim."""
import argparse,json,platform,resource,time
from pathlib import Path
import cv2,numpy as np
from luxoiii.perception.face_detector import FaceDetector
from luxoiii.perception.hands import HandLandmarker, Rect
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--threads',nargs='+',type=int,default=[1,2,4]);a=p.parse_args()
rng=np.random.default_rng(20261013);frame=rng.integers(0,256,(480,640,3),dtype=np.uint8)
rows=[]
for threads in a.threads:
 cv2.setNumThreads(1)
 face=FaceDetector(num_threads=threads);hands=HandLandmarker(num_threads=threads,max_hands=2)
 stages={'face':lambda:face.detect(frame),'palm':lambda:hands._palms(frame,None),'landmark':lambda:hands._landmarks(frame,Rect(320,240,250,0))}
 for stage,fn in stages.items():
  for _ in range(4):fn()
  times=[];cpu=time.process_time();start=time.perf_counter()
  for _ in range(24):
   t=time.perf_counter();fn();times.append((time.perf_counter()-t)*1000)
  row=dict(threads=threads,stage=stage,n=len(times),mean_ms=float(np.mean(times)),p50_ms=float(np.median(times)),p95_ms=float(np.percentile(times,95)),cpu_ms=(time.process_time()-cpu)*1000/len(times),rss_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
  rows.append(row);print(json.dumps(row),flush=True)
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(dict(platform=platform.platform(),input='seeded random 640x480; latency only',opencv_threads=1,results=rows),indent=2)+'\n')
