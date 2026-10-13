#!/usr/bin/env python3
"""Paired old/new gesture latency with identical four-thread image models.

No hardware RPC. The first five manifest images per category are selected
before seeing outcomes. ABBA order reduces warm-up/thermal order effects.
"""
import argparse
import hashlib
import json
import platform
import time
from pathlib import Path

import cv2
import numpy as np
from luxoiii.perception.face_detector import FaceDetector
from luxoiii.perception.hands import HandLandmarker
from luxoiii.perception.gestures import classify, classify_rules


def summary(values):
    return dict(mean_ms=float(np.mean(values)),p50_ms=float(np.median(values)),
                p95_ms=float(np.percentile(values,95)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cache',type=Path,required=True)
    a=p.parse_args();cv2.setNumThreads(1)
    hands=HandLandmarker(num_threads=4,max_hands=2,landmark_threshold=.65)
    face=FaceDetector(num_threads=4)
    path=a.data/'manifest.json';manifest=json.loads(path.read_text());chosen=[]
    for category in sorted({r['category'] for r in manifest['rows']}):
        chosen.extend([r for r in manifest['rows'] if r['category']==category][:5])
    cache=json.loads(a.cache.read_text())
    poses=[h['points'] for row in cache['rows'] for h in row['predictions'] if h['score']>=.65]
    if not poses:raise ValueError('No detected hands to benchmark')
    micro={}
    for label,fn in [('original',classify_rules),('new',classify)]:
        for pose in poses[:20]:fn(pose)
        values=[]
        for _ in range(3):
            for pose in poses:
                t=time.perf_counter();fn(pose);values.append((time.perf_counter()-t)*1000)
        micro[label]=dict(calls=len(values),**summary(values))
    for _ in range(6):
        blank=np.zeros((480,640,3),np.uint8);face.detect(blank);hands.detect(blank)
    result=[]
    for row in chosen:
        raw=(a.data/row['path']).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=row['sha256']:raise ValueError(row['id'])
        frame=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        for label,fn in [('original',classify_rules),('new',classify),('new',classify),('original',classify_rules)]:
            hands.reset();t=time.perf_counter();face.detect(frame);hands.detect(frame)
            gestures=[fn(h.points) for h in hands.last]
            result.append(dict(id=row['id'],variant=label,ms=(time.perf_counter()-t)*1000,hands=len(hands.last),gestures=gestures))
        print(row['category'],row['id'],flush=True)
    report=dict(platform=platform.platform(),threads_per_model=4,parallel=False,opencv_threads=1,
                model_resolution='unchanged',selection='first five images per category in frozen manifest; ABBA order',
                manifest_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),classifier=micro,
                workload={label:summary([r['ms'] for r in result if r['variant']==label]) for label in ['original','new']},rows=result)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))


if __name__=='__main__':main()
