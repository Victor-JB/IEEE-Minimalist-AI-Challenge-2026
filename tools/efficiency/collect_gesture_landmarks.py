#!/usr/bin/env python3
"""Cache actual UNO Q landmark outputs for offline classifier development.

Human labels/boxes remain evaluation truth. Cached MediaPipe annotation landmarks
are not used here. Experiments change palm crop size explicitly; no GT crop is fed
to inference. Confidence filtering can be studied without repeating inference.
"""
import argparse
import hashlib
import json
import platform
import time
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np
from luxoiii.perception import hands as H
from luxoiii.perception.gestures import classify_rules as classify


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data', type=Path, required=True, help='Folder containing manifest.json')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--palm-threshold', type=float, default=.5)
    p.add_argument('--crop-scale', type=float, default=2.6)
    p.add_argument('--threads', type=int, default=4)
    p.add_argument('--canned', action='store_true')
    a = p.parse_args()
    cv2.setNumThreads(1)
    H.PALM_SCALE = a.crop_scale
    hands = H.HandLandmarker(num_threads=a.threads, max_hands=2,
                            landmark_threshold=0, palm_threshold=a.palm_threshold)
    if not hands.available:
        raise RuntimeError('Required models unavailable')
    canned = H.CannedGestures() if a.canned else None
    manifest_path = a.data / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    result = dict(manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
                  split=manifest['split'], platform=platform.platform(),
                  threads=a.threads, palm_threshold=a.palm_threshold, crop_scale=a.crop_scale,
                  landmark_threshold=0, rows=[])
    for _ in range(5):
        hands.detect(np.zeros((480,640,3),np.uint8))
    for i, row in enumerate(manifest['rows']):
        raw = (a.data / row['path']).read_bytes()
        if hashlib.sha256(raw).hexdigest() != row['sha256']:
            raise ValueError('Image hash mismatch: '+row['id'])
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        h,w = image.shape[:2]; scale = min(640/w,480/h)
        frame = cv2.resize(image,(round(w*scale),round(h*scale)))
        hands.reset(); t = time.perf_counter(); hands.detect(frame)
        elapsed = (time.perf_counter()-t)*1000
        predictions=[]
        for hand in hands.last:
            xy=np.asarray(hand.points)[:,:2]
            pred=dict(points=hand.points, world=hand.world.tolist(), score=hand.score,
                      right=hand.right, rect={k:float(v) for k,v in asdict(hand.rect).items()}, label=classify(hand.points),
                      box=[*xy.min(0).tolist(),*np.ptp(xy,axis=0).tolist()])
            if canned:
                pred['canned_probs']=canned.probs(hand.points,hand.right).tolist()
            predictions.append(pred)
        result['rows'].append(dict(id=row['id'],category=row['category'],shape=list(frame.shape),
                                   ms=elapsed, predictions=predictions))
        if (i+1)%20==0:
            print(f'{manifest["split"]}: {i+1}/{len(manifest["rows"])}',flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(a.output,flush=True)


if __name__=='__main__':
    main()
