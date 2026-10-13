#!/usr/bin/env python3
"""Predecode public benchmark photographs to lossless model-sized PNGs.

This saves board transfer/storage, while preserving the exact resized pixels
used by evaluate.py. Source hashes and manifest are recorded for traceability.
No image crop or label filtering is performed.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    path=args.source/'manifest.json';manifest=json.loads(path.read_text())
    rows=[]
    for row in manifest['rows']:
        raw=(args.source/row['path']).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=row['sha256']:raise ValueError(row['id'])
        im=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        h,w=im.shape[:2];s=min(640/w,480/h)
        frame=cv2.resize(im,(round(w*s),round(h*s)))
        out=args.output/'images'/row['category']/(row['id']+'.png')
        out.parent.mkdir(parents=True,exist_ok=True)
        if not cv2.imwrite(str(out),frame):raise OSError(str(out))
        if not np.array_equal(cv2.imread(str(out)),frame):raise ValueError('Lossless roundtrip failed')
        rows.append(dict(row,path=str(out.relative_to(args.output)),
                         source_path=row['path'],source_sha256=row['sha256'],
                         sha256=hashlib.sha256(out.read_bytes()).hexdigest()))
    manifest.update(rows=rows,source_manifest_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    frame_preparation='cv2 INTER_LINEAR fit 640x480; lossless PNG; no crop',
                    preparation_opencv=cv2.__version__)
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'{len(rows)} lossless frames ready: {args.output}')


if __name__=='__main__':main()
