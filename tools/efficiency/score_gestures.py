#!/usr/bin/env python3
"""Score cached image-model outputs against human gesture labels and boxes.

The primary metric remains the original, unpadded landmark box IoU >= .5.
All target hands, including undetected hands and no-command hands, count.
"""
import argparse
import collections
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
from metrics import matches

MAPPING={'palm':'open_palm','like':'thumbs_up','dislike':'thumbs_down',
         'one':'point','point':'point'}
CANNED=['none','none','open_palm','point','thumbs_down','thumbs_up','none','none']


def score(cache, manifest, classifier=None, threshold=.65):
    ids=[r['id'] for r in manifest['rows']]
    cache_ids=[r['id'] for r in cache['rows']]
    if len(set(ids))!=len(ids) or len(set(cache_ids))!=len(cache_ids) or set(ids)!=set(cache_ids):
        raise ValueError('Cache must cover every manifest image exactly once')
    annotations={r['id']:r for r in manifest['rows']}
    confusion=collections.Counter(); category=collections.defaultdict(collections.Counter)
    tp=fp=fn=correct=command_fp=command_predictions=0
    rows=[]
    for row in cache['rows']:
        ann=annotations[row['id']];h,w=row['shape'][:2]
        gt=[[b[0]*w,b[1]*h,b[2]*w,b[3]*h] for b in ann['boxes']]
        pred=[dict(p,label=(classifier(p) if classifier else p['label']) or 'none')
              for p in row['predictions'] if p['score']>=threshold]
        pairs,extra,missing=matches(pred,gt); tp+=len(pairs);fp+=extra;fn+=missing
        selected={j:p['label'] for p,j,_ in pairs}
        outcomes=[]
        for j,label in enumerate(ann['labels']):
            want=MAPPING.get(label,'none');got=selected.get(j,'miss')
            confusion[want,got]+=1;category[label]['total']+=1
            category[label]['matched']+=got!='miss';category[label]['correct']+=got==want
            correct+=got==want;outcomes.append([want,got])
        good_commands=sum(p['label']==MAPPING.get(ann['labels'][j],'none') and p['label']!='none'
                          for p,j,_ in pairs)
        commands=sum(p['label']!='none' for p in pred)
        command_predictions+=commands;command_fp+=commands-good_commands
        rows.append(dict(id=row['id'],confusion=outcomes,predictions=[{k:p[k] for k in ['box','score','label']} for p in pred]))
    return dict(images=len(rows),gt_hands=tp+fn,matched_hands=tp,unmatched_predictions=fp,
                missed_hands=fn,correct=correct,end_to_end_recall=correct/max(1,tp+fn),
                matched_accuracy=correct/max(1,tp),localization_precision=tp/max(1,tp+fp),
                command_predictions=command_predictions,incorrect_or_unmatched_commands=command_fp,
                command_precision=(command_predictions-command_fp)/max(1,command_predictions),
                confusion={f'{a} -> {b}':v for (a,b),v in sorted(confusion.items())},
                categories=dict(category), rows=rows)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cache',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--classifier',choices=['original','canned','current'],default='original')
    p.add_argument('--threshold',type=float,default=.65)
    a=p.parse_args();cache_bytes=a.cache.read_bytes()
    if a.cache.suffix=='.gz':cache_bytes=gzip.decompress(cache_bytes)
    cache=json.loads(cache_bytes);manifest=json.loads(a.manifest.read_text())
    if cache['manifest_sha256']!=hashlib.sha256(a.manifest.read_bytes()).hexdigest():
        raise ValueError('Manifest does not match cached inference')
    classifier=None;model_metadata={}
    if a.classifier=='canned':classifier=lambda h:CANNED[int(np.argmax(h['canned_probs']))]
    elif a.classifier=='current':
        from luxoiii.perception.gestures import classify, _classifier
        from luxoiii.perception.gesture_features import FEATURE_VERSION
        from luxoiii.paths import MODELS
        if _classifier() is None:raise RuntimeError('Trained classifier required; fallback is not a valid evaluation')
        model_metadata=dict(model_sha256=hashlib.sha256((MODELS/'gesture_landmarks_v1.npz').read_bytes()).hexdigest(),
                            feature_version=FEATURE_VERSION,command_threshold=.5)
        classifier=lambda h:classify(h['points'])
    result=score(cache,manifest,classifier,a.threshold)
    result.update(classifier=a.classifier,landmark_threshold=a.threshold,**model_metadata,
                  manifest_sha256=cache['manifest_sha256'],cache_sha256=hashlib.sha256(cache_bytes).hexdigest())
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))


if __name__=='__main__':main()
