#!/usr/bin/env python3
"""Reproducible UNO Q latency + public-data accuracy evaluation, no hardware RPC.

Face metric: greedy score-ordered one-to-one box matches at IoU >= 0.5 on a
fixed WIDER validation subset (NOT the official WIDER easy/medium/hard AP).
Gesture metric: matching landmark bounding boxes at IoU >= 0.5, including
missed hands, with full confusion counts. A reset between unrelated stills
prevents accidental temporal leakage. Synthetic translations below test ROI
consistency and workload only; they are not a real-video accuracy benchmark.
"""
import argparse,collections,hashlib,json,os,platform,resource,sys,time
from pathlib import Path
import cv2,numpy as np
from luxoiii.perception.face_detector import FaceDetector,MODEL_PATH
from luxoiii.perception.hands import HandLandmarker,MODEL_DIR
from luxoiii.perception.gestures import classify
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'spatial'))
from inference import Inference
from metrics import matches, summary

MAPPING={'palm':'open_palm','like':'thumbs_up','dislike':'thumbs_down','one':'point','point':'point'}

def frame_for(root,row):
 path=root/row['path'];raw=path.read_bytes()
 if hashlib.sha256(raw).hexdigest()!=row['sha256']:raise ValueError('Dataset hash mismatch: '+str(path))
 im=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
 if im is None:raise ValueError('Invalid dataset image '+str(path))
 h,w=im.shape[:2];s=min(640/w,480/h)
 return cv2.resize(im,(round(w*s),round(h*s))),s

def run(args):
 cv2.setNumThreads(1)
 face=FaceDetector(num_threads=args.threads)
 options={} if args.discovery_interval is None else dict(discovery_interval=args.discovery_interval)
 hands=HandLandmarker(num_threads=args.threads,max_hands=2,landmark_threshold=.65,**options)
 if not hands.available:raise RuntimeError('Required hand models unavailable')
 workload=Inference(face,hands,parallel=args.parallel)
 blank=np.zeros((480,640,3),np.uint8)
 for _ in range(6):workload.run(blank)
 root=args.data;wf=json.loads((root/'wider-manifest.json').read_text());hg=json.loads((root/'hagrid/manifest.json').read_text())
 result=dict(label=args.label,threads=args.threads,parallel=args.parallel,discovery_interval=getattr(hands,'discovery_interval',1),platform=platform.platform(),numpy=np.__version__,opencv=cv2.__version__,opencv_threads=1,model_files={str(p.relative_to(MODEL_PATH.parents[1])):dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in [MODEL_PATH,*sorted(MODEL_DIR.glob('*.tflite'))]},manifest_hashes={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in ['wider-manifest.json','hagrid/manifest.json']},faces=[],hands=[],temporal=[])
 def timed(frame,run_hands):
  cpu=time.process_time();start=time.perf_counter();detections=workload.run(frame,run_hands=run_hands)
  timing=dict(ms=(time.perf_counter()-start)*1000,cpu_ms=(time.process_time()-cpu)*1000)
  return [dict(box=[d.x,d.y,d.w,d.h],score=d.score) for d in detections],timing
 try:
  for index,row in enumerate([] if args.only_temporal else wf['rows']):
   frame,s=frame_for(root,row);pred,timing=timed(frame,False)
   gt=[[v*s for v in b] for b in row['boxes']];ignored=[[v*s for v in b] for b in row['ignored_boxes']]
   match,fp,fn=matches(pred,gt,ignored)
   large={j for j,b in enumerate(gt) if min(b[2:])>=32}
   result['faces'].append(dict(id=row['id'],**timing,tp=len(match),fp=fp,fn=fn,large_total=len(large),large_tp=sum(j in large for _,j,_ in match),presence_hit=bool(pred),predictions=pred))
   if (index+1)%60==0:print(args.label,'faces',index+1,flush=True)
  for index,row in enumerate([] if args.only_temporal else hg['rows']):
   frame,s=frame_for(root/'hagrid',row);hands.reset();_,timing=timed(frame,True)
   gt=[[b[0]*frame.shape[1],b[1]*frame.shape[0],b[2]*frame.shape[1],b[3]*frame.shape[0]] for b in row['boxes']]
   preds=[]
   for h in hands.last:
    points=np.array(h.points);xy=points[:,:2];low=xy.min(0);size=np.ptp(xy,axis=0)
    preds.append(dict(box=[*low.tolist(),*size.tolist()],score=h.score,label=classify(h.points) or 'none'))
   match,fp,fn=matches(preds,gt);labels=[MAPPING.get(x,'none') for x in row['labels']];matched={j:p['label'] for p,j,_ in match}
   result['hands'].append(dict(id=row['id'],category=row['category'],**timing,tp=len(match),fp=fp,fn=fn,confusion=[[label,matched.get(j,'miss')] for j,label in enumerate(labels)],predictions=preds))
   if (index+1)%35==0:print(args.label,'hands',index+1,flush=True)
  # Two preselected images per active gesture category, irrespective of results.
  seq=[]
  for category in ['palm','like','dislike','one','point','fist']:
   seq.extend([r for r in hg['rows'] if r['category']==category][:2])
  for row in seq:
   frame,s=frame_for(root/'hagrid',row);hands.reset();palms=0
   for j in range(12):
    dx=round(6*np.sin(j/3));dy=round(3*np.cos(j/3))
    shifted=cv2.warpAffine(frame,np.float32([[1,0,dx],[0,1,dy]]),(frame.shape[1],frame.shape[0]))
    # Count actual palm invocations even with the unchanged baseline class.
    original=hands._palms;count=[0]
    def counted(*a,**kw):count[0]+=1;return original(*a,**kw)
    hands._palms=counted
    try:_,timing=timed(shifted,True)
    finally:hands._palms=original
    gt=[[b[0]*frame.shape[1]+dx,b[1]*frame.shape[0]+dy,b[2]*frame.shape[1],b[3]*frame.shape[0]] for b in row['boxes']]
    predictions=[]
    for h in hands.last:
     xy=np.asarray(h.points)[:,:2]
     predictions.append(dict(box=[*xy.min(0).tolist(),*np.ptp(xy,axis=0).tolist()],score=h.score,label=classify(h.points) or 'none'))
    match,fp,fn=matches(predictions,gt);matched={k:p['label'] for p,k,_ in match}
    confusion=[[MAPPING.get(label,'none'),matched.get(k,'miss')] for k,label in enumerate(row['labels'])]
    result['temporal'].append(dict(id=row['id'],frame=j,**timing,palm_calls=count[0],hand_count=len(hands.last),labels=[p['label'] for p in predictions],predictions=predictions,tp=len(match),fp=fp,fn=fn,confusion=confusion))
  result['summary']={stage:summary(result[stage]) for stage in ['faces','hands','temporal']}
  for stage in ['faces','hands']:
   sums={k:sum(r[k] for r in result[stage]) for k in ['tp','fp','fn']};sums.update(precision=sums['tp']/max(1,sums['tp']+sums['fp']),recall=sums['tp']/max(1,sums['tp']+sums['fn']))
   result['summary'][stage].update(sums)
  conf=collections.Counter(tuple(pair) for row in result['hands'] for pair in row['confusion'])
  result['summary']['gesture_confusion']={f'{a} -> {b}':n for (a,b),n in sorted(conf.items())}
  result['summary']['matched_gesture_accuracy']=sum(n for (a,b),n in conf.items() if a==b)/max(1,sum(n for (a,b),n in conf.items() if b!='miss'))
  result['summary']['end_to_end_gesture_recall']=sum(n for (a,b),n in conf.items() if a==b)/max(1,sum(conf.values()))
  result['summary']['face_recall_32px']=sum(r['large_tp'] for r in result['faces'])/max(1,sum(r['large_total'] for r in result['faces']))
  result['summary']['temporal']['correct_gestures']=sum(a==b for r in result['temporal'] for a,b in r['confusion'])
  result['summary']['temporal']['ground_truth_hands']=sum(len(r['confusion']) for r in result['temporal'])
  result['summary']['temporal']['matched_hands']=sum(r['tp'] for r in result['temporal'])
  result['summary']['temporal']['palm_calls']=sum(r['palm_calls'] for r in result['temporal'])
  result['peak_rss_kib']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
  result['end_temperature_c_max_sensor']=max([float(p.read_text())/1000 for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')],default=None)
  args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
  print(json.dumps(result['summary'],indent=2),flush=True)
 finally:workload.close()

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--data',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--label',required=True);p.add_argument('--threads',type=int,default=2);p.add_argument('--parallel',action='store_true');p.add_argument('--only-temporal',action='store_true');p.add_argument('--discovery-interval',type=int);run(p.parse_args())
