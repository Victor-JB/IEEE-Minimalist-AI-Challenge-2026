"""Pure evaluation metrics, independent of the board inference runtime."""
import numpy as np

def iou(a,b):
 x=max(a[0],b[0]);y=max(a[1],b[1]);w=max(0,min(a[0]+a[2],b[0]+b[2])-x);h=max(0,min(a[1]+a[3],b[1]+b[3])-y)
 inter=w*h;return inter/max(a[2]*a[3]+b[2]*b[3]-inter,1e-9)

def matches(pred,truth,ignored=()):
 used=set();out=[];fp=0
 for p in sorted(pred,key=lambda x:x['score'],reverse=True):
  eligible=[(iou(p['box'],b),j) for j,b in enumerate(truth) if j not in used]
  overlap,j=max(eligible,default=(0,-1))
  if overlap>=.5:used.add(j);out.append((p,j,overlap))
  elif not any(iou(p['box'],b)>=.5 for b in ignored):fp+=1
 return out,fp,len(truth)-len(used)

def summary(rows):
 if not rows:return {}
 vals=np.array([x['ms'] for x in rows]);cpu=np.array([x['cpu_ms'] for x in rows])
 return dict(n=len(vals),mean_ms=float(vals.mean()),p50_ms=float(np.median(vals)),p95_ms=float(np.percentile(vals,95)),throughput_fps=float(1000/vals.mean()),mean_cpu_ms=float(cpu.mean()))

