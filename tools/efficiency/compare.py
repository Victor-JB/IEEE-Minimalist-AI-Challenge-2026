#!/usr/bin/env python3
"""Compare paired outputs and measurements; never infer accuracy from throughput."""
import argparse,json
from pathlib import Path
import numpy as np


def compare(b,a):
 if b['manifest_hashes']!=a['manifest_hashes']:raise ValueError('Different datasets')
 if b['model_files']!=a['model_files']:raise ValueError('Different model weights')
 out={'model_weights_unchanged':True,'datasets_identical':True,'latency':{}}
 for stage in ['faces','hands','temporal']:
  before=b['summary'][stage];after=a['summary'][stage]
  out['latency'][stage]=dict(before_mean_ms=before['mean_ms'],after_mean_ms=after['mean_ms'],latency_reduction_percent=100*(1-after['mean_ms']/before['mean_ms']),speedup=before['mean_ms']/after['mean_ms'],cpu_reduction_percent=100*(1-after['mean_cpu_ms']/before['mean_cpu_ms']))
 for stage in ['faces','hands']:
  pairs=list(zip(b[stage],a[stage]));assert len(b[stage])==len(a[stage])
  assert all(x['id']==y['id'] for x,y in pairs)
  out[stage+'_identical_predictions']=sum(x['predictions']==y['predictions'] for x,y in pairs)
  out[stage+'_samples']=len(pairs)
 for label,data in [('baseline',b),('optimized',a)]:
  out[label+'_accuracy']={k:data['summary'][k] for k in ['face_recall_32px','matched_gesture_accuracy','end_to_end_gesture_recall']}
  out[label+'_accuracy']['faces']={k:data['summary']['faces'][k] for k in ['tp','fp','fn','precision','recall']}
  out[label+'_accuracy']['hands']={k:data['summary']['hands'][k] for k in ['tp','fp','fn','precision','recall']}
  out[label+'_palm_calls']=data['summary']['temporal']['palm_calls']
  out[label+'_peak_rss_kib']=data['peak_rss_kib']
 # Sequence retention compares per-frame results, separate from static accuracy.
 out['temporal_label_agreement']=sum(x['labels']==y['labels'] for x,y in zip(b['temporal'],a['temporal']))/len(b['temporal'])
 out['temporal_hand_count_agreement']=sum(x['hand_count']==y['hand_count'] for x,y in zip(b['temporal'],a['temporal']))/len(b['temporal'])
 # Paired bootstrap by source image/sequence (avoid counting correlated frames as independent).
 rng=np.random.default_rng(20261013)
 for stage in ['faces','hands','temporal']:
  ids=sorted({x['id'] for x in b[stage]})
  def group(data):return np.array([np.mean([r['ms'] for r in data[stage] if r['id']==key]) for key in ids])
  bv,av=group(b),group(a);sample=rng.integers(0,len(ids),(2000,len(ids)))
  reductions=100*(1-av[sample].mean(1)/bv[sample].mean(1))
  out['latency'][stage]['paired_bootstrap_95pct_latency_reduction']=np.percentile(reductions,[2.5,97.5]).tolist()
 return out

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('optimized',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 result=compare(json.loads(a.baseline.read_text()),json.loads(a.optimized.read_text()));a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
