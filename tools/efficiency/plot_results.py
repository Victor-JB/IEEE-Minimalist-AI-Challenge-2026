#!/usr/bin/env python3
"""Render the measured on-board comparison for the judging scorecard."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('optimized',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
b=json.loads(a.baseline.read_text());o=json.loads(a.optimized.read_text())
plt.rcParams.update({'font.family':'DejaVu Sans','text.color':'#514a3d','axes.labelcolor':'#514a3d','xtick.color':'#514a3d','ytick.color':'#514a3d','font.size':11})
fig,axes=plt.subplots(1,3,figsize=(13,5));fig.patch.set_facecolor('#fcfaf6')
for ax,stage,title in zip(axes,['faces','hands','temporal'],['Face detection','Face + hand acquisition','Synthetic tracking']):
 v=[b['summary'][stage]['mean_ms'],o['summary'][stage]['mean_ms']]
 ax.set_facecolor('#fcfaf6');ax.bar([0,1],v,color=['#6d685e','#d4a14b'],width=.55)
 ax.set_xticks([0,1],['Baseline','Optimized']);ax.set_ylim(0,max(v)*1.22)
 for i,value in enumerate(v):ax.text(i,value+max(v)*.025,f'{value:.1f} ms',ha='center',fontweight='bold')
 ax.set_title(title+'\n'+f'{100*(1-v[1]/v[0]):.0f}% lower mean latency',pad=16,fontsize=13)
 ax.spines[['top','right','left']].set_visible(False);ax.spines['bottom'].set_color('#d8d2c6');ax.yaxis.grid(True,color='#e6e1d7');ax.set_axisbelow(True)
 ax.set_ylabel('Mean model-workload latency (ms)')
fig.suptitle('luxoiii | measured on Arduino UNO Q',x=.07,ha='left',fontsize=22,fontfamily='DejaVu Serif',y=.98)
fig.text(.07,.025,'240 WIDER FACE images + 210 HaGRID images. Synthetic tracking: 144 translated frames, not a real-video test.\nLatency includes model preprocessing and decoding; excludes camera capture, JPEG decoding and hardware RPC.',fontsize=10,color='#756d60')
fig.tight_layout(w_pad=3,rect=(0,.15,1,.92));a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=180,bbox_inches='tight',facecolor=fig.get_facecolor())
