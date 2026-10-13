#!/usr/bin/env python3
"""Prepare deterministic public evaluation samples. Images stay outside Git.

HaGRID: official subject-disjoint test annotations; one image per subject per class.
WIDER FACE: official validation annotations; deterministic sample, no score filtering.
Neither corpus is used to train weights or tune thresholds by this script.
"""
import argparse,concurrent.futures,hashlib,json,zipfile
from pathlib import Path
from remote_zip import RemoteZipFile

ROOT='https://rndml-team-cv.obs.ru-moscow-1.hc.sbercloud.ru/datasets/'
ANNOT=ROOT+'hagrid_v2/annotations_with_landmarks/annotations.zip'
CLASSES=['palm','like','dislike','one','point','fist','no_gesture']
def rank(value):return hashlib.sha256(('luxoiii-eval-v1:'+value).encode()).hexdigest()
def sha(data):return hashlib.sha256(data).hexdigest()
def hagrid(root,n):
 root.mkdir(parents=True,exist_ok=True)
 with zipfile.ZipFile(RemoteZipFile(ANNOT)) as z:
  selections=[]
  for label in CLASSES:
   cache=root/(label+'-test.json')
   if not cache.exists():cache.write_bytes(z.read('annotations/test/'+label+'.json'))
   annotations=json.loads(cache.read_text());seen=set();selected=[]
   for key in sorted(annotations,key=rank):
    a=annotations[key];user=a['user_id']
    if user in seen:continue
    seen.add(user);selected.append((key,a))
    if len(selected)==n:break
   selections.append((label,selected))
 def download(selection):
  label,selected=selection
  location=('hagrid/hagrid_dataset_new_554800/hagrid_dataset/' if label in ['palm','like','dislike','fist'] else 'hagrid_v2/hagrid_v2_zip/')
  url=ROOT+location+label+'.zip';remote=RemoteZipFile(url)
  with zipfile.ZipFile(remote) as z:
   by_id={Path(info.filename).stem:info for info in z.infolist() if info.filename.endswith('.jpg')}
   rows=[]
   for key,a in selected:
    path=root/'images'/label/(key+'.jpg');path.parent.mkdir(parents=True,exist_ok=True)
    if not path.exists():path.write_bytes(z.read(by_id[key]))
    data=path.read_bytes()
    rows.append(dict(id=key,path=str(path.relative_to(root)),sha256=sha(data),category=label,boxes=a['bboxes'],labels=a['labels'],subject_hash=sha(a['user_id'].encode()),archive=url))
   print(f'HaGRID {label}: {len(rows)} images, {remote.downloaded/1e6:.1f} MB transferred',flush=True)
   return rows
 with concurrent.futures.ThreadPoolExecutor(7) as pool:rows=sum(pool.map(download,selections),[])
 result=dict(dataset='HaGRIDv2',split='test',selection='SHA256 luxoiii-eval-v1, one image per subject per category',per_category=n,annotation_url=ANNOT,rows=rows)
 (root/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
 print('HaGRID ready:',len(rows),flush=True)

def wider(root,n):
 with zipfile.ZipFile(root/'wider_face_split.zip') as z:
  lines=iter(z.read('wider_face_split/wider_face_val_bbx_gt.txt').decode().splitlines())
  rows=[]
  for name in lines:
   count=int(next(lines));ann=[list(map(int,next(lines).split())) for _ in range(max(count,1))]
   rows.append(dict(id=name,boxes=[x[:4] for x in ann if x[2]>0 and x[3]>0 and not x[7]],ignored_boxes=[x[:4] for x in ann if x[2]>0 and x[3]>0 and x[7]]))
 selected=sorted(rows,key=lambda x:rank(x['id']))[:n]
 with zipfile.ZipFile(root/'WIDER_val.zip') as z:
  for row in selected:
   data=z.read('WIDER_val/images/'+row['id']);path=root/'wider-images'/row['id'];path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
   row.update(path=str(path.relative_to(root)),sha256=sha(data))
 result=dict(dataset='WIDER FACE',split='validation',selection='SHA256 luxoiii-eval-v1 sorted image IDs, first N; invalid boxes ignored',rows=selected)
 (root/'wider-manifest.json').write_text(json.dumps(result,indent=2)+'\n');print('WIDER ready:',len(selected),flush=True)

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--hagrid',type=int,default=0);p.add_argument('--wider',type=int,default=0);a=p.parse_args()
 if a.wider:wider(a.root,a.wider)
 if a.hagrid:hagrid(a.root/'hagrid',a.hagrid)
