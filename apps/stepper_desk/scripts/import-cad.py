#!/usr/bin/env python3
"""Rebuild visual assets from the pinned accurate CAD branch, without checking it out."""
from pathlib import Path
import subprocess,xml.etree.ElementTree as ET,json
root=Path(__file__).resolve().parents[1]
repo=str(root.parents[1])
ref=subprocess.check_output(['git','rev-parse','ef1b0d2057c5e449a0e02638a1ff7d5b58dd3cbd'],cwd=repo,text=True).strip()
def read(path): return subprocess.check_output(['git','show',f'{ref}:arm_mujoco/{path}'],cwd=repo)
xml=ET.fromstring(read('robot_arm.xml'))
out=root/'mac/Resources/ArmCAD';out.mkdir(parents=True,exist_ok=True)
materials={el.get('name'):[float(v) for v in el.get('rgba','1 1 1 1').split()] for el in xml.findall('asset/material')}
meshes={el.get('name'):el.get('file') for el in xml.findall('asset/mesh')}
def vec(el,name,default):return [float(v) for v in el.get(name,default).split()]
parts=[]
for index,name in enumerate(['first_link','second_link','head']):
 body=xml.find(f'.//body[@name="{name}"]')
 selected=[(body,[0,0,0])]
 if name=='head':selected.append((body.find('body[@name="head_tilt"]'),[0,-4.5,0]))
 for group,offset in selected:
  for geom in group.findall('geom'):
   mesh=geom.get('mesh');filename=meshes[mesh]
   (out/filename).write_bytes(read('meshes/'+filename))
   parts.append(dict(joint=index,file=filename,position=[v*100+offset[i] for i,v in enumerate(vec(geom,'pos','0 0 0'))],quaternion=vec(geom,'quat','1 0 0 0'),color=materials[geom.get('material')]))
manifest=dict(source=f'origin/full-model-animations@{ref}',model='ARM v29',lengths=[18.5,18.5,4.5],parts=parts)
(out/'model.json').write_text(json.dumps(manifest,indent=2)+'\n')
(out/'SOURCE.txt').write_text(f'Fusion ARM v29 visual meshes and transforms from arm_mujoco/robot_arm.xml\nBranch: full-model-animations\nCommit: {ref}\nSTL units: millimetres; manifest translations: centimetres.\nBase yaw assembly excluded; fixed schematic pedestal replaces it.\nThree measured joints: shoulder, elbow, wrist/head. Head-tilt servo fixed at CAD zero.\nInternal cycloid cam/plate subassemblies omitted (not visible through opaque shells).\nCAD remains rigid; no simulated dynamics or position commands.\n')
print(f'Imported {len(parts)} parts / {len(list(out.glob("*.stl")))} meshes from {ref}')
