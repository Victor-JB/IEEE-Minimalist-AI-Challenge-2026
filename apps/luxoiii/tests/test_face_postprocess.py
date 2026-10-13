"""Check uint8 peak selection against the original float decoder on difficult maps."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
try:
 import cv2,numpy as np
 from luxoiii.perception.face_detector import FaceDetector,Detection
 AVAILABLE=True
except ImportError:AVAILABLE=False


@unittest.skipUnless(AVAILABLE,'OpenCV and a TFLite runtime required')
class FaceDecode(unittest.TestCase):
 def test_optimized_decoder_matches_original_on_random_and_threshold_maps(self):
  model=FaceDetector(num_threads=1)
  original=model.interpreter
  class Tensors:
   def get_tensor(self,index):return tensors[index]
  rng=np.random.default_rng(431)
  try:
   for _ in range(12):
    tensors={d['index']:rng.integers(0,256,d['shape'],dtype=np.uint8) for d in model.outputs.values()}
    model.interpreter=Tensors();model.scale=.75
    # Independent reference: original pre-optimization float32 algorithm.
    heatmap=1/(1+np.exp(-model._output('heatmap')[...,0]));bbox=model._output('bbox')
    ys,xs=np.nonzero((heatmap==cv2.dilate(heatmap,np.ones((3,3),np.uint8))) & (heatmap>model.score_threshold))
    boxes=[];scores=[];k=8/model.scale
    for cx,cy in zip(xs,ys):
     l,t,r,b=bbox[cy,cx];x1,y1=(cx-l)*k,(cy-t)*k;x2,y2=(cx+r)*k,(cy+b)*k
     boxes.append([int(x1),int(y1),int(x2-x1),int(y2-y1)]);scores.append(float(heatmap[cy,cx]))
    keep=cv2.dnn.NMSBoxes(boxes,scores,model.score_threshold,model.nms_threshold)
    expected=[Detection(*boxes[i],scores[i]) for i in np.array(keep).flatten()]
    self.assertEqual(model._postprocess(),expected)
  finally:model.interpreter=original
