"""Metric helpers use one-to-one matching and count missing predictions."""
import importlib.util,sys,unittest
from pathlib import Path
path=Path(__file__).resolve().parents[3]/'tools/efficiency/metrics.py'
spec=importlib.util.spec_from_file_location('efficiency_metrics',path)
metrics=importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)

class Metrics(unittest.TestCase):
 def test_duplicate_detection_is_false_positive(self):
  pred=[dict(box=[0,0,10,10],score=.9),dict(box=[0,0,10,10],score=.8)]
  hit,fp,fn=metrics.matches(pred,[[0,0,10,10]])
  self.assertEqual((len(hit),fp,fn),(1,1,0))
 def test_missed_ground_truth_counts_as_false_negative(self):
  self.assertEqual(metrics.matches([],[[0,0,10,10]])[1:],(0,1))
 def test_invalid_face_does_not_count_as_false_positive(self):
  self.assertEqual(metrics.matches([dict(box=[0,0,10,10],score=.8)],[],[[0,0,10,10]])[1:],(0,0))
 def test_boxes_require_half_iou(self):
  hit,fp,fn=metrics.matches([dict(box=[8,8,10,10],score=.9)],[[0,0,10,10]])
  self.assertEqual((len(hit),fp,fn),(0,1,1))
