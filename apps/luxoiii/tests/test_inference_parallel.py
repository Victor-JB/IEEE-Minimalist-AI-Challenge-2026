"""Concurrent inference must preserve the fresh face guard and drain failed jobs."""
import sys,threading,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools/spatial'))
from inference import Inference


class Detector:
    def __init__(self,fn):self.detect=fn


class ParallelInference(unittest.TestCase):
    def test_models_overlap_and_return_the_same_frame(self):
        barrier=threading.Barrier(2);seen=[];frame=object()
        def detect(f):
            seen.append(f);barrier.wait(timeout=2);return ['face']
        pipeline=Inference(Detector(detect),Detector(detect))
        try:
            self.assertEqual(pipeline.run(frame),['face'])
            self.assertEqual(seen,[frame,frame])
            self.assertEqual(set(pipeline.timings),{'face_ms','hands_ms'})
        finally:pipeline.close()

    def test_sleeping_hands_do_not_skip_face_checks(self):
        calls=[]
        pipeline=Inference(Detector(lambda _:calls.append('face') or []),Detector(lambda _:calls.append('hands')))
        try:
            pipeline.run(None,run_hands=False)
            self.assertEqual(calls,['face'])
        finally:pipeline.close()

    def test_failure_is_propagated_after_other_interpreter_finishes(self):
        barrier=threading.Barrier(2);finished=threading.Event()
        def face(_):barrier.wait(timeout=2);raise ValueError('bad face frame')
        def hand(_):barrier.wait(timeout=2);finished.set()
        pipeline=Inference(Detector(face),Detector(hand))
        try:
            with self.assertRaisesRegex(ValueError,'bad face'):pipeline.run(None)
            self.assertTrue(finished.is_set())
        finally:pipeline.close()
