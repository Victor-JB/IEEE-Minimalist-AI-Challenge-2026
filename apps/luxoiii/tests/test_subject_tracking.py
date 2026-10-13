import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools/spatial'))
from tracking import SubjectTracker


def face(x,w=80):return SimpleNamespace(x=x-w/2,w=w,h=w)
def hand(x):return dict(points=[[x,240,0] for _ in range(21)],score=.9)
def pivot(q):return dict(enabled=True,current=q,fault=0)


class TrackingTests(unittest.TestCase):
    def test_tracks_both_directions_and_stops_in_deadband(self):
        t=SubjectTracker()
        right=t.update(0,[face(500)],[],pivot(0))
        self.assertGreater(right['target'],0)
        left=t.update(.3,[face(100)],[],pivot(right['target']))
        self.assertLess(left['target'],right['target'])
        centered=t.update(.6,[face(320)],[],pivot(left['target']))
        self.assertEqual(centered['target'],left['target'])

    def test_prefers_faces_then_falls_back_to_hand(self):
        t=SubjectTracker()
        s=t.update(0,[face(500)],[hand(50)],pivot(0));self.assertEqual(s['subject'],'face');self.assertGreater(s['target'],0)
        s=t.update(.4,[],[hand(50)],pivot(3));self.assertFalse(s['visible'])
        s=t.update(1.2,[],[hand(50)],pivot(3));self.assertEqual(s['subject'],'hand');self.assertLess(s['target'],3)

    def test_holds_lost_subject_instead_of_scanning_or_recentering(self):
        t=SubjectTracker();s=t.update(0,[face(500)],[],pivot(0))
        for now in (.5,2,20):
            lost=t.update(now,[],[],pivot(s['target']))
            self.assertEqual(lost['target'],s['target']);self.assertFalse(lost['visible'])

    def test_limits_and_stable_subject_selection(self):
        t=SubjectTracker()
        for i in range(30):
            s=t.update(i*.25,[face(600)],[],pivot(t.target))
            self.assertLessEqual(abs(s['target']),10)
        self.assertEqual(s['target'],10)
        t=SubjectTracker();t.update(0,[face(200)],[],pivot(0))
        t.update(.25,[face(205),face(500,200)],[],pivot(t.target))
        self.assertAlmostEqual(t.center,205/640)
