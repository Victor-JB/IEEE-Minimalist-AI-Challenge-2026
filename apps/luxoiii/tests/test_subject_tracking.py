import math
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools/spatial'))
from tracking import SubjectTracker


def face(x,w=80,h=None):return SimpleNamespace(x=x-w/2,w=w,h=w if h is None else h)
def hand(x):return dict(points=[[x,240,0] for _ in range(21)],score=.9)
def pivot(q):return dict(enabled=True,current=q,fault=0)
def pixel(error):return 320+320*math.tan(math.radians(error))/math.tan(math.radians(50))


class TrackingTests(unittest.TestCase):
    def test_confirmation_and_direction_reversal(self):
        t=SubjectTracker()
        self.assertEqual(t.update(0,[face(580)],[],pivot(0))['target'],0)
        right=t.update(.3,[face(580)],[],pivot(0))['target'];self.assertGreater(right,0)
        self.assertLessEqual(right,6)
        # A single far-left detection cannot kick the head back to the other side.
        self.assertEqual(t.update(.6,[face(40)],[],pivot(right))['target'],right)
        self.assertLess(t.update(.9,[face(40)],[],pivot(right))['target'],right)

    def test_wide_hold_zone_and_hysteresis_reject_jitter(self):
        t=SubjectTracker()
        for i,x in enumerate((220,440,460,478,450,320,190,175,160,190,250)):
            s=t.update(i*.3,[face(x)],[],pivot(12))
            self.assertEqual(s['target'],12)
        self.assertEqual(s['deadzone_fraction'],.4);self.assertEqual(s['reengage_fraction'],.52)
        t.update(4,[face(540)],[],pivot(12))
        s=t.update(4.3,[face(540)],[],pivot(12));self.assertGreater(s['target'],12)
        # Once moving, it stops at the inner zone and won't reengage in the hysteresis band.
        self.assertEqual(t.update(4.6,[face(420)],[],pivot(14))['target'],14)
        self.assertEqual(t.update(4.9,[face(475)],[],pivot(14))['target'],14)

    def test_derivative_brakes_but_never_reverses_a_closing_error(self):
        t=SubjectTracker();t.update(0,[face(600)],[],pivot(0));t.update(.3,[face(600)],[],pivot(0))
        s=t.update(.6,[face(505)],[],pivot(6))
        edge=math.degrees(math.atan(SubjectTracker.DEADZONE*2*math.tan(math.radians(50))))
        undamped=min(6,.3*SubjectTracker.KP*(s['error_deg']-edge),.45*(s['error_deg']-edge))
        self.assertGreaterEqual(s['target'],6);self.assertLess(s['target']-6,undamped)

    def test_accounts_for_swivel_travel_during_inference(self):
        t=SubjectTracker();t.update(0,[face(pixel(36))],[],pivot(0))
        # Old frame saw 36° error; the swivel has already moved 15° while processing.
        s=t.update(.3,[face(pixel(36))],[],pivot(15),observed_pivot=pivot(0))
        self.assertAlmostEqual(s['error_deg'],21);self.assertEqual(s['target'],15)
        self.assertFalse(t.moving)

    def test_prefers_faces_then_confirms_hand_fallback(self):
        t=SubjectTracker()
        s=t.update(0,[face(540)],[hand(50)],pivot(0));self.assertEqual(s['subject'],'face')
        s=t.update(.4,[],[hand(50)],pivot(3));self.assertFalse(s['visible'])
        s=t.update(1.2,[],[hand(50)],pivot(3));self.assertEqual(s['subject'],'hand');self.assertEqual(s['target'],3)
        s=t.update(1.5,[],[hand(50)],pivot(3));self.assertLess(s['target'],3)

    def test_limits_lost_target_and_largest_face_selection(self):
        t=SubjectTracker()
        for i in range(40):
            s=t.update(i*.25,[face(600)],[],pivot(t.target));self.assertLessEqual(abs(s['target']),45)
        self.assertEqual(s['target'],45)
        for i in range(40,100):s=t.update(i*.25,[face(40)],[],pivot(t.target))
        self.assertEqual(s['target'],-45)
        lost=t.update(26,[],[],pivot(-30));self.assertEqual(lost['target'],-30);self.assertFalse(lost['visible'])
        t=SubjectTracker();t.update(0,[face(200)],[],pivot(0))
        t.update(.25,[face(205),face(500,200)],[],pivot(0));self.assertAlmostEqual(t.center,500/640)

    def test_larger_head_overrides_visible_previous_subject_and_resets_motion_history(self):
        t=SubjectTracker()
        t.update(0,[face(580)],[],pivot(0))
        first=t.update(.3,[face(580)],[],pivot(0))
        self.assertGreater(first['target'],0)
        # A closer/larger head enters left while the old right-hand head stays.
        switched=t.update(.6,[face(580),face(50,160)],[],pivot(first['target']))
        self.assertAlmostEqual(t.center,50/640)
        self.assertEqual(switched['target'],first['target'])
        self.assertEqual(t.error_rate,0)
        self.assertFalse(t.moving);self.assertEqual(t.pending_frames,1)
        confirmed=t.update(.9,[face(580),face(50,160)],[],pivot(first['target']))
        self.assertLess(confirmed['target'],first['target'])
        self.assertLessEqual(first['target']-confirmed['target'],6)

    def test_head_size_changes_reselect_largest_area_each_frame(self):
        t=SubjectTracker()
        t.update(0,[face(80,120),face(560,80)],[],pivot(0))
        self.assertAlmostEqual(t.center,80/640)
        # Area, rather than width alone, determines the largest face.
        t.update(.3,[face(80,140,40),face(560,90,90)],[],pivot(0))
        self.assertAlmostEqual(t.center,560/640)
        self.assertEqual(t.pending_sign,1);self.assertEqual(t.pending_frames,1)
        t.update(.6,[face(560,80),face(80,150)],[],pivot(0))
        self.assertAlmostEqual(t.center,80/640)
        self.assertEqual(t.pending_sign,-1);self.assertEqual(t.pending_frames,1)

    def test_hand_fallback_keeps_nearest_hand_when_confidences_change(self):
        t=SubjectTracker();left,right=hand(80),hand(560)
        left['score']=.95;right['score']=.8
        t.update(0,[],[left,right],pivot(0))
        self.assertEqual(t.kind,'hand');self.assertAlmostEqual(t.center,80/640)
        left['score']=.7;right['score']=.99
        t.update(.3,[],[right,left],pivot(0))
        self.assertAlmostEqual(t.center,80/640)
        self.assertEqual(t.kind,'hand')

    def test_delayed_noisy_stationary_subject_settles_without_oscillation(self):
        # Model a camera delayed two observations and a slew-limited servo. A stationary
        # subject should enter the wide hold zone without alternating command directions.
        t=SubjectTracker();q=0.;history=[0.,0.];commands=[]
        for i in range(100):
            observed=history[-2];error=42-observed+(.4 if i%2 else -.4)
            s=t.update(i*.3,[face(pixel(error))],[],pivot(q),observed_pivot=pivot(observed))
            commands.append(s['target']-q);q=s['target'];history.append(q)
        self.assertLess(abs(42-q),26)
        self.assertFalse(any(v<-.01 for v in commands))
        self.assertTrue(all(abs(v)<.01 for v in commands[-20:]))
