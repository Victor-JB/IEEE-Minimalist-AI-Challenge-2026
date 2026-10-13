import importlib.util
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'tools/spatial'),str(ROOT/'arm/python'),str(ROOT/'apps/luxoiii')]
from behavior import Demo
from hybrid import Outputs
from pointing import target_from_hand,physical_aim
from kinematics import beam_hit, fk
from luxoiii.spatial import estimate


def hand(gesture='point',x=300):
    points=[[x+(i%4)*10,200+(i//4)*10,0] for i in range(21)]
    points[5]=[x-30,230,0];points[17]=[x+30,230,0]
    return dict(points=points,gesture=gesture,score=.95)


class BehaviorTests(unittest.TestCase):
    def test_point_needs_a_desk_intersection_and_a_hold(self):
        d=Demo();h=hand();target=dict(point=[.4,0,0],pose=[0,-18,65,70,0])
        d.update(10,[],[h],target=target)
        self.assertEqual(d.state,'ATTENTIVE')
        d.update(10.5,[],[h],target=target);self.assertEqual(d.state,'ACK')
        d.update(11.4,[],[],target=None);self.assertEqual(d.state,'ILLUMINATE')
        self.assertEqual(d.snapshot(12)['desk_target'],target['point'])
        d=Demo();d.update(10,[],[h]);d.update(10.5,[],[h]);self.assertEqual(d.state,'ATTENTIVE')
        self.assertFalse(d.snapshot(11)['light_requested'])

    def test_missing_frames_do_not_accumulate_hold_time(self):
        d=Demo();d.update(10,[],[hand()]);d.update(12,[],[hand()],target=dict(point=[.4,0,0],pose=[0,-18,65,70,0]))
        self.assertEqual(d.state,'ATTENTIVE')

    def test_brightness_repeats_only_when_illuminating(self):
        d=Demo();h=hand('thumbs_up');d.update(10,[],[h]);d.update(10.5,[],[h]);self.assertEqual(d.level,.4)
        d.state='ILLUMINATE';d.gesture=None
        for i in range(100):d.update(20+i*.25,[],[h])
        self.assertEqual(d.level,.8)

    def test_stationary_open_palm_greets_once_and_never_stops(self):
        for state in ('IDLE','ATTENTIVE','ACK','ILLUMINATE'):
            d=Demo();d.state=state;d.entered=10;h=hand('open_palm')
            d.update(10,[],[h]);self.assertFalse(d.event.startswith('Open palm'))
            d.update(10.5,[],[h]);self.assertEqual(d.event,'Open palm · hello back')
            serial=d.serial
            for i in range(1,40):d.update(10.5+i*.25,[],[h])
            self.assertNotEqual(d.state,'STOPPED');self.assertEqual(d.serial,serial)

    def test_open_palm_rearms_after_release_with_cooldown(self):
        d=Demo();h=hand('open_palm')
        d.update(10,[],[h]);d.update(10.5,[],[h]);serial=d.serial
        d.update(11,[],[]);d.update(11.3,[],[h]);d.update(11.8,[],[h])
        self.assertEqual(d.serial,serial)
        for t in (12.2,12.6,13,13.5):d.update(t,[],[h])
        self.assertEqual(d.serial,serial+1)

    def test_only_explicit_stop_latches_until_resume(self):
        d=Demo();d.stop(10)
        for i in range(20):d.update(10.2+i*.3,[],[hand('open_palm')])
        self.assertEqual(d.state,'STOPPED');self.assertEqual(d.event,'Stopped · lamp off')
        d.resume(20);d.update(20.2,[],[hand('open_palm')]);d.update(20.7,[],[hand('open_palm')])
        self.assertEqual(d.event,'Open palm · hello back');self.assertEqual(d.state,'ATTENTIVE')

    def test_arrivals_group_greeting_and_quiet_return(self):
        d=Demo();face=SimpleNamespace(x=200,w=80,h=80)
        self.assertEqual(d.update(10,[face],[])['state'],'ATTENTIVE')
        self.assertTrue(d.event.startswith('Hello'))
        d.update(11,[face,face],[]);d.update(11.9,[face,face],[])
        self.assertTrue(d.event.startswith('More company'))
        self.assertEqual(d.snapshot(20)['people_count'],2)
        self.assertGreater(d.snapshot(21)['interaction_idle_s'],8)
        self.assertEqual(d.update(23,[],[])['state'],'IDLE')
        d.state='ILLUMINATE';d.last_seen=23
        self.assertEqual(d.update(34,[],[])['state'],'IDLE')



class Bridge:
    def __init__(self):self.calls=[];self.owner=0
    def connect(self,**kw):return True
    def disconnect(self):pass
    def call(self,method,args=None,**kw):
        self.calls.append((method,args))
        if method=='arm_light':return [4,1,args[0],args[0],0,0,0]
        r=[0]*38;r[0]=1;r[2]=self.owner;r[8]=1
        if method=='arm_command':
            if args[0]==0:self.owner=args[1]
            if args[0]==7:self.owner=0
        return r


class OutputTests(unittest.TestCase):
    def setUp(self):
        self.b=Bridge();self.o=Outputs(self.b);self.o.requested=True;self.o.heartbeat=100
        q=(-18,65,70)
        self.data=dict(joints=[dict(angle=v,healthy=True,age_ms=5) for v in q],detections=[],
            pivot=dict(enabled=True,current=0,fault=0),tracking=dict(target=4,status='Tracking face'),demo=dict(state='ILLUMINATE',light_requested=True,brightness=.4,pivot_target=0,desk_target=beam_hit(q)))

    def test_only_tracking_pivot_commands_and_lamp_off_are_sent(self):
        self.o.step(self.data,100,100)
        self.assertTrue(self.o.claimed)
        ops=[a[0] for m,a in self.b.calls if m=='arm_command']
        self.assertEqual(ops,[0,17,4,16]);self.assertTrue(set(ops)<=Outputs.ALLOWED)
        config=next(a for m,a in self.b.calls if m=='arm_command' and a[0]==17)
        self.assertEqual(config[4:6],[-45000,45000]);self.assertEqual(config[-1],60000)
        move=next(a for m,a in self.b.calls if m=='arm_command' and a[0]==16)
        self.assertEqual(move[-1],60000)
        light=[a for m,a in self.b.calls if m=='arm_light']
        self.assertTrue(all(a[0]==0 for a in light));self.assertEqual(self.o.last_target,4)
        with self.assertRaises(ValueError):self.o.command(11,0)

    def test_heartbeat_and_bad_encoders_disable_outputs(self):
        self.o.step(self.data,100,100);self.o.step(self.data,104,104)
        self.assertFalse(self.o.requested);self.assertFalse(self.o.claimed)
        self.o.requested=True;self.o.heartbeat=105;self.data['joints'][0]['healthy']=False
        self.o.step(self.data,105,105);self.assertFalse(self.o.requested)

    def test_face_or_upward_pose_never_lights(self):
        for case in ('face','up'):
            self.setUp()
            if case=='face':self.data['detections']=[dict(x=0)]
            else:self.data['joints'][2]['angle']=5
            self.o.step(self.data,100,100)
            self.assertFalse(any(a[0]>0 for m,a in self.b.calls if m=='arm_light'))

    def test_gestures_never_override_subject_tracking(self):
        for state in ('IDLE','ATTENTIVE','ACK','ILLUMINATE'):
            self.data['demo'].update(state=state,pivot_target=-8,brightness=.8)
            self.o.step(self.data,100,100)
            self.assertEqual(self.o.last_target,4)
            self.assertEqual(self.o.level,0)
        self.data['tracking']['target']=50
        self.o.step(self.data,100,100);self.assertEqual(self.o.last_target,45)
        self.data['demo']['state']='STOPPED'
        self.o.step(self.data,100,100);self.assertFalse(self.o.requested)

    def test_owned_arm_is_not_claimed(self):
        self.b.owner=99;self.o.step(self.data,100,100)
        self.assertFalse(any(m=='arm_command' and a[0]==0 for m,a in self.b.calls))
        self.assertFalse(self.o.requested)


class HandProjection(unittest.TestCase):
    def test_skeleton_has_camera_and_world_points_and_expires(self):
        s=dict(size=[640,480],age_s=.1,joints=[dict(angle=v,healthy=True,age_ms=1) for v in (0,60,30)],pivot=dict(enabled=True,current=0),hands=[hand()])
        d=estimate(s);self.assertEqual(len(d['hands_3d']),1)
        h=d['hands_3d'][0];self.assertEqual(len(h['world_points']),21)
        self.assertTrue(all(math.isfinite(v) for p in h['world_points'] for v in p))
        s['age_s']=3;self.assertEqual(estimate(s)['hands_3d'],[])

    def test_cropped_hand_is_not_given_a_false_3d_position(self):
        h=hand();h['points'][8][0]=-1
        self.assertEqual(estimate(dict(size=[640,480],age_s=.1,hands=[h]))['hands_3d'],[])

    def test_finger_ray_projects_to_the_expected_desk_spot(self):
        q=(-18,65,70);h=fk(q);f=640/(2*math.tan(math.radians(50)));depth=.3
        center=[h.camera[i]+depth*h.axis[i] for i in range(3)]
        def pixel(right,up):return [320+right*f/depth,240-up*f/depth,0]
        pts=[pixel(0,0) for _ in range(21)]
        pts[5]=pixel(-.04,0);pts[17]=pixel(.04,0)
        pts[6]=pixel(0,.03);pts[8]=pixel(0,-.04)
        hand_data=dict(points=pts,score=.95)
        joints=[dict(angle=v,healthy=True,age_ms=1) for v in q]
        result=target_from_hand(hand_data,joints,dict(enabled=True,current=0))
        self.assertIsNotNone(result)
        expected=[center[i]-center[2]/h.up[2]*h.up[i] for i in range(3)]
        for a,b in zip(expected,result['point']):self.assertAlmostEqual(a,b)

    def test_pointing_requires_downward_reachable_ray(self):
        joints=[dict(angle=v,healthy=True,age_ms=1) for v in (-18,65,70)]
        self.assertIsNone(target_from_hand(hand(),[],{},640,480))
        self.assertIsNone(physical_aim(joints,[2,0,0]))
        self.assertAlmostEqual(physical_aim(joints,beam_hit([-18,65,70])),0)

if __name__=='__main__':unittest.main()
