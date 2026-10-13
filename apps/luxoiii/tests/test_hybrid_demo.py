import importlib.util
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'tools/spatial'),str(ROOT/'arm/python'),str(ROOT/'apps/luxoiii')]
from behavior import Demo, GestureHold, palm_footprint
from hybrid import Outputs
from pointing import target_from_hand,physical_aim,simulated_aim,adjust_pose
from kinematics import beam_hit, fk, ray_plane
from luxoiii.spatial import estimate


def hand(gesture='point',x=300):
    points=[[x+(i%4)*10,200+(i//4)*10,0] for i in range(21)]
    points[5]=[x-30,230,0];points[17]=[x+30,230,0]
    return dict(points=points,gesture=gesture,score=.95)


def sized_hand(gesture='point',x=300,scale=1.,score=.95):
    h=hand(gesture,x);origin=h['points'][0]
    h['points']=[[origin[i]+scale*(p[i]-origin[i]) for i in range(3)] for p in h['points']]
    h['score']=score
    return h


class BehaviorTests(unittest.TestCase):
    def test_initial_pointer_and_command_selection_prefers_larger_palm_over_score(self):
        for gesture in ('point','open_palm','thumbs_up'):
            with self.subTest(gesture=gesture):
                hold=GestureHold()
                small=sized_hand(gesture,150,scale=.7,score=.99)
                large=sized_hand(gesture,440,scale=1.5,score=.65)
                self.assertIs(hold.select(10,[small,large]),large)
                self.assertIs(hold.select(10,[large,small]),large)

    def test_palm_priority_is_rotation_invariant_and_not_changed_by_fingertips(self):
        h=sized_hand(scale=1.5);area=palm_footprint(h)
        for index in (4,8,12,16,20):h['points'][index]=[2000,3000,0]
        self.assertEqual(palm_footprint(h),area)
        h['points']=[[-p[1]+500,p[0]-200,p[2]] for p in h['points']]
        self.assertAlmostEqual(palm_footprint(h),area)

    def test_nearly_equal_hands_stay_associated_but_clearly_larger_hand_takes_over(self):
        d=Demo(simulated_aim);old=sized_hand(x=180);near=sized_hand(x=220,scale=1.1,score=.99)
        first=dict(point=[.4,.2,0]);second=dict(point=[.4,-.2,0])
        d.update(10,[],[old],target=first);d.update(10.5,[],[old],target=first)
        for i in range(3):
            now=10.6+i*.15
            self.assertIs(d.select_pointing_hand(now,[near,old]),old)
            d.update(now,[],[near,old],target=first)
        large=sized_hand(x=220,scale=1.6,score=.6)
        saved=list(d.target)
        self.assertIs(d.select_pointing_hand(11.1,[old,large]),large)
        d.update(11.1,[],[large,old],target=second)
        self.assertEqual(d.target,saved);self.assertFalse(d.point_following)
        # Close wrists still require a new hold when a distinctly larger hand
        # takes over; the old hand's accumulated confirmation cannot transfer.
        d.update(11.6,[],[old,large],target=second)
        self.assertLess(d.target[1],saved[1]);self.assertTrue(d.point_following)

    def test_small_open_palm_still_interrupts_larger_pointing_hand(self):
        d=Demo(simulated_aim)
        pointer=sized_hand('point',180,scale=2.)
        palm=sized_hand('open_palm',450,scale=.7)
        spot=dict(point=[.4,0,0])
        for t in (10,10.5,11.4):d.update(t,[],[pointer],target=spot)
        d.update(11.6,[],[pointer,palm],target=spot)
        snapshot=d.update(12.1,[],[palm,pointer],target=spot)
        self.assertEqual(snapshot['state'],'ATTENTIVE')
        self.assertEqual(snapshot['event'],'Open palm · hello back')
        self.assertFalse(snapshot['light_requested']);self.assertIsNone(snapshot['desk_target'])

    def test_point_needs_a_desk_intersection_and_a_hold(self):
        d=Demo(simulated_aim);h=hand();target=dict(point=[.4,0,0],pose=[0,-18,65,70,0])
        d.update(10,[],[h],target=target)
        self.assertEqual(d.state,'ATTENTIVE')
        d.update(10.5,[],[h],target=target);self.assertEqual(d.state,'ACK')
        d.update(11.4,[],[],target=None);self.assertEqual(d.state,'ILLUMINATE')
        self.assertEqual(d.snapshot(12)['desk_target'],target['point'])
        d=Demo(simulated_aim);d.update(10,[],[h]);d.update(10.5,[],[h]);self.assertEqual(d.state,'ATTENTIVE')
        self.assertFalse(d.snapshot(11)['light_requested'])

    def test_missing_frames_do_not_accumulate_hold_time(self):
        d=Demo(simulated_aim);d.update(10,[],[hand()]);d.update(12,[],[hand()],target=dict(point=[.4,0,0],pose=[0,-18,65,70,0]))
        self.assertEqual(d.state,'ATTENTIVE')

    def test_point_follows_subcentimeter_moves_without_repeated_acknowledgments(self):
        d=Demo(simulated_aim);h=hand();first=dict(point=[.4,0,0],mode='virtual_pointer',source='Virtual desk pointer')
        d.update(10,[],[h],target=first);d.update(10.5,[],[h],target=first)
        serial=d.serial
        self.assertEqual(d.snapshot(10.6)['pose'],simulated_aim(first['point']))
        self.assertEqual(d.snapshot(10.6)['target_mode'],'virtual_pointer')
        for t in (11,11.5,12):d.update(t,[],[h],target=first)
        self.assertEqual(d.state,'ILLUMINATE')
        # No old 3 cm threshold or 1-second repeat timer: all of these move.
        for i in range(1,6):
            previous=list(d.target);desired=[.4+i*.005,i*.003,0]
            snapshot=d.update(12+i*.1,[],[h],target=dict(first,point=desired))
            self.assertGreater(d.target[0],previous[0]);self.assertLess(d.target[0],desired[0])
            self.assertEqual(d.serial,serial);self.assertTrue(snapshot['point_following'])
            f=fk(d.aim_pose[1:4],d.aim_pose[4],d.aim_pose[0])
            self.assertLess(math.dist(ray_plane(f.light,f.axis),d.target),1e-8)
        saved=list(d.target)
        self.assertFalse(d.snapshot(14)['point_following'])  # Expire without extrapolating.
        d.update(13,[],[])
        self.assertIsNone(d.point_preview);self.assertEqual(d.target,saved)
        self.assertFalse(d.snapshot(13)['point_following'])
        # Re-acquire after the brief hold, without restarting the greeting/nod.
        d.update(13.1,[],[h],target=first);self.assertEqual(d.target,saved)
        d.update(13.6,[],[h],target=first);self.assertLess(d.target[0],saved[0])
        self.assertEqual(d.serial,serial);self.assertEqual(d.state,'ILLUMINATE')

    def test_following_filter_is_time_based_and_has_no_overshoot(self):
        def run(step):
            d=Demo(simulated_aim);h=hand()
            d.update(10,[],[h],target=dict(point=[.4,0,0]));d.update(10.5,[],[h],target=dict(point=[.4,0,0]))
            previous=.4
            for i in range(1,round(.4/step)+1):
                d.update(10.5+i*step,[],[h],target=dict(point=[.55,.15,0]))
                self.assertGreater(d.target[0],previous);self.assertLess(d.target[0],.55)
                previous=d.target[0]
            return d.target
        for a,b in zip(run(.05),run(.1)):self.assertAlmostEqual(a,b)
        self.assertLess(math.dist(run(.1),[.55,.15,0]),.02)
        d=Demo(simulated_aim);h=hand();center=dict(point=[.4,0,0])
        d.update(20,[],[h],target=center);d.update(20.5,[],[h],target=center)
        deviations=[]
        for i in range(1,50):
            d.update(20.5+i*.1,[],[h],target=dict(point=[.4+(.008 if i%2 else -.008),0,0]))
            deviations.append(abs(d.target[0]-.4))
        self.assertLess(max(deviations),.004)  # Attenuate 8 mm fingertip jitter.
        saved=list(d.target)
        d.update(26,[],[h],target=None);self.assertEqual(d.target,saved)
        self.assertFalse(d.point_following)
        d.stop(26.1)
        for t in (26.2,26.7,27):d.update(t,[],[h],target=dict(point=[.6,.2,0]))
        self.assertEqual(d.target,saved);self.assertFalse(d.point_following)

    def test_thumb_up_is_far_down_is_close_and_distance_persists_while_pointing(self):
        for gesture,sign,label in [('thumbs_up',-1,'Far'),('thumbs_down',1,'Close')]:
            d=Demo(simulated_aim);h=hand(gesture);spot=[.4,.1,0]
            d.update(10,[],[h]);d.update(10.5,[],[h]);self.assertEqual(d.distance_offset,0)
            d.state='ILLUMINATE';d.command.gesture=None;d.target=spot;d.aim_pose=simulated_aim(spot)
            def distance():
                return math.dist(fk(d.aim_pose[1:4],d.aim_pose[4],d.aim_pose[0]).light,d.target)
            before=distance()
            d.update(20,[],[h]);d.update(20.5,[],[h])
            self.assertAlmostEqual(d.distance_offset,.03*sign)
            self.assertLess((distance()-before)*sign,0)
            self.assertTrue(d.event.startswith(label));self.assertEqual(d.level,.4)
            serial=d.serial
            d.update(20.7,[],[h]);self.assertEqual(d.serial,serial)
            d.update(21,[],[h]);d.update(21.5,[],[h]);self.assertAlmostEqual(d.distance_offset,.06*sign)
            for i in range(80):d.update(21.75+i*.25,[],[h])
            self.assertAlmostEqual(d.distance_offset,.24*sign);self.assertEqual(d.level,.4)
            offset=d.distance_offset
            d.update(42,[],[hand()],target=dict(point=[.5,-.1,0]))
            d.update(42.5,[],[hand()],target=dict(point=[.5,-.1,0]))
            self.assertEqual(d.distance_offset,offset)
            self.assertEqual(d.aim_pose,simulated_aim(d.target,offset))
            f=fk(d.aim_pose[1:4],d.aim_pose[4],d.aim_pose[0])
            self.assertLess(math.dist(ray_plane(f.light,f.axis),d.target),1e-8)

    def test_stationary_open_palm_greets_once_and_never_stops(self):
        for state in ('IDLE','ATTENTIVE','ACK','ILLUMINATE'):
            d=Demo(simulated_aim);d.state=state;d.entered=10;h=hand('open_palm')
            d.update(10,[],[h]);self.assertFalse(d.event.startswith('Open palm'))
            d.update(10.5,[],[h]);self.assertEqual(d.event,'Open palm · hello back')
            serial=d.serial
            for i in range(1,40):d.update(10.5+i*.25,[],[h])
            self.assertNotEqual(d.state,'STOPPED');self.assertEqual(d.serial,serial)

    def test_open_palm_releases_lit_pointing_focus_and_welcomes_before_reacquiring(self):
        d=Demo(simulated_aim);pointer=hand(x=180);palm=hand('open_palm',x=450)
        spot=dict(point=[.4,.1,0],mode='virtual_pointer',source='Virtual desk pointer')
        for t in (10,10.5,11,11.5):d.update(t,[],[pointer],target=spot)
        self.assertTrue(d.snapshot(11.5)['light_requested']);self.assertTrue(d.point_following)
        d.update(11.7,[],[pointer,palm],target=spot)
        welcome=d.update(12.2,[],[palm,pointer],target=spot)
        self.assertEqual(welcome['state'],'ATTENTIVE')
        self.assertEqual(welcome['event'],'Open palm · hello back')
        self.assertFalse(welcome['light_requested']);self.assertFalse(welcome['point_following'])
        self.assertIsNone(welcome['desk_target']);self.assertIsNone(welcome['point_preview'])
        self.assertIsNone(d.aim_pose);self.assertIsNone(d.target_mode)
        serial=d.serial
        # A continuously held second-hand point cannot cancel the greeting, even
        # if the palm leaves the frame. No new point acknowledgments are emitted.
        for t in (12.4,12.8,13.2,13.6,14,14.4,14.8,15.2,15.6,16,16.4,16.8):
            snapshot=d.update(t,[],[pointer],target=spot)
            self.assertEqual(snapshot['state'],'ATTENTIVE')
            self.assertFalse(snapshot['light_requested']);self.assertIsNone(snapshot['desk_target'])
            self.assertEqual(d.serial,serial)
        d.update(17.3,[],[pointer],target=spot);self.assertEqual(d.state,'ATTENTIVE')
        d.update(17.8,[],[pointer],target=spot);self.assertEqual(d.state,'ACK')
        self.assertEqual(d.target,spot['point']);self.assertTrue(d.point_following)

    def test_held_open_palm_keeps_priority_over_second_hand_point_after_welcome(self):
        d=Demo(simulated_aim);hands=[hand('open_palm',450),hand('point',180)]
        spot=dict(point=[.4,0,0])
        d.update(10,[],hands,target=spot);d.update(10.5,[],hands,target=spot)
        serial=d.serial
        for i in range(1,30):
            snapshot=d.update(10.5+i*.25,[],hands,target=spot)
            self.assertEqual(snapshot['state'],'ATTENTIVE')
            self.assertFalse(snapshot['light_requested']);self.assertIsNone(snapshot['desk_target'])
            self.assertEqual(d.serial,serial)

    def test_arrivals_during_palm_welcome_update_count_without_interrupting(self):
        d=Demo(simulated_aim);palm=hand('open_palm')
        face=SimpleNamespace(x=200,w=80,h=80)
        d.update(10,[],[palm]);d.update(10.5,[],[palm]);serial=d.serial
        d.update(11,[face,face],[])
        d.update(12,[face,face],[])
        self.assertEqual(d.group_count,2)
        self.assertEqual(d.serial,serial);self.assertEqual(d.event,'Open palm · hello back')
        # Those arrivals are not replayed after the welcome ends.
        d.update(15.8,[face,face],[])
        self.assertEqual(d.serial,serial)
        # A subsequent, genuinely new arrival still receives the group greeting.
        d.update(16,[face,face,face],[]);d.update(16.9,[face,face,face],[])
        self.assertEqual(d.group_count,3);self.assertEqual(d.serial,serial+1)
        self.assertTrue(d.event.startswith('More company'))

    def test_two_hands_point_and_adjust_on_independent_hold_timers(self):
        d=Demo(simulated_aim);pointer=hand(x=180);thumb=hand('thumbs_up',x=450)
        spot=dict(point=[.4,0,0])
        for t in (10,10.5,11,11.5):d.update(t,[],[pointer],target=spot)
        serial=d.serial
        d.update(11.7,[],[thumb,pointer],target=dict(point=[.42,.01,0]))
        self.assertEqual(d.distance_offset,0);self.assertTrue(d.point_following)
        d.update(11.9,[],[pointer,thumb],target=dict(point=[.44,.02,0]))
        d.update(12.2,[],[thumb,pointer],target=dict(point=[.46,.03,0]))
        self.assertAlmostEqual(d.distance_offset,-.03);self.assertGreater(d.target[0],.44)
        self.assertEqual(d.serial,serial+1);self.assertTrue(d.point_following)
        self.assertEqual(d.snapshot(12.2)['gestures'],['point','thumbs_up'])
        # Changing only the other hand's gesture must not reacquire the pointer.
        thumb['gesture']='thumbs_down'
        d.update(12.4,[],[thumb,pointer],target=dict(point=[.48,.04,0]))
        self.assertTrue(d.point_following);self.assertGreater(d.target[0],.46)
        d.update(12.9,[],[pointer,thumb],target=dict(point=[.5,.05,0]))
        self.assertAlmostEqual(d.distance_offset,0)
        self.assertTrue(d.event.startswith('Close'));self.assertTrue(d.point_following)
        d.stop(13)
        for t in (13.1,13.6,14):d.update(t,[],[thumb,pointer],target=spot)
        self.assertEqual(d.state,'STOPPED');self.assertEqual(d.distance_offset,0)

    def test_two_pointers_keep_the_selected_hand_across_detection_reordering(self):
        d=Demo(simulated_aim);left=hand(x=150);right=hand(x=440)
        d.update(10,[],[left],target=dict(point=[.4,.2,0]))
        d.update(10.5,[],[left],target=dict(point=[.4,.2,0]))
        for i in range(6):
            now=10.7+i*.2;hands=[left,right] if i%2 else [right,left]
            selected=d.select_pointing_hand(now,hands)
            self.assertIs(selected,left)
            d.update(now,[],hands,target=dict(point=[.4,.2,0]))
            self.assertTrue(d.point_following)
        # If that hand disappears, a different distant hand needs its own hold.
        saved=list(d.target)
        self.assertIs(d.select_pointing_hand(12.1,[right]),right)
        d.update(12.1,[],[right],target=dict(point=[.4,-.2,0]))
        self.assertEqual(d.target,saved);self.assertFalse(d.point_following)
        d.update(12.6,[],[right],target=dict(point=[.4,-.2,0]))
        self.assertLess(d.target[1],saved[1]);self.assertTrue(d.point_following)

    def test_opposite_thumbs_cancel_instead_of_oscillating(self):
        d=Demo(simulated_aim);d.state='ILLUMINATE';d.target=[.4,0,0];d.aim_pose=simulated_aim(d.target)
        hands=[hand('thumbs_up',150),hand('thumbs_down',440)]
        for i in range(20):d.update(10+i*.25,[],hands if i%2 else hands[::-1])
        self.assertEqual(d.distance_offset,0);self.assertEqual(d.serial,0)

    def test_open_palm_rearms_after_release_with_cooldown(self):
        d=Demo(simulated_aim);h=hand('open_palm')
        d.update(10,[],[h]);d.update(10.5,[],[h]);serial=d.serial
        d.update(11,[],[]);d.update(11.3,[],[h]);d.update(11.8,[],[h])
        self.assertEqual(d.serial,serial)
        for t in (12.2,12.6,13,13.5):d.update(t,[],[h])
        self.assertEqual(d.serial,serial+1)

    def test_only_explicit_stop_latches_until_resume(self):
        d=Demo(simulated_aim);d.stop(10)
        for i in range(20):d.update(10.2+i*.3,[],[hand('open_palm')])
        self.assertEqual(d.state,'STOPPED');self.assertEqual(d.event,'Stopped · lamp off')
        d.resume(20);d.update(20.2,[],[hand('open_palm')]);d.update(20.7,[],[hand('open_palm')])
        self.assertEqual(d.event,'Open palm · hello back');self.assertEqual(d.state,'ATTENTIVE')

    def test_arrivals_group_greeting_and_quiet_return(self):
        d=Demo(simulated_aim);face=SimpleNamespace(x=200,w=80,h=80)
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
        if method=='arm_roll_status':return [3,1,0,0,1500,0]
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
            pivot=dict(enabled=True,current=0,target=0,fault=0),tracking=dict(target=4,status='Tracking face'),demo=dict(state='ILLUMINATE',light_requested=True,brightness=.4,pivot_target=0,desk_target=beam_hit(q)))

    def downward(self):
        for j,v in zip(self.data['joints'],(0,90,90)):j['angle']=v
        self.data['tracking']['target']=0
        self.data['face_guard']=dict(checked=True,frame_at=100)

    def frame(self,now):
        self.data['face_guard']['frame_at']=now
        self.o.heartbeat=now;self.o.step(self.data,now,now)

    def illuminate(self):
        self.downward()
        for t in (100,100.4,100.8,101.2):self.frame(t)
        self.assertEqual(self.o.level,1.)

    def test_physical_light_needs_clear_frames_and_whole_downward_cone(self):
        self.downward();self.data['demo'].update(state='IDLE',light_requested=False,brightness=.1)
        for t in (100,100.4,100.8):
            self.frame(t);self.assertEqual(self.o.level,0)
        self.frame(101.2);self.assertEqual(self.o.level,1.)
        on=[args for method,args in self.b.calls if method=='arm_light' and args[0]>0]
        self.assertEqual(on,[[1000,0,800]])
        self.assertLess(self.o.snapshot()['lamp_gate']['beam_top_deg'],-5)
        self.frame(101.4)
        self.assertEqual([args for method,args in self.b.calls if method=='arm_light'][-1],[1000,0,800])
        # A face cuts the light before any tracking/motion RPC on that cycle.
        self.b.calls.clear();self.data['detections']=[dict(score=.51)]
        self.frame(101.5);self.assertEqual(self.o.level,0)
        self.assertEqual(self.b.calls[0],('arm_light',[0,0]))
        self.data['detections']=[]
        for t in (101.6,102,102.4):self.frame(t);self.assertEqual(self.o.level,0)
        self.frame(102.8);self.assertEqual(self.o.level,1.)

    def test_automatic_light_turns_off_on_missing_inputs_or_horizon_crossing(self):
        for failure in ('face','horizon','bad_encoder','swivel_fault','swivel_released','future_swivel','in_flight_swivel','skipped_face','stale_face','missing_detections','camera_error','stop'):
            with self.subTest(failure=failure):
                self.setUp();self.illuminate()
                if failure=='face':self.data['detections']=[dict(score=.7)]
                elif failure=='horizon':self.data['joints'][2]['angle']=5
                elif failure=='bad_encoder':self.data['joints'][0]['healthy']=False
                elif failure=='swivel_fault':self.data['pivot']['fault']=-1
                elif failure=='swivel_released':self.data['pivot']['enabled']=False
                elif failure=='future_swivel':self.data['tracking']['target']=45
                elif failure=='in_flight_swivel':self.data['pivot']['target']=45
                elif failure=='skipped_face':self.data['face_guard']['checked']=False
                elif failure=='stale_face':self.data['face_guard']['frame_at']=99
                elif failure=='missing_detections':self.data.pop('detections')
                elif failure=='camera_error':self.data['error']='camera disconnected'
                elif failure=='stop':self.data['demo']['state']='STOPPED'
                self.o.step(self.data,101.3,101.3)
                self.assertEqual(self.o.level,0)
                self.assertEqual([a for m,a in self.b.calls if m=='arm_light'][-1],[0,0])

    def test_replayed_or_skipped_face_check_cannot_accumulate_clear_frames(self):
        self.downward()
        for i in range(30):
            now=100+i*.05;self.o.heartbeat=now;self.o.step(self.data,now,now)
            self.assertEqual(self.o.level,0)
        self.assertEqual(self.o.clear_frames,0)
        self.data['face_guard']['checked']=False
        for t in (102,102.5,103,103.5):self.frame(t);self.assertEqual(self.o.level,0)

    def test_horizon_margin_hysteresis_and_current_pose_override_camera_pose(self):
        self.downward()
        def margin(degrees):self.data['joints'][2]['angle']=self.o.BEAM_HALF_DEG+degrees
        margin(4)
        for t in (100,100.4,100.8,101.2):self.frame(t)
        self.assertEqual(self.o.level,0)
        margin(6);self.frame(101.4);self.assertEqual(self.o.level,1.)
        margin(3);self.frame(101.6);self.assertEqual(self.o.level,1.)
        margin(1);self.frame(101.8);self.assertEqual(self.o.level,0)
        margin(6);self.frame(102);self.assertEqual(self.o.level,1.)
        # New physical readings always beat the earlier camera-time pose.
        self.o.read_pose=lambda bridge:dict(self.data,pivot=dict(enabled=True,target=45,current=45,fault=0))
        self.frame(102.2);self.assertEqual(self.o.level,0)

    def test_automatic_light_refuses_firmware_without_expiry_support(self):
        call=self.b.call
        def old_firmware(method,args=None,**kw):
            if method=='arm_light' and len(args)==3:
                self.b.calls.append((method,args));return [4,1,0,0,0,0,-22]
            return call(method,args,**kw)
        self.b.call=old_firmware;self.downward();self.frame(100)
        self.assertFalse(self.o.requested);self.assertFalse(self.o.claimed)
        self.assertFalse(any(a[0]>0 for m,a in self.b.calls if m=='arm_light'))

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

    def test_heartbeat_disables_outputs_but_pitch_loss_only_blocks_light(self):
        self.o.step(self.data,100,100);self.o.step(self.data,104,104)
        self.assertFalse(self.o.requested);self.assertFalse(self.o.claimed)
        self.o.requested=True;self.o.heartbeat=105;self.data['joints'][0]['healthy']=False
        self.o.step(self.data,105,105)
        self.assertTrue(self.o.requested);self.assertTrue(self.o.claimed)
        self.assertEqual(self.o.level,0)
        self.assertIn('Shoulder angle unavailable',self.o.lamp_reason)

    def test_missing_elbow_keeps_tracking_with_light_off_before_swivel(self):
        self.data['joints'][1].update(healthy=False,angle=None,reason='Elbow magnet not detected')
        self.o.step(self.data,100,100)
        self.assertTrue(self.o.requested);self.assertTrue(self.o.claimed)
        self.assertEqual(self.o.last_target,4);self.assertEqual(self.o.level,0)
        self.assertIn('Elbow magnet not detected',self.o.lamp_reason)
        moves=[i for i,(m,a) in enumerate(self.b.calls) if m=='arm_command' and a[0]==16]
        off=[i for i,(m,a) in enumerate(self.b.calls) if m=='arm_light' and a[0]==0]
        self.assertTrue(moves and off);self.assertLess(off[-1],moves[0])
        self.assertFalse(any(a[0]>0 for m,a in self.b.calls if m=='arm_light'))

    def test_released_servo_at_limit_can_wake_with_bad_elbow_and_lamp_off(self):
        self.data['joints'][1].update(healthy=False,angle=None,reason='Elbow magnet not detected')
        self.data['pivot'].update(enabled=False,current=45,target=45)
        self.data['tracking']['target']=45
        self.o.step(self.data,100,100)
        self.assertTrue(self.o.requested);self.assertTrue(self.o.claimed)
        self.assertEqual(self.o.last_target,45);self.assertEqual(self.o.level,0)
        moves=[a for m,a in self.b.calls if m=='arm_command' and a[0]==16]
        self.assertEqual(moves[0][2],45000)
        self.assertFalse(any(a[0]>0 for m,a in self.b.calls if m=='arm_light'))

    def test_pitch_loss_cuts_lit_lamp_and_recovery_still_requires_face_and_beam_gates(self):
        self.illuminate();self.b.calls.clear()
        self.data['joints'][1].update(healthy=False,angle=None)
        self.data['tracking']['target']=4
        self.frame(101.4)
        self.assertEqual(self.b.calls[0],('arm_light',[0,0]))
        self.assertTrue(self.o.requested);self.assertEqual(self.o.last_target,4)
        self.assertEqual(self.o.level,0);self.assertIsNone(self.o.beam_top_deg)
        self.data['joints'][1].update(healthy=True,angle=90)
        self.data['detections']=[dict(score=.9)]
        self.frame(101.6);self.assertEqual(self.o.level,0)
        self.data['detections']=[]
        self.data['joints'][2]['angle']=5
        for t in (101.8,102.2,102.6,103):self.frame(t)
        self.assertEqual(self.o.level,0)
        self.data['joints'][2]['angle']=90
        self.frame(103.2)
        self.assertEqual(self.o.level,1.)
        self.assertEqual([a for m,a in self.b.calls if m=='arm_light'][-1],[1000,0,800])

    def test_pitch_read_failure_fetches_fresh_swivel_after_light_off(self):
        self.illuminate();self.b.calls.clear()
        def unavailable(_):raise TimeoutError('Pitch RPC failed')
        self.o.read_pose=unavailable;self.data['tracking']['target']=4
        self.frame(101.4)
        self.assertTrue(self.o.requested);self.assertTrue(self.o.claimed)
        self.assertEqual(self.o.level,0);self.assertEqual(self.o.last_target,4)
        off=next(i for i,(m,a) in enumerate(self.b.calls) if m=='arm_light' and a[0]==0)
        read=next(i for i,(m,a) in enumerate(self.b.calls) if m=='arm_roll_status')
        move=next(i for i,(m,a) in enumerate(self.b.calls) if m=='arm_command' and a[0]==16)
        self.assertLess(off,read);self.assertLess(read,move)

    def test_pitch_rpc_failure_cannot_use_stale_camera_swivel_if_fresh_swivel_fails(self):
        self.illuminate();self.b.calls.clear()
        def unavailable(_):raise TimeoutError('Pitch RPC failed')
        self.o.read_pose=unavailable
        call=self.b.call
        def no_servo(method,args=None,**kw):
            if method=='arm_roll_status':raise TimeoutError('Servo RPC failed')
            return call(method,args,**kw)
        self.b.call=no_servo;self.data['tracking']['target']=4
        self.frame(101.4)
        self.assertFalse(self.o.requested);self.assertFalse(self.o.claimed)
        self.assertEqual(self.o.level,0)
        self.assertFalse(any(m=='arm_command' and a[0]==16 for m,a in self.b.calls))

    def test_invalid_swivel_or_firmware_rejection_still_disables_everything(self):
        for failure in ('fault','missing','nonfinite','out_of_range','firmware_power_lock'):
            with self.subTest(failure=failure):
                self.setUp();self.data['joints'][1].update(healthy=False,angle=None)
                if failure=='fault':self.data['pivot']['fault']=-1
                elif failure=='missing':self.data.pop('pivot')
                elif failure=='nonfinite':self.data['pivot']['current']=math.nan
                elif failure=='out_of_range':self.data['pivot']['target']=60
                else:
                    call=self.b.call
                    def locked(method,args=None,**kw):
                        result=call(method,args,**kw)
                        if method=='arm_command' and args[0]==16:result[1]=-1
                        return result
                    self.b.call=locked
                self.o.step(self.data,100,100)
                self.assertFalse(self.o.requested);self.assertFalse(self.o.claimed)
                self.assertEqual(self.o.level,0)

    def test_camera_and_stop_guards_still_disable_with_missing_pitch(self):
        for failure in ('stale','camera_error','stopped'):
            with self.subTest(failure=failure):
                self.setUp();self.data['joints'][1].update(healthy=False,angle=None)
                self.o.step(self.data,100,100)
                self.o.heartbeat=101
                if failure=='camera_error':self.data['error']='Camera unavailable'
                elif failure=='stopped':self.data['demo']['state']='STOPPED'
                self.o.step(self.data,98 if failure=='stale' else 101,101)
                self.assertFalse(self.o.requested);self.assertFalse(self.o.claimed)
                self.assertEqual(self.o.level,0)

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
        self.assertEqual(result['mode'],'finger_ray')
        # A virtual drag must not jump back to a ray hit when the pose fluctuates.
        locked=target_from_hand(hand_data,joints,dict(enabled=True,current=0),prefer_virtual=True)
        self.assertEqual(locked['mode'],'virtual_pointer')
        ray_only=target_from_hand(hand_data,joints,dict(enabled=True,current=0),allow_virtual=False,prefer_virtual=True)
        self.assertEqual(ray_only['mode'],'finger_ray')
        expected=[center[i]-center[2]/h.up[2]*h.up[i] for i in range(3)]
        for a,b in zip(expected,result['point']):self.assertAlmostEqual(a,b)

    def test_pointing_requires_downward_reachable_ray(self):
        joints=[dict(angle=v,healthy=True,age_ms=1) for v in (-18,65,70)]
        self.assertIsNone(target_from_hand(hand(),[],{},640,480,allow_virtual=False))
        self.assertIsNone(physical_aim(joints,[2,0,0]))
        self.assertAlmostEqual(physical_aim(joints,beam_hit([-18,65,70])),0)

    def test_virtual_cursor_recovers_a_ray_miss_at_current_real_pose(self):
        joints=[dict(angle=v,healthy=True,age_ms=1) for v in (-8.9,59.9,43.2)]
        h=hand();h['points'][6]=[310,220,0];h['points'][8]=[310,160,0]
        pivot=dict(enabled=True,current=45)
        self.assertIsNone(target_from_hand(h,joints,pivot,allow_virtual=False))
        selected=target_from_hand(h,joints,pivot)
        self.assertEqual(selected['mode'],'virtual_pointer')
        self.assertIn('Virtual',selected['source']);self.assertIsNotNone(selected['ray_note'])
        self.assertEqual(selected['image_tip'],[310/640,160/480])
        h['points'][8]=[440,350,0]
        moved=target_from_hand(h,joints,pivot)
        self.assertLess(moved['point'][0],selected['point'][0])
        self.assertLess(moved['point'][1],selected['point'][1])

    def test_virtual_cursor_rejects_invalid_or_cropped_index(self):
        for value in (-1,641,float('nan')):
            h=hand();h['points'][8][0]=value
            self.assertIsNone(target_from_hand(h,[],{}))

    def test_digital_yaw_aim_covers_entire_desk_and_preserves_beam_on_adjust(self):
        for x in (.2,.25,.3,.4,.5,.6,.65):
            for y in (-.3,-.15,0,.15,.3):
                target=[x,y,0];pose=simulated_aim(target)
                self.assertIsNotNone(pose)
                for delta in (0,.03,-.03,.24,-.24):
                    p=adjust_pose(pose,target,delta)
                    h=fk(p[1:4],p[4],p[0]);hit=ray_plane(h.light,h.axis)
                    self.assertLess(math.dist(hit,target),1e-8)
                    if delta>0:self.assertLess(math.dist(h.light,target),math.dist(fk(pose[1:4],pose[4],pose[0]).light,target))

    def test_close_far_saturates_without_reversing_at_workspace_edges(self):
        for x in (.2,.25,.4,.55,.65):
            for y in (-.3,0,.3):
                target=[x,y,0]
                for sign in (-1,1):
                    previous=None
                    for i in range(9):
                        p=simulated_aim(target,i*.03*sign);h=fk(p[1:4],p[4],p[0])
                        distance=math.dist(h.light,target)
                        if previous is not None:self.assertLessEqual((distance-previous)*sign,1e-7)
                        self.assertLess(math.dist(ray_plane(h.light,h.axis),target),1e-8)
                        previous=distance

if __name__=='__main__':unittest.main()
