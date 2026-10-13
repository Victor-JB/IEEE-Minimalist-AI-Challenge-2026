"""Projection and freshness checks for the read-only spatial preview."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from luxoiii.spatial import estimate

OBS=Path(__file__).resolve().parents[3]/'tools/spatial/observe.py'
spec=importlib.util.spec_from_file_location('spatial_observer',OBS)
observer=importlib.util.module_from_spec(spec)
spec.loader.exec_module(observer)


class Spatial(unittest.TestCase):
    def setUp(self):
        self.s=dict(size=[640,480],age_s=.2,pivot=dict(enabled=True,current=0,fault=0),
          joints=[dict(angle=v,age_ms=10,healthy=True) for v in (0,60,30)],
          detections=[dict(x=276,y=196,w=88,h=88,score=.95)])

    def test_center_ray_follows_head_and_distance_scales_with_width(self):
        d=estimate(self.s)
        p=d['people'][0];c=d['camera']
        self.assertAlmostEqual(p['camera'][0],0)
        self.assertAlmostEqual(p['camera'][1],0)
        self.assertAlmostEqual(p['depth_m'],640/(2*math.tan(math.radians(50)))*.16/88)
        for i in range(3):
            self.assertAlmostEqual(p['world'][i],c['position'][i]+p['depth_m']*c['forward'][i])
        self.assertAlmostEqual(estimate(self.s,face_width=.24)['people'][0]['depth_m'],p['depth_m']*1.5)

    def test_image_right_and_up_follow_camera_basis(self):
        self.s['detections'][0].update(x=364,y=108)
        d=estimate(self.s);p=d['people'][0];c=d['camera']
        self.assertGreater(p['camera'][0],0)
        self.assertGreater(p['camera'][1],0)
        for key,index in [('right',0),('up',1),('forward',2)]:
            delta=[p['world'][i]-c['position'][i] for i in range(3)]
            self.assertAlmostEqual(sum(a*b for a,b in zip(delta,c[key])),p['camera'][index])

    def test_pivot_and_mount_rotate_the_physical_camera(self):
        a=estimate(self.s)['camera']
        self.s['pivot']['current']=30
        b=estimate(self.s)['camera']
        self.assertLess(b['forward'][1],a['forward'][1])
        c=estimate(self.s,camera_pitch=15,camera_yaw=20)['camera']
        for axis in ['right','up','forward']:
            self.assertAlmostEqual(math.hypot(*c[axis]),1)
        self.assertAlmostEqual(sum(x*y for x,y in zip(c['up'],c['forward'])),0)

    def test_released_servo_uses_explicit_fallback_not_reported_angle(self):
        self.s['pivot'].update(enabled=False,current=42)
        d=estimate(self.s,pivot_fallback=-5)
        self.assertEqual(d['camera']['roll'],-5)
        self.assertIn('manual estimate',d['pivot_source'])

    def test_stale_frames_remove_camera_and_people(self):
        self.s['age_s']=2.1
        d=estimate(self.s)
        self.assertFalse(d['fresh']);self.assertEqual(d['people'],[]);self.assertIsNone(d['camera'])

    def test_bad_joints_never_create_world_positions(self):
        for update in [dict(age_ms=500),dict(healthy=False),dict(angle=None),dict(angle=math.nan)]:
            s=copy.deepcopy(self.s);s['joints'][0].update(update)
            d=estimate(s)
            self.assertIsNone(d['camera']);self.assertNotIn('world',d['people'][0])

    def test_cropped_and_tiny_faces_excluded(self):
        for box in [dict(x=-3),dict(w=2),dict(x=600),dict(y=-1)]:
            s=copy.deepcopy(self.s);s['detections'][0].update(box)
            self.assertEqual(estimate(s)['people'],[])

    def test_invalid_calibration_rejected(self):
        for settings in [dict(hfov=0),dict(face_width=0),dict(camera_pitch=math.nan)]:
            with self.assertRaises(ValueError):estimate(self.s,**settings)

    def test_unclaimed_status_raw_counts_calibrated_in_sensor_order(self):
        raw=[7,0,0,-13]+[0]*10+[144,32,0,5]+[0]*13
        j=observer.encoder(raw,0,dict(zero=22.5,reversed=False))
        self.assertAlmostEqual(j['angle'],-9.84375)
        self.assertTrue(j['healthy'])
        raw[16]=1
        self.assertIsNone(observer.encoder(raw,0,dict(zero=22.5,reversed=False))['angle'])

    def test_encoder_explains_missing_magnet_without_publishing_an_angle(self):
        raw=[7,1,0,-13]+[0]*10+[3710,16,0,7]+[0]*13
        joint=observer.encoder(raw,1,dict(zero=0,reversed=False))
        self.assertFalse(joint['healthy'])
        self.assertIsNone(joint['angle'])
        self.assertEqual(joint['reason'],'Elbow magnet not detected')
        raw[15]=32
        self.assertIsNone(observer.encoder(raw,1,dict(zero=0,reversed=False))['reason'])

if __name__=='__main__':unittest.main()
