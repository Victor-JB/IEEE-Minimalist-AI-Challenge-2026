"""Offline local-target gates and cleanup; never connects to hardware."""
import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('local_pose',ROOT/'scripts/validate-local-pose.py')
local=importlib.util.module_from_spec(spec);spec.loader.exec_module(local)

class FakeSession:
    def __init__(self,profile,initial):
        self.fingerprint=profile['fingerprint'];self.initial=initial;self.calls=[];self.closed=False;self.events=[];self.error=None;self.modify=None
    def __enter__(self):return self
    def __exit__(self,*args):self.closed=True
    def _ready(self):
        axes=[{'angle':0} for _ in range(3)]
        for j,a in enumerate(local.client.MOTOR):axes[a]['angle']=round(self.initial[j]*1000)
        return {'axes':axes}
    def _event(self,kind,**values):self.events.append((kind,values))
    def _jog(self,joint,direction,seconds,travel,trial,rate,powered):
        self.calls.append((joint,direction,seconds,travel,trial,rate,powered))
        if self.error:raise self.error
        j=local.client.JOINTS.index(joint);sign=(1 if j==0 else -1)*(1 if direction=='forward' else -1)
        final=self.initial[:];final[j]+=travel*sign
        result=dict(initial_angles=self.initial[:],powered_initial_angles=self.initial[:],powered_final_angles=final[:],final_angles=final[:],
                    pulses=200,reason='travel reached',recording='fake.jsonl')
        if self.modify:self.modify(result)
        return result

class LocalPoseTests(unittest.TestCase):
    def setUp(self):
        self.profile_path=ROOT/'calibration/local-endpoint-profile-v1.json'
        _,self.fingerprint=local.client.load_calibration(local.client.DEFAULT_CALIBRATION)
        self.profile=local.load_profile(self.profile_path,self.fingerprint)
        self.initial=[-11.7,62.7,3.8]
    def test_real_profile_provenance_and_wrong_fingerprint(self):
        self.assertIs(self.profile['globally_verified'],False)
        with self.assertRaises(ValueError):local.load_profile(self.profile_path,'wrong')
        altered=copy.deepcopy(self.profile);altered['provenance'][0]['sha256']='0'*64
        with tempfile.TemporaryDirectory() as tmp:
            file=Path(tmp)/'bad.json';file.write_text(json.dumps(altered))
            with self.assertRaises(ValueError):local.load_profile(file,self.fingerprint)
    def test_directions_and_finite_single_joint_bounds(self):
        for joint,start,target,expected in [('shoulder',-11.7,-9.5,'forward'),('shoulder',-9.5,-11.7,'reverse'),
                ('elbow',62.7,60.5,'forward'),('elbow',60.5,62.7,'reverse'),('head',3.8,1.5,'forward'),('head',1.5,3.8,'reverse')]:
            self.assertEqual(local.target_check(self.profile,joint,target,start)[3],expected)
        for target in [float('nan'),float('inf'),True,-50,30,-11,-5]:
            with self.subTest(target=target),self.assertRaises(ValueError):local.target_check(self.profile,'shoulder',target,-11.7)
        with self.assertRaises(ValueError):local.target_check(self.profile,'head',1.5,30)
    def test_success_uses_bounded_existing_jog_and_reports_separate_prediction(self):
        arm=FakeSession(self.profile,self.initial)
        report=local.validate_move(arm,self.profile,'elbow',60.5)
        self.assertTrue(report['passed']);self.assertEqual(report['powered_error_deg'],0)
        self.assertEqual(arm.calls,[('elbow','forward',6,2.200000000000003,False,1333,True)])
        self.assertIn('without_reversal',report['model_predicted_counts'])
        self.assertIs(report['globally_verified'],False)
        self.assertEqual(arm.events[-1][0],'local_validation_result')
    def test_no_progress_deadline_error_and_other_joint_motion_rejected(self):
        def noop(r):r.update(pulses=0,powered_final_angles=r['powered_initial_angles'][:])
        def deadline(r):r['reason']='deadline'
        def bad_endpoint(r):r['final_angles'][0]+=2
        def other_joint(r):r['powered_final_angles'][1]+=1
        for modify in (noop,deadline,bad_endpoint,other_joint):
            arm=FakeSession(self.profile,self.initial);arm.modify=modify
            with self.subTest(modify=modify),self.assertRaises(local.client.ArmError):local.validate_move(arm,self.profile,'shoulder',-9.5)
    def test_postconditions_reject_out_of_scope_endpoints_and_reread_start(self):
        # A target inside the envelope can be within 1.5 degrees of an
        # out-of-scope endpoint: tolerance alone must never certify it.
        initial=[-1.0,62.7,3.8]
        def powered_out(r):r['powered_final_angles'][0]=2.4
        def released_out(r):r['final_angles'][0]=2.4
        def reread_out(r):r['initial_angles'][0]=-18
        def reread_too_near(r):r['initial_angles'][0]=1.0
        def powered_baseline_out(r):r['powered_initial_angles'][0]=-18
        def powered_baseline_too_near(r):r['powered_initial_angles'][0]=1.0
        for modify in (powered_out,released_out,reread_out,reread_too_near,powered_baseline_out,powered_baseline_too_near):
            arm=FakeSession(self.profile,initial);arm.modify=modify
            with self.subTest(modify=modify),self.assertRaises(local.client.ArmError):
                local.validate_move(arm,self.profile,'shoulder',1.5)
            self.assertFalse(arm.events[-1][1]['passed'])

    def test_preview_never_connects_and_execution_exception_closes(self):
        with tempfile.TemporaryDirectory() as tmp:
            args=['--profile',str(self.profile_path),'--joint','shoulder','--target','-9.5','--output',str(Path(tmp)/'new.jsonl')]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(local.main(args,session_factory=lambda **kw:self.fail('preview connected')),0)
            arm=FakeSession(self.profile,self.initial);arm.error=local.client.ArmError('sensor fault')
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(local.main(args+['--execute'],session_factory=lambda **kw:arm),1)
            self.assertTrue(arm.closed)
    def test_profile_cannot_weaken_tolerance_or_global_gate(self):
        for field,value in [('tolerance_deg',1.51),('max_move_deg',6),('globally_verified',True)]:
            profile=copy.deepcopy(self.profile);profile[field]=value
            with tempfile.TemporaryDirectory() as tmp:
                file=Path(tmp)/'invalid.json';file.write_text(json.dumps(profile))
                with self.assertRaises(ValueError):local.load_profile(file,self.fingerprint)

if __name__=='__main__':unittest.main()
