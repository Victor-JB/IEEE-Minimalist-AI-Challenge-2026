#!/usr/bin/env python3
"""Bounded single-joint sensor-target check; never installs global ratios."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
import arm_client as client


def number(value,lo,hi,name):
    return client.finite(value,lo,hi,name)


def load_profile(path,fingerprint):
    profile=json.loads(Path(path).read_text())
    if (profile.get('schema_version')!=1 or profile.get('profile_type')!='local_single_joint_endpoint' or
        profile.get('status')!='accepted_local_retrospective' or profile.get('globally_verified') is not False):
        raise ValueError('An accepted local-only endpoint profile is required')
    if profile.get('fingerprint')!=fingerprint:raise ValueError('Profile does not match angle calibration/microstep fingerprint')
    number(profile.get('tolerance_deg'),.001,1.5,'Profile tolerance')
    number(profile.get('max_move_deg'),1.5,5,'Profile move cap')
    if set(profile.get('joints',{}))!=set(client.JOINTS):raise ValueError('Profile needs all three joint definitions')
    for j,name in enumerate(client.JOINTS):
        entry=profile['joints'][name]
        if entry.get('motor_axis')!=client.MOTOR[j] or entry.get('forward_sign')!=(1 if j==0 else -1):raise ValueError('Profile joint mapping mismatch')
        bounds=entry.get('bounds_deg')
        if not isinstance(bounds,list) or len(bounds)!=2:raise ValueError('Missing local bounds')
        lo,hi=client.LIMITS[j]
        left=number(bounds[0],lo,hi,'Local lower bound');right=number(bounds[1],lo,hi,'Local upper bound')
        if left>=right:raise ValueError('Invalid local bounds')
        model=entry.get('model',{})
        number(model.get('reversal_pulses'),0,3600,'Reversal pulses')
        if model.get('kind')=='gain_plus_reversal':number(model.get('gain_pulses_per_degree'),1,10000,'Model gain')
        elif model.get('kind')=='piecewise_cumulative_gain_plus_reversal':
            number(model.get('knot_deg'),lo,hi,'Model knot')
            for field in ('low_pulses_per_degree','high_pulses_per_degree'):number(model.get(field),1,10000,'Model gain')
        else:raise ValueError('Unsupported local model')
    provenance=profile.get('provenance')
    if not isinstance(provenance,list) or not provenance:raise ValueError('Missing profile provenance')
    for item in provenance:
        source=Path(item.get('source',''))
        resolved=(client.ROOT/source).resolve()
        if source.is_absolute() or not resolved.is_relative_to(client.ROOT.resolve()) or not resolved.is_file():raise ValueError('Invalid provenance source')
        expected=item.get('sha256')
        if not isinstance(expected,str) or hashlib.sha256(resolved.read_bytes()).hexdigest()!=expected:raise ValueError('Provenance hash mismatch')
    return profile


def target_check(profile,joint,target,initial=None):
    j=client.JOINTS.index(joint);entry=profile['joints'][joint]
    target=number(target,*entry['bounds_deg'],'Target within local bounds')
    number(target,client.LIMITS[j][0]+2,client.LIMITS[j][1]-2,'Target with hard-limit margin')
    if initial is None:return target
    start=number(initial,*entry['bounds_deg'],'Initial angle within local bounds')
    travel=number(abs(target-start),1.5,min(5,profile['max_move_deg']),'Requested displacement')
    sign=1 if target>start else -1
    direction='forward' if sign==entry['forward_sign'] else 'reverse'
    return target,travel,sign,direction


def predicted_counts(model,start,target):
    if model['kind']=='gain_plus_reversal':base=abs(target-start)*model['gain_pulses_per_degree']
    else:
        def integral(angle):
            delta=angle-model['knot_deg']
            return delta*(model['low_pulses_per_degree'] if delta<0 else model['high_pulses_per_degree'])
        base=abs(integral(target)-integral(start))
    return {'without_reversal':base,'with_reversal':base+model['reversal_pulses'],
            'history':'unknown; predictions are reported only, never used as pulse commands'}


def validate_move(arm,profile,joint,target):
    if arm.fingerprint!=profile['fingerprint']:raise ValueError('Session fingerprint changed')
    j=client.JOINTS.index(joint);sample=arm._ready();initial=client.angles(sample)
    target,travel,sign,direction=target_check(profile,joint,target,initial[j])
    rate=1000 if joint=='head' else 1333
    prediction=predicted_counts(profile['joints'][joint]['model'],initial[j],target)
    arm._event('local_validation_request',joint=joint,target_deg=target,tolerance_deg=profile['tolerance_deg'],model_predicted_counts=prediction)
    # _jog retains readiness, lease, sensor, wrong-direction, other-joint, travel,
    # powered settlement and STOP/failure cleanup. No ratios are supplied.
    result=arm._jog(joint,direction,6,travel,False,rate,True)
    actual_initial=result['initial_angles'];before=result['powered_initial_angles'];powered=result['powered_final_angles'];released=result['final_angles']
    if any(not isinstance(v,list) or len(v)!=3 for v in (actual_initial,before,powered,released)):raise client.ArmError('Missing settled endpoint feedback')
    for values in (actual_initial,before,powered,released):
        for value in values:number(value,-180,180,'Endpoint angle')
    tolerance=profile['tolerance_deg']
    errors={'powered_error_deg':powered[j]-target,'released_error_deg':released[j]-target}
    failures=[]
    # The existing jog re-reads readiness after our preflight. Do not certify a
    # stale-baseline move or an endpoint outside the accepted local envelope.
    # These are postconditions, not an atomic MCU local-envelope interlock.
    try:
        target_check(profile,joint,target,actual_initial[j])
        target_check(profile,joint,target,before[j])
        for endpoint in (powered[j],released[j]):
            number(endpoint,*profile['joints'][joint]['bounds_deg'],'Endpoint within local bounds')
    except ValueError as exc:failures.append(str(exc))
    if result['reason']!='travel reached':failures.append('requested travel was not reached')
    if number(result['pulses'],0,3600,'Issued count')<1 or sign*(powered[j]-before[j])<.176:failures.append('insufficient issued pulses or signed sensor progress')
    if any(abs(error)>tolerance for error in errors.values()):failures.append('endpoint exceeds accepted tolerance')
    if any(abs(v[k]-initial[k])>.75 for k in range(3) if k!=j for v in (powered,released)):failures.append('another joint moved')
    report={'validation_type':'single_joint_sensor_target_validation','joint':joint,'target_deg':target,
            'tolerance_deg':tolerance,'passed':not failures,'failures':failures,**errors,
            'model_predicted_counts':prediction,'issued_pulses':result['pulses'],
            'recording':result['recording'],'globally_verified':False,
            'scope':'local sensor endpoint only; not coordinated or feedforward validation'}
    arm._event('local_validation_result',**report)
    if failures:raise client.ArmError('; '.join(failures))
    return report


def main(argv=None,*,session_factory=client.ArmSession):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--calibration',type=Path,default=client.DEFAULT_CALIBRATION)
    parser.add_argument('--joint',choices=client.JOINTS,required=True)
    parser.add_argument('--target',type=float,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args(argv)
    try:
        _,fingerprint=client.load_calibration(args.calibration)
        profile=load_profile(args.profile,fingerprint);target_check(profile,args.joint,args.target)
        if args.output.exists():raise ValueError('Output must be a new file')
        if not args.execute:
            print(json.dumps({'preview':True,'connected':False,'joint':args.joint,'target_deg':args.target,
                              'initial_angle':'not read; local bounds and 1.5–5 degree displacement checked on execution',
                              'tolerance_deg':profile['tolerance_deg'],'globally_verified':False}))
            return 0
        with session_factory(calibration=args.calibration,output=args.output) as arm:
            report=validate_move(arm,profile,args.joint,args.target)
        print(json.dumps(report));return 0
    except (ValueError,KeyError,TypeError,OSError,client.ArmError) as exc:
        print(str(exc),file=sys.stderr);return 1

if __name__=='__main__':raise SystemExit(main())
