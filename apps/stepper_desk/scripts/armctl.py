#!/usr/bin/env python3
"""Direct Mac scripting interface for Arduino Q. Motion requires --execute."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from arm_client import ArmSession, Transport, DEFAULT_CALIBRATION, JOINTS, load_calibration
from arm_motion import ArmMotion

def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='action',required=True)
    sub.add_parser('status',help='Read without claiming or renewing control')
    sub.add_parser('stop',help='Stop every motor and lock drivers asleep')
    for name in ('record','jog','trial','pose','enable'):
        q=sub.add_parser(name)
        q.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION)
        q.add_argument('--output',type=Path,help='New JSONL recording; existing files are never replaced')
        q.add_argument('--ratios',type=Path,help='Ratio trial file matching this calibration')
        if name in ('jog','trial','pose','enable'):q.add_argument('--execute',action='store_true',help='Required for motion or clearing sleep lock')
        if name in ('jog','trial'):
            q.add_argument('--rate-hz',type=int,default=100,help='Nominal STEP pulses/sec, 20–2400');q.add_argument('joint',choices=JOINTS);q.add_argument('--direction',choices=('forward','reverse'),required=True)
        if name=='record':q.add_argument('--seconds',type=float,default=5)
        if name=='jog':
            q.add_argument('--seconds',type=float,default=.5);q.add_argument('--max-degrees',type=float,default=2)
        if name=='trial':
            q.add_argument('--powered-settle',action='store_true',help='Measure with coils on for up to 0.5 seconds before sleep');q.add_argument('--degrees',type=float,default=6);q.add_argument('--timeout',type=float,default=30)
        if name=='pose':
            q.add_argument('--angles',nargs=3,type=float,required=True,metavar=('SHOULDER','ELBOW','HEAD'));q.add_argument('--timeout',type=float,default=120)
    q=sub.add_parser('move',help='Closed-loop move to a pose, then hold or relax')
    q.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION)
    q.add_argument('--output',type=Path,help='New JSONL recording; existing files are never replaced')
    q.add_argument('--execute',action='store_true',help='Required: the arm moves')
    q.add_argument('--angles',nargs=3,type=float,required=True,metavar=('SHOULDER','ELBOW','HEAD'))
    q.add_argument('--duration',type=float,help='Seconds; stretched if faster than the limits allow')
    q.add_argument('--speed',type=float,help='Degrees/second cap, default 12')
    q.add_argument('--hold',type=float,default=0,help='Seconds to hold powered before relaxing')
    q.add_argument('--tuck',default='auto',help="Shoulder recovery: 'auto', 'off', or an elbow angle")
    q=sub.add_parser('verify-ratios',help='Offline repeatability check; never controls hardware')
    q.add_argument('recordings',nargs='+',type=Path);q.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION);q.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.action in ('jog','trial','pose','enable','move') and not args.execute:p.error('Add --execute to authorize this bounded operation')
    if args.action in ('status','stop'):
        with Transport() as t:result=t.request('OBSERVE' if args.action=='status' else 'LOCK')
    elif args.action=='move':
        tuck=False if args.tuck=='off' else 'auto' if args.tuck=='auto' else float(args.tuck)
        with ArmMotion(calibration=args.calibration,output=args.output,tuck=tuck) as arm:
            result=arm.move_to(args.angles,duration=args.duration,speed=args.speed)
            if result['ok'] and args.hold:result['held_angles']=arm.hold_for(args.hold)
            arm.relax()
    elif args.action=='verify-ratios':
        _,fingerprint=load_calibration(args.calibration)
        spec=importlib.util.spec_from_file_location('recording_analysis',Path(__file__).with_name('analyze-recording.py'))
        analyzer=importlib.util.module_from_spec(spec);spec.loader.exec_module(analyzer)
        trials={j:[] for j in JOINTS}
        seen=set()
        for path in dict.fromkeys(p.resolve() for p in args.recordings):
            report=analyzer.analyze(analyzer.read_events(path),path)
            if report['fingerprint']!=fingerprint:raise ValueError('Recording calibration does not match')
            for move in report['moves']:
                key=(move.get('owner'),move.get('id'))
                if move.get('ratio_candidate') and key not in seen:
                    seen.add(key)
                    joint=move['joint'];trials[joint].append({'direction':move['direction'],'pulses':move['issued_pulses'][joint],
                        'measurement_mode':move['measurement_mode'],'sleep_drift_deg':move.get('sleep_drift_deg'),'degrees':move['signed_delta_deg'][joint],'source':str(path.resolve()),'id':move['id']})
        result={'version':1,'fingerprint':fingerprint,'trials':trials}
        # Validate before publishing a usable ratio file.
        from arm_client import validate_ratio_data
        result['pulses_per_degree']=validate_ratio_data(result,fingerprint)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    else:
        with ArmSession(calibration=args.calibration,ratios=args.ratios,output=args.output) as arm:
            if args.action=='record':result=arm.record(args.seconds)
            elif args.action=='jog':result=arm.jog(args.joint,args.direction,args.seconds,args.max_degrees,args.rate_hz)
            elif args.action=='trial':result=arm.trial(args.joint,args.direction,args.degrees,args.timeout,args.rate_hz,args.powered_settle)
            elif args.action=='pose':result=arm.move_pose(args.angles,args.timeout)
            else:result=arm.power(True)
            result=dict(result,recording=str(arm.output))
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0
if __name__=='__main__':
    try:sys.exit(main())
    except KeyboardInterrupt:print('Interrupted; STOP/release attempted. MCU watchdog remains active.',file=sys.stderr);sys.exit(130)
    except Exception as exc:print(str(exc),file=sys.stderr);sys.exit(1)
