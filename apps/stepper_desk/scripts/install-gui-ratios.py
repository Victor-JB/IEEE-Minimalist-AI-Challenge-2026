#!/usr/bin/env python3
"""Validate CLI ratio evidence and install GUI preferences while the app is closed."""
import argparse,json,plistlib,subprocess,sys,uuid
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from arm_client import validate_ratio_data,JOINTS,MOTOR
DOMAIN='dev.tongtong.stepperdeskq'
def prepare(data,prefs):
    joints=json.loads(prefs['q.arm.geometry.v1'])['joints']
    fingerprint='|'.join(f"{j}:{float(c['zero'])}:{str(c['reversed']).lower()}:{str(c['calibrated']).lower()}:{prefs[f'q.axis{MOTOR[j]}.microsteps']}:{prefs[f'q.axis{MOTOR[j]}.fullSteps']}" for j,c in enumerate(joints))
    if len(joints)!=3 or any(c.get('calibrated') is not True for c in joints):raise ValueError('Calibrate all GUI angles first')
    ratios=validate_ratio_data(data,fingerprint)
    previous=json.loads(prefs.get('q.arm.poses.v1',b'{}'))
    trials=[[dict(id=str(uuid.uuid4()).upper(),direction=t['direction'],pulses=t['pulses'],degrees=t['degrees']) for t in data['trials'][name]] for name in JOINTS]
    if any(type(t['pulses']) is not int for rows in trials for t in rows):raise ValueError('Pulse counts must be integers')
    return dict(fingerprint=fingerprint,trials=trials,ratios=ratios,poses=previous.get('poses',[]))
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('ratios',type=Path);p.add_argument('--install',action='store_true');args=p.parse_args()
    if subprocess.run(['pgrep','-x','StepperDeskQ'],stdout=subprocess.DEVNULL).returncode==0:raise SystemExit('Close Stepper Desk Q before installing preferences')
    prefs=plistlib.loads(subprocess.check_output(['defaults','export',DOMAIN,'-']))
    result=prepare(json.loads(args.ratios.read_text()),prefs)
    if args.install:
        payload=json.dumps(result,separators=(',',':')).encode()
        subprocess.run(['defaults','write',DOMAIN,'q.arm.poses.v1','-data',payload.hex()],check=True)
        saved=plistlib.loads(subprocess.check_output(['defaults','export',DOMAIN,'-']))
        if json.loads(saved['q.arm.poses.v1'])!=result:raise RuntimeError('Preference readback mismatch')
    print(json.dumps(dict(installed=args.install,ratios=result['ratios'],preserved_poses=len(result['poses'])),indent=2))
if __name__=='__main__':main()
