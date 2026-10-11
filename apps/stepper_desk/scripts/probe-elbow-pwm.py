#!/usr/bin/env python3
"""Bounded elbow-only PWM diagnostic. Does not certify counted pulses or ratios."""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from arm_client import Transport,healthy,angles,finite

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--rpm',type=float,default=200);p.add_argument('--seconds',type=float,default=.4)
 p.add_argument('--max-degrees',type=float,default=3);p.add_argument('--direction',choices=['forward','reverse'],default='forward')
 p.add_argument('--output',type=Path,required=True);p.add_argument('--execute',action='store_true')
 a=p.parse_args()
 if not a.execute:p.error('--execute required')
 finite(a.rpm,3,200,'RPM');finite(a.seconds,.1,1,'Seconds');finite(a.max_degrees,.1,6,'Travel')
 hz=round(a.rpm*400/60);direction=int(a.direction=='reverse');sign=1 if direction else -1
 a.output.parent.mkdir(parents=True,exist_ok=True)
 began=time.monotonic();claimed=False
 with a.output.open('x') as f, Transport() as observer, Transport(axis=0) as motor:
  def event(kind,**kw):
   f.write(json.dumps(dict(type=kind,time=time.monotonic()-began,**kw),allow_nan=False)+'\n');f.flush()
  def cmd(text):
   event('command',text=text);s=motor.request(text);event('motor_sample',sample=s)
   if s['error'] or s['fault']:raise RuntimeError('Motor error/fault')
   return s
  def observe():
   s=observer.request('OBSERVE');event('feedback',sample=s)
   if not healthy(s):raise RuntimeError('Fresh feedback required for all three joints')
   return s
  event('metadata',rpm=a.rpm,nominal_hz=hz,pulses_per_revolution=400,direction=a.direction,max_seconds=a.seconds,max_degrees=a.max_degrees,note='PWM diagnostic; hz telemetry is commanded frequency, not counted STEP edges.')
  try:
   pre=observer.request('OBSERVE')
   if pre['awake'] or not pre['enabled']:raise RuntimeError('Arm must be idle, asleep and movement enabled')
   cmd('HELLO');claimed=True
   s=cmd('LIMITS 53349609 1 -1');initial=angles(observe())
   if not -70<=initial[1]+sign*a.max_degrees<=77:raise RuntimeError('Probe endpoint lacks joint-limit margin')
   started=time.monotonic();last_progress=started;last_angle=initial[1]
   s=cmd(f"CURVE 1 {direction} 400 2 0 0 20 {hz} {s['power_generation']}")
   while True:
    current=angles(observe());delta=current[1]-initial[1]
    if sign*delta<-.3:raise RuntimeError('Opposite direction response')
    if any(abs(current[j]-initial[j])>.75 for j in (0,2)):raise RuntimeError('Uncommanded joint movement')
    if abs(current[1]-last_angle)>=.176:last_progress=time.monotonic();last_angle=current[1]
    if time.monotonic()-last_progress>.6:raise RuntimeError('No measurable response for 0.6 seconds')
    if abs(delta)>=a.max_degrees or time.monotonic()-started>=a.seconds:break
    if s['run']!=1 or s['limit_state']!=0:raise RuntimeError('Unexpected motor/limit stop')
    time.sleep(min(.025,max(0,a.seconds-(time.monotonic()-started))))
    if time.monotonic()-started>=a.seconds:break
    s=cmd('HOLD 1')
   cmd('STOP')
   for _ in range(8):time.sleep(.075);final=observe()
   if final['awake']:raise RuntimeError('Drivers did not sleep after STOP')
   result=dict(initial_angles=initial,final_angles=angles(final),delta_deg=[b-a for a,b in zip(initial,angles(final))],nominal_hz=hz,rpm=a.rpm,asleep=True,recording=str(a.output))
   if sign*result['delta_deg'][1]<-.3 or any(abs(result['delta_deg'][j])>.75 for j in (0,2)):raise RuntimeError('Unexpected settled movement')
   event('result',**result);print(json.dumps(result,indent=2))
  except BaseException as exc:event('error',message=str(exc));raise
  finally:
   if claimed:
    try:
     if not motor.closed:cmd('STOP')
    finally:motor.close()
if __name__=='__main__':main()
