#!/usr/bin/env python3
"""Run on Arduino Q. Test physical STEP pad edges with D8 sleep locked LOW."""
import json,sys
sys.path.insert(0,'/home/arduino/IEEE-Minimalist-AI-Challenge-2026/apps/stepper_desk/python')
from arduino.router_bridge import Bridge
from arm_gateway import ArmGateway
b=Bridge();g=ArmGateway(b);out={}
try:
 if not b.connect(timeout=3):raise RuntimeError('Router unavailable')
 s=g.command('HELLO')
 s=g.command(f"POWER 0 {s['generation']}")
 if s['error'] or s['awake'] or s['enabled']:raise RuntimeError('Sleep lock required')
 out['static']=b.call('arm_selftest',g.owner,timeout=2)
 if out['static'][1]:raise RuntimeError('Static pad test failed')
 out['rates']=[]
 for rate in (100,1333,2400):
  result=b.call('arm_pulse_selftest',[g.owner,rate,100],timeout=6)
  out['rates'].append(result)
  if len(result)!=12 or result[1] or any(result[3+3*a]!=result[4+3*a] or result[5+3*a] for a in range(3)):
   raise RuntimeError('Counted pad test failed')
 out['final']=g.command('STATUS')
 if out['final']['awake'] or out['final']['enabled']:raise RuntimeError('Drivers did not stay locked asleep')
finally:
 try:g.stop()
 finally:b.disconnect();print(json.dumps(out,indent=2))
