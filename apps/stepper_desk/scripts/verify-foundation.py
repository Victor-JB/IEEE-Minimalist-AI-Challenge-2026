"""Run on Q: no JOG/POSE/HOLD; pad test only with physical SLEEP low."""
import sys,json,time
sys.path.insert(0,'/home/arduino/IEEE-Minimalist-AI-Challenge-2026/apps/stepper_desk/python')
from arduino.router_bridge import Bridge
from arm_gateway import ArmGateway
b=Bridge();g=ArmGateway(b);report={}
try:
    assert b.connect(timeout=3)
    s=g.command('HELLO');report['boot']=s
    assert s['run']==0 and s['awake']==0
    s=g.command('POWER 0 '+str(s['generation']))
    assert s['error']==0 and s['enabled']==0 and s['awake']==0
    check=b.call('arm_selftest',g.owner,timeout=1)
    report['sleep_locked_pad_check']=check
    assert check==[1,0,0,1,0,2,0,4,0],check
    configs=[[0,53349609,1,-1,0,0],[1,22500000,0,1,0,0],[2,-30322266,1,-1,0,0]]
    for cfg in configs:
        s=g.command('CONFIG '+' '.join(map(str,cfg)))
        assert s['error']==0 and s['configuration_echo']==cfg
    assert s['configured']==7 and s['verified']==0
    report['idle_samples']=[]
    for _ in range(20):
        s=g.command('STATUS');report['idle_samples'].append(s)
        assert s['run']==0 and s['awake']==0 and all(a['count']==0 for a in s['axes'])
        time.sleep(.1)
    s=g.command('POWER 1 '+str(s['generation']))
    assert s['error']==0 and s['run']==0 and s['awake']==0 and s['enabled']==1
    report['final']=s
    report['note']='No JOG, POSE, CURVE or HOLD sent. Three diagnostic STEP pad transitions occurred with physical D8 LOW and movement locked; drivers ignored them. This does not measure pulse width or loaded motion.'
finally:
    try:g.stop()
    finally:b.disconnect()
print(json.dumps(report,indent=2))
