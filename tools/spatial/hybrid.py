"""Head swivel exclusively tracks subjects. D7 stays off. No stepper motion.

Browser heartbeat expires after 3 seconds; Stop and stale data release the servo.
"""
import math
import secrets
from tracking import SubjectTracker


class Outputs:
    ALLOWED={0,4,7,16,17,18}
    def __init__(self,bridge):
        self.bridge=bridge;self.owner=secrets.randbelow(2**30)+1
        self.connected=False;self.claimed=False;self.requested=False;self.heartbeat=-1e9
        self.reason='Physical outputs disabled';self.level=0.;self.last_target=None

    def command(self,op,*args):
        if op not in self.ALLOWED:raise ValueError('Stepper operations are forbidden in hybrid mode')
        raw=self.bridge.call('arm_command',[op,self.owner,*args],timeout=.2)
        if len(raw)!=38 or raw[0]!=1 or raw[1]:raise RuntimeError('Pivot control refused: '+str(raw[:4]))
        return raw

    def light(self,level,fade=0):
        r=self.bridge.call('arm_light',[level,fade],timeout=.2)
        if len(r)!=7 or r[0]!=4 or not r[1] or r[5] or r[6]:raise RuntimeError('D7 lamp output unavailable')

    def disable(self,force_light=False):
        self.requested=False;self.level=0.;self.last_target=None
        try:
            if force_light and not self.connected:self.connected=self.bridge.connect(timeout=1)
            if self.connected:
                try:self.light(0)
                finally:
                    if self.claimed:
                        try:self.command(18)
                        finally:self.command(7)
        finally:
            self.claimed=False
            if self.connected:self.bridge.disconnect()
            self.connected=False

    def step(self,data,stamp,now):
        age=now-stamp
        try:
            if not self.requested:return
            if now-self.heartbeat>3:
                self.reason='Connection expired · outputs off';self.disable();return
            demo=data.get('demo') or {}
            joints=data.get('joints',[])
            healthy=len(joints)==3 and all(j.get('healthy') and isinstance(j.get('angle'),(int,float)) and math.isfinite(j['angle']) and 0<=j.get('age_ms',999)<=100 for j in joints)
            if age>1.5 or not healthy or data.get('error') or demo.get('state')=='STOPPED':
                self.reason='Stopped or stale camera · outputs off';self.disable();return
            # Never seize a running arm. Claiming a quiet arm does not configure or move steppers.
            if not self.connected:
                self.connected=self.bridge.connect(timeout=1)
                if not self.connected:raise ConnectionError('Board bridge unavailable')
            if not self.claimed:
                status=self.bridge.call('arm_observe',timeout=.2)
                if len(status)!=38 or status[0]!=1 or status[2] or status[6] or status[7]:
                    raise RuntimeError('Disconnect other arm controls and let the stepper drivers sleep first')
                self.command(0);self.claimed=True
                self.light(0)
                self.command(17,1500,7407,-45000,45000,-1,round(SubjectTracker.SPEED*1000))
            status=self.command(4)
            if status[6] or status[7]:raise RuntimeError('Unexpected stepper activity · hybrid stopped')
            tracking=data.get('tracking') or {}
            value=tracking.get('target')
            if not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError('Subject tracking unavailable')
            target=max(-SubjectTracker.LIMIT,min(SubjectTracker.LIMIT,value))
            if self.last_target is None or abs(target-self.last_target)>.5:
                self.command(16,round(target*1000),round(SubjectTracker.SPEED*1000));self.last_target=target
            # The camera points at the subject. Gesture lighting is digital only.
            self.light(0);self.level=0.
            self.reason=tracking.get('status','Tracking subject')+' · real lamp off'
        except Exception as exc:
            self.reason=str(exc)
            try:self.disable()
            except Exception:pass

    def snapshot(self):
        return dict(enabled=self.requested,connected=self.claimed,brightness=self.level,
                    reason=self.reason,swivel_mode='subject tracking',swivel_target=self.last_target,steppers='disabled · no motion commands')
