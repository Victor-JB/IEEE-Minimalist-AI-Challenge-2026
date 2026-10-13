"""Optional pivot + D7 driver. Explicit RPC allowlist excludes every stepper move/power op.

Browser heartbeat expires after 3 seconds. Light writes are short renewable fades
back to zero. The two-write refresh is best effort, not an atomic firmware lease.
"""
import math
import secrets
import time
from pointing import physical_aim


def light_gate(data,age):
    joints=data.get('joints',[])
    if age>1.5 or len(joints)!=3 or any(not j.get('healthy') or j.get('angle') is None or j.get('age_ms',999)>100 for j in joints):
        return False,'Waiting for fresh encoders'
    phi=sum(j['angle'] for j in joints)
    if not 110<=phi<=170:return False,'Lamp off · aim the supported head down at the desk'
    if data.get('detections'):return False,'Lamp off · face in the camera view'
    pivot=data.get('pivot') or {}
    if not pivot.get('enabled') or pivot.get('fault') or abs(pivot.get('current',999))>12:
        return False,'Lamp off · pivot position unavailable'
    return True,'Desk-directed lamp enabled'


class Outputs:
    ALLOWED={0,4,7,16,17,18}
    def __init__(self,bridge):
        self.bridge=bridge;self.owner=secrets.randbelow(2**30)+1
        self.connected=False;self.claimed=False;self.requested=False;self.heartbeat=-1e9
        self.reason='Physical outputs disabled';self.level=0.;self.last_target=None;self.manual_target=None

    def swivel(self,angle):
        if angle is not None and (isinstance(angle,bool) or not isinstance(angle,(int,float)) or not math.isfinite(angle) or abs(angle)>10):
            raise ValueError('Swivel must be between -10 and 10 degrees')
        if not self.requested:raise ValueError('Enable real outputs before moving the swivel')
        self.manual_target=angle

    def command(self,op,*args):
        if op not in self.ALLOWED:raise ValueError('Stepper operations are forbidden in hybrid mode')
        raw=self.bridge.call('arm_command',[op,self.owner,*args],timeout=.2)
        if len(raw)!=38 or raw[0]!=1 or raw[1]:raise RuntimeError('Pivot control refused: '+str(raw[:4]))
        return raw

    def light(self,level,fade=0):
        r=self.bridge.call('arm_light',[level,fade],timeout=.2)
        if len(r)!=7 or r[0]!=4 or not r[1] or r[5] or r[6]:raise RuntimeError('D7 lamp output unavailable')

    def disable(self,force_light=False):
        self.requested=False;self.level=0.;self.last_target=None;self.manual_target=None
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
                self.command(17,1500,7407,-12000,12000,-1,20000)
            status=self.command(4)
            if status[6] or status[7]:raise RuntimeError('Unexpected stepper activity · hybrid stopped')
            aimed=physical_aim(data.get('joints',[]),demo.get('desk_target')) if demo.get('light_requested') else None
            target=aimed if aimed is not None else max(-10.,min(10.,float(demo.get('pivot_target',0))))
            if self.manual_target is not None:target=self.manual_target
            if self.last_target is None or abs(target-self.last_target)>.5:
                self.command(16,round(target*1000),20000);self.last_target=target
            allowed,self.reason=light_gate(data,age)
            if demo.get('light_requested') and aimed is None:
                allowed=False;self.reason='Lamp off · selected spot is outside the fixed arm reach'
            if abs((data.get('pivot') or {}).get('current',999)-target)>2:
                allowed=False;self.reason='Lamp off · pivot is settling'
            if self.manual_target is not None:
                allowed=False;self.reason='Manual swivel · lamp off'
            desired=demo.get('brightness',0) if allowed and demo.get('light_requested') else 0.
            if desired:
                self.level=max(0.,min(.8,float(desired)))
                self.light(round(self.level**2.2*1000))
                self.light(0,800)  # on-MCU expiry, renewed at 10 Hz; no persistent nonzero target
            else:
                if self.level:self.light(0)
                self.level=0.
                if allowed:self.reason='Pivot live · lamp waiting for pointing gesture'
        except Exception as exc:
            self.reason=str(exc)
            try:self.disable()
            except Exception:pass

    def snapshot(self):
        return dict(enabled=self.requested,connected=self.claimed,brightness=self.level,
                    reason=self.reason,swivel_mode='auto' if self.manual_target is None else 'manual',swivel_target=self.last_target,steppers='disabled · no motion commands')
