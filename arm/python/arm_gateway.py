"""Compact all-joint transport. Does not generate HOLD or execute recordings."""
import errno
import secrets

HEADER = ['v','error','owner','state','id','ms','run','awake','enabled','generation','configured','verified','duration','elapsed']
AXIS = ['angle','reference','target','count','direction','age','magnet','sensor_error']
SERVO_HEADER = ['v','state','reason','id','retries','stalls','stall_axis','stall_angle','duration','elapsed','scale']
SERVO_AXIS = ['estimate','reference','goal','velocity','rate','ppd_forward','ppd_reverse','direction','count']
ROLL_FIELDS = ['v','enabled','target','current','pulse_us','fault']
def decode_roll(raw):
    if not isinstance(raw,(list,tuple)) or len(raw)!=6 or raw[0]!=3 or any(type(v)!=int for v in raw):
        raise ValueError('Roll servo firmware required')
    result=dict(zip(ROLL_FIELDS,raw)); result.update(kind='roll',supported=True)
    return result
def decode_servo(raw):
    if not isinstance(raw,(list,tuple)) or len(raw)!=38 or raw[0]!=2 or any(type(v)!=int for v in raw):
        raise ValueError('Closed-loop servo firmware required')
    result=dict(zip(SERVO_HEADER,raw[:11]))
    result['axes']=[dict(zip(SERVO_AXIS,raw[11+9*a:20+9*a])) for a in range(3)]
    result['kind']='servo'
    return result
class ArmGateway:
    def __init__(self, bridge):
        self.bridge = bridge
        self.owner = secrets.randbelow(2**31-1)+1
        self.claimed = False
    def rpc(self, op, *args):
        raw = self.bridge.call('arm_command', [op,self.owner,*args], timeout=0.2)
        if not isinstance(raw,(list,tuple)) or len(raw)!=38 or raw[0]!=1 or any(type(v)!=int for v in raw):
            raise ValueError('Coordinated firmware required')
        if raw[1]==-errno.EACCES:
            raise ConnectionError('Arm ownership lost; reconnect')
        result=dict(zip(HEADER,raw[:14]))
        result['axes']=[dict(zip(AXIS,raw[14+8*a:22+8*a])) for a in range(3)]
        result['kind']='arm'
        return result
    def command(self,line):
        words=line.split()
        if not words: return None
        name,args=words[0],words[1:]
        if name=='HELLO' and not args and not self.claimed:
            result=self.rpc(0)
            if result['error']: raise RuntimeError('Disconnect manual controls before connecting the pose workspace')
            self.claimed=True
            return result
        if not self.claimed: raise ValueError('HELLO required')
        if name=='SERVOSTAT' and not args: return self.servo_status()
        if name=='ROLLSTAT' and not args: return self.roll_status()
        # 10-15: closed-loop MOVE, STREAM setpoint, HALT, SERVO axis config, SERVOTIME, SCALE.
        # 16-18: head-tilt ROLL angle, ROLLCONFIG, ROLLOFF (limp).
        commands={'CONFIG':(1,6),'POSE':(2,5),'JOG':(3,4),'STATUS':(4,0),'HOLD':(5,1),'STOP':(6,0),'POWER':(8,2),'PAUSE':(9,1),
                  'MOVE':(10,7),'STREAM':(11,9),'HALT':(12,0),'SERVO':(13,4),'SERVOTIME':(14,3),'SCALE':(15,1),
                  'ROLL':(16,2),'ROLLCONFIG':(17,6),'ROLLOFF':(18,0)}
        if name not in commands: raise ValueError('Unknown arm command')
        op,n=commands[name]
        if (len(args)!=n and not (name=='JOG' and len(args) in (5,6))) or any(not w.isascii() or not w.lstrip('-').isdecimal() or len(w)>11 or '-' in w[1:] for w in args):
            raise ValueError('Malformed arm command')
        values=list(map(int,args))
        if any(v < -2147483647 or v > 2147483647 for v in values): raise ValueError('Argument out of range')
        if name=='JOG' and len(values)>=5 and not 20<=values[3]<=2400: raise ValueError('Pulse rate out of range')
        if name=='JOG' and len(values)==6 and not 80<=values[4]<=500: raise ValueError('Preparation time out of range')
        result=self.rpc(op,*values)
        if op==1 and result['error']==0:
            result['configuration_echo']=values
        return result
    def servo_status(self):
        raw = self.bridge.call('arm_servo_status', timeout=0.2)
        return decode_servo(raw)
    def roll_status(self):
        # Firmware from before the roll servo has no such RPC: report that instead of failing,
        # so a session on old firmware keeps working without head tilt.
        try:
            raw = self.bridge.call('arm_roll_status', timeout=0.2)
        except Exception:
            return {'kind':'roll','supported':False}
        try:
            return decode_roll(raw)
        except ValueError:
            return {'kind':'roll','supported':False}
    def stop(self):
        if self.claimed:
            try: self.rpc(6)
            finally:
                try: self.rpc(7)
                finally: self.claimed=False
