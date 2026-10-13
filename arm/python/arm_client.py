"""Mac-side Arduino Q API. No GUI, third-party Python packages, or background HOLDs."""
from __future__ import annotations
import json
from datetime import datetime, timezone
import uuid
import math
import os
from pathlib import Path
import selectors
import shutil
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CALIBRATION = ROOT/'calibration/arm-calibration-2026-10-11.json'
JOINTS = ('shoulder','elbow','head')
MOTOR = (1,0,2)
LIMITS = ((-45,38),(-72,79),(-80,87))

ROUTER_SOCKET='/var/run/arduino-router.sock'

class ArmError(RuntimeError): pass
class LinkLost(ArmError):
    """The gateway, router socket or SSH link stopped answering (timeout, EOF, broken pipe)."""
class SensorStale(ArmError):
    """Encoder feedback is stale or invalid (age > 100 ms, I2C error, magnet); the MCU stops itself."""

def ssh_target():
    """SSH arguments for the board and the per-request timeout.

    On the Arduino Q itself (the router socket exists) the gateway runs as a local
    process. Otherwise STEPPER_LINK=auto (default) uses USB (ADB forward to localhost:2222) when the board
    is plugged in, else Tailscale (STEPPER_HOST, default daboard). Tailscale round trips
    are ~13 ms with occasional ~250 ms spikes, so requests there get a longer deadline;
    the MCU's 300 ms lease still halts and holds the arm if renewals stop.
    """
    link=os.environ.get('STEPPER_LINK','auto')
    if link not in ('auto','usb','tailscale','local'): raise ValueError('STEPPER_LINK must be auto, usb, tailscale or local')
    if link=='local' or (link=='auto' and os.path.exists(ROUTER_SOCKET)):
        return None,.25   # running on the Arduino Q itself: no SSH hop
    if link!='tailscale':
        adb=shutil.which('adb') or '/opt/homebrew/bin/adb'
        try:
            subprocess.run([adb,'-s','2709356903','forward','tcp:2222','tcp:22'],check=True,timeout=4,
                           stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            return ['-p','2222','arduino@localhost'],.25
        except (OSError,subprocess.SubprocessError):
            if link=='usb': raise ArmError('Arduino Q not found over USB')
    return ['arduino@'+os.environ.get('STEPPER_HOST','daboard')],.6

def gateway_python():
    """Interpreter for a local gateway: this one if it has the router bridge, else the deploy venv."""
    import importlib.util
    if importlib.util.find_spec('arduino') is not None:
        return sys.executable
    for venv in (os.environ.get('STEPPER_VENV'),'/home/arduino/IEEE-Minimalist-AI-Challenge-2026/arm/.venv'):
        if venv and os.path.exists(venv+'/bin/python'): return venv+'/bin/python'
    raise ArmError('arduino-router-bridge not installed: run bash arm/scripts/deploy.sh or pip install -r arm/requirements.txt')

class Transport:
    """Correlated JSON over one SSH process. Closing stdin releases ownership."""
    def __init__(self, axis=None, command=None, timeout=.25):
        """command (with its per-request timeout) replaces the gateway process; for tests."""
        if axis is not None and axis not in (0,1,2): raise ValueError('Invalid motor')
        target,self.timeout=(None,timeout) if command else ssh_target()
        mode=['--group'] if axis is None else ['--axis',str(axis)]
        if command:pass
        elif target is None:
            command=[gateway_python(),'-u',str(ROOT/'python/script_gateway.py'),*mode]
        else:
            remote=' '.join(['/home/arduino/IEEE-Minimalist-AI-Challenge-2026/apps/stepper_desk/.venv/bin/python -u /home/arduino/IEEE-Minimalist-AI-Challenge-2026/apps/stepper_desk/python/script_gateway.py',*mode])
            command=['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=5',
                     '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2',*target,remote]
        self.process=subprocess.Popen(command,
                                      stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,bufsize=0)
        self.selector=selectors.DefaultSelector()
        for name,pipe in [('out',self.process.stdout),('err',self.process.stderr)]:
            os.set_blocking(pipe.fileno(),False);self.selector.register(pipe,selectors.EVENT_READ,name)
        self.buffer=b'';self.errors=b'';self.serial=0;self.first=True;self.closed=False
    def request(self,command):
        if self.closed: raise ArmError('Transport closed')
        self.serial+=1
        wire=(json.dumps({'id':self.serial,'command':command},separators=(',',':'))+'\n').encode()
        if len(wire)>2048: raise ValueError('Command too long')
        # Bounded: the first reply includes gateway start-up, then every request gets self.timeout.
        # Silence, EOF or a dead pipe is LinkLost (the MCU's 300 ms lease already halts the arm).
        wait=8 if self.first else self.timeout
        deadline=time.monotonic()+wait
        try:
            try:self.process.stdin.write(wire)
            except OSError as e:raise LinkLost(f'Arm gateway is gone ({e}) '+self.errors.decode(errors='replace')[-500:]) from e
            while True:
                if b'\n' in self.buffer:
                    line,self.buffer=self.buffer.split(b'\n',1);result=json.loads(line)
                    if result.get('id')!=self.serial: raise ArmError('Mismatched response ID')
                    self.first=False
                    if result.get('ok') is not True:
                        kind={'link_lost':LinkLost,'sensor_stale':SensorStale}.get(result.get('kind'),ArmError)
                        raise kind(result.get('error','Scripting request failed'))
                    return result.get('status')
                left=deadline-time.monotonic()
                if left<=0: raise LinkLost(f'No reply from the arm gateway in {wait:g} s; MCU watchdog will stop motion')
                for key,_ in self.selector.select(left):
                    chunk=os.read(key.fileobj.fileno(),8192)
                    if not chunk:
                        self.selector.unregister(key.fileobj)
                        if key.data=='out': raise LinkLost('Arm transport closed: '+self.errors.decode(errors='replace')[-500:])
                    elif key.data=='out':self.buffer+=chunk
                    else:self.errors=(self.errors+chunk)[-4096:]
                if len(self.buffer)>65536:raise ArmError('Oversized response')
        except BaseException:
            self.close();raise
    def close(self):
        if self.closed:return
        self.closed=True
        try:self.process.stdin.close()
        except OSError:pass
        try:self.process.wait(timeout=.5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:self.process.wait(timeout=.5)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.selector.close()
        self.process.stdout.close();self.process.stderr.close()
    def __enter__(self):return self
    def __exit__(self,*_):self.close()

def finite(value,minimum,maximum,name):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not minimum<=value<=maximum:
        raise ValueError(f'{name} must be finite and between {minimum} and {maximum}')
    return float(value)

def load_calibration(path=DEFAULT_CALIBRATION):
    data=json.loads(Path(path).read_text());joints=data['saved_calibration']['joints'];motors=data['motor_settings']
    if len(joints)!=3 or len(motors)!=3:raise ValueError('Three calibrated joints/motors required')
    for c in joints:
        if c.get('calibrated') is not True or type(c.get('reversed')) is not bool:raise ValueError('Invalid angle calibration')
        finite(c['zero'],-180,180,'Sensor zero')
    fingerprint='|'.join(f"{j}:{float(c['zero'])}:{str(c['reversed']).lower()}:true:{motors[MOTOR[j]]['microsteps_per_full_step']}:{motors[MOTOR[j]]['full_steps_per_revolution']}" for j,c in enumerate(joints))
    return data,fingerprint

def ratio_values(path,fingerprint):
    if path is None:return [0.,0.,0.]
    return validate_ratio_data(json.loads(Path(path).read_text()),fingerprint)

def validate_ratio_data(data,fingerprint):
    if data.get('version')!=1 or data.get('fingerprint')!=fingerprint:raise ValueError('Ratio calibration fingerprint mismatch')
    signs=[]
    try:
        signs=[-1 if part.split(':')[2]=='true' else 1 for part in fingerprint.split('|')]
        if len(signs)!=3:raise ValueError()
    except (IndexError,ValueError):raise ValueError('Malformed calibration fingerprint')
    values=[]
    for j,name in enumerate(JOINTS):
        trials=data['trials'][name]
        if len(trials)<3 or {t['direction'] for t in trials}!={0,1}:raise ValueError('Each ratio needs >=3 trials including both directions')
        ppd=[]
        for t in trials:
            pulses=finite(t['pulses'],50,3600,'Trial pulses');degrees=finite(abs(t['degrees']),5,13,'Trial travel')
            if type(t['direction']) is not int or t['direction'] not in (0,1):raise ValueError('Invalid trial direction')
            expected=signs[j]*(1 if t['direction']==0 else -1)
            if t['degrees']*expected<=0:raise ValueError('Trial direction mismatch')
            ppd.append(finite(pulses/degrees,1,10000,'Pulses/degree'))
        value=statistics.median(ppd)
        if (max(ppd)-min(ppd))/value>.05:raise ValueError('Ratio trials disagree by more than 5%')
        values.append(value)
    return values

def healthy(s):
    axes=s.get('axes',[])
    return len(axes)==3 and all(a['angle']!=-2147483648 and 0<=a['age']<=100 and a['sensor_error']==0 and a['magnet']&0x28==0x20 for a in axes)

def stale_joints(s):
    """The joints failing healthy(), with why, e.g. ['head (age 250 ms)']."""
    axes=s.get('axes',[])
    if len(axes)!=3:return ['all (no axis data)']
    out=[]
    for j,name in enumerate(JOINTS):
        a=axes[MOTOR[j]];why=[]
        if a['angle']==-2147483648:why.append('no angle')
        if not 0<=a['age']<=100:why.append(f"age {a['age']} ms")
        if a['sensor_error']!=0:why.append(f"sensor error {a['sensor_error']}")
        if a['magnet']&0x28!=0x20:why.append(f"magnet 0x{a['magnet']:02x}")
        if why:out.append(f"{name} ({', '.join(why)})")
    return out

def angles(s):return [s['axes'][a]['angle']/1000 for a in MOTOR]

def observe():
    with Transport() as t:return t.request('OBSERVE')

class ArmSession:
    """Exclusive, finite operations. Use a with block; no automatic retry/re-arm.

    Every jog/trial/pose writes a recording. The MCU lease expires at 300 ms;
    HOLD is sent only from the active bounded loop in the calling thread.
    """
    def __init__(self,calibration=DEFAULT_CALIBRATION,ratios=None,output=None,*,transport_factory=Transport,clock=time.monotonic,sleep=time.sleep):
        self.calibration,self.fingerprint=load_calibration(calibration)
        self.ratios=ratio_values(ratios,self.fingerprint)
        self.output=Path(output) if output else ROOT/'recordings'/('script-'+time.strftime('%Y%m%d-%H%M%S')+'-'+str(time.time_ns())[-6:]+'.jsonl')
        self.factory=transport_factory;self.clock=clock;self.sleep=sleep;self.transport=None;self.file=None;self.owned=False;self.motion_id=0;self.sample=None;self.started=0
    def __enter__(self):
        self.output.parent.mkdir(parents=True,exist_ok=True)
        self.file=self.output.open('x');self.started=self.clock()
        self.started_at=datetime.now(timezone.utc).isoformat(timespec='microseconds').replace('+00:00','Z')
        self.session_id=str(uuid.uuid4())
        try:
            self._event('metadata',started_at=self.started_at,session_id=self.session_id,fingerprint=self.fingerprint,ratios=self.ratios,text='Python bounded commissioning; motor-order arrays; millidegrees; counts are issued edges; no torque measurement')
            self.transport=self.factory()
            self._request('HELLO');self.owned=True
            for j,c in enumerate(self.calibration['saved_calibration']['joints']):
                # Saved zero is in [-180,180); round like Swift for this calibration.
                zero=int(math.copysign(math.floor(abs(c['zero']*1e6)+.5),c['zero']))
                values=[MOTOR[j],zero,int(c['reversed']),-1 if c['reversed'] else 1,round(self.ratios[j]*1000),int(self.ratios[j]>0)]
                s=self._request('CONFIG '+' '.join(map(str,values)))
                if s.get('configuration_echo')!=values:raise ArmError('Configuration not acknowledged')
            return self
        except BaseException:
            self.close();raise
    def __exit__(self,*_):self.close()
    def _event(self,kind,**values):
        if self.file:
            self.file.write(json.dumps(dict(type=kind,version=1,time=self.clock()-self.started,**values),allow_nan=False)+'\n');self.file.flush()
    def _request(self,command,allow=()):
        self._event('command',text=command)
        s=self.transport.request(command)
        if s is not None:
            self.sample=s;self._event('sample',sample=s)
            if s.get('error',0) and s['error'] not in allow:raise ArmError(f"Firmware rejected command: {s['error']}")
        return s
    def status(self):return self._request('STATUS')
    def stop(self):return self._request('STOP')
    def power(self,enabled):
        if type(enabled) is not bool:raise ValueError('enabled must be bool')
        s=self.stop();return self._request(f"POWER {int(enabled)} {s['generation']}")
    def close(self):
        try:
            if self.transport and self.owned:
                try:self._request('SHUTDOWN')
                except Exception:pass
        finally:
            if self.transport:self.transport.close()
            self.owned=False
            if self.file:self.file.close();self.file=None
    def _ready(self):
        s=self.status()
        if not healthy(s) or s['configured']!=7:raise ArmError('Fresh calibrated feedback required')
        if s['run'] or not s['enabled'] or s['awake']:raise ArmError('Arm must be stopped, asleep, and movement enabled')
        return s
    def record(self,seconds):
        seconds=finite(seconds,.1,3600,'Recording seconds');deadline=self.clock()+seconds
        while self.clock()<deadline:self.status();self.sleep(min(.05,max(0,deadline-self.clock())))
        return self.sample
    def jog(self,joint,direction,seconds=.5,max_degrees=2,rate_hz=100):
        seconds=finite(seconds,.1,60,'Jog seconds');max_degrees=finite(max_degrees,.1,10,'Maximum travel')
        return self._jog(joint,direction,seconds,max_degrees,False,rate_hz)
    def trial(self,joint,direction,degrees=6,timeout=30,rate_hz=100,powered_settle=False):
        if type(powered_settle) is not bool:raise ValueError('powered_settle must be bool')
        degrees=finite(degrees,5,10,'Trial degrees');timeout=finite(timeout,.1,60,'Trial timeout')
        return self._jog(joint,direction,timeout,degrees,True,rate_hz,powered_settle)
    def _jog(self,joint,direction,seconds,travel,trial,rate_hz,powered_settle=False):
        rate_hz=finite(rate_hz,20,2400,'Pulse rate')
        if int(rate_hz)!=rate_hz:raise ValueError('Pulse rate must be an integer')
        if joint not in JOINTS or direction not in ('forward','reverse'):raise ValueError('Use a named joint and forward/reverse')
        j=JOINTS.index(joint);d=0 if direction=='forward' else 1;s=self._ready();initial=angles(s)
        forward=-1 if self.calibration['saved_calibration']['joints'][j]['reversed'] else 1
        sign=forward*(1 if d==0 else -1)
        lo,hi=LIMITS[j]
        if (initial[j]<=lo and sign<0) or (initial[j]>=hi and sign>0):raise ArmError('Jog would move outward at a joint limit')
        self.motion_id+=1;command=f"JOG {self.motion_id} {MOTOR[j]} {d} {int(rate_hz)} {'400 ' if powered_settle else ''}{s['generation']}"
        result=self._run(command,initial,seconds,travel,j,sign,powered_settle)
        delta=(result['powered_final_angles'][j]-result['powered_initial_angles'][j]) if powered_settle else result['final_angles'][j]-initial[j];count=result['sample']['axes'][MOTOR[j]]['count']
        result.update(joint=joint,direction=d,pulses=count,degrees=delta,
                      pulses_per_degree=count/abs(delta) if abs(delta)>=.176 else None,
                      ratio_candidate=trial and count>=50 and 5<=abs(delta)<=13 and sign*delta>0 and result['sample']['state'] in (5,11))
        self._event('result',text=json.dumps({k:v for k,v in result.items() if k!='sample'}))
        return result
    def move_pose(self,targets,timeout=120):
        if len(targets)!=3:raise ValueError('Pose requires shoulder, elbow, head')
        targets=[finite(v,*LIMITS[j],name='Target angle') for j,v in enumerate(targets)]
        timeout=finite(timeout,1,123,'Pose timeout');s=self._ready();initial=angles(s)
        if not all(self.ratios) or s['verified']!=7:raise ArmError('All ratios must be verified first')
        if any(not LIMITS[j][0]<=initial[j]<=LIMITS[j][1] or not LIMITS[j][0]+2<=targets[j]<=LIMITS[j][1]-2 or abs(targets[j]-initial[j])>5 for j in range(3)):
            raise ArmError('Pose exceeds 5 degree commissioning cap or 2 degree limit margin')
        self.motion_id+=1;values=[round(targets[j]*1000) for j in MOTOR]
        result=self._run(f"POSE {self.motion_id} {' '.join(map(str,values))} {s['generation']}",initial,timeout,6,None,0)
        if result['sample']['state']!=4:raise ArmError('Pose did not complete before its deadline')
        return result
    def _run(self,command,initial,seconds,travel,joint,sign,powered_settle=False):
        began=self.clock();reason='deadline';failure=None;powered_initial=None;powered_final=None;preparation_samples=[]
        progress_at=began;progress_angle=initial[joint] if joint is not None else 0
        try:
            s=self._request(command)
            while True:
                elapsed=self.clock()-began
                if not healthy(s):raise ArmError('Sensor feedback unavailable')
                if s['id']!=self.motion_id:raise ArmError('Motion ID mismatch')
                current=angles(s)
                if powered_settle and s['state']==1 and s['awake'] and s['axes'][MOTOR[joint]]['count']==0:
                    if s['elapsed']>=200:
                        preparation_samples.append(current[:]);powered_initial=current[:]
                if joint is not None:
                    delta=current[joint]-initial[joint]
                    if delta*sign<-.3:raise ArmError('Measured movement is opposite the requested direction')
                    if any(abs(current[j]-initial[j])>.75 for j in range(3) if j!=joint):raise ArmError('An uncommanded joint moved')
                    if abs(current[joint]-progress_angle)>=.176:
                        progress_angle=current[joint];progress_at=self.clock()
                    elif s['run'] and self.clock()-progress_at>=2:raise ArmError('No measurable response for 2 seconds; release and investigate')
                if s['run']==0:
                    if s['state'] not in (4,5,11):raise ArmError(f"Motion fault state {s['state']}")
                    reason='firmware stop';break
                if joint is not None and abs(current[joint]-initial[joint])>=travel:reason='travel reached';break
                if elapsed>=seconds:break
                self.sleep(min(.025,seconds-elapsed))
                if self.clock()-began>=seconds:break
                s=self._request(f'HOLD {self.motion_id}')
            if powered_settle:
                if powered_initial is None or len(preparation_samples)<3 or any(max(v[j] for v in preparation_samples)-min(v[j] for v in preparation_samples)>.176001 for j in range(3)):
                    raise ArmError('Missing stable powered zero-count baseline')
                powered_initial=[statistics.median(v[j] for v in preparation_samples) for j in range(3)]
                s=self._request(f'PAUSE {self.motion_id}')
                paused=self.clock();counts=[a['count'] for a in s['axes']];tail=[]
                while self.clock()-paused<.35:
                    if not healthy(s) or s['state']!=12 or not s['run'] or not s['awake'] or [a['count'] for a in s['axes']]!=counts:
                        raise ArmError('Powered settling ended early or pulse count changed')
                    if any(abs(angles(s)[j]-initial[j])>.75 for j in range(3) if j!=joint):raise ArmError('An uncommanded joint moved during settling')
                    if self.clock()-paused>=.2:tail.append(angles(s))
                    self.sleep(.025);s=self._request(f'HOLD {self.motion_id}')
                if len(tail)<2 or any(max(v[j] for v in tail)-min(v[j] for v in tail)>.176001 for j in range(3)):
                    raise ArmError('Powered endpoint did not settle')
                powered_final=[statistics.median(v[j] for v in tail) for j in range(3)]
        except BaseException as exc:
            failure=exc
        finally:
            try:s=self.stop()
            except BaseException:
                self.close()
                if failure is None:raise
        if failure is not None:raise failure
        # Read settled feedback; never send HOLD during this interval.
        deadline=self.clock()+.5;settled=[]
        while self.clock()<deadline:
            self.sleep(.05);s=self.status()
            if s['run'] or s['awake']:raise ArmError('Stop did not leave drivers asleep')
            if healthy(s):settled.append(angles(s))
        if not settled:raise ArmError('No settled feedback after stop')
        if any(max(v[j] for v in settled[-5:])-min(v[j] for v in settled[-5:])>.176001 for j in range(3)):
            raise ArmError('Joint feedback did not settle after stop')
        final=[statistics.median(v[j] for v in settled[-5:]) for j in range(3)]
        if joint is not None and (sign*(final[joint]-initial[joint])<-.3 or any(abs(final[j]-initial[j])>.75 for j in range(3) if j!=joint)):
            raise ArmError('Unexpected movement in settled feedback')
        return {'reason':reason,'initial_angles':initial,'final_angles':final,'powered_initial_angles':powered_initial,'powered_final_angles':powered_final,'sleep_delta_deg':[final[j]-powered_final[j] for j in range(3)] if powered_final else None,'sample':s,'recording':str(self.output)}
