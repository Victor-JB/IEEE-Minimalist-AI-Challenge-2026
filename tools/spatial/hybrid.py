"""Subject-tracking swivel and a face/physical-orientation gate for D7. No stepper motion.

Browser heartbeat expires after 3 seconds; Stop and stale data release the servo.
"""
import math
import secrets
from tracking import SubjectTracker
from kinematics import shade_frame, SHADE_RADIUS, SHADE_DEPTH, LIGHT_OFFSET


class Outputs:
    ALLOWED={0,4,7,16,17,18}
    LIGHT_LEVEL=1.
    LIGHT_LEASE_MS=800
    # Aperture geometry, not the narrower artistic cone in the web renderer.
    # Assumes a level base and the modeled axial LED location inside the shade.
    BEAM_HALF_DEG=math.degrees(math.atan2(SHADE_RADIUS,SHADE_DEPTH-LIGHT_OFFSET))
    ON_MARGIN_DEG=5.
    OFF_MARGIN_DEG=2.
    def __init__(self,bridge,read_pose=None):
        self.bridge=bridge;self.owner=secrets.randbelow(2**30)+1
        self.read_pose=read_pose
        self.connected=False;self.claimed=False;self.requested=False;self.heartbeat=-1e9
        self.reason='Physical outputs disabled';self.level=0.;self.last_target=None
        self.clear_since=None;self.clear_frames=0;self.face_stamp=None
        self.beam_top_deg=None;self.axis_elevation_deg=None;self.lamp_reason='Physical outputs disabled'

    def command(self,op,*args):
        if op not in self.ALLOWED:raise ValueError('Stepper operations are forbidden in hybrid mode')
        raw=self.bridge.call('arm_command',[op,self.owner,*args],timeout=.2)
        if len(raw)!=38 or raw[0]!=1 or raw[1]:raise RuntimeError('Pivot control refused: '+str(raw[:4]))
        return raw

    def light(self,level,fade=0,lease=0):
        args=[round(level*1000),fade]
        if lease:args.append(lease)
        r=self.bridge.call('arm_light',args,timeout=.2)
        if len(r)!=7 or r[0]!=4 or not r[1] or r[5] or r[6]:raise RuntimeError('D7 lamp output unavailable')

    def clear_faces(self,data,now):
        guard=data.get('face_guard') or {};stamp=guard.get('frame_at')
        checked=guard.get('checked') is True and isinstance(stamp,(int,float)) and math.isfinite(stamp) and 0<=now-stamp<=1.
        detections=data.get('detections')
        if not checked or not isinstance(detections,list) or detections:
            self.clear_since=None;self.clear_frames=0;self.face_stamp=None
            return False,'Face in frame' if detections else 'Waiting for a fresh face check'
        if self.clear_since is None:self.clear_since=now
        if stamp!=self.face_stamp:self.clear_frames+=1;self.face_stamp=stamp
        ready=self.clear_frames>=3 and now-self.clear_since>=1.
        return ready,'No faces detected' if ready else 'Waiting for one clear second'

    @staticmethod
    def pitch_problem(pose):
        """Pitch encoders gate the beam, independently of the swivel servo."""
        joints=pose.get('joints',[])
        if not isinstance(joints,(list,tuple)) or len(joints)!=3:
            return 'Physical pitch readings unavailable · lamp off'
        for name,j in zip(('Shoulder','Elbow','Head'),joints):
            if not isinstance(j,dict):return name+' angle unavailable · lamp off'
            angle,age=j.get('angle'),j.get('age_ms')
            if not (j.get('healthy') and isinstance(angle,(int,float)) and math.isfinite(angle)
                    and isinstance(age,(int,float)) and math.isfinite(age) and 0<=age<=100):
                reason=j.get('reason')
                return (reason if isinstance(reason,str) and reason else name+' angle unavailable')+' · lamp off'
        return None

    def read_swivel(self):
        """Fresh servo telemetry even when a pitch-encoder RPC failed."""
        r=self.bridge.call('arm_roll_status',timeout=.2)
        if len(r)!=6 or r[0]!=3:raise ValueError('Head swivel telemetry unavailable')
        return dict(enabled=bool(r[1]),target=r[2]/1000,current=r[3]/1000,fault=r[5])

    @staticmethod
    def check_swivel(pose):
        p=pose.get('pivot') or {}
        if p.get('fault'):raise ValueError('Head swivel fault · outputs off')
        if any(not isinstance(p.get(k),(int,float)) or not math.isfinite(p[k])
               or abs(p[k])>SubjectTracker.LIMIT+.5 for k in ('current','target')):
            raise ValueError('Head swivel pose unavailable · outputs off')
        # Released is a valid starting state: op16 wakes the servo at its last
        # command and firmware still checks power.enabled and rollFault.

    def downward_cone(self,pose,target):
        self.beam_top_deg=None;self.axis_elevation_deg=None
        p=pose.get('pivot') or {}
        rolls=[p.get('current'),p.get('target'),target]
        if not p.get('enabled') or p.get('fault') or any(not isinstance(r,(int,float)) or not math.isfinite(r) or abs(r)>SubjectTracker.LIMIT+.5 for r in rolls):
            return False,'Head swivel pose unavailable'
        phi=sum(j['angle'] for j in pose['joints'])
        elevations=[math.degrees(math.asin(max(-1,min(1,shade_frame(phi,r)[0][2])))) for r in rolls]
        self.axis_elevation_deg=elevations[0]
        # For a downward axis within ±45°, the highest elevation along the servo
        # sweep is at an endpoint. Check the command in flight and the next target.
        self.beam_top_deg=max(elevations)+self.BEAM_HALF_DEG
        margin=self.OFF_MARGIN_DEG if self.level else self.ON_MARGIN_DEG
        permitted=self.beam_top_deg<=-margin
        return permitted,'Beam below horizon' if permitted else 'Tilt the whole beam farther downward'

    def disable(self,force_light=False):
        self.requested=False;self.level=0.;self.last_target=None
        self.clear_since=None;self.clear_frames=0;self.face_stamp=None
        self.beam_top_deg=None;self.axis_elevation_deg=None;self.lamp_reason='Physical outputs disabled'
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
            if not 0<=age<=1.5 or data.get('error') or demo.get('state')=='STOPPED':
                self.reason='Stopped or stale camera · outputs off';self.disable();return
            face_clear,face_reason=self.clear_faces(data,now)
            if self.connected and self.level and (not face_clear or self.pitch_problem(data)):
                self.light(0);self.level=0.
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
                # Older firmware rejects the optional lease. Refuse automatic
                # lighting instead of falling back to a persistent on command.
                self.light(0,lease=self.LIGHT_LEASE_MS)
                self.command(17,1500,7407,-45000,45000,-1,round(SubjectTracker.SPEED*1000))
            status=self.command(4)
            if status[6] or status[7]:raise RuntimeError('Unexpected stepper activity · hybrid stopped')
            try:
                pose=self.read_pose(self.bridge) if self.read_pose else data
                if not isinstance(pose,dict):raise ValueError('Invalid physical pose')
                pitch_reason=self.pitch_problem(pose)
            except Exception:
                # Loss of pitch telemetry cannot authorize light. Check the
                # independent servo status after switching D7 off, then permit
                # only swivel tracking if that fresh status is still valid.
                self.light(0);self.level=0.
                pose=dict(joints=[],pivot=self.read_swivel())
                pitch_reason='Physical pitch readings unavailable · lamp off'
            self.check_swivel(pose)
            tracking=data.get('tracking') or {}
            value=tracking.get('target')
            if not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError('Subject tracking unavailable')
            target=max(-SubjectTracker.LIMIT,min(SubjectTracker.LIMIT,value))
            if pitch_reason:
                self.beam_top_deg=None;self.axis_elevation_deg=None
                downward,direction_reason=False,pitch_reason
            else:
                downward,direction_reason=self.downward_cone(pose,target)
            level=self.LIGHT_LEVEL if face_clear and downward else 0.
            if not level:self.light(0);self.level=0.
            if self.last_target is None or abs(target-self.last_target)>.5:
                self.command(16,round(target*1000),round(SubjectTracker.SPEED*1000));self.last_target=target
            if level:self.light(level,lease=self.LIGHT_LEASE_MS)
            self.level=level
            self.lamp_reason=pitch_reason or ('No faces · whole beam below horizon' if level else face_reason if not face_clear else direction_reason)
            self.reason=tracking.get('status','Tracking subject')+' · '+self.lamp_reason+' · real lamp '+('on' if level else 'off')
        except Exception as exc:
            self.reason=str(exc)
            try:self.disable()
            except Exception:pass

    def snapshot(self):
        return dict(enabled=self.requested,connected=self.claimed,brightness=self.level,
                    reason=self.reason,swivel_mode='subject tracking',swivel_target=self.last_target,steppers='disabled · no motion commands',
                    lamp_gate=dict(reason=self.lamp_reason,beam_top_deg=self.beam_top_deg,axis_elevation_deg=self.axis_elevation_deg,
                                   half_angle_deg=self.BEAM_HALF_DEG,lease_ms=self.LIGHT_LEASE_MS,clear_frames=self.clear_frames))
