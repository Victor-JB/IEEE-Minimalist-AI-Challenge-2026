"""Image-based head swivel tracking, independent of the digital behavior state."""
import math


class SubjectTracker:
    LIMIT=45.0
    HFOV=100.0
    SPEED=60.0
    DEADZONE=.10       # hold within the middle 20% of the image
    REENGAGE=.14       # wider threshold avoids chatter at the boundary
    def __init__(self):
        self.target=0.;self.last_seen=-1e9;self.previous=None;self.center=None;self.kind=None
        self.status='Waiting for a subject';self.error=0.;self.moving=False

    def update(self,now,faces,hands,pivot,width=640):
        dt=min(.5,max(0.,now-self.previous)) if self.previous is not None else .25
        self.previous=now
        current=pivot.get('current') if pivot.get('enabled') and not pivot.get('fault') else None
        if not isinstance(current,(int,float)) or not math.isfinite(current):current=self.target
        candidates=[((f.x+f.w/2)/width,f.w*f.h) for f in faces]
        kind='face'
        if not candidates and not (self.kind=='face' and now-self.last_seen<1):
            # Palm centre is steadier than the fingertip used for pointing.
            candidates=[(sum(h['points'][i][0] for i in (0,5,9,13,17))/(5*width),h.get('score',0)) for h in hands]
            kind='hand'
        candidates=[p for p in candidates if 0<=p[0]<=1]
        if not candidates:
            self.status='Subject lost · holding swivel' if self.kind else 'Waiting for a subject'
            self.error=0.;self.moving=False;self.target=max(-self.LIMIT,min(self.LIMIT,current))
        else:
            # Keep the nearest image-space subject instead of hopping to the largest face.
            x=(min(candidates,key=lambda c:abs(c[0]-self.center)) if self.center is not None and kind==self.kind and now-self.last_seen<1.5 else max(candidates,key=lambda c:c[1]))[0]
            self.center=x;self.kind=kind;self.last_seen=now
            self.error=math.degrees(math.atan((x-.5)*2*math.tan(math.radians(self.HFOV/2))))
            # Hysteresis: once centered, wait for a wider departure before moving.
            offset=abs(x-.5)
            if offset<=self.DEADZONE:self.moving=False
            elif offset>=self.REENGAGE:self.moving=True
            if self.moving:
                edge=math.degrees(math.atan(self.DEADZONE*2*math.tan(math.radians(self.HFOV/2))))
                correction=math.copysign(max(0,abs(self.error)-edge)*.9,self.error)
                desired=max(-self.LIMIT,min(self.LIMIT,current+correction))
                step=self.SPEED*dt
                self.target=max(current-step,min(current+step,desired))
                self.status=('At swivel limit · subject outside centre' if abs(self.target)>=self.LIMIT-.1 else 'Tracking '+kind)
            else:
                # Cancel any outstanding slew at the reported current command.
                self.target=max(-self.LIMIT,min(self.LIMIT,current))
                self.status='Subject centered · holding swivel'
        return dict(target=self.target,status=self.status,subject=self.kind,
                    visible=bool(candidates),error_deg=self.error,limit_deg=self.LIMIT,hfov=self.HFOV,
                    speed_deg_s=self.SPEED,deadzone_fraction=self.DEADZONE*2,reengage_fraction=self.REENGAGE*2)
