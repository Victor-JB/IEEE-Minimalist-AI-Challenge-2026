"""Damped image-based swivel tracking, independent of digital behavior.

Large hysteresis, filtered derivative braking and small bounded corrections avoid
chasing detector noise/delayed camera frames. No integral term can accumulate at
±45°. This uses the servo's command estimate, not measured servo position.
"""
import math


class SubjectTracker:
    LIMIT=45.0
    HFOV=100.0
    SPEED=60.0          # firmware ceiling; controller normally requests smaller steps
    DEADZONE=.20       # hold within the middle 40% of the image
    REENGAGE=.26       # restart outside the middle 52%, after two consistent observations
    KP=1.8             # proportional velocity per degree outside the hold zone
    KD=.35             # brake when the visual error is closing
    MAX_STEP=6.0        # limit motion before the next camera observation
    FILTER_TAU=.35

    def __init__(self):
        self.target=0.;self.last_seen=-1e9;self.previous=None;self.center=None;self.kind=None
        self.status='Waiting for a subject';self.error=0.;self.moving=False
        self.last_error=None;self.error_rate=0.;self.pending_sign=0;self.pending_frames=0

    def reset_motion(self):
        self.moving=False;self.last_error=None;self.error_rate=0.;self.pending_sign=0;self.pending_frames=0

    def update(self,now,faces,hands,pivot,width=640,observed_pivot=None):
        gap=self.previous is None or now-self.previous>.9
        dt=min(.5,max(.01,now-self.previous)) if self.previous is not None else .25
        self.previous=now
        if gap:self.reset_motion()
        current=pivot.get('current') if pivot.get('enabled') and not pivot.get('fault') else None
        if not isinstance(current,(int,float)) or not math.isfinite(current):current=self.target
        candidates=[((f.x+f.w/2)/width,f.w*f.h) for f in faces]
        kind='face'
        if not candidates and not (self.kind=='face' and now-self.last_seen<1):
            candidates=[(sum(h['points'][i][0] for i in (0,5,9,13,17))/(5*width),h.get('score',0)) for h in hands]
            kind='hand'
        candidates=[p for p in candidates if 0<=p[0]<=1]
        self.target=max(-self.LIMIT,min(self.LIMIT,current))
        if not candidates:
            self.status='Subject lost · holding swivel' if self.kind else 'Waiting for a subject'
            self.error=0.;self.reset_motion()
        else:
            same=kind==self.kind and now-self.last_seen<1.5
            nearest=min(candidates,key=lambda c:abs(c[0]-self.center)) if self.center is not None and same else None
            if kind=='face':
                # The largest visible head always wins, including when the old
                # subject remains visible. Association is only used to detect a
                # switch, never to keep following a smaller face.
                selected=max(candidates,key=lambda c:c[1])
                if nearest is not None and (selected is not nearest or abs(selected[0]-self.center)>.35):
                    same=False
            else:
                # Keep hand identity stable when detection order/score changes.
                selected=nearest if nearest is not None else max(candidates,key=lambda c:c[1])
            x=selected[0]
            if not same:self.reset_motion()
            self.center=x;self.kind=kind;self.last_seen=now
            error=math.degrees(math.atan((x-.5)*2*math.tan(math.radians(self.HFOV/2))))
            # Detection finishes hundreds of milliseconds after this frame's telemetry.
            # Remove the swivel movement already made while inference was running.
            observed=(observed_pivot or {}).get('current')
            if (observed_pivot or {}).get('enabled') and not (observed_pivot or {}).get('fault') and isinstance(observed,(int,float)) and math.isfinite(observed):
                error-=current-observed
            self.error=error
            rate=0 if self.last_error is None else max(-120,min(120,(error-self.last_error)/dt))
            alpha=dt/(self.FILTER_TAU+dt)
            self.error_rate+=alpha*(rate-self.error_rate);self.last_error=error
            edge=math.degrees(math.atan(self.DEADZONE*2*math.tan(math.radians(self.HFOV/2))))
            restart=math.degrees(math.atan(self.REENGAGE*2*math.tan(math.radians(self.HFOV/2))))
            sign=1 if error>0 else -1
            if abs(error)<=edge:
                self.reset_motion()
                self.status='Subject in hold zone · swivel steady'
            else:
                # Reject one-frame excursions and require confirmation before reversing.
                reversing=self.moving and self.pending_sign!=sign
                if reversing:self.reset_motion()
                if not self.moving and abs(error)>=restart:
                    self.pending_frames=self.pending_frames+1 if self.pending_sign==sign else 1
                    self.pending_sign=sign
                    if self.pending_frames>=2:self.moving=True
                elif not self.moving:self.pending_frames=0
                if self.moving:
                    excess=max(0,abs(error)-edge)
                    # D only reduces motion; it must never reverse the servo on noisy frames.
                    closing=max(0,-sign*self.error_rate)
                    velocity=max(0,min(self.SPEED,self.KP*excess-self.KD*closing))
                    step=min(self.MAX_STEP,velocity*dt,.45*excess)
                    self.target=max(-self.LIMIT,min(self.LIMIT,current+sign*step))
                    self.status=('At swivel limit · holding' if abs(self.target)>=self.LIMIT-.1 else 'Tracking '+kind+' · damped')
                else:self.status='Holding swivel · confirming subject movement'
        return dict(target=self.target,status=self.status,subject=self.kind,
                    visible=bool(candidates),error_deg=self.error,limit_deg=self.LIMIT,hfov=self.HFOV,
                    speed_deg_s=self.SPEED,deadzone_fraction=self.DEADZONE*2,reengage_fraction=self.REENGAGE*2,
                    controller='PD with hysteresis · no integral',max_step_deg=self.MAX_STEP)
