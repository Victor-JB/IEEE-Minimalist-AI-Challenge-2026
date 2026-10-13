"""Hybrid demo behavior. Failed arm joints are represented only by simulated poses.

This is the desk-target demo, not the app's unimplemented book-search pipeline.
All inputs are fresh, actual inference results. No UI example can generate events.
"""
import math


class Demo:
    def __init__(self):
        self.state='IDLE'; self.entered=0.; self.last_seen=0.; self.level=.4
        self.gesture=None; self.since=0.; self.fired=-1e9; self.frames=0; self.previous=-1e9
        self.event='Waiting for someone'; self.serial=0; self.event_at=0.
        self.wave_ext=None; self.wave_dir=0; self.wave_turns=[]; self.wave_at=-1e9
        self.last_interaction=0.;self.group_count=0;self.count_candidate=0;self.count_since=0.;self.palm_anchor=None
        self.adjust=None; self.point_x=.5; self.wrist=None; self.target=None; self.aim_pose=None

    def emit(self,event,now):
        self.event=event; self.serial+=1; self.event_at=now

    def transition(self,state,now):
        if self.state!=state:self.state=state;self.entered=now

    def stop(self,now):
        self.transition('STOPPED',now);self.emit('Stopped · lamp off',now)
        self.gesture=None;self.frames=0;self.wave_ext=None;self.wave_turns=[]

    def resume(self,now):
        self.transition('IDLE',now);self.emit('Ready for gestures',now)
        self.gesture=None;self.frames=0;self.previous=now

    def update(self,now,faces,hands,width=640,target=None):
        hand=next((h for h in hands if h.get('gesture')=='open_palm'),hands[0] if hands else None)
        gesture=hand.get('gesture') if hand else None
        wrist=hand['points'][0][:2] if hand else None
        size=math.dist(hand['points'][0][:2],hand['points'][9][:2]) if hand else 0
        changed_hand=wrist is not None and self.wrist is not None and math.dist(wrist,self.wrist)>max(80,2*size)
        gap=now-self.previous>.9
        if gesture!=self.gesture or gap or changed_hand:
            self.gesture=gesture;self.since=now;self.frames=0;self.fired=-1e9;self.palm_anchor=wrist
        if gesture=='open_palm' and self.palm_anchor is not None and math.dist(wrist,self.palm_anchor)>max(15,size*.25):
            self.since=now;self.frames=0;self.palm_anchor=wrist
        self.previous=now;self.frames+=1;self.wrist=wrist
        if faces or hands:self.last_seen=now
        if self.state!='STOPPED':
            if self.state=='IDLE' and (faces or hands):
                self.transition('ATTENTIVE',now);self.emit('Hello · ready to help',now);self.group_count=len(faces);self.last_interaction=now
            elif self.state=='ACK' and now-self.entered>=.8:self.transition('ILLUMINATE',now)
            elif self.state=='ILLUMINATE' and now-self.last_seen>10:
                self.transition('IDLE',now);self.emit('Nobody here · lamp off',now)
            elif self.state=='ATTENTIVE' and now-self.last_seen>10:
                self.transition('IDLE',now);self.group_count=0;self.emit('Quiet room · idle',now)
            count=len(faces)
            if count!=self.count_candidate:self.count_candidate=count;self.count_since=now
            if self.state=='ATTENTIVE' and now-self.count_since>=.8:
                if count>self.group_count:
                    self.emit('More company · hello everyone',now);self.last_interaction=now
                self.group_count=count
        hold=1.0 if gesture=='open_palm' else .4
        repeat=gesture in ('thumbs_up','thumbs_down') or gesture=='point' and self.state in ('IDLE','ATTENTIVE')
        if gesture and self.frames>=2 and now-self.since>=hold and (self.fired<self.since or repeat and now-self.fired>=1):
            self.fired=now;self.last_interaction=now
            if gesture=='open_palm': self.stop(now)
            elif self.state!='STOPPED' and gesture=='point':
                self.point_x=max(0,min(1,hand['points'][8][0]/width))
                if target:
                    self.target=target['point'];self.aim_pose=target['pose']
                    self.transition('ACK',now);self.emit('Point accepted · estimated desk spot selected',now)
                else:self.emit('Point downward at a reachable desk spot',now)
            elif self.state=='ILLUMINATE' and repeat:
                self.level=max(.1,min(.8,self.level+(.1 if gesture=='thumbs_up' else -.1)))
                if self.adjust and self.aim_pose and self.target:
                    self.aim_pose=self.adjust(self.aim_pose,self.target,.03 if gesture=='thumbs_up' else -.03)
                self.emit('Brighter / closer' if gesture=='thumbs_up' else 'Dimmer / further',now)
        # A wave is separate from the static open-palm stop; a held palm always wins.
        if not hand or gap or changed_hand:
            self.wave_ext=wrist[0] if wrist else None;self.wave_dir=0;self.wave_turns=[]
        elif self.state=='ATTENTIVE' and size>10:
            x=wrist[0]; self.wave_turns=[t for t in self.wave_turns if now-t<=2.5]
            if self.wave_ext is None:self.wave_ext=x
            elif self.wave_dir==0:
                if abs(x-self.wave_ext)>.5*size:self.wave_dir=1 if x>self.wave_ext else -1;self.wave_ext=x
            elif (x-self.wave_ext)*self.wave_dir>0:self.wave_ext=x
            elif (self.wave_ext-x)*self.wave_dir>.5*size:
                self.wave_turns.append(now);self.wave_dir*=-1;self.wave_ext=x
                if len(self.wave_turns)>=2 and now-self.wave_at>3:
                    self.wave_at=now;self.wave_turns=[];self.last_interaction=now;self.emit('Wave · hello back',now)
        return self.snapshot(now)

    def snapshot(self,now):
        t=now-self.entered; wave=max(0,now-self.wave_at)
        # A displayed desk zone is an intent cue, not an inferred book or precise pointing ray.
        target=self.target
        yaw=max(-20,min(20,(self.point_x-.5)*35)) if self.state in ('ACK','ILLUMINATE') else 0
        pose=[yaw,-12,63,5,0]
        if self.state=='IDLE':pose=[0,-20+2*math.sin(t*.3),65,40,0]
        elif self.state=='ATTENTIVE':
            pose[3]+=2*math.sin(t)
            if wave<2.5:pose[4]=12*math.sin(wave*6)
        elif self.state=='ACK':pose[3]+=12*math.sin(min(t/.8,1)*math.pi)
        elif self.state=='ILLUMINATE':
            amount=(self.level-.4)*15
            pose=list(self.aim_pose or [yaw,-18,65,70,0])
        elif self.state=='STOPPED':pose=[0,-12,63,5,0]
        return dict(state=self.state,people_count=self.group_count,interaction_idle_s=max(0,now-self.last_interaction),event=self.event,event_id=self.serial,event_age_s=now-self.event_at,
            gesture=self.gesture,hold_progress=min(1,max(0,(now-self.since)/(1 if self.gesture=='open_palm' else .4))) if self.gesture else 0,
            brightness=self.level,light_requested=self.state=='ILLUMINATE',
            pose=pose,desk_target=target if self.state in ('ACK','ILLUMINATE') else None,
            target_source='estimated finger ray on desk · 60° FOV / 8 cm palm; not a detected book')
