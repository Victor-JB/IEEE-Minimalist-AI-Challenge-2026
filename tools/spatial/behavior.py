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
        self.wave_at=-1e9
        self.last_interaction=0.;self.group_count=0;self.count_candidate=0;self.count_since=0.
        self.adjust=None; self.point_x=.5; self.wrist=None; self.target=None; self.aim_pose=None
        self.point_preview=None;self.target_source=None;self.target_mode=None

    def emit(self,event,now):
        self.event=event; self.serial+=1; self.event_at=now

    def transition(self,state,now):
        if self.state!=state:self.state=state;self.entered=now

    def stop(self,now):
        self.transition('STOPPED',now);self.emit('Stopped · lamp off',now)
        self.gesture=None;self.frames=0
        self.point_preview=None

    def resume(self,now):
        self.transition('IDLE',now);self.emit('Ready for gestures',now)
        self.gesture=None;self.frames=0;self.previous=now

    def update(self,now,faces,hands,width=640,target=None):
        hand=next((h for h in hands if h.get('gesture')=='open_palm'),hands[0] if hands else None)
        gesture=hand.get('gesture') if hand else None
        self.point_preview=target if gesture=='point' and self.state!='STOPPED' else None
        wrist=hand['points'][0][:2] if hand else None
        size=math.dist(hand['points'][0][:2],hand['points'][9][:2]) if hand else 0
        changed_hand=wrist is not None and self.wrist is not None and math.dist(wrist,self.wrist)>max(80,2*size)
        gap=now-self.previous>.9
        if gesture!=self.gesture or gap or changed_hand:
            self.gesture=gesture;self.since=now;self.frames=0;self.fired=-1e9
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
        hold=.4
        repeat=gesture in ('thumbs_up','thumbs_down','point')
        greeting_ready=gesture!='open_palm' or now-self.wave_at>=3
        if self.state!='STOPPED' and greeting_ready and gesture and self.frames>=2 and now-self.since>=hold and (self.fired<self.since or repeat and now-self.fired>=1):
            self.fired=now;self.last_interaction=now
            if gesture=='open_palm':
                self.wave_at=now;self.emit('Open palm · hello back',now)
            elif self.state!='STOPPED' and gesture=='point':
                self.point_x=max(0,min(1,hand['points'][8][0]/width))
                if target:
                    changed=self.target is None or math.dist(self.target,target['point'])>.03 or self.state not in ('ACK','ILLUMINATE')
                    if changed:
                        self.target=target['point'];self.aim_pose=target['pose']
                        self.target_source=target.get('source','Estimated desk target');self.target_mode=target.get('mode','finger_ray')
                        if self.state!='ILLUMINATE':self.transition('ACK',now)
                        kind='virtual desk spot' if self.target_mode=='virtual_pointer' else 'estimated desk spot'
                        self.emit('Point accepted · '+kind+' selected',now)
                else:self.emit('Point unavailable · keep fingertip and palm in view',now)
            elif self.state=='ILLUMINATE' and repeat:
                self.level=max(.1,min(.8,self.level+(.1 if gesture=='thumbs_up' else -.1)))
                if self.adjust and self.aim_pose and self.target:
                    self.aim_pose=self.adjust(self.aim_pose,self.target,.03 if gesture=='thumbs_up' else -.03)
                self.emit('Brighter / closer' if gesture=='thumbs_up' else 'Dimmer / further',now)
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
        elif self.state=='ACK':pose=list(self.aim_pose or pose)
        elif self.state=='ILLUMINATE':
            amount=(self.level-.4)*15
            pose=list(self.aim_pose or [yaw,-18,65,70,0])
        elif self.state=='STOPPED':pose=[0,-12,63,5,0]
        return dict(state=self.state,people_count=self.group_count,interaction_idle_s=max(0,now-self.last_interaction),event=self.event,event_id=self.serial,event_age_s=now-self.event_at,
            gesture=self.gesture,hold_progress=min(1,max(0,(now-self.since)/.4)) if self.gesture else 0,
            brightness=self.level,light_requested=self.state=='ILLUMINATE',
            pose=pose,desk_target=target if self.state in ('ACK','ILLUMINATE') else None,
            point_preview=self.point_preview,target_mode=self.target_mode,target_source=self.target_source)
