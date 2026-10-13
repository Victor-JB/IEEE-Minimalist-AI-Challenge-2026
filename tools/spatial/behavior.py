"""Hybrid demo behavior. Failed arm joints are represented only by simulated poses.

This is the desk-target demo, not the app's unimplemented book-search pipeline.
All inputs are fresh, actual inference results. No UI example can generate events.
"""
import math


class GestureHold:
    """One gesture channel, stable across changes in detector result order."""
    def __init__(self):
        self.gesture=None;self.since=0.;self.frames=0;self.previous=-1e9
        self.fired=-1e9;self.wrist=None;self.seen=-1e9

    def select(self,now,hands):
        if not hands:return None
        if self.wrist is not None and now-self.seen<=.9:
            return min(hands,key=lambda h:math.dist(h['points'][0][:2],self.wrist))
        return max(hands,key=lambda h:h.get('score',0))

    def update(self,now,hand):
        gesture=hand.get('gesture') if hand else None
        wrist=hand['points'][0][:2] if hand else None
        size=math.dist(hand['points'][0][:2],hand['points'][9][:2]) if hand else 0
        changed=wrist is not None and self.wrist is not None and math.dist(wrist,self.wrist)>max(80,2*size)
        if gesture!=self.gesture or now-self.previous>.9 or changed:
            self.gesture=gesture;self.since=now;self.frames=0;self.fired=-1e9
        self.previous=now;self.frames+=1
        if wrist is not None:self.wrist=wrist;self.seen=now
        return bool(gesture and self.frames>=2 and now-self.since>=.4)

    def progress(self,now):
        return min(1,max(0,(now-self.since)/.4)) if self.gesture else 0


class Demo:
    def __init__(self,aim):
        self.state='IDLE';self.entered=0.;self.last_seen=0.;self.level=.4
        self.pointer=GestureHold();self.command=GestureHold()
        self.event='Waiting for someone';self.serial=0;self.event_at=0.
        self.wave_at=-1e9
        self.last_interaction=0.;self.group_count=0;self.count_candidate=0;self.count_since=0.
        self.aim=aim;self.distance_offset=0.
        self.point_x=.5;self.target=None;self.aim_pose=None
        self.point_preview=None;self.target_source=None;self.target_mode=None
        self.point_following=False;self.point_at=-1e9

    def emit(self,event,now):
        self.event=event;self.serial+=1;self.event_at=now

    def transition(self,state,now):
        if self.state!=state:self.state=state;self.entered=now

    def stop(self,now):
        self.transition('STOPPED',now);self.emit('Stopped · lamp off',now)
        self.pointer=GestureHold();self.command=GestureHold()
        self.point_preview=None;self.point_following=False

    def resume(self,now):
        self.transition('IDLE',now);self.emit('Ready for gestures',now)
        self.pointer=GestureHold();self.command=GestureHold();self.point_following=False

    def select_pointing_hand(self,now,hands):
        return self.pointer.select(now,[h for h in hands if h.get('gesture')=='point'])

    def update(self,now,faces,hands,width=640,target=None):
        hand=self.select_pointing_hand(now,hands)
        point_ready=self.pointer.update(now,hand)
        commands=[h for h in hands if h.get('gesture') in ('open_palm','thumbs_up','thumbs_down')]
        palms=[h for h in commands if h['gesture']=='open_palm']
        # Two opposite thumbs cancel rather than alternately pulling the lamp.
        conflict={h['gesture'] for h in commands}>={'thumbs_up','thumbs_down'}
        command_hand=self.command.select(now,palms or ([] if conflict else commands))
        command_ready=self.command.update(now,command_hand)
        self.point_preview=target if hand and self.state!='STOPPED' else None
        self.point_following=False
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
        # Acquire once, then follow EVERY valid observation. Only the first selection
        # acknowledges; dragging must not restart a nod or wait for a repeat timer.
        if self.state!='STOPPED' and point_ready:
            if target:
                first=self.target is None or self.state not in ('ACK','ILLUMINATE')
                alpha=1 if first else -math.expm1(-min(max(now-self.point_at,0),.5)/.16)
                point=list(target['point']) if first else [a+(b-a)*alpha for a,b in zip(self.target,target['point'])]
                pose=self.aim(point,self.distance_offset)
                if pose is not None:
                    self.point_x=max(0,min(1,hand['points'][8][0]/width))
                    self.target=point;self.aim_pose=pose;self.point_at=now;self.point_following=True
                    self.target_source=target.get('source','Estimated desk target');self.target_mode=target.get('mode','finger_ray')
                    self.last_interaction=now
                    if first:
                        self.transition('ACK',now)
                        kind='virtual desk spot' if self.target_mode=='virtual_pointer' else 'estimated desk spot'
                        self.emit('Point accepted · '+kind+' selected',now)
            elif self.pointer.fired<self.pointer.since:
                self.pointer.fired=now;self.emit('Point unavailable · keep fingertip and palm in view',now)
        gesture=self.command.gesture
        repeat=gesture in ('thumbs_up','thumbs_down')
        greeting_ready=gesture!='open_palm' or now-self.wave_at>=3
        if self.state!='STOPPED' and command_ready and greeting_ready and (self.command.fired<self.command.since or repeat and now-self.command.fired>=1):
            self.command.fired=now;self.last_interaction=now
            if gesture=='open_palm':
                self.wave_at=now;self.emit('Open palm · hello back',now)
            elif self.state=='ILLUMINATE' and repeat and self.target:
                # Up = farther, down = closer. Keep lamp power fixed; distance
                # changes the illuminated footprint naturally in the renderer.
                offset=max(-.24,min(.24,self.distance_offset+(.03 if gesture=='thumbs_down' else -.03)))
                pose=self.aim(self.target,offset)
                if pose is not None:self.distance_offset=offset;self.aim_pose=pose
                self.emit('Close · moving toward your spot' if gesture=='thumbs_down' else 'Far · moving away from your spot',now)
        return self.snapshot(now)

    def snapshot(self,now):
        t=now-self.entered;wave=max(0,now-self.wave_at)
        target=self.target
        yaw=max(-20,min(20,(self.point_x-.5)*35)) if self.state in ('ACK','ILLUMINATE') else 0
        pose=[yaw,-12,63,5,0]
        if self.state=='IDLE':pose=[0,-20+2*math.sin(t*.3),65,40,0]
        elif self.state=='ATTENTIVE':
            pose[3]+=2*math.sin(t)
            if wave<2.5:pose[4]=12*math.sin(wave*6)
        elif self.state=='ACK':pose=list(self.aim_pose or pose)
        elif self.state=='ILLUMINATE':pose=list(self.aim_pose or [yaw,-18,65,70,0])
        elif self.state=='STOPPED':pose=[0,-12,63,5,0]
        active=self.command if self.command.gesture else self.pointer
        return dict(state=self.state,people_count=self.group_count,interaction_idle_s=max(0,now-self.last_interaction),event=self.event,event_id=self.serial,event_age_s=now-self.event_at,
            gesture=active.gesture,gestures=[h.gesture for h in (self.pointer,self.command) if h.gesture],hold_progress=active.progress(now),
            brightness=self.level,light_requested=self.state=='ILLUMINATE',
            pose=pose,desk_target=target if self.state in ('ACK','ILLUMINATE') else None,
            point_following=self.point_following and now-self.point_at<=.9,
            point_preview=self.point_preview,target_mode=self.target_mode,target_source=self.target_source)
