/* Full-model choreography + ELEGNT additive layers. Browser animation only. */
(function(root) {
'use strict';
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const smooth=t=>{const s=clamp(t,0,1);return s*s*s*(10+s*(-15+6*s));};
const mix=(a,b,t)=>a.map((v,i)=>v+(b[i]-v)*t);
const zero=()=>[0,0,0,0,0];
const labels={wake:'Oh, hello!',greet:'Hello back!',curious:'What are you thinking?',peek:'Can I help?',nod:'Got it',shake:'Try pointing a little lower',joy:'Happy to see you',look_around:'Hello, everyone',sleep:'Getting comfortable',stretch:'A little stretch',full_show:'Original full-model choreography'};

function trajectory(clip,t) {
 const q=[...clip.start];
 for(const s of clip.segments) {
  if(t<s.t)break;
  for(let j=0;j<5;j++) {
   const local=t-s.t-j*s.stagger;
   if(local>0)q[j]=s.start[j]+(s.end[j]-s.start[j])*smooth(local/s.duration);
  }
 }
 return q;
}
function oscillation(spec,t,loop=false) {
 if(!spec || t<0 || !loop&&t>spec.duration)return zero();
 // Preserve ELEGNT's soft attack/release; looping breath stays continuous.
 const ramp=loop?1:clamp(Math.min(t/spec.ramp,(spec.duration-t)/spec.ramp),0,1);
 const env=Math.sin(Math.PI*.5*ramp)**2;
 return spec.amplitude.map((a,i)=>env*a*Math.sin(2*Math.PI*spec.frequency*t+spec.phase[i]));
}
class Emotions {
 constructor(library={},random=Math.random) {
  this.clips=library;this.random=random;this.current=null;this.layers=[];
  this.event=null;this.state=null;this.until=0;this.cycle=0;this.invites=0;
  this.label='Resting';this.detail='Gentle breathing';this.lastPose=null;
  this.lastTime=null;this.attention=zero();this.energy=.35;this.showcase=false;
  this.attentionIndex=0;this.attentionUntil=0;this.attentionPoint=null;
 }
 play(ref,now) {
  const clip=this.clips.clips?.[ref];if(!clip)return;
  this.current={ref,clip,started:now,from:[...(this.lastPose||clip.start)],entry:.65};
  this.until=now+.65+clip.duration+2+this.random()*2;
  this.label=labels[ref]||ref;this.returning=null;
 }
 layer(ref,now,weight=1) {
  const spec=this.clips.overlays?.[ref];if(!spec)return;
  this.layers.push({ref,spec,started:now,weight});
 }
 release(now) {
  this.current=null;
  this.returning=this.lastPose?{from:[...this.lastPose],started:now}:null;
 }
 reset(now) {
  this.release(now);this.layers=[];this.state=null;this.event=null;this.until=now;
 }
 preview(ref,accents,now) {
  this.reset(now);this.showcase=true;this.state='SHOWCASE';this.play(ref,now);
  for(const accent of accents)this.layer(accent,now+.8,accent==='sway'?.45:.7);
 }
 lookAt(data,now,dt,enabled) {
  const people=(data?.people||[]).filter(p=>p.world?.every(Number.isFinite));
  let goal=zero();
  if(enabled && people.length) {
   // Hold attention for several seconds, then include another person in the group.
   // Proximity matching stabilizes reordered detections; this is not identity tracking.
   if(now>=this.attentionUntil) {
    this.attentionIndex=(this.attentionIndex+1)%people.length;
    this.attentionPoint=people[this.attentionIndex].world;
    this.attentionUntil=now+3.5+this.random()*2;
   }
   const old=this.attentionPoint||people[0].world;
   const person=people.reduce((best,p)=>Math.hypot(...p.world.map((v,i)=>v-old[i]))<Math.hypot(...best.world.map((v,i)=>v-old[i]))?p:best);
   this.attentionPoint=person.world;
   const [x,y]=person.world;
   goal[0]=clamp(Math.atan2(y,x)*180/Math.PI,-65,65);
   goal[4]=clamp(goal[0]*.12,-7,7);
  }
  // Different response times give overlapping head/body follow-through.
  this.attention=this.attention.map((v,i)=>v+(goal[i]-v)*(1-Math.exp(-dt*(i===4?5:2))));
  return this.attention;
 }
 sample(data,now,showcase=false) {
  const lib=this.clips;if(!lib.poses)return null;
  const d=data?.demo||{},state=showcase?'SHOWCASE':(d.state||'IDLE');
  const dt=this.lastTime===null?0:clamp(now-this.lastTime,0,.1);this.lastTime=now;
  if(state==='STOPPED') {this.reset(now);this.state=state;this.label='Stopped';this.detail='Resume when ready';return null;}
  if(this.showcase!==showcase){this.reset(now);this.showcase=showcase;}
  const task=state==='ILLUMINATE'||state==='ACK';
  const following=!showcase&&state==='ILLUMINATE'&&d.point_following;
  const visible=(data?.people?.length||data?.hands?.length||0)>0;
  let base=[...(task&&d.pose?d.pose:state==='IDLE'?lib.poses.sleep:lib.poses.look)];
  const previous=this.state;
  if(previous!==state) {
   this.release(now);this.until=now+(state==='IDLE'?12:3);
   if(state==='IDLE' && previous && previous!=='STOPPED')this.play('sleep',now);
   this.state=state;
  }
  // Gesture events preempt invitations. Never queue stale reactions or replay a reconnect.
  if(!showcase && d.event_id!==this.event) {
   this.event=d.event_id;
   if(d.event_age_s>=0 && d.event_age_s<3) {
    const event=d.event||'';
    if(/^(Hello|More company|Open palm|Point |Close ·|Far ·)/.test(event)) {
     this.release(now);this.layers=[];this.invites=0;this.until=now+5;
     this.energy=Math.min(1,this.energy+.3);
     if(event.startsWith('Hello'))this.play(previous==='IDLE'?'wake':'greet',now);
     else if(event.startsWith('More company')){this.play('look_around',now);this.layer('nod',now+.8,.5);}
     else if(event.startsWith('Open palm')) {
      if(!task)this.play(this.cycle++%2?'joy':'greet',now);
      this.layer('shake',now+.2,task?.35:1);
      this.layer('bounce',now+.65,task?.12:.6);
      this.label='Hello back!';
     } else if(event.startsWith('Point accepted')) {
      this.layer('nod',now,.6);this.label='That spot? Got it';
     } else if(event.startsWith('Point downward')||event.startsWith('Point unavailable')) {
      if(!task)this.play('shake',now);
      this.layer('shake',now,.5);this.label='Keep your fingertip and palm in view';
     } else {this.layer('nod',now,.3);this.label=event.startsWith('Close ·')?'A little closer':'A little farther';}
    }
   }
  }
  // Active pointing owns the beam. The outer frame-rate smoother handles pose
  // continuity; no acknowledgment or return blend can pull it off the finger.
  if(following){this.current=null;this.returning=null;this.layers=[];}
  if(showcase && now>=this.until) {
   const choices=['wake','curious','peek','nod','shake','joy','look_around','sleep'];
   const ref=choices[this.cycle++%choices.length];this.play(ref,now);
   if(ref==='joy'){this.layer('dance',now+.7,.7);this.layer('sway',now+.7,.45);}
   else if(ref==='curious')this.layer('nod',now+1,.4);
  } else if(!showcase && now>=this.until && !this.current && !task) {
   if(state==='IDLE') {
    this.play(this.cycle++%3===2?'look_around':'stretch',now);
    this.until+=12+this.random()*8;
   } else if(visible && d.interaction_idle_s>8) {
    const choices=['curious','peek','joy','look_around'];
    const ref=choices[this.cycle++%choices.length];this.play(ref,now);
    if(ref==='joy'){this.layer('dance',now+.7,.65);this.layer('sway',now+.7,.4);}
    else if(ref==='curious')this.layer('nod',now+1,.35);
    // Invite, then wait for an answer. Back off when invitations go unanswered.
    this.invites++;this.until+=Math.min(20,this.invites*4);
   }
  }
  if(this.current) {
   const action=this.current,elapsed=now-action.started;
   const target=trajectory(action.clip,Math.max(0,elapsed-action.entry));
   base=mix(action.from,target,smooth(elapsed/action.entry));
   if(elapsed>=action.clip.duration+action.entry)this.release(now);
  } else if(this.returning) {
   const amount=smooth((now-this.returning.started)/.75);
   base=mix(this.returning.from,base,amount);
   if(amount>=1)this.returning=null;
  }
  this.layers=this.layers.filter(l=>now-l.started<=l.spec.duration);
  const targetEnergy=state==='IDLE'?.2:task?.25:(d.interaction_idle_s>15?.4:.7);
  this.energy+=(targetEnergy-this.energy)*(1-Math.exp(-dt*.35));
  const breath=oscillation(lib.overlays.breathe,now,true);
  // Breathing can mix with a body action; a selected desk pose stays steady.
  if(!task)base=base.map((v,i)=>v+breath[i]*(state==='IDLE'?.5:.3));
  const attention=this.lookAt(data,now,dt,!task&&!showcase&&state==='ATTENTIVE');
  const attentionWeight=task||showcase||state==='IDLE'?0:this.current?.ref==='look_around'?.15:.7;
  base=base.map((v,i)=>v+attention[i]*attentionWeight);
  for(const layer of this.layers) {
   const offset=oscillation(layer.spec,now-layer.started);
   base=base.map((v,i)=>v+offset[i]*layer.weight*(.65+.35*this.energy));
  }
  if(!this.current&&!this.layers.length) {
   this.label=following?'Following your finger':task?'Holding your spot':state==='IDLE'?'Resting':visible?'Listening to you':'Where did you go?';
  }
  const activeLayers=this.layers.filter(l=>now>=l.started).map(l=>({shake:'head wag',bounce:'happy bounce',nod:'a small nod',dance:'dance',sway:'sway'})[l.ref]||l.ref);
  this.detail=[this.current?'Full-body expression':following?'Move your finger to guide the light':task?'Steady desk light':'A quiet pause',
   ...activeLayers,!task?'gentle breathing':null,attentionWeight&&visible?'attention toward you':null].filter(Boolean).join(' · ');
  // CAD joint ranges, independent of the failed steppers and ±45° physical swivel.
  this.lastPose=base.map((v,i)=>clamp(v,i===4?-135:-180,i===4?135:180));
  return [...this.lastPose];
 }
}
root.LuxoiiiEmotions=Emotions;
if(typeof module!=='undefined')module.exports={Emotions,trajectory,oscillation,smooth};
})(globalThis);
