const assert=require('node:assert/strict');
const lib=require('../../../arm/clips/full-model.json');
const {Emotions,trajectory,oscillation}=require('../luxoiii/desk/emotion_view.js');
const nearly=(a,b,tol=1e-8)=>a.forEach((v,i)=>assert(Math.abs(v-b[i])<tol,`${a} != ${b}`));
const data=(state='ATTENTIVE',event='',id=1,extra={})=>({fresh:true,camera:{joints:[-10,60,25],roll:-40},
 people:[{world:[1,0,.4]}],demo:{state,event,event_id:id,event_age_s:0,interaction_idle_s:0,...extra}});
const engine=()=>new Emotions(lib,()=>.5);
assert.equal(lib.source.commit,'ef1b0d2057c5e449a0e02638a1ff7d5b58dd3cbd');
// Every source key pose and per-joint delay survives the branch import unchanged.
for(const clip of Object.values(lib.clips)) {
 nearly(trajectory(clip,0),clip.start);
 for(const s of clip.segments) {
  nearly(trajectory(clip,s.t),s.start);
  nearly(trajectory(clip,s.t+s.duration+4*s.stagger),s.end);
  if(s.stagger)assert.equal(trajectory(clip,s.t+s.stagger/2)[4],s.start[4]);
 }
 nearly(trajectory(clip,clip.duration),clip.segments.at(-1).end);
}
assert.equal(Math.max(...lib.clips.peek.segments.map(s=>s.end[1])),50); // Old clamp was 20°.
assert.equal(lib.poses.look_left[0],55);
assert.equal(lib.poses.sleep[2],90);
for(const spec of Object.values(lib.overlays)) {
 nearly(oscillation(spec,0),[0,0,0,0,0]);
 nearly(oscillation(spec,spec.duration),[0,0,0,0,0]);
}
// Hardware pose does not anchor the digital arm any more.
const a=engine(),b=engine();
const hello=data('ATTENTIVE','Hello · ready to help');
nearly(a.sample(hello,0),b.sample({...hello,camera:{joints:[80,-40,10],roll:40}},0));
assert.equal(a.current.ref,'greet');
const greet=a.sample(hello,.9);assert(greet.every(Number.isFinite));
const palm=data('ATTENTIVE','Open palm · hello back',2);
a.sample(palm,1);assert.equal(a.label,'Hello back!');
assert.deepEqual(a.layers.map(l=>l.ref),['shake','bounce']);
const started=a.current.started;
a.sample(palm,1.2);assert.equal(a.current.started,started); // A held palm does not retrigger.
// A fresh desk gesture interrupts a long idle action and preserves the target.
const aim=[0,-18,65,70,0];
a.sample(data('ACK','Point accepted · estimated desk spot selected',3,{pose:aim}),1.4);
assert.equal(a.current,null);assert.deepEqual(a.layers.map(l=>l.ref),['nod']);
a.sample(data('ILLUMINATE','Point accepted',3,{pose:aim}),2.2);
nearly(a.sample(data('ILLUMINATE','Point accepted',3,{pose:aim}),4),aim);
a.sample(data('ILLUMINATE','Open palm · hello back',4,{pose:aim}),5);
assert.equal(a.current,null);assert(a.layers.length===2);
nearly(a.sample(data('ILLUMINATE','Open palm · hello back',4,{pose:aim}),9),aim);
assert.equal(a.sample(data('STOPPED'),10),null);assert.equal(a.current,null);assert.equal(a.layers.length,0);
// Reconnects must not replay a stale event; old data alone cannot become a greeting.
const stale=engine();stale.sample(data('ATTENTIVE','Open palm · hello back',10,{event_age_s:20}),0);
assert.equal(stale.current,null);assert.equal(stale.layers.length,0);
// Simultaneous dance and sway actually change the same joint, not just the label.
const mixed=engine(),plain=engine();mixed.preview('joy',['dance','sway'],0);plain.preview('joy',[],0);
mixed.sample(null,0,true);plain.sample(null,0,true);
const m=mixed.sample(null,1.2,true),p=plain.sample(null,1.2,true);
assert(Math.abs(m[0]-p[0])>.1);assert.equal(mixed.layers.length,2);
// Entry/interruption are continuous; no 50° shoulder snap on switching to peek.
const continuity=engine();continuity.preview('peek',[],0);
let last=continuity.sample(null,0,true),largest=0;
for(let t=1/60;t<8;t+=1/60) {
 if(t>1.5&&t<1.5+1/60)continuity.preview('sleep',[],t);
 const q=continuity.sample(null,t,true);
 largest=Math.max(largest,...q.map((v,i)=>Math.abs(v-last[i])));last=q;
}
assert(largest<8,`One frame jumped ${largest}°`);
// Invitations leave longer pauses when ignored, and stop when nobody is visible.
const social=engine(),linger=data('ATTENTIVE','',1,{interaction_idle_s:30,event_age_s:30});
const seen=new Set();let invitedAt=[];let previous=null;
for(let time=0;time<130;time+=.05) {
 social.sample(linger,time);const active=social.current?.started;
 if(active!==undefined&&active!==previous){invitedAt.push(time);previous=active;seen.add(social.current.ref);}
}
assert(seen.has('curious')&&seen.has('peek')&&seen.has('joy'));
assert(invitedAt[2]-invitedAt[1]>invitedAt[1]-invitedAt[0]);
const alone=engine();for(let t=0;t<20;t++)alone.sample({...linger,people:[]},t);
assert.equal(alone.invites,0);
// Departures settle; arrivals interrupt rest; geometry-based attention turns toward a person.
const attention=engine();attention.sample(data('IDLE'),0);
attention.sample(data('ATTENTIVE','Hello',2),1);assert.equal(attention.current.ref,'wake');
for(let t=1;t<3;t+=.05)attention.sample({...hello,people:[{world:[1,.5,.4]}]},t);
assert(attention.attention[0]>10);
console.log('Full-model source poses, layering, interruptions, palm greeting, target hold, stop, attention and social pacing passed');
