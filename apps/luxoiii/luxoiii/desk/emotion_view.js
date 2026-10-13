/* Original authored curves, sampled locally at display rate. Simulation only. */
(function(root) {
'use strict';
const names=['base_yaw','shoulder_pitch','elbow_pitch','wrist_pitch','head_roll'];
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
function curve(frames,name,t) {
 const ks=frames.filter(k=>Number.isFinite(k.joints[name]));
 if(!ks.length)return 0;
 if(t<=ks[0].t)return ks[0].joints[name];
 const i=ks.findIndex(k=>k.t>=t);
 if(i<0)return ks.at(-1).joints[name];
 const a=ks[i-1],b=ks[i],h=b.t-a.t,s=(t-a.t)/h,v=a.joints[name],w=b.joints[name];
 if(b.ease==='cubic') {
  const tangent=j=>j>0&&j<ks.length-1&&ks[j].ease==='cubic'&&ks[j+1].ease==='cubic'?
   (ks[j+1].joints[name]-ks[j-1].joints[name])/(ks[j+1].t-ks[j-1].t):0;
  return (2*s**3-3*s*s+1)*v+(s**3-2*s*s+s)*h*tangent(i-1)+(-2*s**3+3*s*s)*w+(s**3-s*s)*h*tangent(i);
 }
 const e=b.ease==='linear'?s:b.ease==='ease_in'?s**3:b.ease==='ease_out'?1-(1-s)**3:s**3*(s*(6*s-15)+10);
 return v+(w-v)*e;
}
class Emotions {
 constructor(clips={}) {this.clips=clips;this.current=null;this.event=null;this.state=null;this.cycle=0;this.until=0;this.label='Ready';this.demoIndex=0;}
 play(ref,now) {
  const clip=this.clips[ref];if(!clip)return;
  this.current=clip;this.started=now;this.scale=1.15;this.duration=Math.max(...clip.keyframes.map(k=>k.t))*this.scale;
  this.until=now+this.duration+1.3;this.label=ref.split('__')[0].replaceAll('_',' ');
 }
 sample(data,now,showcase=false) {
  const d=data?.demo||{},state=d.state||'IDLE';
  const measured=data?.camera?[0,...data.camera.joints,data.camera.roll]:[0,-20,65,35,0];
  const base=!showcase&&state==='ILLUMINATE'&&d.pose?[...d.pose]:measured;
  if(!showcase && state==='STOPPED'){this.current=null;this.state=state;this.label='Stopped';return null;}
  let ref=null;
  if(showcase) {
   if(now>=this.until)ref=['perk_up__small','curious_head_tilt__single','tail_wag__small','nod_yes__medium','shrug_fail__small','idle_breathe__calm'][this.demoIndex++%6];
   // Return to live behavior immediately when the showcase ends.
   this.event=null;this.state=null;
  } else {
   if(d.event_id!==this.event) {
    const e=d.event||'';this.event=d.event_id;
    // Do not replay an old gesture when opening/reconnecting the page.
    if(d.event_age_s<3) {
     if(e.startsWith('Hello'))ref='perk_up__small';
     else if(e.startsWith('Wave'))ref='tail_wag__small';
     else if(e.startsWith('Point accepted'))ref='nod_yes__medium';
     else if(e.startsWith('Point downward'))ref='shrug_fail__small';
     else if(e.startsWith('Brighter')||e.startsWith('Dimmer'))ref='nod_yes__small_calm';
    }
   }
   if(state!==this.state) {
    if(!ref&&state==='ATTENTIVE')ref='double_take__subtle';
    if(state==='ILLUMINATE'){this.current=null;this.until=now+2;}
    if(state==='IDLE'){this.current=null;this.until=now;}
    this.state=state;
   }
   if(!ref&&now>=this.until&&state!=='ACK') {
    const pool=state==='ATTENTIVE'?['idle_breathe__alert','curious_head_tilt__single','lean_in_interest__subtle']:
     state==='ILLUMINATE'?[]:['idle_breathe__calm','idle_breathe__calm','scan_room__calm'];
    if(pool.length)ref=pool[this.cycle++%pool.length];
   }
  }
  if(ref)this.play(ref,now);
  if(this.current) {
   const c=this.current,t=Math.max(0,(now-this.started)/this.scale);
   // Use source curves with sparse per-joint timing and their original easing.
   names.forEach((name,i)=>{
    if(c.joint_mask.includes(name)) {
     const offset=curve(c.keyframes,name,t);
     base[i]=c.mode==='additive'?base[i]+offset*.8:offset;
    }
   });
   if(now-this.started>this.duration){this.current=null;this.label=state==='ILLUMINATE'?'Holding the selected spot':'Listening';}
  } else if(state==='ILLUMINATE')this.label='Holding the selected spot';
  return base.map((v,i)=>clamp(v,...[[-60,60],[-35,20],[-10,90],[-60,110],[-35,35]][i]));
 }
}
root.LuxoiiiEmotions=Emotions;
if(typeof module!=='undefined')module.exports={Emotions,curve};
})(globalThis);
