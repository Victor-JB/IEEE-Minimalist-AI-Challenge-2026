const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const {Emotions,curve}=require('../luxoiii/desk/emotion_view.js');
const clipsDir=path.resolve(__dirname,'../../../arm/clips/authored');
const clips=Object.fromEntries(fs.readdirSync(clipsDir).map(f=>[f.replace('.json',''),JSON.parse(fs.readFileSync(path.join(clipsDir,f)))]));
const data={camera:{joints:[-10,60,25],roll:3},demo:{state:'ATTENTIVE',event_id:1,event_age_s:0,event:'Hello'}};
const e=new Emotions(clips);
assert.deepEqual(e.sample(data,0),[0,-10,60,25,0]);
assert.notDeepEqual(e.sample(data,.6),[0,-10,60,25,0]);
const old={...data,demo:{...data.demo,event_id:2,event:'Wave',event_age_s:0}};
e.sample(old,1);assert.equal(e.label,'tail wag');
e.sample({...old,demo:{state:'STOPPED'}},2);assert.equal(e.current,null);
assert.equal(e.sample({...old,demo:{state:'STOPPED'}},3),null);
const aim=[0,-18,65,70,0];
assert.deepEqual(e.sample({...data,demo:{state:'ILLUMINATE',pose:aim,event_id:3,event_age_s:10}},4),aim);
assert.deepEqual(e.sample({...data,demo:{state:'ILLUMINATE',pose:aim,event_id:3,event_age_s:10}},20),aim);
assert.notDeepEqual(e.sample({...data,demo:{state:'ILLUMINATE',pose:aim,event_id:3,event_age_s:10}},21),aim);
const independent=new Emotions(clips);
assert.deepEqual(independent.sample({...data,camera:{joints:[-10,60,25],roll:-8}},0),[0,-10,60,25,0]);
// Sparse per-joint timing: an unrelated keyframe must not disturb the shoulder.
assert.equal(curve([{t:0,joints:{shoulder_pitch:0}},{t:1,joints:{head_roll:50}},{t:2,joints:{shoulder_pitch:20},ease:'linear'}],'shoulder_pitch',1),10);
const keys=[{t:0,joints:{x:0},ease:'cubic'},{t:1,joints:{x:10},ease:'cubic'},{t:2,joints:{x:0},ease:'cubic'}];
assert.equal(curve(keys,'x',0),0);assert.equal(curve(keys,'x',1),10);assert.equal(curve(keys,'x',2),0);
// Showcase uses the same curves but never constructs a hardware request.
const show=new Emotions(clips);assert(show.sample(null,0,true).every(Number.isFinite));
assert(show.sample(null,.5,true).every(Number.isFinite));
console.log('Emotion curves, measured baseline, event reactions, stop, and desk hold passed');

const social=new Emotions(clips);
const linger={...data,demo:{state:'ATTENTIVE',event_id:1,event_age_s:20,interaction_idle_s:15}};
const seen=new Set();
for(let time=0;time<90;time+=.25){social.sample(linger,time);seen.add(social.label);}
assert(seen.has('excited bounce'));assert(seen.has('curious head tilt'));
social.sample({...linger,demo:{...linger.demo,event_id:2,event_age_s:0,event:'Point accepted',state:'ACK'}},91);
assert.equal(social.label,'nod yes');
console.log('Playful lingering loops and gesture priority passed');
