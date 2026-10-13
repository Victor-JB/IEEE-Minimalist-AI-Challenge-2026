const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const html=fs.readFileSync(require('node:path').join(__dirname,'../luxoiii/desk/preview.html'),'utf8');
const source=html.slice(html.indexOf('function controlFeedback('),html.indexOf("document.getElementById('hardware').onclick="));
const elements=new Map();
const element=id=>{
 if(!elements.has(id))elements.set(id,{textContent:'',hidden:true,disabled:false,setAttribute(k,v){this[k]=v;}});
 return elements.get(id);
};
const pending=[];
const context=vm.createContext({document:{getElementById:element},live:{outputs:{enabled:false}},
 controlPending:false,controlSequence:0,wantHardware:false,
 control:action=>new Promise((resolve,reject)=>pending.push({action,resolve,reject}))});
vm.runInContext(source,context);

(async()=>{
 const actions=[];
 const enable=context.requestControl('enable','Enabled',()=>actions.push('enable'));
 assert.equal(element('hardware').disabled,true);
 await context.requestControl('enable','Duplicate',()=>actions.push('duplicate'));
 assert.equal(pending.length,1);
 const stop=context.requestControl('stop','Stopped',()=>actions.push('stop'),true);
 pending[0].resolve({ok:true});await enable;
 assert.deepEqual(actions,[],'Late enable must not undo Stop');
 assert.equal(element('hardware').disabled,true);
 pending[1].resolve({ok:true});await stop;
 assert.deepEqual(actions,['stop']);
 assert.equal(element('controlFeedback').textContent,'Stopped');
 assert.equal(element('hardware').disabled,false);

 const failure=context.requestControl('enable','Enabled',()=>actions.push('bad'));
 pending[2].reject(new Error('Board unavailable'));await failure;
 assert.equal(element('controlFeedback').hidden,false);
 assert.match(element('controlFeedback').textContent,/Could not enable tracking: Board unavailable/);
 context.live={outputs:{enabled:false}};context.renderControlButtons();
 assert.match(element('controlFeedback').textContent,/Board unavailable/,'Live button refresh must preserve the error');
 assert.deepEqual(actions,['stop']);

 const resume=context.requestControl('resume','Gestures resumed',()=>actions.push('resume'));
 pending[3].resolve({ok:true});await resume;
 assert.equal(element('controlFeedback').textContent,'Gestures resumed');
 assert.deepEqual(actions,['stop','resume']);
 context.wantHardware=true;context.live={outputs:{enabled:false}};context.renderControlButtons();
 assert.equal(element('hardware').textContent,'Disable tracking + auto light');
 console.log('Control feedback, pending request suppression, and Stop priority pass');
})().catch(error=>{console.error(error);process.exitCode=1;});
