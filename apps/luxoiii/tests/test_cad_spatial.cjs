const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');

// Exercise the renderer's actual frame path without requiring a GPU or CAD files.
const frames=[],draws=[],texts=[];
const gl=new Proxy({
  getShaderParameter:()=>true,getProgramParameter:()=>true,
  getAttribLocation:()=>0,getUniformLocation:(_,name)=>name,
  drawArrays:(mode,...args)=>draws.push({mode,args}),
}, {get:(target,key)=>key in target?target[key]:/^[A-Z_]+$/.test(key)?key:()=>({})});
const canvas={clientWidth:800,clientHeight:600,dataset:{},getContext:()=>gl,addEventListener(){}};
const labels={getContext:()=>({
  clearRect:()=>texts.splice(0),measureText:t=>({width:t.length*7}),fillRect(){},
  fillText:t=>texts.push(t),
})};
const document={documentElement:{},querySelectorAll:()=>[]};
const sandbox={document,devicePixelRatio:1,addEventListener(){},
  requestAnimationFrame:callback=>frames.push(callback),
  getComputedStyle:()=>({getPropertyValue:()=>'#ffffff'}),
  fetch:async()=>({ok:true,json:async()=>({parts:[],bodies:[],couplings:[],model:'test'})}),
};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(require.resolve('../luxoiii/desk/cad_view.js'),'utf8'),sandbox);
const cad=sandbox.createLuxoiiiCAD({canvas,labels,status:{},controls:document});
const camera={position:[.3,0,.2],right:[0,-1,0],up:[0,0,1],forward:[1,0,0],chain:[[0,0,0],[.3,0,.2]]};
const live={fresh:true,camera,hfov:100,vfov:75,
  people:[{world:[1,0,.5],depth_m:.7,range_m:.8}],
  hands_3d:[{world_points:Array.from({length:21},(_,i)=>[.7,i/100,.4]),depth_m:.4,gesture:'point'}],
  demo:{state:'ILLUMINATE',light_requested:true,brightness:.4,desk_target:[.4,.1,0],
    point_following:true,target_mode:'virtual_pointer'},
};
function render(data) {
  draws.length=0;
  cad.spatial(data);
  frames.shift()();
  return draws.filter(d=>d.mode==='LINES').length;
}
(async()=>{
  await new Promise(resolve=>setImmediate(resolve));
  const completeLines=render(live);
  assert(completeLines>100);
  assert(texts.includes('Physical camera'));
  assert(texts.some(t=>t.startsWith('Face 1')));
  assert(texts.some(t=>t.startsWith('Hand 1')));

  const missing={...live,camera:null,joints:[{}, {reason:'Elbow magnet not detected'}, {}]};
  const digitalLines=render(missing);
  assert(digitalLines>0 && digitalLines<completeLines,'Digital desk lines survive a missing real pose');
  assert(texts.includes('Following your finger'));
  assert(texts.includes('Simulated light · 40%'));
  assert(texts.includes('3D camera pose unavailable'));
  assert(texts.includes('Elbow magnet not detected'));
  assert(!texts.some(t=>/^(Physical camera|Face 1|Hand 1)/.test(t)),'Never reuse old physical world markers');
  assert.equal(canvas.dataset.lightPower,'0.4');

  render({...missing,demo:{point_preview:{point:[.5,0,0],mode:'virtual_pointer'}}});
  assert(texts.includes('Virtual desk cursor · hold to select'));
  assert(draws.some(d=>d.mode==='LINES'),'Unconfirmed pointer preview survives missing real pose');

  assert.equal(render({...missing,fresh:false}),0);
  assert.equal(texts.length,0,'Stale frames clear all live markers');
  assert.equal(canvas.dataset.lightPower,'0');

  assert.equal(render(live),completeLines);
  assert(texts.includes('Physical camera'));
  assert(!texts.includes('3D camera pose unavailable'),'Valid pose recovery clears the warning');
  console.log('CAD overlays retain fresh digital pointing, hide unavailable physical pose, and clear stale data');
})().catch(error=>{console.error(error);process.exitCode=1;});
