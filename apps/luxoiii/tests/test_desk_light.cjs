const assert=require('node:assert/strict');
const {luxoiiiLight}=require('../luxoiii/desk/cad_view.js');
// Independent reference values from arm/python/kinematics.py (metres -> renderer cm).
const cases=[
 {q:[20,55,60],r:0,y:0,p:[30.91451485914483,20.354951397663733,0],d:[.7071067811865476,-.7071067811865475,0]},
 {q:[-18,65,70],r:22,y:-35,p:[11.993926718741758,30.963894656362527,10.684788951321027],d:[.4618579942266626,-.4209326614852151,.7807066591645657]},
 {q:[15,45,55],r:-32,y:55,p:[14.30898532185077,28.325842547163933,-25.054779081729706],d:[.006762152082234962,-.3584006122701382,-.933543397182802]}
];
for(const c of cases) {
 const h=luxoiiiLight(c.q,c.r,c.y);
 c.p.forEach((v,i)=>assert(Math.abs(v-h.position[i])<1e-9));
 c.d.forEach((v,i)=>assert(Math.abs(v-h.direction[i])<1e-9));
 for(const basis of [h.right,h.up,h.direction])assert(Math.abs(Math.hypot(...basis)-1)<1e-9);
 assert(Math.abs(h.right.reduce((s,v,i)=>s+v*h.up[i],0))<1e-9);
}
// The preview's cone points down and lands in the selectable desk area.
const h=luxoiiiLight([20,55,60]);const t=-h.position[1]/h.direction[1];
const hit=h.position.map((v,i)=>v+t*h.direction[i]);
assert(hit[0]>20&&hit[0]<65);assert(Math.abs(hit[2])<1e-9);
console.log('Rendered spotlight matches calibrated FK for pitch, swivel and base yaw');
