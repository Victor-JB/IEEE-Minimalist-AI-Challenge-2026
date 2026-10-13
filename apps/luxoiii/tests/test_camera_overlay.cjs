const assert=require('node:assert/strict');
const fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const html=fs.readFileSync(path.join(__dirname,'../luxoiii/desk/preview.html'),'utf8');
const boxes={children:[],replaceChildren(){this.children=[];},append(el){this.children.push(el);}};
const caption={textContent:''};
const context=vm.createContext({document:{getElementById:id=>id==='faceBoxes'?boxes:caption,
 createElement:()=>({style:{}})},live:{fresh:true,size:[640,480],camera:null,people:[],
 outputs:{enabled:true},tracking:{subject:'face',visible:true,status:'Tracking face'},
 detections:[{x:-20,y:10,w:100,h:120},{x:300,y:200,w:40,h:40}]}});
vm.runInContext(html.slice(html.indexOf('function renderImageTracking()'),html.indexOf('function renderMetrics()')),context);
context.renderImageTracking();
assert.equal(boxes.children.length,2,'Image boxes must not depend on 3D projection or encoders');
assert.equal(boxes.children[0].textContent,'Tracking · largest face');
assert.match(boxes.children[0].style.cssText,/left:0%;/,'Cropped face box must stay inside the frame');
assert.match(boxes.children[0].style.cssText,/width:12.5%;/);
assert.match(caption.textContent,/Tracking on/);
context.live.outputs.enabled=false;context.renderImageTracking();
assert.equal(boxes.children[0].className,'faceBox');
assert.equal(caption.textContent,'Physical tracking off');
context.live.fresh=false;context.renderImageTracking();
assert.equal(boxes.children.length,0);assert.match(caption.textContent,/Waiting for fresh/);
context.live=null;context.renderImageTracking();assert.equal(boxes.children.length,0);
console.log('Raw image tracking survives missing 3D pose, clips cropped faces, and clears stale data');
