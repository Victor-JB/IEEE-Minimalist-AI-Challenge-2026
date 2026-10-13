// Full ARM v29 CAD, including base and mechanically coupled internal cams.
// Kinematic visualization only; this renderer never sends hardware commands.
function createLuxoiiiCAD({ canvas, status, controls = document, assetBase = "/cad/", labels = null }) {
  const cv = canvas, gl = cv.getContext("webgl", { antialias: true });
  const $ = () => status;
  const REF = [-20, 75, -40];
  let spatial = null, sceneFit = false;
  let manifest, bodies = [], yaw = 0, shellMode = "xray";
  let parts = [], ready = false, angles = [null, null, null], roll = null, dirty = true;
  let az = 0.6, el = 0.22, zoom = 37;          // the Mac app's 3D camera: (48, 30, 85) looking at (0, 22, 0)
  const M = {
    I: () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    mul(a, b) { const o = new Array(16); for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) { let v = 0; for (let k = 0; k < 4; k++) v += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = v; } return o; },
    T: (x, y, z) => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1],
    Rz(t) { const c = Math.cos(t), s = Math.sin(t); return [c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]; },
    Rx(t) { const c = Math.cos(t), s = Math.sin(t); return [1, 0, 0, 0, 0, c, s, 0, 0, -s, c, 0, 0, 0, 0, 1]; },
    Q(w, x, y, z) { const n = Math.hypot(w, x, y, z); w /= n; x /= n; y /= n; z /= n;
      return [1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y), 0, 2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x), 0,
        2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y), 0, 0, 0, 0, 1]; },
    S: (k) => [k, 0, 0, 0, 0, k, 0, 0, 0, 0, k, 0, 0, 0, 0, 1],
  };
  if (!gl) { $("cadmsg").textContent = "WebGL unavailable in this browser: see the schematic below."; return { pose() {}, spatial() {}, fitPeople() {} }; }
  const sh = (type, src) => { const o = gl.createShader(type); gl.shaderSource(o, src); gl.compileShader(o); return o; };
  const prog = gl.createProgram();
  gl.attachShader(prog, sh(gl.VERTEX_SHADER, `attribute vec3 p; attribute vec3 n; uniform mat4 VP, Mo; uniform mat3 N; varying vec3 vn;
    void main() { vn = N * n; gl_Position = VP * Mo * vec4(p, 1.0); }`));
  gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, `precision mediump float; varying vec3 vn; uniform vec3 col; uniform float a;
    void main() { vec3 q = normalize(vn); float l = 0.32 + 0.55 * abs(dot(q, normalize(vec3(0.45, 0.8, 0.55)))) + 0.22 * abs(dot(q, normalize(vec3(-0.5, 0.25, -0.6))));
      gl_FragColor = vec4(col * l, a); }`));
  gl.linkProgram(prog); gl.useProgram(prog);
  const U = (k) => gl.getUniformLocation(prog, k), aP = gl.getAttribLocation(prog, "p"), aN = gl.getAttribLocation(prog, "n");
  function mesh(buf) {                            // binary STL -> interleaved buffer (cm)
    const dv = new DataView(buf), n = dv.getUint32(80, true), f = new Float32Array(n * 18);
    for (let t = 0; t < n; t++) { const o = 84 + t * 50; const nx = dv.getFloat32(o, true), ny = dv.getFloat32(o + 4, true), nz = dv.getFloat32(o + 8, true);
      for (let v = 0; v < 3; v++) { const b = t * 18 + v * 6, q = o + 12 + v * 12;
        f[b] = dv.getFloat32(q, true) * 0.1; f[b + 1] = dv.getFloat32(q + 4, true) * 0.1; f[b + 2] = dv.getFloat32(q + 8, true) * 0.1; f[b + 3] = nx; f[b + 4] = ny; f[b + 5] = nz; } }
    const g = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, g); gl.bufferData(gl.ARRAY_BUFFER, f, gl.STATIC_DRAW);
    return { g, count: n * 3 };
  }
  (async () => {
    try {
      const response = await fetch(assetBase + "full-model.json");
      if (!response.ok) throw new Error(`CAD manifest: ${response.status}`);
      const man = manifest = await response.json(), cache = {};
      const files = [...new Set(man.parts.map((p) => p.file))];
      let done = 0;
      const pending = [...files];
      await Promise.all(Array.from({ length: 4 }, async () => {
       while (pending.length) {
        const f = pending.shift();
        const response = await fetch(assetBase + f);
        if (!response.ok) throw new Error(`Mesh ${f}: ${response.status}`);
        cache[f] = mesh(await response.arrayBuffer());
        status.textContent = `Loading full assembly… ${++done}/${files.length}`;
       }
      }));
      bodies = man.bodies.map((b) => ({ ...b, local: M.mul(M.T(...b.position), M.Q(...b.quaternion)) }));
      parts = man.parts.map((p) => ({ ...p, m: cache[p.file], col: p.color.slice(0, 3),
        local: M.mul(M.T(...p.position), M.Q(...p.quaternion)) }));
      ready = true;
      cv.dataset.parts = String(parts.length);
      cv.dataset.meshes = String(files.length);
      status.textContent = `${man.model} · ${parts.length} parts · base + internal drives`;
      dirty = true;
    } catch (e) { $("cadmsg").textContent = `CAD unavailable: ${e}`; }
  })();
  function frame() {
    requestAnimationFrame(frame);
    if (!ready || !dirty) return; dirty = false;
    const W = cv.width = cv.clientWidth * devicePixelRatio, H = cv.height = cv.clientHeight * devicePixelRatio;
    gl.viewport(0, 0, W, H); gl.enable(gl.DEPTH_TEST);
    const bg = getComputedStyle(document.documentElement).getPropertyValue("--panel2").trim();
    const m = bg.match(/\w\w/g) || ["1e", "23", "2b"]; gl.clearColor(...m.slice(0, 3).map((h) => parseInt(h, 16) / 255), 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    // orthographic camera orbiting (0, 22, 0); zoom = half the visible height (cm), like SceneKit's orthographicScale
    const scenePoints = spatial && spatial.camera ? [worldPoint(spatial.camera.position),
      ...spatial.people.filter(p=>p.world).map(p=>worldPoint(p.world))] : [];
    if(spatial?.camera) {
      const c=spatial.camera, depth=Math.min(1.5,Math.max(.5,...spatial.people.map(p=>p.depth_m)));
      const fw=depth*Math.tan(spatial.hfov*Math.PI/360),fh=depth*Math.tan(spatial.vfov*Math.PI/360);
      for(const x of [-fw,fw]) for(const y of [-fh,fh]) scenePoints.push(worldPoint(c.position.map((v,i)=>v+x*c.right[i]+y*c.up[i]+depth*c.forward[i])));
    }
    let target = [8,22,0], radius = 37;
    if(sceneFit && scenePoints.length) {
      const all=[[0,0,0],[0,55,0],...scenePoints];
      const lo=[0,1,2].map(i=>Math.min(...all.map(p=>p[i]))), hi=[0,1,2].map(i=>Math.max(...all.map(p=>p[i])));
      target=lo.map((v,i)=>(v+hi[i])/2); radius=Math.max(45,Math.hypot(...hi.map((v,i)=>v-lo[i]))*.65);
    }
    const [tx,ty,tz] = target, r = 1000, ex = tx + r * Math.cos(el) * Math.sin(az), ey = ty + r * Math.sin(el), ez = tz + r * Math.cos(el) * Math.cos(az);
    let fx = tx - ex, fy = ty - ey, fz = tz - ez; const fl = Math.hypot(fx, fy, fz); fx /= fl; fy /= fl; fz /= fl;
    let sx = fy * 0 - fz * 1, sy = fz * 0 - fx * 0, sz = fx * 1 - fy * 0; const sl = Math.hypot(sx, sy, sz) || 1; sx /= sl; sy /= sl; sz /= sl;
    const ux = sy * fz - sz * fy, uy = sz * fx - sx * fz, uz = sx * fy - sy * fx;
    const V = [sx, ux, -fx, 0, sy, uy, -fy, 0, sz, uz, -fz, 0, -(sx * ex + sy * ey + sz * ez), -(ux * ex + uy * ey + uz * ez), fx * ex + fy * ey + fz * ez, 1];
    const hy = Math.max(sceneFit ? radius : zoom, (sceneFit ? radius : 35) * H / W), hx = hy * W / H, n = 1, f = 2500;
    const P = [1 / hx, 0, 0, 0, 0, 1 / hy, 0, 0, 0, 0, -2 / (f - n), 0, 0, 0, -(f + n) / (f - n), 1];
    const VP=M.mul(P,V);
    gl.uniformMatrix4fv(U("VP"), false, VP);
    // Preserve the branch's complete hierarchy, then map MuJoCo Z-up into browser Y-up.
    const values = { yaw: yaw * Math.PI / 180, head_tilt: (roll || 0) * Math.PI / 180 };
    ["shoulder", "elbow", "wrist"].forEach((name, j) => {
      values[name] = (Number.isFinite(angles[j]) ? angles[j] : REF[j]) * Math.PI / 180;
    });
    for (const c of manifest.couplings) {
      values[c.joint] = c.coefficients.reduce((v, k, i) => v + k * values[c.source] ** i, 0);
    }
    const world = [], rig = M.Rx(-Math.PI / 2);
    for (const b of bodies) {
      let local = b.local;
      if (b.joint) {
        const j = b.joint, a = values[j.name] || 0, norm = Math.hypot(...j.axis);
        const q = M.Q(Math.cos(a / 2), ...j.axis.map((v) => Math.sin(a / 2) * v / norm));
        local = M.mul(local, M.mul(M.T(...j.position), M.mul(q, M.T(...j.position.map((v) => -v)))));
      }
      world.push(M.mul(b.parent < 0 ? rig : world[b.parent], local));
    }
    const draws = parts.map((p) => {
      const Mo = M.mul(world[p.body], p.local);
      const a = p.shell ? (shellMode === "solid" ? 1 : shellMode === "hidden" ? 0 : p.color[3]) : p.color[3];
      // Camera-space depth for back-to-front translucent shells.
      const depth = V[2] * Mo[12] + V[6] * Mo[13] + V[10] * Mo[14] + V[14];
      return { p, Mo, a, depth };
    }).filter((d) => d.a > 0);
    const draw = ({ p, Mo, a }) => {
      gl.uniformMatrix4fv(U("Mo"), false, Mo);
      gl.uniformMatrix3fv(U("N"), false, [Mo[0], Mo[1], Mo[2], Mo[4], Mo[5], Mo[6], Mo[8], Mo[9], Mo[10]]);
      gl.uniform3fv(U("col"), p.col); gl.uniform1f(U("a"), a);
      gl.bindBuffer(gl.ARRAY_BUFFER, p.m.g);
      gl.enableVertexAttribArray(aP); gl.vertexAttribPointer(aP, 3, gl.FLOAT, false, 24, 0);
      gl.enableVertexAttribArray(aN); gl.vertexAttribPointer(aN, 3, gl.FLOAT, false, 24, 12);
      gl.drawArrays(gl.TRIANGLES, 0, p.m.count);
    };
    gl.disable(gl.BLEND); gl.depthMask(true);
    draws.filter((d) => d.a >= 1).forEach(draw);
    gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA); gl.depthMask(false);
    draws.filter((d) => d.a < 1).sort((a, b) => a.depth - b.depth).forEach(draw);
    gl.depthMask(true); gl.disable(gl.BLEND);
    drawSpatial(VP, W, H);
    cv.dataset.shellMode = shellMode;
    cv.dataset.people = String(spatial?.people?.length || 0);
  }

  const lineBuffer=gl.createBuffer();
  const worldPoint=p=>[p[0]*100,p[2]*100,-p[1]*100];
  function drawSpatial(VP,W,H) {
    const ctx=labels?.getContext('2d');
    if(ctx) { labels.width=W; labels.height=H; ctx.clearRect(0,0,W,H); }
    if(!spatial?.fresh || !spatial.camera) return;
    const c=spatial.camera, origin=worldPoint(c.position), lines=[];
    const add=(a,b,color)=>lines.push({a,b,color});
    const amber=[1,.7,.3], teal=[.25,1,.82], muted=[.32,.58,.65];
    const camPoint=(x,y,z)=>worldPoint(c.position.map((v,i)=>v+x*c.right[i]+y*c.up[i]+z*c.forward[i]));
    const ring=(center,r,color)=> {
      for(let axis=0;axis<3;axis++) for(let j=0;j<24;j++) {
        const point=t=>center.map((v,i)=>v+(i===axis?0:r*(i===(axis+1)%3?Math.cos(t):Math.sin(t))));
        add(point(j*Math.PI/12),point((j+1)*Math.PI/12),color);
      }
    };
    for(let i=1;i<c.chain.length;i++) add(worldPoint(c.chain[i-1]),worldPoint(c.chain[i]),amber);
    ring(origin,2.5,amber);
    const depth=Math.min(1.5,Math.max(.5,...spatial.people.map(p=>p.depth_m)));
    const fw=depth*Math.tan(spatial.hfov*Math.PI/360), fh=depth*Math.tan(spatial.vfov*Math.PI/360);
    const corners=[[-fw,-fh],[fw,-fh],[fw,fh],[-fw,fh]].map(p=>camPoint(...p,depth));
    corners.forEach((p,i)=>{add(origin,p,muted);add(p,corners[(i+1)%4],muted);});
    const text=(p,label,color)=> {
      if(!ctx) return;
      const clip=[0,1,2,3].map(i=>VP[i]*p[0]+VP[4+i]*p[1]+VP[8+i]*p[2]+VP[12+i]);
      const x=(clip[0]/clip[3]+1)*W/2,y=(1-clip[1]/clip[3])*H/2;
      ctx.font=`${12*devicePixelRatio}px system-ui`; const tw=ctx.measureText(label).width;
      ctx.fillStyle='rgba(10,18,28,.86)'; ctx.fillRect(x+5,y-19*devicePixelRatio,tw+10,20*devicePixelRatio);
      ctx.fillStyle=color; ctx.fillText(label,x+10,y-5*devicePixelRatio);
    };
    text(origin,'Physical camera','#ffbc65');
    spatial.people.forEach((p,i)=> {
      if(!p.world) return;
      const point=worldPoint(p.world);ring(point,8,teal);add(origin,point,teal);
      // A head-and-shoulders symbol, not estimated body joints.
      const center=point.map((v,j)=>v-14*c.up[[0,2,1][j]]*(j===2?-1:1));
      const right=worldPoint(c.right).map(v=>v*.18);
      add(center.map((v,j)=>v-right[j]),center.map((v,j)=>v+right[j]),teal);
      text(point,`${spatial.example ? "EXAMPLE" : "Face "+(i+1)} · ~${p.range_m.toFixed(2)} m`,'#80f9d8');
    });
    const values=new Float32Array(lines.flatMap(l=>[...l.a,0,1,0,...l.b,0,1,0]));
    gl.bindBuffer(gl.ARRAY_BUFFER,lineBuffer);gl.bufferData(gl.ARRAY_BUFFER,values,gl.DYNAMIC_DRAW);
    gl.vertexAttribPointer(aP,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(aN,3,gl.FLOAT,false,24,12);
    gl.uniformMatrix4fv(U('Mo'),false,M.I()); gl.uniformMatrix3fv(U('N'),false,[1,0,0,0,1,0,0,0,1]);gl.uniform1f(U('a'),1);
    gl.disable(gl.DEPTH_TEST);
    lines.forEach((l,i)=>{gl.uniform3fv(U('col'),l.color);gl.drawArrays(gl.LINES,i*2,2);});
    gl.enable(gl.DEPTH_TEST);
  }
  requestAnimationFrame(frame);
  let drag = null;
  cv.addEventListener("pointerdown", (e) => { drag = [e.clientX, e.clientY, az, el]; cv.setPointerCapture(e.pointerId); });
  cv.addEventListener("pointermove", (e) => { if (!drag) return; az = drag[2] - (e.clientX - drag[0]) * 0.01; el = Math.max(-1.4, Math.min(1.4, drag[3] + (e.clientY - drag[1]) * 0.01)); dirty = true; });
  cv.addEventListener("pointerup", () => (drag = null));
  cv.addEventListener("wheel", (e) => { e.preventDefault(); sceneFit=false; zoom = Math.max(8, Math.min(500, zoom * Math.exp(e.deltaY * 0.001))); dirty = true; }, { passive: false });
  addEventListener("resize", () => (dirty = true));
  const CAMS = { "3d": [0.51, 0.08], side: [0, 0], front: [Math.PI / 2, 0] };
  controls.querySelectorAll("[data-cam]").forEach((b) => (b.onclick = () => { [az, el] = CAMS[b.dataset.cam]; sceneFit=false; zoom = 37; dirty = true; }));
  controls.querySelectorAll("[data-shell]").forEach((button) => {
    button.onclick = () => {
      shellMode = button.dataset.shell;
      controls.querySelectorAll("[data-shell]").forEach((b) => b.setAttribute("aria-pressed", String(b === button)));
      dirty = true;
    };
  });
  return { spatial(data) { spatial=data; dirty=true; },
    fitPeople() { sceneFit=true; dirty=true; },
    pose(a, r, baseYaw = 0) {
    const same = a && angles && a.every((v, i) => v === angles[i]) && r === roll && baseYaw === yaw;
    if (!same) { angles = a ? [...a] : [null, null, null]; roll = r; yaw = baseYaw; dirty = true; }
  } };
}
