/* IMPULS v5 module viewer: WebGL2, no external libraries. */
(function () {
  'use strict';
  const G = window.IMPULS_GEOM, SD = window.SIMDATA;
  const { V, Mx } = G, PI = Math.PI, TAU = 2 * PI;
  const $ = (id) => document.getElementById(id);
  const canvas = $('gl');
  const gl = canvas.getContext('webgl2', { antialias: true, preserveDrawingBuffer: true, alpha: false });
  if (!gl) { const e = $('err'); e.style.display = 'flex'; e.textContent = 'Браузер не підтримує WebGL2 — відкрийте сторінку в Chrome, Edge, Firefox або Safari 15+.'; return; }

  // ------------------------------------------------------------------ shaders
  const VS = `#version 300 es
  in vec3 aPos; in vec3 aNrm; in vec3 aCol; in float aU;
  uniform mat4 uModel, uVP;
  out vec3 vW; out vec3 vN; out vec3 vCol; out float vU;
  void main(){ vec4 w = uModel*vec4(aPos,1.0); vW=w.xyz; vN=mat3(uModel)*aNrm; vCol=aCol; vU=aU; gl_Position=uVP*w; }`;
  const FS = `#version 300 es
  precision highp float;
  in vec3 vW; in vec3 vN; in vec3 vCol; in float vU;
  uniform vec3 uColor, uCam, uPickCol; uniform int uUseVCol, uClipOn, uPick; uniform vec4 uClip;
  uniform float uMetal, uRough, uAlpha, uHi; uniform vec4 uStripe;
  out vec4 o;
  vec3 toLin(vec3 c){ return pow(c, vec3(2.2)); }
  void main(){
    if(uClipOn==1 && dot(vec4(vW,1.0),uClip)>0.0) discard;
    if(uPick==1){ o=vec4(uPickCol,1.0); return; }
    vec3 base = uUseVCol==1 ? vCol : uColor;
    if(uStripe.x>0.0){ float s=0.5+0.5*sin(vU*uStripe.z-uStripe.y); base=mix(base, min(base*1.8+0.08, vec3(1.0)), uStripe.w*s*s); }
    vec3 N=normalize(vN); vec3 Vd=normalize(uCam-vW);
    if(!gl_FrontFacing){ N=-N;
      if(uClipOn==1){ float h=step(0.82,fract((gl_FragCoord.x+gl_FragCoord.y)/6.0)); vec3 c=mix(base*0.78+vec3(0.06),base*0.45,h); o=vec4(c,1.0); return; } }
    vec3 b=toLin(base);
    vec3 L1=normalize(vec3(0.55,0.75,0.45)), L2=normalize(vec3(-0.6,0.35,-0.7)), L3=normalize(vec3(0.1,-0.6,0.8));
    float shin=mix(90.0,10.0,uRough);
    vec3 col=vec3(0.0);
    vec3 amb=mix(vec3(0.20,0.19,0.18),vec3(0.42,0.45,0.50),0.5+0.5*N.y);
    col+=amb*b*(uMetal>0.5?0.75:1.0);
    vec3 Ls[3]; Ls[0]=L1; Ls[1]=L2; Ls[2]=L3; float I[3]; I[0]=1.0; I[1]=0.38; I[2]=0.22;
    for(int k=0;k<3;k++){ vec3 L=Ls[k]; float d=max(dot(N,L),0.0); vec3 H=normalize(L+Vd); float sp=pow(max(dot(N,H),0.0),shin)*(shin+8.0)/40.0;
      vec3 spc = uMetal>0.5 ? b*1.2+0.06 : vec3(0.06*(1.0-uRough)+0.015);
      col+=I[k]*(b*d*(uMetal>0.5?0.55:0.95)+spc*sp); }
    float fr=pow(1.0-max(dot(N,Vd),0.0),3.0); col+=fr*0.10*vec3(0.8,0.85,0.9);
    col=col/(1.0+col*0.35);
    col=pow(col,vec3(1.0/2.2));
    if(uHi>0.0){ col=mix(col,vec3(1.0,0.55,0.10),0.35*uHi+0.15*uHi*fr); }
    o=vec4(col,uAlpha);
  }`;
  function sh(type, src) { const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s); if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s)); return s; }
  const prog = gl.createProgram(); gl.attachShader(prog, sh(gl.VERTEX_SHADER, VS)); gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, FS));
  gl.bindAttribLocation(prog, 0, 'aPos'); gl.bindAttribLocation(prog, 1, 'aNrm'); gl.bindAttribLocation(prog, 2, 'aCol'); gl.bindAttribLocation(prog, 3, 'aU');
  gl.linkProgram(prog); if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
  gl.useProgram(prog);
  const U = {}; for (const n of ['uModel', 'uVP', 'uColor', 'uCam', 'uPickCol', 'uUseVCol', 'uClipOn', 'uPick', 'uClip', 'uMetal', 'uRough', 'uAlpha', 'uHi', 'uStripe']) U[n] = gl.getUniformLocation(prog, n);

  // ------------------------------------------------------------------ GPU meshes
  function gpu(mesh, u, dyn) {
    const o = { vao: gl.createVertexArray(), b: [gl.createBuffer(), gl.createBuffer(), gl.createBuffer(), gl.createBuffer()], ib: gl.createBuffer(), count: 0, hasCol: false, hasU: false };
    upload(o, mesh, u, dyn); return o;
  }
  function upload(o, mesh, u, dyn) {
    const usage = dyn ? gl.DYNAMIC_DRAW : gl.STATIC_DRAW;
    gl.bindVertexArray(o.vao);
    const set = (k, data, size) => { gl.bindBuffer(gl.ARRAY_BUFFER, o.b[k]); gl.bufferData(gl.ARRAY_BUFFER, data, usage); gl.enableVertexAttribArray(k); gl.vertexAttribPointer(k, size, gl.FLOAT, false, 0, 0); };
    set(0, new Float32Array(mesh.p), 3); set(1, new Float32Array(mesh.n), 3);
    if (mesh.c && mesh.c.length === mesh.p.length) { set(2, new Float32Array(mesh.c), 3); o.hasCol = true; } else { gl.disableVertexAttribArray(2); gl.vertexAttrib3f(2, 1, 1, 1); o.hasCol = false; }
    if (u && u.length === mesh.p.length / 3) { set(3, new Float32Array(u), 1); o.hasU = true; } else { gl.disableVertexAttribArray(3); gl.vertexAttrib1f(3, 0); o.hasU = false; }
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, o.ib); gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint32Array(mesh.i), usage);
    o.count = mesh.i.length; gl.bindVertexArray(null);
  }

  // ------------------------------------------------------------------ scene state
  const S = {
    scen: 0, dial: 0, dialT: 0, tau: -0.2, playing: false, slow: 20, mode: 'full', cutH: 30, cutT: 'h', expK: 0.8, skin: false, body: true, sel: null,
  };
  const T0 = -0.10;                       // the pull of the handle starts (s); it leaves the module and the drums are free at 0
  let geo, objs = [], byId = {};
  const groupsUI = [
    ['Корпус', ['cover', 'saddle', 'pad', 'slotWipers', 'screws', 'buttonGuard']],
    ['Кріплення до пояса', ['strapTongue', 'strap']],
    ['Рама', ['plateYm', 'plateYp', 'tieRod', 'bush1', 'thrust', 'arbor', 'arborLock', 'cupWasher', 'rollerPinsA']],
    ['Барабан 1: пружина, храповик', ['barrel1', 'capM1', 'spring', 'springWasher', 'gear1', 'ratchet', 'clutchDisk', 'pawl1', 'pawl2', 'pawl3', 'pawlSprings', 'cam']],
    ['Барабан 2 і балончик', ['barrel2', 'cap2M', 'bush2', 'gear2', 'needle2', 'axle2', 'nose2', 'capSpring', 'capsule']],
    ['Привід', ['actBody', 'actCover', 'needleA', 'needleB', 'shaftSeal', 'pulleyShaft', 'pulley', 'thrustWasher', 'clutchRing', 'clutchSpring', 'piston', 'pistonSeal', 'cordEnd', 'capA', 'chamberGas', 'cable', 'relief', 'burstDisc', 'vent']],
    ['Газова система', ['gasBlock', 'capSeal', 'lance', 'pierceLever', 'pierceCap', 'lanceSeal', 'lanceWasher', 'regPiston', 'valveStem', 'gasTube']],
    ['Запуск і датчик натягу', ['trigger', 'pullLoop', 'backupButton', 'tripLever', 'tripShaft', 'tripSpring', 'tripSpringLeg', 'rollerB', 'rollerBushB', 'rollerPinsB']],
    ['Стрічка і тіло', ['band', 'bandRoll', 'rollerA', 'rollerBushA', 'sleeve', 'trousers', 'thigh', 'artery', 'vein', 'femur']],
  ];
  const BODY = new Set(['sleeve', 'trousers', 'thigh', 'artery', 'vein', 'femur', 'band']);

  function sc() { return SD.scenarios[S.scen]; }
  function interp(arr, t, tt) { // tt sorted
    if (t <= tt[0]) return arr[0]; const n = tt.length; if (t >= tt[n - 1]) return arr[n - 1];
    let lo = 0, hi = n - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (tt[m] <= t) lo = m; else hi = m; }
    const f = (t - tt[lo]) / (tt[hi] - tt[lo]); return arr[lo] + f * (arr[hi] - arr[lo]);
  }
  const smooth = (a, b, x) => { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };

  function stateAt(tau) {
    const s = sc(), st = { tau };
    const t = Math.max(0, tau);
    for (const k of ['th', 'X', 'Td', 'pmin', 'pmax', 'pmean', 'Pc', 'Ps', 'Tc', 'Ts', 'gas', 'dCt', 'dCc', 'L', 'Lgas']) st[k] = interp(s[k], t, s.t);
    st.theta = st.th;
    if (tau < s.t_gas) st.Pc = 1.013;
    st.flowGas = tau < s.t_gas ? 0 : Math.max(0, (interp(s.gas, t + 0.004, s.t) - interp(s.gas, Math.max(0, t - 0.004), s.t)) / 0.008);
    st.dial = S.dial;
    if (st.dial > 0 && geo) st.theta = st.th + G.releaseWind(geo.parts.filter(q => q.pawl).map(q => q.pawl), st.th, st.dial);   // drum winds on ≤ 1° while a seated pawl climbs out
    st.rd = geo ? geo.K.rd(st.theta) : 15.3;
    st.h = (G.P.gas.pullExit + 0.4) * smooth(T0, 0, tau) + 80 * smooth(0, 0.05, tau);   // handle lift: pierces the capsule, then leaves
    st.pk = G.pierceKin(st.h);
    st.trip = smooth(s.t_gas - 0.004, s.t_gas + 0.002, tau);                    // tension lever opens the feed valve
    const R0 = s.C * 1000 / TAU;
    st.R0 = R0; st.Rt = R0 - st.dCt / TAU;
    st.trT = s.wear === 'trousers' ? 1.5 * Math.max(0.2, 1 - st.dCc / (TAU * 1.8)) : 0;
    st.rem = Math.max(0, s.slack - st.L);                                       // slack still in the loop, mm
    st.Rsl = st.Rt + st.trT + G.P.sleeveT + st.rem / TAU;                        // loose loop drawn concentric
    st.Rb = st.Rsl + G.P.band.t / 2;
    st.mountX = G.mountFor(st.Rsl);
    st.occ = 0; // anatomical reference only: artery wall collapse is not solved
    st.criterionRatio = st.pmin / s.P_occ;
    st.flow = 1; // blood flow is not solved; do not animate it as a simulation result
    st.releaseGeometryOnly = st.dial > 0;
    return st;
  }

  // ------------------------------------------------------------------ build scene
  function colorOf(m) { return G.MAT[m].color; }
  function makeObj(o) { o.visible = o.visible !== false; byId[o.id] = o; objs.push(o); return o; }

  function build() {
    for (const o of objs) { gl.deleteVertexArray(o.g.vao); o.g.b.forEach(b => gl.deleteBuffer(b)); gl.deleteBuffer(o.g.ib); }
    const keepVis = {}; for (const o of objs) keepVis[o.id] = o.visible;
    objs = []; byId = {};
    G.P.R = sc().C * 1000 / TAU;
    geo = G.buildParts({});
    for (const p of geo.parts) {
      const u = p.id === 'gasTube' ? pathU(p.mesh, p.flowPath) : null;
      makeObj({ id: p.id, name: p.name, mat: p.mat, part: p, g: gpu(p.mesh, u), module: true, explode: p.explode, motion: motionOf(p), alpha: G.MAT[p.mat].alpha, color: p.id === 'pullLoop' ? [0.74, 0.13, 0.10] : undefined });
    }
    // dynamic objects
    const empty = new G.Mesh();
    makeObj({ id: 'band', name: 'Стрічка UHMWPE 90 × 0,3 мм (тиск — кольором)', mat: 'webbing', g: gpu(empty, null, true), module: true, dyn: true, explode: [0, 0, 0] });
    makeObj({ id: 'bandRoll', name: 'Рулони стрічки на барабанах', mat: 'webbing', g: gpu(empty, null, true), module: true, dyn: true, explode: [0, 0, 0] });
    makeObj({ id: 'cable', name: 'Шнур Dyneema SK99 Ø3', mat: 'cord', g: gpu(empty, null, true), module: true, dyn: true, explode: [0, 0, 0] });
    makeObj({ id: 'chamberGas', name: 'Газ у робочій камері', mat: 'ptfe', g: gpu(empty, null, true), module: true, dyn: true, explode: [0, 34, 14], alpha: 0.35, color: [0.30, 0.75, 0.95] });
    makeObj({ id: 'thigh', name: 'Стегно', mat: 'skin', g: gpu(empty, null, true), module: false, dyn: true });
    makeObj({ id: 'sleeve', name: 'Рукав із ковзним підкладом', mat: 'sleeve', g: gpu(empty, null, true), module: false, dyn: true });
    makeObj({ id: 'trousers', name: 'Штани', mat: 'trousers', g: gpu(empty, null, true), module: false, dyn: true });
    makeObj({ id: 'artery', name: 'Стегнова артерія', mat: 'skin', g: gpu(empty, null, true), module: false, dyn: true, color: [0.80, 0.10, 0.10] });
    makeObj({ id: 'vein', name: 'Стегнова вена', mat: 'skin', g: gpu(empty, null, true), module: false, dyn: true, color: [0.20, 0.25, 0.60] });
    makeObj({ id: 'femur', name: 'Стегнова кістка', mat: 'skin', g: gpu(scaleBody(G.lathe([[0, -170], [13.5, -170], [12, -60], [11.5, 0], [12.5, 80], [14, 170], [0, 170]], 32).xf(Mx.T(10, 0, -6))), null), module: false, color: [0.90, 0.87, 0.78] });
    for (const id in keepVis) if (byId[id]) byId[id].visible = keepVis[id];
    lastDyn = {};
    buildTree();
  }
  function scaleBody(m) { const k = sc().C * 1000 / TAU / 87.5; return m.xf(Mx.S(k, 1, k)); }
  function pathU(mesh, path) { // arc-length coordinate for stripes: nearest path vertex
    const L = [0]; for (let k = 1; k < path.length; k++) L.push(L[k - 1] + V.len(V.sub(path[k], path[k - 1])));
    const u = []; for (let v = 0; v < mesh.p.length; v += 3) { let best = 1e9, bu = 0; const p = [mesh.p[v], mesh.p[v + 1], mesh.p[v + 2]]; for (let k = 0; k < path.length; k++) { const d = V.len(V.sub(p, path[k])); if (d < best) { best = d; bu = L[k]; } } u.push(bu); }
    return u;
  }
  // motions: the same sets and signs as in ichk.js (interference check)
  const D1 = new Set(['barrel1', 'capM1', 'spring', 'springWasher', 'gear1', 'ratchet', 'clutchDisk']);
  const D2 = new Set(['barrel2', 'cap2M', 'bush2', 'gear2', 'needle2']);
  const PUL = new Set(['pulleyShaft', 'pulley', 'clutchRing']);
  const TRIP = new Set(['tripLever', 'tripShaft', 'rollerB', 'rollerBushB', 'rollerPinsB', 'tripSpringLeg']);
  const PIST = new Set(['piston', 'pistonSeal', 'cordEnd']);
  function motionOf(p) {
    const { xd, z1, z2 } = G.P, T = G.P.trip;
    if (D1.has(p.id)) return (st) => Mx.rotAbout([0, 1, 0], -st.theta, [xd, 0, z1]);
    if (D2.has(p.id)) return (st) => Mx.rotAbout([0, 1, 0], st.theta, [xd, 0, z2]);
    if (PUL.has(p.id)) return (st) => Mx.rotAbout([0, 1, 0], -st.X / G.P.pulley.r, [xd, 0, z1]);
    if (PIST.has(p.id)) return (st) => Mx.T(0, 0, st.X);
    if (p.pawl) return (st) => Mx.rotAbout([0, 1, 0], G.pawlAngle(p.pawl, -st.theta, st.dial), p.pivot);
    if (p.id === 'cam') return (st) => Mx.rotAbout([0, 1, 0], G.P.cam.throw * st.dial, [xd, 0, z1]);
    if (TRIP.has(p.id)) return (st) => Mx.rotAbout([0, 1, 0], T.dAng * st.trip, [T.piv[1], 0, T.piv[0]]);
    if (p.id === 'valveStem') return (st) => Mx.T(0, 0, -Math.max(0, T.valveArm * T.dAng * st.trip - 0.1));
    if (p.id === 'pierceLever') return (st) => Mx.rotAbout([0, 1, 0], -st.pk.psi, [xd, 0, z2]);
    if (p.id === 'lance') return (st) => Mx.T(0, st.pk.lance, 0);
    if (p.id === 'trigger' || p.id === 'pullLoop') return (st) => Mx.T(st.h, 0, 0);
    return null;
  }

  // ------------------------------------------------------------------ dynamic geometry
  let lastDyn = {};
  function pressureColor(p) { // mmHg -> sequential ramp (light yellow -> red -> dark red)
    const stops = [[0, [0.99, 0.94, 0.72]], [150, [0.98, 0.76, 0.38]], [300, [0.93, 0.45, 0.20]], [450, [0.78, 0.18, 0.12]], [600, [0.48, 0.05, 0.08]]];
    p = Math.max(0, Math.min(600, p)); for (let k = 1; k < stops.length; k++) if (p <= stops[k][0]) { const f = (p - stops[k - 1][0]) / (stops[k][0] - stops[k - 1][0]); return stops[k - 1][1].map((c, j) => c + f * (stops[k][1][j] - c)); }
    return stops[stops.length - 1][1];
  }
  function updateDynamic(st) {
    const s = sc(), mu = SD.design.mu, w = SD.design.w * 1000;
    // band
    const key = (v) => Math.round(v * 200) / 200;
    const kb = [key(st.theta), key(st.Rsl), key(st.trip), st.Td.toFixed(1), st.releaseGeometryOnly].join();
    if (lastDyn.band !== kb) {
      const MMHG = 133.322;
      const web = colorOf('webbing');
      const bst = { theta: st.theta, Rb: st.Rb, mountX: st.mountX, trip: st.trip };
      const bg = G.bandGeometry(geo, bst, (phi) => {
        const p = st.releaseGeometryOnly ? 0 : st.pmax * Math.exp(-mu * phi);
        const c = pressureColor(p), k = Math.min(1, p / 40); return web.map((x, j) => x + (c[j] - x) * k);
      });
      const cols = bg.cols.map(c => c || web);
      byId.band.g && upload(byId.band.g, G.strip(bg.pts, bg.ups, G.P.band.w, 0.45, cols), null, true);
      upload(byId.bandRoll.g, G.bandRoll(geo, bst), null, true);
      lastDyn.band = kb;
    }
    const kc = key(st.X);
    if (lastDyn.cable !== kc) {
      upload(byId.cable.g, G.cableGeometry(geo, st.X).mesh, null, true);
      const C = G.P.act, f0 = C.zB + 0.05, f1 = C.z0 + st.X - 0.05;
      upload(byId.chamberGas.g, G.alongZ(G.tubeY(3.3, C.ri - 0.15, f0, Math.max(f0 + 0.05, f1), 32), C.x, C.y), null, true);
      lastDyn.cable = kc;
    }
    const kt = [key(st.Rt), key(st.trT), key(st.Rsl), st.occ.toFixed(2)].join();
    if (lastDyn.thigh !== kt) {
      const R0 = st.R0, dR = R0 - st.Rt;
      upload(byId.thigh.g, G.thighMesh(R0, dR, 0, 110, 96, 110).mesh, null, true);
      upload(byId.sleeve.g, G.tubeY(st.Rsl - G.P.sleeveT, st.Rsl, -48, 48, 120), null, true);
      if (s.wear === 'trousers') upload(byId.trousers.g, G.thighMesh(R0 + 1.5, dR + (1.5 - st.trT), 0, 110, 96, 110).mesh, null, true);
      else upload(byId.trousers.g, new G.Mesh(), null, true);
      // artery and vein: flattened under the band as pressure rises
      const k = R0 / 87.5;
      const vessel = (r0, occ, xz) => {
        const prof = [[0, -170]]; for (let j = 0; j <= 80; j++) { const y = -170 + 340 * j / 80; const u = Math.abs(y) / 45; const c = u < 1 ? 1 : Math.exp(-Math.pow((u - 1) * 3, 2)); prof.push([r0 * (1 - 0.88 * occ * c), y]); } prof.push([0, 170]);
        return G.lathe(prof, 20).xf(Mx.T(xz[0] * k, 0, xz[1] * k));
      };
      const am = vessel(3.8, Math.pow(st.occ, 1.4), [-40, 40]);
      const au = []; for (let v = 0; v < am.p.length; v += 3) au.push(am.p[v + 1]);
      upload(byId.artery.g, am, au, true);
      upload(byId.vein.g, vessel(4.6, Math.min(1, st.occ * 2.2), [-48, 30]), null, true);
      lastDyn.thigh = kt;
    }
  }

  // ------------------------------------------------------------------ camera
  const cam = { az: 0.95, el: 0.38, d: 420, tgt: [110, 0, 0] };
  const CAMS = {
    iso: () => ({ az: 0.95, el: 0.38, d: 420, tgt: [S.mount + 20, 6, 0] }),
    top: () => ({ az: PI / 2, el: 0.02, d: 330, tgt: [S.mount + 22, 8, 0] }),
    side: () => ({ az: PI / 2 + 0.0001, el: 1.45, d: 300, tgt: [S.mount + 16, 0, 0] }),
    ratchet: () => ({ az: PI / 2 + 0.0001, el: -1.42, d: 150, tgt: [S.mount + 21, -53, -14] }),
    valve: () => ({ az: PI / 2 + 0.0001, el: -1.42, d: 150, tgt: [S.mount + 14, -56, 24] }),
    gas: () => ({ az: 0.0001, el: 0.02, d: 250, tgt: [S.mount + 22, -12, 18] }),
    cyl: () => ({ az: -PI / 2 + 0.0001, el: 1.42, d: 200, tgt: [S.mount + 23, 72, 6] }),
    drum: () => ({ az: -PI / 2 + 0.0001, el: 1.42, d: 200, tgt: [S.mount + 12, 0, 0] }),
    leg: () => ({ az: 0.6, el: 0.22, d: 760, tgt: [20, 0, 0] }),
  };
  S.mount = 92;
  let camAnim = null;
  function setCam(name) { const to = CAMS[name](); cam.follow = name !== 'leg'; camAnim = { from: { ...cam, tgt: [...cam.tgt] }, to, t0: performance.now() }; }
  function viewProj() {
    const ce = Math.cos(cam.el), eye = [cam.tgt[0] + cam.d * ce * Math.sin(cam.az), cam.tgt[1] + cam.d * Math.sin(cam.el), cam.tgt[2] + cam.d * ce * Math.cos(cam.az)];
    const f = V.norm(V.sub(cam.tgt, eye)), r = V.norm(V.cross(f, [0, 1, 0])), u = V.cross(r, f);
    const view = [r[0], u[0], -f[0], 0, r[1], u[1], -f[1], 0, r[2], u[2], -f[2], 0, -V.dot(r, eye), -V.dot(u, eye), V.dot(f, eye), 1];
    const asp = canvas.width / canvas.height, fov = 32 * PI / 180, n = 2, fa = 6000, t = 1 / Math.tan(fov / 2);
    const proj = [t / asp, 0, 0, 0, 0, t, 0, 0, 0, 0, (fa + n) / (n - fa), -1, 0, 0, 2 * fa * n / (n - fa), 0];
    return { vp: Mx.mul(proj, view), eye };
  }
  // pointer controls
  let drag = null; const ptrs = new Map();
  canvas.addEventListener('pointerdown', (e) => { canvas.setPointerCapture(e.pointerId); ptrs.set(e.pointerId, [e.clientX, e.clientY]); drag = { x: e.clientX, y: e.clientY, x0: e.clientX, y0: e.clientY, pan: e.button === 2 || e.shiftKey, moved: false }; camAnim = null; });
  canvas.addEventListener('pointermove', (e) => {
    if (!drag) return; const prev = ptrs.get(e.pointerId); ptrs.set(e.pointerId, [e.clientX, e.clientY]);
    if (ptrs.size === 2) { const [a, b] = [...ptrs.values()]; const d = Math.hypot(a[0] - b[0], a[1] - b[1]); if (drag.pd) cam.d *= drag.pd / d; drag.pd = d; drag.moved = true; return; }
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag.x = e.clientX; drag.y = e.clientY;
    if (Math.hypot(e.clientX - drag.x0, e.clientY - drag.y0) > 4) drag.moved = true;
    if (drag.pan) { const k = cam.d / canvas.clientHeight * 0.6; const ce = Math.cos(cam.el); const right = [Math.cos(cam.az), 0, -Math.sin(cam.az)], up = [-Math.sin(cam.el) * Math.sin(cam.az), ce, -Math.sin(cam.el) * Math.cos(cam.az)]; cam.tgt = V.add(cam.tgt, V.add(V.mul(right, -dx * k), V.mul(up, dy * k))); }
    else { cam.az -= dx * 0.008; cam.el = Math.max(-1.5, Math.min(1.5, cam.el + dy * 0.008)); }
  });
  canvas.addEventListener('pointerup', (e) => { ptrs.delete(e.pointerId); if (drag && !drag.moved) pick(e); if (ptrs.size === 0) drag = null; });
  canvas.addEventListener('contextmenu', (e) => e.preventDefault());
  canvas.addEventListener('wheel', (e) => { e.preventDefault(); cam.d *= Math.exp(e.deltaY * 0.0012); cam.d = Math.max(60, Math.min(3000, cam.d)); camAnim = null; }, { passive: false });

  // ------------------------------------------------------------------ draw
  let fbo = null, fboTex = null, fboDepth = null, fboSize = [0, 0];
  function resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2), w = Math.max(1, Math.round(canvas.clientWidth * dpr)), h = Math.max(1, Math.round(canvas.clientHeight * dpr));
    if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  }
  function modelMatrix(o, st) {
    const W = Mx.T(o.module ? st.mountX : 0, 0, 0);
    let M = Mx.I();
    if (o.motion) M = o.motion(st);
    if (o.module && S.mode === 'explode') { const e = o.explode || [0, 0, 0]; M = Mx.mul(Mx.T(e[0] * S.expK * 1.6, e[1] * S.expK * 1.6, e[2] * S.expK * 1.6), M); }
    return Mx.mul(W, M);
  }
  function visibleNow(o) {
    if (!o.visible) return false;
    if (S.mode === 'explode' && (o.id === 'band' || o.id === 'cable' || !o.module)) return false;
    if (!S.body && !o.module) return false;
    if (S.mode !== 'full' && S.mode !== 'cut' && (o.id === 'cover' || o.id === 'screws' || o.id === 'buttonGuard')) return false;
    if (S.mode === 'open' && o.id === 'pullLoop') return false;
    if (o.id === 'chamberGas' && (S.mode === 'full')) return false;
    if (o.id === 'trousers' && sc().wear !== 'trousers') return false;
    if ((o.id === 'trigger' || o.id === 'pullLoop') && S.tau > 0.045 && S.mode !== 'explode') return false;
    if ((o.id === 'artery' || o.id === 'vein' || o.id === 'femur') && !S.skin) return false;
    return true;
  }
  function drawScene(st, pickMode) {
    const { vp, eye } = viewProj();
    gl.uniformMatrix4fv(U.uVP, false, vp); gl.uniform3fv(U.uCam, eye);
    gl.uniform1i(U.uPick, pickMode ? 1 : 0);
    const clipPlane = S.cutT === 'ratchet' ? [0, -1, 0, -53.0] : S.cutT === 'valve' ? [0, -1, 0, G.P.valve.y] : S.cutT === 'gas' ? [0, 0, 1, -G.P.z2] : S.cutT === 'cyl' ? [0, 1, 0, -G.P.act.y] : S.cutT === 'drum' ? [0, 1, 0, 0] : [1, 0, 0, -(st.mountX + S.cutH)];
    const list = objs.filter(visibleNow);
    const trans = [];
    for (const o of list) {
      let alpha = o.alpha || 1;
      if (o.id === 'thigh' && S.skin) alpha = 0.28;
      if (o.id === 'trousers' && S.skin) alpha = 0.3;
      if (o.id === 'sleeve' && S.skin) alpha = 0.5;
      if (alpha < 1 && !pickMode) { trans.push([o, alpha]); continue; }
      drawObj(o, st, 1, pickMode, clipPlane);
    }
    if (!pickMode && trans.length) {
      gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA); gl.depthMask(false);
      for (const [o, a] of trans) drawObj(o, st, a, false, clipPlane);
      gl.depthMask(true); gl.disable(gl.BLEND);
    }
  }
  function drawObj(o, st, alpha, pickMode, clipPlane) {
    const m = G.MAT[o.mat];
    gl.uniformMatrix4fv(U.uModel, false, modelMatrix(o, st));
    gl.uniform3fv(U.uColor, o.color || m.color);
    gl.uniform1i(U.uUseVCol, o.g.hasCol ? 1 : 0);
    gl.uniform1f(U.uMetal, m.metal); gl.uniform1f(U.uRough, m.rough); gl.uniform1f(U.uAlpha, alpha);
    gl.uniform1f(U.uHi, S.sel === o.id ? 1 : 0);
    const clip = S.mode === 'cut' && o.id !== 'trigger' && o.id !== 'pullLoop' && (o.module || S.cutT !== 'h');
    gl.uniform1i(U.uClipOn, clip ? 1 : 0); gl.uniform4fv(U.uClip, clipPlane);
    let stripe = [0, 0, 0, 0];
    if (o.id === 'artery') stripe = [0, 0, 0, 0]; // no simulated blood-flow animation
    if (o.id === 'gasTube') stripe = [1, S.gphase, 0.35, Math.min(1, st.flowGas * 6)];
    gl.uniform4fv(U.uStripe, stripe);
    if (pickMode) { const id = objs.indexOf(o) + 1; gl.uniform3f(U.uPickCol, (id & 255) / 255, ((id >> 8) & 255) / 255, 0); }
    gl.bindVertexArray(o.g.vao); gl.drawElements(gl.TRIANGLES, o.g.count, gl.UNSIGNED_INT, 0);
  }
  function pick(e) {
    resize();
    const w = canvas.width, h = canvas.height;
    if (!fbo || fboSize[0] !== w || fboSize[1] !== h) {
      if (fbo) { gl.deleteFramebuffer(fbo); gl.deleteTexture(fboTex); gl.deleteRenderbuffer(fboDepth); }
      fbo = gl.createFramebuffer(); fboTex = gl.createTexture(); gl.bindTexture(gl.TEXTURE_2D, fboTex); gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
      fboDepth = gl.createRenderbuffer(); gl.bindRenderbuffer(gl.RENDERBUFFER, fboDepth); gl.renderbufferStorage(gl.RENDERBUFFER, gl.DEPTH_COMPONENT24, w, h);
      gl.bindFramebuffer(gl.FRAMEBUFFER, fbo); gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, fboTex, 0); gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.RENDERBUFFER, fboDepth);
      fboSize = [w, h];
    }
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo); gl.viewport(0, 0, w, h); gl.clearColor(0, 0, 0, 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    drawScene(curState, true);
    const r = canvas.getBoundingClientRect(), px = new Uint8Array(4);
    gl.readPixels(Math.round((e.clientX - r.left) * w / r.width), Math.round(h - (e.clientY - r.top) * h / r.height), 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, px);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    const id = px[0] + (px[1] << 8) - 1;
    select(id >= 0 && objs[id] ? objs[id].id : null);
  }

  // ------------------------------------------------------------------ UI: tree, info, BOM
  function massOf(o) { if (!o.part) return null; if (o.part.massOverride) return o.part.massOverride; const m = G.MAT[o.mat]; return o.part.mesh.volume() / 1000 * m.rho; }
  function buildTree() {
    const t = $('tree'); t.innerHTML = '';
    for (const [gname, ids] of groupsUI) {
      const g = document.createElement('div'); g.className = 'grp';
      const h = document.createElement('div'); h.className = 'h';
      const cb = document.createElement('input'); cb.type = 'checkbox'; cb.checked = ids.some(id => byId[id] && byId[id].visible);
      cb.onchange = () => { ids.forEach(id => { if (byId[id]) byId[id].visible = cb.checked; }); buildTree(); };
      h.append(cb, document.createTextNode(gname)); g.append(h);
      for (const id of ids) {
        const o = byId[id]; if (!o) continue;
        const row = document.createElement('div'); row.className = 'pi' + (S.sel === id ? ' sel' : '');
        const c = document.createElement('input'); c.type = 'checkbox'; c.checked = o.visible; c.onclick = (e) => { e.stopPropagation(); o.visible = c.checked; };
        const sw = document.createElement('span'); sw.className = 'sw'; const col = o.color || G.MAT[o.mat].color; sw.style.background = `rgb(${col.map(x => Math.round(x * 255)).join(',')})`;
        const n = document.createElement('span'); n.className = 'n'; n.textContent = o.name;
        row.append(c, sw, n); row.onclick = () => select(id); g.append(row);
      }
      t.append(g);
    }
  }
  const DYN_INFO = {
    band: ['Геометрія v5.1: стрічка 90 × 0,3 мм, два кінці намотуються на окремі барабани. Конкретний матеріал, шви, клей і тертя ще потребують вимірювань. Колір відображає розрахунковий контактний тиск.', 'webbing'],
    bandRoll: ['Кожен барабан намотує свій кінець стрічки. Радіус намотування визначається прийнятою товщиною 0,3 мм; заміна стрічки змінить кінематику.', 'webbing'],
    cable: ['Шнур Ø3 мм у геометрії v5.1. Артикул і міцність кінцевих закріплень не підтверджені; оцінку запасу попереднього пакета не перенесено як результат перевірки.', 'cord'],
    chamberGas: ['Газова камера в розрахунку має зосереджені параметри. Регулятор і отвір подачі ідеалізовані; реальна витрата потребує паспорта та вимірювань.', null],
    thigh: ['Схематичне стегно. Зміна радіуса походить із припущеної емпіричної залежності стискання. Це не анатомічна скінченноелементна модель.', null],
    sleeve: ['Ковзний рукав із геометрії v5.1; матеріал і коефіцієнт тертя належить підтвердити.', 'sleeve'],
    trousers: ['Прийнятий шар одягу для порівняння сценаріїв. Характеристика стискання не виміряна.', 'trousers'],
    artery: ['Схематичне положення артерії. Деформація її стінки, кровотік та кровотеча не розраховуються.', null],
    vein: ['Схематичне положення вени. Венозний кровотік не розраховується.', null],
    femur: ['Схематична стегнова кістка.', null],
  };
  function select(id) {
    S.sel = id; buildTree();
    const inf = $('info'); if (!id) { inf.innerHTML = '<p class="meta">Клікніть по деталі в моделі або в списку ліворуч.</p>'; return; }
    const o = byId[id]; const p = o.part; const mat = G.MAT[o.mat];
    let h = `<h2>${o.name}</h2>`;
    if (p) {
      const ms = massOf(o);
      h += `<p class="meta">Дані деталі з вихідної геометрії v5.1; це не підтвердження міцності або готовності до виготовлення.</p><p>${p.func || ''}</p><div class="kv"><div>Матеріал</div><div>${mat.name}</div><div>Розміри</div><div>${p.dims || '—'}</div><div>Кількість</div><div>${p.qty}</div>` + (o.mat !== 'eva' ? `<div>Маса (з моделі)</div><div>${ms < 1 ? ms.toFixed(2) : ms.toFixed(1)} г</div>` : '') + `</div>`;
    } else if (DYN_INFO[id]) { h += `<p>${DYN_INFO[id][0]}</p>`; if (DYN_INFO[id][1]) h += `<div class="kv"><div>Матеріал</div><div>${G.MAT[DYN_INFO[id][1]].name}</div></div>`; }
    inf.innerHTML = h;
  }
  // ------------------------------------------------------------------ charts
  const charts = [
    { title: 'Тиск під стрічкою, мм рт. ст.', keys: ['pmin', 'pmax'], names: ['найслабша точка', 'біля модуля'], ref: (s) => [[s.P_occ, 'довідковий поріг ' + Math.round(s.P_occ)], [s.P_occ * 1.3, 'ціль ×1,3']] },
    { title: 'Вибрано стрічки, мм', keys: ['L', 'Lgas'], names: ['усього', 'газом'], ref: (s) => [[s.slack, 'слабина ' + Math.round(s.slack) + ' мм']] },
    { title: 'Тиск у камері циліндра, бар', keys: ['Pc'], names: ['камера'], ref: (s) => [[s.P_set - 1.013, 'уставка ' + s.P_set.toFixed(1) + ' бар (абс.)']], conv: (v) => v - 1.013 },
    { title: 'Хід поршня, мм', keys: ['X'], names: ['хід'], ref: () => [[G.P.travel, `упор ${G.P.travel} мм`]] },
  ];
  function drawCharts(tau) {
    $('charts').style.visibility = S.dial > 0 ? 'hidden' : 'visible';
    const s = sc(), tEnd = s.t[s.t.length - 1];
    charts.forEach((c, k) => {
      const cv = $('c' + k), dpr = Math.min(2, window.devicePixelRatio || 1), W = cv.clientWidth, H = cv.clientHeight;
      const width = Math.round(W * dpr), height = Math.round(H * dpr);
      if (cv.width !== width) cv.width = width;
      if (cv.height !== height) cv.height = height;
      const x = cv.getContext('2d'); x.setTransform(dpr, 0, 0, dpr, 0, 0); x.clearRect(0, 0, W, H);
      const L = 40, R = 10, Tp = 24, B = 20, pw = W - L - R, ph = H - Tp - B;
      const conv = c.conv || ((v) => v);
      let ymax = 0; for (const key of c.keys) for (const v of s[key]) ymax = Math.max(ymax, conv(v));
      for (const [v] of c.ref(s)) ymax = Math.max(ymax, v);
      ymax *= 1.08; const nice = niceMax(ymax); ymax = nice;
      const X = (t) => L + pw * (t - T0) / (tEnd - T0), Y = (v) => Tp + ph * (1 - v / ymax);
      x.font = '600 11px system-ui'; x.fillStyle = '#16181b'; x.fillText(c.title, L - 30, 14);
      x.strokeStyle = '#e3e6e9'; x.lineWidth = 1; x.font = '10px system-ui'; x.fillStyle = '#7a8088';
      for (let g = 0; g <= 4; g++) { const v = ymax * g / 4, y = Y(v); x.beginPath(); x.moveTo(L, y); x.lineTo(L + pw, y); x.stroke(); x.fillText(fmt(v), 2, y + 3); }
      for (const tt of [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]) if (tt <= tEnd) { x.fillText(tt.toFixed(2) + ' с', X(tt) - 12, H - 5); }
      // reference lines
      for (const [v, lab] of c.ref(s)) { x.setLineDash([4, 4]); x.strokeStyle = '#9a9fa6'; x.beginPath(); x.moveTo(L, Y(v)); x.lineTo(L + pw, Y(v)); x.stroke(); x.setLineDash([]); x.fillStyle = '#6b7178'; x.fillText(lab, L + pw - x.measureText(lab).width - 2, Y(v) - 3); }
      // series: first key accent, second grey
      c.keys.forEach((key, j) => {
        x.strokeStyle = j === 0 ? '#2a78d6' : '#a3a8ae'; x.lineWidth = 2; x.beginPath();
        x.moveTo(X(T0), Y(conv(key === 'Pc' ? 1.013 : s[key][0]))); x.lineTo(X(0), Y(conv(key === 'Pc' ? 1.013 : s[key][0])));
        for (let i = 0; i < s.t.length; i++) x.lineTo(X(s.t[i]), Y(conv(s[key][i])));
        x.stroke();
        const lv = conv(s[key][s[key].length - 1]); x.fillStyle = j === 0 ? '#1d5fae' : '#6b7178'; x.font = '10px system-ui';
        if (c.keys.length > 1) x.fillText(c.names[j], X(tEnd) - x.measureText(c.names[j]).width - 2, Y(lv) + (j ? 12 : -5));
      });
      // cursor
      x.strokeStyle = '#16181b'; x.lineWidth = 1; x.beginPath(); x.moveTo(X(tau), Tp); x.lineTo(X(tau), Tp + ph); x.stroke();
      if (k === 0 && s.t_occ) { x.fillStyle = '#c2412d'; x.beginPath(); x.arc(X(s.t_occ), Y(s.P_occ), 3.5, 0, TAU); x.fill(); }
    });
  }
  function niceMax(v) { const p = Math.pow(10, Math.floor(Math.log10(v))); for (const m of [1, 1.2, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10]) if (m * p >= v) return m * p; return v; }
  function fmt(v) { return v >= 100 ? Math.round(v) : v >= 10 ? v.toFixed(0) : v.toFixed(1); }

  // ------------------------------------------------------------------ readouts
  function readouts(st) {
    const s = sc(), t = st.tau, tEnd = s.t[s.t.length - 1];
    let ph = `Носіння: стрічка вільна (слабина ${Math.round(s.slack)} мм), барабан тримає штифт A чеки`;
    if (t >= T0) ph = 'Ривок чеки: штифт A звільняє барабан, важіль проколює балончик — газ стає перед клапаном';
    if (t >= 0) ph = 'Пружина обертає обидва барабани: кожен намотує свій кінець';
    if (t >= s.t_gas) ph = 'Стрічка лягла (≈ 5 Н) → ролик Б відкрив клапан подачі, газ іде в циліндр';
    if (t >= s.t_gas + 0.012) ph = 'Газ тягне шнур: шків крутить барабан 1, шестерні — барабан 2';
    if (Number.isFinite(s.t_occ) && t >= s.t_occ) ph = 'Досягнуто розрахункового критерію тиску: p_min ≥ оцінки порога';
    if (t > Math.min(tEnd - 0.02, (s.t_occ || 0) + 0.15)) ph = 'Утримання: газ тримає силу, храповик — довжину';
    if (st.releaseGeometryOnly) ph = 'Геометрія розблокування: фізичні показники під час послаблення не розраховані';
    $('phase').textContent = ph;
    const kv = [
      ['Час від ривка чеки', (Math.max(0, t) * 1000).toFixed(0) + ' мс'],
      ['Слабина в петлі', st.rem.toFixed(0) + ' мм'], ['Оберти барабанів', (st.theta / TAU).toFixed(2)],
      ['Радіус рулону', st.rd.toFixed(2) + ' мм'], ['Хід поршня', st.X.toFixed(1) + ' мм'], ['Вибрано стрічки', st.L.toFixed(0) + ' мм'],
      ['Тиск у балончику', st.Ps.toFixed(0) + ' бар'], ['Тиск у камері (надл.)', Math.max(0, st.Pc - 1.013).toFixed(1) + ' бар'], ['Газу витрачено', (st.gas * 100).toFixed(0) + ' %'],
      ['Натяг кожного кінця', st.Td.toFixed(0) + ' Н'], ['p_min / p_max', Math.round(st.pmin) + ' / ' + Math.round(st.pmax) + ' мм рт. ст.'],
      ['Довідковий поріг (не виміряний)', Math.round(s.P_occ) + ' мм рт. ст.'], ['Уставка регулятора', s.P_set.toFixed(1) + ' бар'],
    ];
    $('read').innerHTML = st.releaseGeometryOnly ? '<div>Режим</div><div>Лише геометрія</div>' : kv.map(([a, b]) => `<div>${a}</div><div>${b}</div>`).join('');
    $('flow').firstChild.style.width = (st.releaseGeometryOnly ? 0 : Math.min(100, Math.max(0, st.criterionRatio * 100))) + '%';
    $('flow').title = st.releaseGeometryOnly ? 'Розрахунок послаблення відсутній' : 'p_min / прийнятий поріг = ' + st.criterionRatio.toFixed(2) + '; це не вимірювання кровотоку';
    $('clock').textContent = `t = ${(t * 1000).toFixed(0)} мс  (реально ×${S.slow} повільніше)`;
  }

  // ------------------------------------------------------------------ loop & controls
  let curState = null, lastT = performance.now();
  S.phase = 0; S.gphase = 0;
  function tauRange() { const s = sc(); return [T0 - 0.04, s.t[s.t.length - 1] + 0.05]; }
  function frame(now) {
    const dt = Math.min(0.05, (now - lastT) / 1000); lastT = now;
    const [a, b] = tauRange();
    if (S.playing) { S.tau += dt / S.slow; if (S.tau >= b) { S.tau = b; S.playing = false; $('bPlay').textContent = '▶ Спрацювання'; } $('scrub').value = Math.round(1000 * (S.tau - a) / (b - a)); }
    const st = stateAt(S.tau); curState = st;
    if (cam.follow !== false) { const dm = st.mountX - S.mount; cam.tgt[0] += dm; if (camAnim) { camAnim.from.tgt[0] += dm; camAnim.to.tgt[0] += dm; } }   // close-ups follow the module while the slack is taken up
    S.mount = st.mountX;
    S.phase += dt; S.gphase += dt * 40 / S.slow * 10;
    S.dial += Math.max(-dt * 2.5, Math.min(dt * 2.5, S.dialT - S.dial));
    if (camAnim) { const k = Math.min(1, (now - camAnim.t0) / 600), e = k * k * (3 - 2 * k); for (const f of ['az', 'el', 'd']) cam[f] = camAnim.from[f] + (camAnim.to[f] - camAnim.from[f]) * e; cam.tgt = V.lerp(camAnim.from.tgt, camAnim.to.tgt, e); if (k >= 1) camAnim = null; }
    updateDynamic(st);
    resize();
    gl.bindFramebuffer(gl.FRAMEBUFFER, null); gl.viewport(0, 0, canvas.width, canvas.height);
    gl.clearColor(0.925, 0.933, 0.941, 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST); gl.disable(gl.CULL_FACE);
    drawScene(st, false);
    if (!frame.k || frame.k++ % 3 === 0) { drawCharts(S.tau); readouts(st); }
    frame.k = (frame.k || 0) + 1;
    requestAnimationFrame(frame);
  }
  $('bPlay').onclick = () => { const [a, b] = tauRange(); if (S.tau >= b - 1e-6) S.tau = a; S.playing = !S.playing; $('bPlay').textContent = S.playing ? '❚❚ Пауза' : '▶ Спрацювання'; };
  $('bReset').onclick = () => { S.tau = tauRange()[0]; S.playing = false; $('bPlay').textContent = '▶ Спрацювання'; $('scrub').value = 0; };
  $('scrub').oninput = (e) => { const [a, b] = tauRange(); S.tau = a + (b - a) * e.target.value / 1000; S.playing = false; $('bPlay').textContent = '▶ Спрацювання'; };
  $('slow').onchange = (e) => { S.slow = +e.target.value; };
  for (const b of document.querySelectorAll('[data-mode]')) b.onclick = () => {
    S.mode = b.dataset.mode; document.querySelectorAll('[data-mode]').forEach(x => x.classList.toggle('on', x === b));
    $('cutBox').style.display = S.mode === 'cut' ? 'flex' : 'none'; $('expBox').style.display = S.mode === 'explode' ? 'flex' : 'none';
    if (S.mode === 'open') setCam('top');
    if (S.mode === 'cut') setCam(S.cutT === 'h' ? 'top' : S.cutT);
    if (S.mode === 'explode') { setCam('iso'); camAnim.to.d = 640; }
  };
  $('cutH').oninput = (e) => { S.cutH = +e.target.value; $('cutV').textContent = S.cutH + ' мм'; };
  $('cutT').onchange = (e) => { S.cutT = e.target.value; $('cutHBox').style.display = S.cutT === 'h' ? '' : 'none'; setCam(S.cutT === 'h' ? 'top' : S.cutT); };
  $('expK').oninput = (e) => { S.expK = +e.target.value; };
  $('bSkin').onclick = () => { S.skin = !S.skin; $('bSkin').classList.toggle('on', S.skin); };
  $('bBody').onclick = () => { S.body = !S.body; $('bBody').classList.toggle('on', S.body); };
  for (const b of document.querySelectorAll('[data-cam]')) b.onclick = () => setCam(b.dataset.cam);
  for (const b of document.querySelectorAll('[data-rk]')) b.onclick = () => { S.dialT = +b.dataset.rk; document.querySelectorAll('[data-rk]').forEach(x => x.classList.toggle('on', x === b)); select('cam'); };
  // scenarios
  const scen = $('scen');
  SD.scenarios.forEach((s, k) => { const o = document.createElement('option'); o.value = k; o.textContent = `${s.size ? 'Розмір ' + s.size + ' · ' : ''}стегно ${Math.round(s.C * 100)} см · слабина ${Math.round(s.slack)} мм · ${s.T > 0 ? '+' : ''}${Math.round(s.T)} °C · ${s.wear === 'skin' ? 'на шкірі' : 'поверх штанів'}`; scen.append(o); });
  scen.onchange = () => { S.scen = +scen.value; build(); S.tau = tauRange()[0]; };
  // legend
  { const st = []; for (let p = 0; p <= 600; p += 50) { const c = pressureColor(p).map(v => Math.round(v * 255)); st.push(`rgb(${c}) ${p / 6}%`); } $('lgBar').style.background = `linear-gradient(90deg, ${st.join(',')})`; }
  // hash -> initial state (for screenshots / links): #t-300 etc.
  window.IMPULS_VIEW = { S, cam, setCam, select, build, stateAt, setTau: (t) => { S.tau = t; }, setMode: (m) => document.querySelector(`[data-mode="${m}"]`).click(), setCut: (t) => { const e = $('cutT'); e.value = t; e.onchange({ target: e }); }, setDial: (d) => { S.dialT = d; S.dial = d; } };

  build();
  const st0 = stateAt(S.tau); S.mount = st0.mountX; Object.assign(cam, CAMS.iso());
  requestAnimationFrame(frame);
})();
