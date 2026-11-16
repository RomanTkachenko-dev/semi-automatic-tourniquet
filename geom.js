/* IMPULS v5.1 module (hold-down exit rollers, two geared drums winding from their outer sides, gas cylinder pulling drum 1 through a cord that never leaves the gas volume) - parametric geometry, no dependencies.
 * Units: mm. Module-local frame: X = out of the thigh (up), Y = along the thigh,
 * Z = tangential (anterior +Z). World frame = module frame shifted by the mount radius.
 * Works in the browser (window.IMPULS_GEOM) and in Node (module.exports).
 */
(function (root) {
  'use strict';
  const PI = Math.PI, TAU = 2 * Math.PI;

  // ------------------------------------------------------------------ vec / mat
  const V = {
    add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
    sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
    mul: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
    dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
    cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
    len: (a) => Math.hypot(a[0], a[1], a[2]),
    norm: (a) => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; },
    lerp: (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t],
  };
  // column-major 4x4
  const Mx = {
    I: () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    mul(a, b) {
      const o = new Array(16);
      for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
        let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = s;
      }
      return o;
    },
    T: (x, y, z) => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1],
    S: (x, y, z) => [x, 0, 0, 0, 0, y, 0, 0, 0, 0, z, 0, 0, 0, 0, 1],
    RX(a) { const c = Math.cos(a), s = Math.sin(a); return [1, 0, 0, 0, 0, c, s, 0, 0, -s, c, 0, 0, 0, 0, 1]; },
    RY(a) { const c = Math.cos(a), s = Math.sin(a); return [c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, 0, 0, 0, 1]; },
    RZ(a) { const c = Math.cos(a), s = Math.sin(a); return [c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]; },
    chain(...ms) { return ms.reduce((a, b) => Mx.mul(a, b), Mx.I()); },
    apply(m, p) { return [m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12], m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13], m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]]; },
    applyN(m, n) { return V.norm([m[0] * n[0] + m[4] * n[1] + m[8] * n[2], m[1] * n[0] + m[5] * n[1] + m[9] * n[2], m[2] * n[0] + m[6] * n[1] + m[10] * n[2]]); },
    // rotation about axis through point
    rotAbout(axis, ang, pt) {
      const [x, y, z] = V.norm(axis), c = Math.cos(ang), s = Math.sin(ang), t = 1 - c;
      const R = [t * x * x + c, t * x * y + s * z, t * x * z - s * y, 0, t * x * y - s * z, t * y * y + c, t * y * z + s * x, 0,
        t * x * z + s * y, t * y * z - s * x, t * z * z + c, 0, 0, 0, 0, 1];
      return Mx.chain(Mx.T(pt[0], pt[1], pt[2]), R, Mx.T(-pt[0], -pt[1], -pt[2]));
    },
  };

  // ------------------------------------------------------------------ mesh builder
  class Mesh {
    constructor() { this.p = []; this.n = []; this.i = []; this.c = null; }
    v(p, n, col) { this.p.push(p[0], p[1], p[2]); this.n.push(n[0], n[1], n[2]); if (col) { (this.c || (this.c = [])).push(col[0], col[1], col[2]); } return this.p.length / 3 - 1; }
    tri(a, b, c) {   // auto-orient so the geometric normal agrees with the vertex normals
      const P = this.p, N = this.n;
      const ax = P[3 * a], ay = P[3 * a + 1], az = P[3 * a + 2];
      const e1 = [P[3 * b] - ax, P[3 * b + 1] - ay, P[3 * b + 2] - az], e2 = [P[3 * c] - ax, P[3 * c + 1] - ay, P[3 * c + 2] - az];
      const g = V.cross(e1, e2);
      const nn = [N[3 * a] + N[3 * b] + N[3 * c], N[3 * a + 1] + N[3 * b + 1] + N[3 * c + 1], N[3 * a + 2] + N[3 * b + 2] + N[3 * c + 2]];
      if (V.dot(g, nn) < 0) this.i.push(a, c, b); else this.i.push(a, b, c);
    }
    quad(a, b, c, d) { this.tri(a, b, c); this.tri(a, c, d); }
    append(m, mat) {
      const off = this.p.length / 3;
      for (let k = 0; k < m.p.length; k += 3) {
        let p = [m.p[k], m.p[k + 1], m.p[k + 2]], n = [m.n[k], m.n[k + 1], m.n[k + 2]];
        if (mat) { p = Mx.apply(mat, p); n = Mx.applyN(mat, n); }
        this.p.push(p[0], p[1], p[2]); this.n.push(n[0], n[1], n[2]);
      }
      if (m.c) { this.c = this.c || []; for (const x of m.c) this.c.push(x); }
      for (const x of m.i) this.i.push(x + off);
      return this;
    }
    xf(mat) { const m = new Mesh(); m.append(this, mat); return m; }
    volume() { // signed volume (closed meshes), mm3
      let v = 0; const P = this.p, I = this.i;
      for (let k = 0; k < I.length; k += 3) {
        const a = 3 * I[k], b = 3 * I[k + 1], c = 3 * I[k + 2];
        v += (P[a] * (P[b + 1] * P[c + 2] - P[b + 2] * P[c + 1]) - P[a + 1] * (P[b] * P[c + 2] - P[b + 2] * P[c]) + P[a + 2] * (P[b] * P[c + 1] - P[b + 1] * P[c])) / 6;
      }
      return v;
    }
    bbox() {
      const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
      for (let k = 0; k < this.p.length; k += 3) for (let j = 0; j < 3; j++) { lo[j] = Math.min(lo[j], this.p[k + j]); hi[j] = Math.max(hi[j], this.p[k + j]); }
      return [lo, hi];
    }
  }
  const merge = (...ms) => { const o = new Mesh(); for (const m of ms) if (m) o.append(Array.isArray(m) ? m[0] : m, Array.isArray(m) ? m[1] : null); return o; };

  // ------------------------------------------------------------------ primitives
  // Revolve a closed (r, y) profile (CCW: out along the bottom, up the outside, in along the top) about +Y.
  function lathe(prof, seg = 48, a0 = 0, sweep = TAU) {
    const m = new Mesh(), full = Math.abs(sweep - TAU) < 1e-9, ns = full ? seg : seg + 1;
    for (let k = 0; k < prof.length - 1; k++) {
      const [r1, y1] = prof[k], [r2, y2] = prof[k + 1];
      if (r1 === r2 && y1 === y2) continue;
      const dr = r2 - r1, dy = y2 - y1, L = Math.hypot(dr, dy), nr = dy / L, ny = -dr / L;
      const A = [], B = [];
      for (let j = 0; j < ns; j++) {
        const a = a0 + sweep * j / seg, ca = Math.cos(a), sa = Math.sin(a);
        // angle measured from +Z toward +X (matches drum angle convention)
        A.push(m.v([r1 * sa, y1, r1 * ca], [nr * sa, ny, nr * ca]));
        B.push(m.v([r2 * sa, y2, r2 * ca], [nr * sa, ny, nr * ca]));
      }
      for (let j = 0; j < (full ? seg : seg); j++) {
        const j2 = full ? (j + 1) % seg : j + 1;
        if (r1 < 1e-9) m.tri(A[j], B[j2], B[j]); else if (r2 < 1e-9) m.tri(A[j], A[j2], B[j]); else m.quad(A[j], A[j2], B[j2], B[j]);
      }
    }
    if (!full) { // close the cut faces of a partial revolve
      for (const a of [a0, a0 + sweep]) {
        const pts = prof.map(([r, y]) => [r * Math.sin(a), y, r * Math.cos(a)]);
        const nrm = V.norm(V.cross([Math.sin(a), 0, Math.cos(a)], [0, 1, 0]));
        const n2 = a === a0 ? V.mul(nrm, -1) : nrm;
        const idx = pts.map(p => m.v(p, n2));
        const tris = earclip(prof.map(([r, y]) => [r, y]));
        for (const [a1, b1, c1] of tris) m.tri(idx[a1], idx[b1], idx[c1]);
      }
    }
    return m;
  }
  const cylY = (r, y0, y1, seg = 40) => lathe([[0, y0], [r, y0], [r, y1], [0, y1]], seg);
  const tubeY = (ri, ro, y0, y1, seg = 40) => lathe([[ri, y0], [ro, y0], [ro, y1], [ri, y1], [ri, y0]], seg);

  function box(x0, x1, y0, y1, z0, z1) {
    const m = new Mesh();
    const F = [[[1, 0, 0], [[x1, y0, z0], [x1, y1, z0], [x1, y1, z1], [x1, y0, z1]]], [[-1, 0, 0], [[x0, y0, z0], [x0, y0, z1], [x0, y1, z1], [x0, y1, z0]]],
      [[0, 1, 0], [[x0, y1, z0], [x0, y1, z1], [x1, y1, z1], [x1, y1, z0]]], [[0, -1, 0], [[x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1]]],
      [[0, 0, 1], [[x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]]], [[0, 0, -1], [[x0, y0, z0], [x0, y1, z0], [x1, y1, z0], [x1, y0, z0]]]];
    for (const [n, q] of F) { const id = q.map(p => m.v(p, n)); m.quad(id[0], id[1], id[2], id[3]); }
    return m;
  }

  // simple ear clipping for a simple polygon [[x,y],...]
  function earclip(poly) {
    const n = poly.length; if (n < 3) return [];
    let area = 0; for (let k = 0; k < n; k++) { const a = poly[k], b = poly[(k + 1) % n]; area += a[0] * b[1] - b[0] * a[1]; }
    const idx = [...Array(n).keys()]; if (area < 0) idx.reverse();
    const out = []; let guard = 0;
    const inside = (p, a, b, c) => { const s = (u, v, w) => (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0]); return s(a, b, p) >= -1e-12 && s(b, c, p) >= -1e-12 && s(c, a, p) >= -1e-12; };
    while (idx.length > 3 && guard++ < 10000) {
      let cut = false;
      for (let k = 0; k < idx.length; k++) {
        const i0 = idx[(k + idx.length - 1) % idx.length], i1 = idx[k], i2 = idx[(k + 1) % idx.length];
        const a = poly[i0], b = poly[i1], c = poly[i2];
        if ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 1e-12) continue;
        let ok = true;
        for (const j of idx) { if (j === i0 || j === i1 || j === i2) continue; if (inside(poly[j], a, b, c)) { ok = false; break; } }
        if (!ok) continue;
        out.push([i0, i1, i2]); idx.splice(k, 1); cut = true; break;
      }
      if (!cut) break;
    }
    if (idx.length === 3) out.push([idx[0], idx[1], idx[2]]);
    return out;
  }

  // extrude polygon (in local u,v) along w from w0 to w1; returns mesh in (u,v,w) -> mapped by `map`
  function extrude(poly, w0, w1, map) {
    const m = new Mesh(), n = poly.length;
    let area = 0; for (let k = 0; k < n; k++) { const a = poly[k], b = poly[(k + 1) % n]; area += a[0] * b[1] - b[0] * a[1]; }
    const sgn = area > 0 ? 1 : -1;
    const P = (u, v, w) => map ? map(u, v, w) : [u, v, w];
    const NV = (nu, nv, nw) => { if (!map) return [nu, nv, nw]; const o = map(0, 0, 0), q = map(nu, nv, nw); return V.norm(V.sub(q, o)); };
    for (let k = 0; k < n; k++) {
      const a = poly[k], b = poly[(k + 1) % n], du = b[0] - a[0], dv = b[1] - a[1], L = Math.hypot(du, dv) || 1;
      const nrm = NV(sgn * dv / L, -sgn * du / L, 0);
      m.quad(m.v(P(a[0], a[1], w0), nrm), m.v(P(b[0], b[1], w0), nrm), m.v(P(b[0], b[1], w1), nrm), m.v(P(a[0], a[1], w1), nrm));
    }
    const tris = earclip(poly);
    for (const [w, s] of [[w0, -1], [w1, 1]]) {
      const nn = NV(0, 0, s), id = poly.map(p => m.v(P(p[0], p[1], w), nn));
      for (const [a, b, c] of tris) m.tri(id[a], id[b], id[c]);
    }
    return m;
  }
  // star-shaped outer profile (polar list [[angle, r]...] ascending, full turn) with circular bore, extruded along Y
  function ringY(outer, rin, y0, y1, flatOuter = true) {
    const m = new Mesh(), n = outer.length;
    const pt = (a, r, y) => [r * Math.sin(a), y, r * Math.cos(a)];
    for (const [y, s] of [[y0, -1], [y1, 1]]) {
      const O = outer.map(([a, r]) => m.v(pt(a, r, y), [0, s, 0])), I = outer.map(([a]) => m.v(pt(a, rin, y), [0, s, 0]));
      for (let k = 0; k < n; k++) { const k2 = (k + 1) % n; m.quad(O[k], O[k2], I[k2], I[k]); }
    }
    for (let k = 0; k < n; k++) {
      const [a1, r1] = outer[k], [a2, r2] = outer[(k + 1) % n];
      const p1 = pt(a1, r1, 0), p2 = pt(a2, r2, 0);
      let nn = V.norm(V.cross([0, 1, 0], V.sub(p2, p1)));
      if (V.dot(nn, V.add(p1, p2)) < 0) nn = V.mul(nn, -1);
      if (!flatOuter) nn = null;
      const n1 = nn || V.norm([Math.sin(a1), 0, Math.cos(a1)]), n2 = nn || V.norm([Math.sin(a2), 0, Math.cos(a2)]);
      m.quad(m.v(pt(a1, r1, y0), n1), m.v(pt(a2, r2, y0), n2), m.v(pt(a2, r2, y1), n2), m.v(pt(a1, r1, y1), n1));
      const i1 = [-Math.sin(a1), 0, -Math.cos(a1)], i2 = [-Math.sin(a2), 0, -Math.cos(a2)];
      if (rin > 0) m.quad(m.v(pt(a1, rin, y0), i1), m.v(pt(a2, rin, y0), i2), m.v(pt(a2, rin, y1), i2), m.v(pt(a1, rin, y1), i1));
    }
    return m;
  }
  // rectangular plate (in u,v) with one circular hole, extruded along w; built as a star-shaped ring
  function plateHole(u0, u1, v0, v1, hu, hv, hr, w0, w1, map, seg = 64) {
    const m = new Mesh(), outer = [];
    for (let k = 0; k < seg; k++) {
      const a = TAU * k / seg, du = Math.cos(a), dv = Math.sin(a);
      let t = Infinity;
      if (du > 1e-9) t = Math.min(t, (u1 - hu) / du); if (du < -1e-9) t = Math.min(t, (u0 - hu) / du);
      if (dv > 1e-9) t = Math.min(t, (v1 - hv) / dv); if (dv < -1e-9) t = Math.min(t, (v0 - hv) / dv);
      outer.push([hu + du * t, hv + dv * t, hu + du * hr, hv + dv * hr]);
    }
    // add exact corners so edges stay straight
    const P = (u, v, w) => map(u, v, w), NV = (a, b, c) => { const o = map(0, 0, 0), q = map(a, b, c); return V.norm(V.sub(q, o)); };
    const corners = [[u1, v1], [u0, v1], [u0, v0], [u1, v0]].map(([u, v]) => [Math.atan2(v - hv, u - hu), u, v]);
    let pts = outer.map((o, k) => [TAU * k / seg, o[0], o[1]]);
    for (const c of corners) { let a = c[0]; if (a < 0) a += TAU; pts.push([a, c[1], c[2]]); }
    pts.sort((a, b) => a[0] - b[0]);
    const n = pts.length;
    for (const [w, s] of [[w0, -1], [w1, 1]]) {
      const nn = NV(0, 0, s);
      const O = pts.map(p => m.v(P(p[1], p[2], w), nn)), I = pts.map(p => m.v(P(hu + Math.cos(p[0]) * hr, hv + Math.sin(p[0]) * hr, w), nn));
      for (let k = 0; k < n; k++) { const k2 = (k + 1) % n; m.quad(O[k], O[k2], I[k2], I[k]); }
    }
    for (const [a, b, nu, nv] of [[[u0, v0], [u1, v0], 0, -1], [[u1, v0], [u1, v1], 1, 0], [[u1, v1], [u0, v1], 0, 1], [[u0, v1], [u0, v0], -1, 0]]) {
      const nn = NV(nu, nv, 0);
      m.quad(m.v(P(a[0], a[1], w0), nn), m.v(P(b[0], b[1], w0), nn), m.v(P(b[0], b[1], w1), nn), m.v(P(a[0], a[1], w1), nn));
    }
    for (let k = 0; k < 96; k++) {
      const a1 = TAU * k / 96, a2 = TAU * (k + 1) / 96;
      const n1 = NV(-Math.cos(a1), -Math.sin(a1), 0), n2 = NV(-Math.cos(a2), -Math.sin(a2), 0);
      const q = (a, w) => P(hu + Math.cos(a) * hr, hv + Math.sin(a) * hr, w);
      m.quad(m.v(q(a1, w0), n1), m.v(q(a2, w0), n2), m.v(q(a2, w1), n2), m.v(q(a1, w1), n1));
    }
    return m;
  }

  // tube swept along a 3D polyline (parallel-transport frames)
  function sweep(pts, r, sides = 12, caps = true, closed = false) {
    const m = new Mesh(), n = pts.length; if (n < 2) return m;
    const T = pts.map((p, k) => V.norm(V.sub(pts[Math.min(k + 1, n - 1)], pts[Math.max(k - 1, 0)])));
    let N0 = Math.abs(T[0][1]) < 0.9 ? V.norm(V.cross(T[0], [0, 1, 0])) : V.norm(V.cross(T[0], [1, 0, 0]));
    const rings = [];
    let Nk = N0;
    for (let k = 0; k < n; k++) {
      if (k > 0) { const b = V.cross(T[k - 1], T[k]), s = V.len(b); if (s > 1e-9) { const ax = V.mul(b, 1 / s), ang = Math.asin(Math.min(1, s)) * (V.dot(T[k - 1], T[k]) < 0 ? -1 : 1); Nk = Mx.applyN(Mx.rotAbout(ax, Math.atan2(s, V.dot(T[k - 1], T[k])), [0, 0, 0]), Nk); } }
      const Bk = V.cross(T[k], Nk), ring = [];
      for (let j = 0; j < sides; j++) {
        const a = TAU * j / sides, d = V.add(V.mul(Nk, Math.cos(a)), V.mul(Bk, Math.sin(a)));
        ring.push(m.v(V.add(pts[k], V.mul(d, r)), d));
      }
      rings.push(ring);
    }
    for (let k = 0; k < n - 1 + (closed ? 1 : 0); k++) {
      const A = rings[k], B = rings[(k + 1) % n];
      for (let j = 0; j < sides; j++) m.quad(A[j], A[(j + 1) % sides], B[(j + 1) % sides], B[j]);
    }
    if (caps && !closed) for (const [k, s] of [[0, -1], [n - 1, 1]]) {
      const c = m.v(pts[k], V.mul(T[k], s)), ring = [];
      for (let j = 0; j < sides; j++) { const p = [m.p[3 * rings[k][j]], m.p[3 * rings[k][j] + 1], m.p[3 * rings[k][j] + 2]]; ring.push(m.v(p, V.mul(T[k], s))); }
      for (let j = 0; j < sides; j++) m.tri(c, ring[j], ring[(j + 1) % sides]);
    }
    return m;
  }
  // flat strip along a path: up = surface normal per point, width across (side = up x tangent), thickness t
  function strip(pts, ups, width, t, colors, closed = false) {
    const m = new Mesh(), n = pts.length;
    const T = pts.map((p, k) => V.norm(V.sub(pts[closed ? (k + 1) % n : Math.min(k + 1, n - 1)], pts[closed ? (k - 1 + n) % n : Math.max(k - 1, 0)])));
    const S = pts.map((p, k) => V.norm(V.cross(ups[k], T[k])));
    ups = ups.slice();
    for (let k = 0; k < n; k++) if (Math.abs(S[k][1]) > 0.5 && S[k][1] < 0) { S[k] = V.mul(S[k], -1); ups[k] = V.mul(ups[k], -1); }
    const ring = (k) => {
      const p = pts[k], u = V.mul(ups[k], t / 2), s = V.mul(S[k], width / 2), col = colors ? colors[k] : null;
      const c = [V.add(V.add(p, u), s), V.add(V.sub(p, s), u), V.sub(V.sub(p, s), u), V.add(V.sub(p, u), s)];
      return {
        top: [m.v(c[0], ups[k], col), m.v(c[1], ups[k], col)],
        bot: [m.v(c[3], V.mul(ups[k], -1), col), m.v(c[2], V.mul(ups[k], -1), col)],
        s1: [m.v(c[0], S[k], col), m.v(c[3], S[k], col)],
        s2: [m.v(c[1], V.mul(S[k], -1), col), m.v(c[2], V.mul(S[k], -1), col)],
      };
    };
    const R = pts.map((_, k) => ring(k));
    for (let k = 0; k < n - 1 + (closed ? 1 : 0); k++) {
      const a = R[k], b = R[(k + 1) % n];
      for (const f of ['top', 'bot', 's1', 's2']) m.quad(a[f][0], a[f][1], b[f][1], b[f][0]);
    }
    return m;
  }
  function helix(R, r, pitch, turns, y0, seg = 24, sides = 8) {
    const pts = [], N = Math.max(8, Math.round(turns * seg));
    for (let k = 0; k <= N; k++) { const a = TAU * turns * k / N; pts.push([R * Math.sin(a), y0 + pitch * turns * k / N, R * Math.cos(a)]); }
    return sweep(pts, r, sides);
  }



  // ------------------------------------------------------------------ extra primitives (v5)
  // ear clipping that tolerates bridged polygons (repeated vertices at the bridges)
  function earclipH(poly) {
    const n = poly.length; if (n < 3) return [];
    let area = 0; for (let k = 0; k < n; k++) { const a = poly[k], b = poly[(k + 1) % n]; area += a[0] * b[1] - b[0] * a[1]; }
    const idx = [...Array(n).keys()]; if (area < 0) idx.reverse();
    const out = []; let guard = 0;
    const same = (p, q) => Math.abs(p[0] - q[0]) < 1e-9 && Math.abs(p[1] - q[1]) < 1e-9;
    const cr = (u, v, w) => (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0]);
    const inside = (p, a, b, c) => cr(a, b, p) > 1e-10 && cr(b, c, p) > 1e-10 && cr(c, a, p) > 1e-10;
    while (idx.length > 3 && guard++ < 40000) {
      let cut = false;
      for (let k = 0; k < idx.length; k++) {
        const i0 = idx[(k + idx.length - 1) % idx.length], i1 = idx[k], i2 = idx[(k + 1) % idx.length];
        const a = poly[i0], b = poly[i1], c = poly[i2];
        if (cr(a, b, c) <= 1e-10) continue;
        let ok = true;
        for (const j of idx) { const p = poly[j]; if (j === i0 || j === i1 || j === i2 || same(p, a) || same(p, b) || same(p, c)) continue; if (inside(p, a, b, c)) { ok = false; break; } }
        if (!ok) continue;
        out.push([i0, i1, i2]); idx.splice(k, 1); cut = true; break;
      }
      if (!cut) { // degenerate leftovers (collinear bridge pieces): drop one zero-area vertex and continue
        let dropped = false;
        for (let k = 0; k < idx.length; k++) { const a = poly[idx[(k + idx.length - 1) % idx.length]], b = poly[idx[k]], c = poly[idx[(k + 1) % idx.length]]; if (Math.abs(cr(a, b, c)) <= 1e-10) { idx.splice(k, 1); dropped = true; break; } }
        if (!dropped) break;
      }
    }
    if (idx.length === 3) out.push([idx[0], idx[1], idx[2]]);
    return out;
  }
  const sArea = (p) => { let a = 0; for (let k = 0; k < p.length; k++) { const u = p[k], v = p[(k + 1) % p.length]; a += u[0] * v[1] - v[0] * u[1]; } return a / 2; };
  const ccw = (p) => sArea(p) < 0 ? p.slice().reverse() : p.slice();
  const circ = (c, r, n = 40, a0 = 0) => { const o = []; for (let k = 0; k < n; k++) { const a = a0 + TAU * k / n; o.push([c[0] + r * Math.cos(a), c[1] + r * Math.sin(a)]); } return o; };
  const rrect = (u0, u1, v0, v1, r, n = 6) => { const o = []; const cs = [[u1 - r, v0 + r, -PI / 2], [u1 - r, v1 - r, 0], [u0 + r, v1 - r, PI / 2], [u0 + r, v0 + r, PI]]; for (const [cu, cv, a0] of cs) for (let k = 0; k <= n; k++) { const a = a0 + PI / 2 * k / n; o.push([cu + r * Math.cos(a), cv + r * Math.sin(a)]); } return o; };
  // plate: outer polygon with holes (all in u,v), extruded along w, mapped by `map`
  function plate(outer, holes, w0, w1, map) {
    const m = new Mesh();
    const O = ccw(outer), H = holes.map(ccw);
    const P = (u, v, w) => map(u, v, w), NV = (a, b, c) => { const o = map(0, 0, 0), q = map(a, b, c); return V.norm(V.sub(q, o)); };
    const walls = (c, hole) => {
      for (let k = 0; k < c.length; k++) {
        const a = c[k], b = c[(k + 1) % c.length], du = b[0] - a[0], dv = b[1] - a[1], L = Math.hypot(du, dv) || 1, s = hole ? -1 : 1;
        const nn = NV(s * dv / L, -s * du / L, 0);
        m.quad(m.v(P(a[0], a[1], w0), nn), m.v(P(b[0], b[1], w0), nn), m.v(P(b[0], b[1], w1), nn), m.v(P(a[0], a[1], w1), nn));
      }
    };
    walls(O, false); for (const h of H) walls(h, true);
    // caps: exact slab decomposition (even-odd rule) between consecutive vertex levels in v
    const edges = []; for (const c of [O, ...H]) for (let k = 0; k < c.length; k++) { const p = c[k], q = c[(k + 1) % c.length]; if (Math.abs(p[1] - q[1]) > 1e-9) edges.push(p[1] < q[1] ? [p, q] : [q, p]); }
    const lv = [...new Set([O, ...H].flat().map(p => +p[1].toFixed(7)))].sort((x, y) => x - y);
    const tris = [];
    for (let k = 0; k + 1 < lv.length; k++) {
      const va = lv[k], vb = lv[k + 1], vm = (va + vb) / 2; if (vb - va < 1e-7) continue;
      const at = (e, v) => e[0][0] + (e[1][0] - e[0][0]) * (v - e[0][1]) / (e[1][1] - e[0][1]);
      const cr = edges.filter(e => e[0][1] <= vm && e[1][1] >= vm).map(e => [at(e, vm), at(e, va), at(e, vb)]).sort((x, y) => x[0] - y[0]);
      for (let j = 0; j + 1 < cr.length; j += 2) { const L = cr[j], R = cr[j + 1]; tris.push([[L[1], va], [R[1], va], [R[2], vb]], [[L[1], va], [R[2], vb], [L[2], vb]]); }
    }
    for (const [w, s] of [[w0, -1], [w1, 1]]) {
      const nn = NV(0, 0, s);
      for (const t of tris) { if (Math.abs((t[1][0] - t[0][0]) * (t[2][1] - t[0][1]) - (t[2][0] - t[0][0]) * (t[1][1] - t[0][1])) < 1e-10) continue; m.tri(m.v(P(t[0][0], t[0][1], w), nn), m.v(P(t[1][0], t[1][1], w), nn), m.v(P(t[2][0], t[2][1], w), nn)); }
    }
    return m;
  }
  // ring with a circular OUTER edge and a star-shaped inner profile (internal teeth); polar list [[angle, r]...]
  function ringIn(inner, rOut, y0, y1) {
    const m = new Mesh(), n = inner.length;
    const pt = (a, r, y) => [r * Math.sin(a), y, r * Math.cos(a)];
    for (const [y, s] of [[y0, -1], [y1, 1]]) {
      const O = inner.map(([a]) => m.v(pt(a, rOut, y), [0, s, 0])), I = inner.map(([a, r]) => m.v(pt(a, r, y), [0, s, 0]));
      for (let k = 0; k < n; k++) { const k2 = (k + 1) % n; m.quad(O[k], O[k2], I[k2], I[k]); }
    }
    for (let k = 0; k < n; k++) {
      const [a1, r1] = inner[k], [a2, r2] = inner[(k + 1) % n];
      const o1 = [Math.sin(a1), 0, Math.cos(a1)], o2 = [Math.sin(a2), 0, Math.cos(a2)];
      m.quad(m.v(pt(a1, rOut, y0), o1), m.v(pt(a2, rOut, y0), o2), m.v(pt(a2, rOut, y1), o2), m.v(pt(a1, rOut, y1), o1));
      const p1 = pt(a1, r1, 0), p2 = pt(a2, r2, 0);
      let nn = V.norm(V.cross([0, 1, 0], V.sub(p2, p1))); if (V.dot(nn, V.add(p1, p2)) > 0) nn = V.mul(nn, -1);
      m.quad(m.v(pt(a1, r1, y0), nn), m.v(pt(a2, r2, y0), nn), m.v(pt(a2, r2, y1), nn), m.v(pt(a1, r1, y1), nn));
    }
    return m;
  }
  function hull(pts) {   // 2D convex hull (monotone chain), CCW
    const p = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]), cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    const lo = [], up = [];
    for (const q of p) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
    for (let k = p.length - 1; k >= 0; k--) { const q = p[k]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
    return lo.slice(0, -1).concat(up.slice(0, -1));
  }
  // 2D tangent helpers in the (z, x) plane, points as [z, x]; angle from +Z toward +X
  function tanPC(S, C, r, s) {   // tangent point on circle (C, r) of a line from point S; s = +-1 picks the side
    const dz = C[0] - S[0], dx = C[1] - S[1], d = Math.hypot(dz, dx), b = Math.atan2(dx, dz), a = Math.asin(Math.min(1, r / d)), L = Math.sqrt(Math.max(0, d * d - r * r)), u = b + s * a;
    return [S[0] + L * Math.cos(u), S[1] + L * Math.sin(u)];
  }
  function tanInt(C1, r1, C2, r2, s) {  // internal tangent: circle 1 on the -n side of the line, circle 2 on the +n side
    const D = [C2[0] - C1[0], C2[1] - C1[1]], d = Math.hypot(D[0], D[1]), g = Math.acos(Math.min(1, (r1 + r2) / d)), a = Math.atan2(D[1], D[0]) + s * g, n = [Math.cos(a), Math.sin(a)];
    return { n, p1: [C1[0] + r1 * n[0], C1[1] + r1 * n[1]], p2: [C2[0] - r2 * n[0], C2[1] - r2 * n[1]] };
  }
  const ang2 = (C, p) => Math.atan2(p[1] - C[1], p[0] - C[0]);
  const zx3 = (p, y = 0) => [p[1], y, p[0]];


  // ------------------------------------------------------------------ v5.1 design parameters (mm)
  // X = out of the thigh (x = 0: lowest point of the saddle, centre line), Y = along the thigh (+Y proximal), Z = tangential.
  // Drum 1 (z < 0) winds CLOCKWISE in the (z right, x up) view = rotation -theta about +Y; drum 2 winds counter-clockwise (+theta).
  // Polar angles about an axis parallel to Y are measured from +Z toward +X.
  const DEG = PI / 180;
  const P = {
    ver: '5.1',
    HX: 44.0, coverT: 0.9, cornerR: 3.0, floor: 2.0, padT: 2.5, Rs: 78,
    Ym: -63.5, Yp: 86.4, ZM: -44.0, HZ: 44.0, wallX0: -1.5,
    band: { w: 90, t: 0.3 },
    xd: 21, z1: -18.75, z2: 18.75, R0: 15, barrelT: 0.8, barrel: [-45.75, 45.75], preWrap: PI,
    flangeR: 17.0, flange: [-46.95, -45.75], thrustT: 0.4,
    roll: { z: 38.5, x: -7.2, r: 5, wall: 0.8, y: [-45.6, 45.6], pin: 1.5 },
    saddleZ: 31.9, wingZ: 43.0, bandZone: 46.4,
    gear: { m: 1.25, n: 30, rp: 18.75, y: [45.9, 50.9], web: 1.4 },
    plateYm: [-49.1, -47.5], plateYp: [51.25, 52.85], plateZ: 42.6, plateTop: 41.6, legPad: 2.4, legPadM: 1.0,
    shaft1: { ro: 6, ri: 4.15, yM: -58.3, yP: 53.95, hexR: 5.0 },
    axle2: { ro: 11.4, ri: 10.75, flange: [-51.3, -50.3], plug: 22.0, bottomT: 2.0, bossR: 6.0, bossEnd: 33.0, noseR: 4.0, noseBore: 2.5, nose: [27.0, 52.85] },
    capsule: { y0: -47.0, L: 69 },
    stator: { y: [-55.1, -49.1], r: 17.4, rLow: 14.2, buttress: 0.55, rPocket: 14.9, rIn: 12.6, hub: [7.0, 9.0], web: 3.0 },
    ratchet: { teeth: 60, rTip: 18.5, rRoot: 19.3, rOut: 21.3, y: [-55.3, -51.05], web: [-57.5, -55.3], webThin: -56.3, hub: [-58.3, -55.3], hubR: 6.5,
      hole: [-54.95, -52.85], holeHalf: 1.1 },
    pawl: { n: 3, rHeel: 16.6, L: 7.0, th: 2.6, y: [-54.7, -50.9], ang0: 17 * DEG, pinAt: 5.5, lift: 0.145 },
    cam: { y: [-50.2, -49.2], rIn: 17.6, rOut: 20.9, throw: 24 * DEG, a0: 78 * DEG, armR: 24.5, knobR: [24.8, 26.6], knobHalf: 3.0, knobY: [-50.7, -46.7],
      fanR: 21.8, fanHalf: 28.5 * DEG },
    spring: { Dc: 27.4, da: 8.0, b: 7.5, t: 0.5, y: [-44.8, -37.3], L: 539, mass: 15.9 },
    arbor: { r: 4.0, bore: 2.0, y: [-62.0, -37.0], lock: [-61.6, -60.1], standA: [145 * DEG, 215 * DEG], standR: 24.0, standr: 2.2 },
    trip: { piv: [39.8, 4.0], travel: 2.0, armM: [-50.6, -49.3], armP: [53.05, 54.35], valveArm: 3.2, shaftEnd: 60.0, spring: [54.8, 59.5], hole: [41.0, 0.8] },
    gas: { flange: [-52.7, -51.3], flangeR: 12.2, lugR: 13.4, lugr: 2.6, lugA: [60 * DEG, 120 * DEG, 240 * DEG, 300 * DEG], pad: [-50.3, -49.1],
      pilot: [-51.3, -48.8], bossR: 6.8, face: -42.5, pocket: -47.3, pocketR: 4.43, bore: 1.525, neck: [-61.7, -52.7], neckR: 4.0,
      sleeveR: 6.5, lever: [-60.1, -58.5], camLow: -57.9, lift: 3.0, camSwing: 56 * DEG, camLead: 5 * DEG, arm: 25.75, beta: 28 * DEG, forkR: 30.56, rodZ: -7.0, rodR: 0.9 },
    reg: { z: 36.9, y: -56.0, r: 5.6, x: [10.3, 23.3] },
    valve: { x: 7.2, y: -55.7, r: 2.8, z: [28.0, 37.6], stemOut: 0.85 },
    act: { y: 71.9, x: 30.0, ri: 11.5, ro: 12.3, lockZ: 1.5, lockR: 0.42, lockWire: 0.8, zB: -4.4, zEnd: 42.5, bottom: 2.6, stroke: 38, z0: -4.0, pistonL: 5.5,
      wallA: [56.3, 67.9], hubR: 8.3, flangeY: 65.4, chamber: [67.9, 75.9], wallB: [75.9, 85.1], coverT: 2.5, chR: 11.8, chRo: 13.4,
      slotX: [26.6, 33.4], lumpX: 35.3, postA: { z: 36.0, x: 37.5, r: 3.2 } },
    pulley: { r: 9, root: 7.5, rf: 11.2, y: [68.5, 75.6] },
    clutch: { disk: [53.05, 53.95], ring: [54.0, 55.2], hub: [55.2, 55.7], r: 10.5, teeth: 54 },
    tie: { z: 0, x: 39.0 },
    pinA: { y: -53.9, xh: 47.2 },
    boss: { z: 36.0, half: 2.5, x: [38.6, 43.1], len: 4.0 },
    tube: { z: 3.0, x: 4.4 },
    R: 87.5, sleeveT: 2.5,
  };
  P.HY = (P.Yp - P.Ym) / 2;
  P.travel = P.act.stroke;
  P.trip.arm = Math.hypot(P.trip.piv[0] - P.roll.z, P.trip.piv[1] - P.roll.x);
  P.trip.dAng = P.trip.travel / P.trip.arm;
  { // roller B moves on an arc about the pivot; e = direction of the chord (z, x), chord = its length
    const a = P.trip.dAng, rz = P.roll.z - P.trip.piv[0], rx = P.roll.x - P.trip.piv[1], ez = rz * Math.cos(a) - rx * Math.sin(a) - rz, ex = rz * Math.sin(a) + rx * Math.cos(a) - rx, L = Math.hypot(ez, ex);
    P.trip.e = [ez / L, ex / L]; P.trip.chord = L;
  }
  P.gas.rodX0 = P.xd - P.gas.arm * Math.tan(P.gas.beta);            // height of the pull-rod pin at rest
  P.gas.pull = 2 * P.gas.arm * Math.tan(P.gas.beta);                // handle travel for the full cam lift
  P.gas.betaExit = Math.acos(P.gas.arm / (P.gas.forkR - 0.3));      // lever angle (above horizontal) at which the pin leaves the fork
  P.gas.pullExit = P.gas.arm * (Math.tan(P.gas.betaExit) + Math.tan(P.gas.beta));
  const DEFAULT_D = { R0: 0.015, t_band: 0.0003, bore: 0.023, stroke: 0.038, r_p: 0.009, teeth: 60, pawls: 3, pre_wrap: Math.PI };
  // drum kinematics (one ply per drum), mirrors sim5.py: rd = R0 + t/2 + t pre/2pi + t th/2pi
  function makeKin(D) {
    const r0 = (D.R0 || 0.015) * 1000, t = (D.t_band || 0.0003) * 1000, pre = D.pre_wrap === undefined ? PI : D.pre_wrap, rf = r0 + t / 2 + t * pre / TAU;
    return { rd: (th) => rf + t * th / TAU, L: (th) => 2 * (rf * th + t * th * th / (4 * PI)) };
  }
  // saddle underside and how the module sits on a limb whose sleeve radius is Rsl (distance limb axis -> x = 0)
  const saddleX = (z) => -(P.Rs - Math.sqrt(Math.max(0, P.Rs * P.Rs - z * z)));
  function mountFor(Rsl) {
    const Rb = Rsl + P.band.t / 2, rr = P.roll.r + P.band.t / 2;
    return Math.max(Rsl + P.padT, Math.sqrt(Math.max(0, Rb * Rb - P.roll.z * P.roll.z)) - (P.roll.x - rr) + 0.1);
  }
  // lever angle (rad, rotation about axis 2, negative = lever tip goes up) and lance travel for a handle lift h (mm)
  function pierceKin(h) {
    const G = P.gas, hh = Math.max(0, Math.min(h, G.pullExit));
    const psi = Math.atan(hh / G.arm - Math.tan(G.beta)) + G.beta;
    return { psi, lance: G.lift * Math.max(0, Math.min(1, psi / G.camSwing)) };
  }
  // internal ratchet teeth of the cup (60), in the cup frame; angle from +Z toward +X. The steep face is at the start of each
  // pitch, the gap (root radius) on its counter-clockwise side: the cup can turn clockwise (winding) and is blocked the other way.
  function toothR(psi) {
    const RT = P.ratchet, pitch = TAU / RT.teeth; let g = (psi / pitch) % 1; if (g < 0) g += 1;
    return g < 0.04 ? RT.rTip + (RT.rRoot - RT.rTip) * g / 0.04 : RT.rRoot - (RT.rRoot - RT.rTip) * (g - 0.04) / 0.96;
  }
  // pocket pawls: heel (pivot) in the stator, tip in the tooth gap. k-th pawl is shifted by 1/3 pitch.
  function pawlGeo(k) {
    const PW = P.pawl, RT = P.ratchet, pitch = TAU / RT.teeth, C = [P.z1, P.xd];
    const rT = RT.rRoot - 0.05, cosD = (rT * rT + PW.rHeel * PW.rHeel - PW.L * PW.L) / (2 * rT * PW.rHeel), dA = Math.acos(cosD);
    const tipA = Math.round((PW.ang0 + k * TAU / 3) / pitch) * pitch + (0.04 + 0.012) * pitch + k * pitch / 3;    // tip corner just past a steep face
    const heelA = tipA + dA;
    const heel = [C[0] + PW.rHeel * Math.cos(heelA), C[1] + PW.rHeel * Math.sin(heelA)], tip = [C[0] + rT * Math.cos(tipA), C[1] + rT * Math.sin(tipA)];
    const u = [(tip[0] - heel[0]) / PW.L, (tip[1] - heel[1]) / PW.L]; let v = [-u[1], u[0]];
    const mid = [(tip[0] + heel[0]) / 2 - C[0], (tip[1] + heel[1]) / 2 - C[1]]; if (v[0] * mid[0] + v[1] * mid[1] < 0) v = [-v[0], -v[1]];   // v points outward
    const at = (a, b) => [heel[0] + u[0] * a + v[0] * b, heel[1] + u[1] * a + v[1] * b];
    return { k, tipA, heelA, heel, tip, u, v, at, L: PW.L, pin: at(PW.pinAt, -0.2), seat: at(3.4, -1.3) };
  }
  // outline in the pawl frame (a along the load line heel centre -> tooth corner, b outward): round heel r 1.3; the outer face stays
  // under the tooth tips and only the last 1.5 mm (nose) enters the tooth gap; the end face is radial (parallel to the tooth face)
  function pawlPoly(L) {
    const o = [];
    for (let j = 0; j <= 10; j++) { const a = PI / 2 + PI * j / 10; o.push([1.3 * Math.cos(a), 1.3 * Math.sin(a)]); }
    o.push([L - 1.6, -1.3], [L - 0.48, -0.76], [L, 0], [4.4, 0.25], [1.0, 1.25]);
    return o;
  }
  function pawlAngle(g, cupAngle, lift = 0, rampOnly = false) {    // rotation about the heel (Mx.rotAbout([0,1,0], d, heel)); lift 0..1 = release-ring position
    const C = [P.z1, P.xd], H = g.heel;
    if (!g._pts) {
      const poly = pawlPoly(g.L), pts = [];
      for (let k = 0; k < poly.length; k++) { const a = poly[k], b = poly[(k + 1) % poly.length], n = Math.max(2, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / 0.1)); for (let j = 0; j < n; j++) { const q = g.at(a[0] + (b[0] - a[0]) * j / n, a[1] + (b[1] - a[1]) * j / n); pts.push([q[0] - H[0], q[1] - H[1]]); } }
      g._pts = pts;
      const q = [g.tip[0] - H[0], g.tip[1] - H[1]], rad = (d) => { const c = Math.cos(d), s = Math.sin(d); return Math.hypot(H[0] + c * q[0] - s * q[1] - C[0], H[1] + s * q[0] + c * q[1] - C[1]); };
      g._out = rad(0.05) > rad(0) ? 1 : -1;
    }
    const worst = (d) => { const c = Math.cos(d), s = Math.sin(d); let w = -1e9; for (const [dz, dx] of g._pts) { const z = H[0] + c * dz - s * dx - C[0], x = H[1] + s * dz + c * dx - C[1], r = Math.hypot(z, x); if (r < P.ratchet.rTip - 0.4) continue; const e = r - toothR(Math.atan2(x, z) - cupAngle); if (e > w) w = e; } return w; };
    // release ring: its inward ramp (same profile as the 'cam' part) bears on the side pin Ø1,2 of the pawl
    const CM = P.cam, pin0 = g.pin, qp = [pin0[0] - H[0], pin0[1] - H[1]], pr0 = Math.hypot(pin0[0] - C[0], pin0[1] - C[1]), pa0 = Math.atan2(pin0[1] - C[1], pin0[0] - C[0]);
    const rProf = (a) => { const d = Math.atan2(Math.sin(a - pa0), Math.cos(a - pa0)); let r = CM.rIn; if (d > -(CM.throw + 0.10) && d < 0.14) r = Math.max(r, pr0 + 0.75 - 0.95 * Math.max(0, Math.min(1, (-d - 0.04) / (CM.throw - 0.06)))); return r; };
    const ramp = (d) => { if (lift <= 0) return -1; const c = Math.cos(d), s = Math.sin(d), pz = H[0] + c * qp[0] - s * qp[1], px = H[1] + s * qp[0] + c * qp[1]; let w = -1e9;
      for (let k = 0; k < 24; k++) { const t = TAU * k / 24, z = pz + 0.6 * Math.cos(t) - C[0], x = px + 0.6 * Math.sin(t) - C[1], e = Math.hypot(z, x) - rProf(Math.atan2(x, z) - CM.throw * lift); if (e > w) w = e; } return w; };
    // the springs push the nose outward: take the most outward rotation that penetrates neither the teeth nor the ramp of the release ring
    const ok = (m) => (rampOnly || worst(m * out) <= 1e-4) && ramp(m * out) <= 1e-4;
    const out = g._out, top = 0.03, bot = -0.32; let prev = top, found = bot;
    for (let k = 0; k <= 140; k++) {
      const m = top + (bot - top) * k / 140;
      if (ok(m)) { found = m; if (k > 0) { let a = m, b = prev; for (let it = 0; it < 14; it++) { const c = (a + b) / 2; if (ok(c)) a = c; else b = c; } found = a; } break; }
      prev = m;
    }
    return found * out;
  }
  // The pawls are self-engaging (the load draws the nose into the tooth gap), so while the release ring lifts a seated pawl
  // the drum has to wind on a little (≤ ≈ 1°) for the nose to climb out; with the gas holding this needs almost no force,
  // with the gas lost the knob force covers it (verify: «Сила на ручці…»). Returns that extra winding angle (rad, + = winding).
  const _rw = new Map();
  function releaseWind(gs, th, dial) {
    if (!(dial > 0)) return 0;
    const pitch = TAU / P.ratchet.teeth, ph = ((th % pitch) + pitch) % pitch, key = Math.round(ph / 2e-4) + '|' + Math.round(dial / 0.005);
    if (_rw.has(key)) return _rw.get(key);
    const okAt = (d) => gs.every(g => { const cup = -(th + d), o = g._out || 1, f = pawlAngle(g, cup, dial) * (g._out || o); const s0 = pawlAngle(g, cup, 0) * g._out, r0 = pawlAngle(g, cup, dial, true) * g._out; return f >= Math.min(s0, r0) - 3e-4; });
    let d = 0; const step = 0.2 * DEG;
    while (d < 3 * DEG && !okAt(d)) d += step;
    if (d > 0 && d < 3 * DEG) { let a = d - step, b = d; for (let it = 0; it < 7; it++) { const c = (a + b) / 2; if (okAt(c)) b = c; else a = c; } d = b; }
    _rw.set(key, d); return d;
  }

  // ------------------------------------------------------------------ materials
  const MAT = {
    al: { name: 'Алюміній 6061-T6, анодований', rho: 2.70, color: [0.70, 0.72, 0.75], metal: 1, rough: 0.35 },
    alDark: { name: 'Алюміній 7075-T6, чорне анодування', rho: 2.81, color: [0.18, 0.19, 0.21], metal: 1, rough: 0.4 },
    d16: { name: 'Дюраль Д16Т (2024-T3)', rho: 2.78, color: [0.64, 0.66, 0.69], metal: 1, rough: 0.35 },
    steel: { name: 'Сталь 42CrMo4 (40ХФА), загартована', rho: 7.85, color: [0.58, 0.59, 0.61], metal: 1, rough: 0.25 },
    steelBlk: { name: 'Сталь, чорне оксидування', rho: 7.85, color: [0.22, 0.22, 0.23], metal: 1, rough: 0.35 },
    ss: { name: 'Нержавіюча сталь AISI 304', rho: 7.93, color: [0.78, 0.79, 0.80], metal: 1, rough: 0.2 },
    spring: { name: 'Пружинна сталь 65Г / EN 10270', rho: 7.85, color: [0.35, 0.36, 0.38], metal: 1, rough: 0.3 },
    brass: { name: 'Латунь CW614N', rho: 8.47, color: [0.80, 0.64, 0.30], metal: 1, rough: 0.3 },
    bronze: { name: 'Бронза CuSn8 / спечена бронза (втулки)', rho: 8.8, color: [0.72, 0.50, 0.30], metal: 1, rough: 0.35 },
    pa: { name: 'PA12-GF30 (склонаповнений поліамід)', rho: 1.45, color: [0.24, 0.27, 0.22], metal: 0, rough: 0.8 },
    pom: { name: 'POM-C (поліацеталь)', rho: 1.41, color: [0.92, 0.92, 0.90], metal: 0, rough: 0.6 },
    ptfe: { name: 'PTFE', rho: 2.2, color: [0.95, 0.95, 0.93], metal: 0, rough: 0.7 },
    seal: { name: 'PTFE-манжета з пружиною (−80…+200 °C)', rho: 2.1, color: [0.93, 0.93, 0.90], metal: 0, rough: 0.7 },
    webbing: { name: 'Поліамідна стрічка 90×1,2 мм, PTFE-просочення', rho: 1.1, color: [0.30, 0.34, 0.24], metal: 0, rough: 0.9 },
    sleeve: { name: 'Рукав: нейлон 500D + PTFE-підклад', rho: 1.15, color: [0.40, 0.42, 0.33], metal: 0, rough: 0.95 },
    eva: { name: 'EVA-піна 45 Shore A', rho: 0.1, color: [0.15, 0.15, 0.15], metal: 0, rough: 1 },
    cable: { name: 'Трос 7×19 Ø2,4 нерж., нейлонова оболонка Ø3', rho: 4.5, color: [0.85, 0.55, 0.15], metal: 0, rough: 0.5 },
    capsule: { name: 'Балончик iSi Nitro N₂ 2,4 г (сталь, бронзовий лак)', rho: 7.85, color: [0.62, 0.42, 0.22], metal: 1, rough: 0.3 },
    red: { name: 'Алюміній, червоне анодування', rho: 2.7, color: [0.75, 0.12, 0.10], metal: 1, rough: 0.35 },
    tpu: { name: 'TPU 85A (щітки-ущільнювачі)', rho: 1.2, color: [0.12, 0.30, 0.55], metal: 0, rough: 0.8 },
    pc: { name: 'Полікарбонат, прозорий', rho: 1.2, color: [0.70, 0.85, 0.95], metal: 0, rough: 0.1, alpha: 0.35 },
    green: { name: 'Алюміній, зелене анодування', rho: 2.7, color: [0.15, 0.55, 0.25], metal: 1, rough: 0.35 },
    skin: { name: 'Стегно (модель м’яких тканин)', rho: 1.05, color: [0.87, 0.67, 0.56], metal: 0, rough: 0.8 },
    trousers: { name: 'Штани (бавовна/нейлон)', rho: 0.5, color: [0.33, 0.36, 0.30], metal: 0, rough: 1 },
  };

  // helpers for axis orientation: lathe is built about +Y
  const alongZ = (m, x, y) => m.xf(Mx.chain(Mx.T(x, y, 0), Mx.RX(PI / 2)));      // profile y -> world z
  const alongX = (m, y, z) => m.xf(Mx.chain(Mx.T(0, y, z), Mx.RZ(-PI / 2)));     // profile y -> world x
  const rodZ = (x, y, r, z0, z1, seg = 16) => alongZ(cylY(r, z0, z1, seg), x, y);

  function fixOrientation(m) {   // make closed meshes outward-facing (positive volume)
    if (m.volume() >= 0) return m;
    for (let k = 0; k < m.i.length; k += 3) { const t = m.i[k + 1]; m.i[k + 1] = m.i[k + 2]; m.i[k + 2] = t; }
    for (let k = 0; k < m.n.length; k++) m.n[k] = -m.n[k];
    return m;
  }

  Object.assign(MAT, {
    pacf: { name: 'PA12-CF (SLS) або PA6/PA12-CF (FDM), E ≥ 7 ГПа', rho: 1.15, color: [0.20, 0.22, 0.19], metal: 0, rough: 0.85 },
    cfrp: { name: 'Вуглепластикова труба (CFRP)', rho: 1.55, color: [0.12, 0.12, 0.13], metal: 0, rough: 0.45 },
    ti: { name: 'Титан ВТ6 (Ti-6Al-4V)', rho: 4.43, color: [0.55, 0.56, 0.58], metal: 1, rough: 0.35 },
    uhmw: { name: 'Стрічка UHMWPE (Dyneema) 90 × 0,3 мм, PU-покриття', rho: 0.97, color: [0.33, 0.36, 0.27], metal: 0, rough: 0.85 },
    cord: { name: 'Шнур Dyneema SK99 Ø2,5 в PU-оболонці Ø3,2', rho: 1.0, color: [0.90, 0.55, 0.12], metal: 0, rough: 0.6 },
  });
  MAT.webbing = MAT.uhmw;

  function gearProfile(n, m, phase = 0) {
    const rp = m * n / 2, ra = rp + m, rf = rp - 1.25 * m, hw = PI / (2 * n), out = [];
    for (let k = 0; k < n; k++) {
      const c = phase + TAU * k / n;
      for (const [da, r] of [[-1.3, rf], [-1.0, rp], [-0.62, ra - 0.12], [-0.45, ra], [0.45, ra], [0.62, ra - 0.12], [1.0, rp], [1.3, rf]]) out.push([c + da * hw, r]);
    }
    return out.map(([a, r]) => [((a % TAU) + TAU) % TAU, r]).sort((p, q) => p[0] - q[0]);
  }
  const fixHoles = (m) => m;


  Object.assign(MAT, {
    cord: { name: 'Шнур Dyneema SK99 Ø3, 12 пасом, термовитягнутий', rho: 0.97, color: [0.90, 0.55, 0.12], metal: 0, rough: 0.6 },
    s174: { name: 'Нержавіюча сталь 17-4PH (H900)', rho: 7.8, color: [0.70, 0.71, 0.73], metal: 1, rough: 0.22 },
    igl: { name: 'Втулка ковзання PTFE-композит (DU / iglidur J)', rho: 1.5, color: [0.80, 0.72, 0.35], metal: 0, rough: 0.6 },
    brg: { name: 'Голчастий підшипник (сталь 100Cr6)', rho: 7.8, color: [0.45, 0.46, 0.48], metal: 1, rough: 0.2 },
    nbr: { name: 'NBR 90, низькотемпературний (до −40 °C)', rho: 1.2, color: [0.08, 0.08, 0.08], metal: 0, rough: 0.9 },
    redSteel: { name: 'Сталь 1.4310 / AISI 303, червоне покриття', rho: 7.9, color: [0.75, 0.12, 0.10], metal: 1, rough: 0.35 },
    strapPA: { name: 'Стропа PA/PES', rho: 1.14, color: [0.16, 0.17, 0.15], metal: 0, rough: 0.9 },
  });
  // names as on the drawings (the v5 table named the grades of the sketch)
  MAT.alDark.name = 'Алюміній EN AW-7075 T651, тверде анодування'; MAT.steel.name = 'Загартована сталь (440C / 100Cr6 / 1.4310 — за кресленням)';
  MAT.ss.name = 'Нержавіюча сталь (440C / 1.4310 / AISI 316 — за кресленням)'; MAT.spring.name = 'Пружинна сталь 1.4310 (EN 10270-3 / EN 10151)';
  MAT.tpu.name = 'TPU 95A (губи-ущільнювачі)'; MAT.ti.name = 'Титан Ti-6Al-4V Grade 5 (ВТ6)'; MAT.red.name = 'Алюміній EN AW-7075, червоне анодування';

  // ---- 2D helpers. Points are [z, x]; polar angle about a centre is measured from +Z toward +X.
  const pol = (C, r, a) => [C[0] + r * Math.cos(a), C[1] + r * Math.sin(a)];
  const hexP = (C, S, ph = 0) => Array.from({ length: 6 }, (_, k) => pol(C, S / Math.sqrt(3), ph + k * Math.PI / 3));   // hexagon, across flats S
  const HX = { m10: 9.989, f10: 10.024, m7: 6.989, f7: 7.024 };   // S10 h8 / F8, S7 h8 / F8 at mid-tolerance
  const arcPts = (C, r, a0, a1, n) => { const o = []; for (let k = 0; k <= n; k++) o.push(pol(C, r, a0 + (a1 - a0) * k / n)); return o; };
  const mapZX = (u, v, w) => [v, w, u], mapZY = (u, v, w) => [w, v, u], mapYX = (u, v, w) => [v, u, w], mapXY = (u, v, w) => [u, v, w];
  const sagS = (z) => P.Rs - Math.sqrt(Math.max(0, P.Rs * P.Rs - z * z));
  const clamp01 = (v) => Math.max(0, Math.min(1, v));
  // disc of radius r0 with round lugs (centres on radius R, radius rl) as a polar profile [[a, r]...] for ringY
  function lugProfile(r0, R, rl, angs, n = 360) {
    const o = [], half = Math.asin(rl / R);
    for (let k = 0; k < n; k++) {
      const a = TAU * k / n; let r = r0;
      for (const aL of angs) { const d = Math.atan2(Math.sin(a - aL), Math.cos(a - aL)); if (Math.abs(d) < half) r = Math.max(r, R * Math.cos(d) + Math.sqrt(Math.max(0, rl * rl - R * R * Math.sin(d) * Math.sin(d)))); }
      o.push([a, r]);
    }
    return o;
  }
  // wavy spring washer about +Y
  function waveWasher(ri, ro, t, yMid, amp, waves, n = 72) {
    const m = new Mesh(), pt = (a, r, y) => [r * Math.sin(a), y, r * Math.cos(a)], f = (a) => yMid + amp * Math.sin(waves * a);
    for (let k = 0; k < n; k++) {
      const a1 = TAU * k / n, a2 = TAU * (k + 1) / n, y1 = f(a1), y2 = f(a2);
      for (const s of [1, -1]) { const h = s * t / 2; m.quad(m.v(pt(a1, ri, y1 + h), [0, s, 0]), m.v(pt(a2, ri, y2 + h), [0, s, 0]), m.v(pt(a2, ro, y2 + h), [0, s, 0]), m.v(pt(a1, ro, y1 + h), [0, s, 0])); }
      for (const [r, s] of [[ro, 1], [ri, -1]]) { const n1 = [s * Math.sin(a1), 0, s * Math.cos(a1)], n2 = [s * Math.sin(a2), 0, s * Math.cos(a2)]; m.quad(m.v(pt(a1, r, y1 - t / 2), n1), m.v(pt(a2, r, y2 - t / 2), n2), m.v(pt(a2, r, y2 + t / 2), n2), m.v(pt(a1, r, y1 + t / 2), n1)); }
    }
    return m;
  }
  // tube about +Y from y0 up to an end face whose height depends on the angle: face(a) -> [yLeft, yRight] (limits before/after a)
  function camTube(ri, ro, y0, face, n = 180) {
    const m = new Mesh(), pt = (a, r, y) => [r * Math.sin(a), y, r * Math.cos(a)];
    for (let k = 0; k < n; k++) {
      const a1 = TAU * k / n, a2 = TAU * (k + 1) / n, f1 = face(a1)[1], f2 = face(a2)[0];
      const o1 = [Math.sin(a1), 0, Math.cos(a1)], o2 = [Math.sin(a2), 0, Math.cos(a2)], i1 = V.mul(o1, -1), i2 = V.mul(o2, -1);
      m.quad(m.v(pt(a1, ro, y0), o1), m.v(pt(a2, ro, y0), o2), m.v(pt(a2, ro, f2), o2), m.v(pt(a1, ro, f1), o1));
      m.quad(m.v(pt(a1, ri, y0), i1), m.v(pt(a2, ri, y0), i2), m.v(pt(a2, ri, f2), i2), m.v(pt(a1, ri, f1), i1));
      m.quad(m.v(pt(a1, ri, y0), [0, -1, 0]), m.v(pt(a2, ri, y0), [0, -1, 0]), m.v(pt(a2, ro, y0), [0, -1, 0]), m.v(pt(a1, ro, y0), [0, -1, 0]));
      const tn = V.norm([-(f2 - f1) / ((ri + ro) / 2 * (a2 - a1)) * Math.cos((a1 + a2) / 2), 1, (f2 - f1) / ((ri + ro) / 2 * (a2 - a1)) * Math.sin((a1 + a2) / 2)]);
      m.quad(m.v(pt(a1, ri, f1), tn), m.v(pt(a2, ri, f2), tn), m.v(pt(a2, ro, f2), tn), m.v(pt(a1, ro, f1), tn));
      const [fl, fr] = face(a1);
      if (Math.abs(fl - fr) > 1e-6) { const s = fr > fl ? -1 : 1, nn = [s * Math.cos(a1), 0, -s * Math.sin(a1)]; m.quad(m.v(pt(a1, ri, fl), nn), m.v(pt(a1, ro, fl), nn), m.v(pt(a1, ro, fr), nn), m.v(pt(a1, ri, fr), nn)); }
    }
    return m;
  }

  function buildParts(opts = {}) {
    const D = Object.assign({}, DEFAULT_D, opts.D || {});
    const K = makeKin(D);
    const parts = [];
    const add = (id, name, group, mat, mesh, extra = {}) => { parts.push(Object.assign({ id, name, group, mat, mesh, explode: [0, 0, 0], qty: 1 }, extra)); };
    const { xd, z1, z2 } = P, C1 = [z1, xd], C2 = [z2, xd];
    const at1 = (m) => m.xf(Mx.T(xd, 0, z1)), at2 = (m) => m.xf(Mx.T(xd, 0, z2));
    const ct = P.coverT, RO = P.roll, e = P.trip.e, G = P.gas, CM = P.cam, A = P.act;
    const yMi = P.Ym + ct, yPi = P.Yp - ct, sz = P.saddleZ, wz = P.wingZ;
    const rodY = G.lever[0] - 1.2;                              // pull rod runs outside the lever plane
    const bossZ = { m: [-36.0, 36.0], p: [-29.5, 29.5] };        // cover-screw bosses on the two cheeks
    const yBossM = P.plateYm[0] - P.boss.len / 2, yBossP = P.plateYp[1] + P.boss.len / 2;
    const sideScrewY = [-56.0, 70.0];

    // ================= saddle (structural lower shell), pad, cover
    {
      // one y-slice of the saddle. pockets: [zc, xc, r]; cells: {a, b, port} (a < b)
      const slice = (S, y0, y1, za, zb, pockets, cells, notch = []) => {
        const topAt = (z) => { let t = P.floor; for (const [zc, xc, r] of pockets) { const d = Math.abs(z - zc); if (d < r) t = Math.min(t, xc - Math.sqrt(r * r - d * d)); } return t; };
        const floorAt = (z) => -sagS(z) + 1.0;
        const bp = [za, zb];
        for (const [zc, xc, r] of pockets) { const h = xc - P.floor; if (h < r) { const d = Math.sqrt(r * r - h * h); for (const z of [zc - d, zc + d]) if (z > za && z < zb) bp.push(z); } }
        const ports = cells.filter(c => c.port);
        for (const c of ports) bp.push(c.a, c.b);
        for (let z = Math.ceil((za + 0.05) / 0.6) * 0.6; z < zb - 0.05; z += 0.6) bp.push(z);
        bp.sort((p, q) => p - q);
        const zs = bp.filter((z, k) => k === 0 || z - bp[k - 1] > 0.02 || ports.some(c => Math.abs(z - c.a) < 1e-9 || Math.abs(z - c.b) < 1e-9)).filter((z, k, arr) => k === 0 || z - arr[k - 1] > 1e-9);
        const inPort = (z) => ports.some(c => z > c.a + 1e-9 && z < c.b - 1e-9);
        const top = [];
        for (let k = zs.length - 1; k >= 0; k--) {
          const z = zs[k], pe = ports.find(c => Math.abs(z - c.a) < 1e-9 || Math.abs(z - c.b) < 1e-9);
          if (pe) { if (Math.abs(z - pe.b) < 1e-9) top.push([z, topAt(z)], [z, floorAt(z)]); else top.push([z, floorAt(z)], [z, topAt(z)]); }
          else top.push([z, inPort(z) ? floorAt(z) : topAt(z)]);
        }
        const bot = zs.map(z => [z, -sagS(z)]);
        const holes = cells.filter(c => !c.port).map(c => {
          const o = [], n = Math.max(2, Math.ceil((c.b - c.a) / 0.6));
          for (let k = 0; k <= n; k++) { const z = c.a + (c.b - c.a) * k / n; o.push([z, floorAt(z)]); }
          for (let k = n; k >= 0; k--) { const z = c.a + (c.b - c.a) * k / n; o.push([z, topAt(z) - 1.0]); }
          return o;
        });
        let poly = top.slice();
        for (const n of notch.filter(q => q.end === 'a')) poly.push([za, n.xc + n.h], [za + n.depth, n.xc + n.h], [za + n.depth, n.xc - n.h], [za, n.xc - n.h]);
        poly = poly.concat(bot);
        for (const n of notch.filter(q => q.end === 'b')) poly.push([zb, n.xc - n.h], [zb - n.depth, n.xc - n.h], [zb - n.depth, n.xc + n.h], [zb, n.xc + n.h]);
        S.append(plate(poly, holes, y0, y1, mapZX));
      };
      // slices with drilled holes: side = pilot holes along z from the wing faces, vert = through holes along x (staircase of 6 sub-slices per hole)
      const sliceH = (S, y0, y1, za, zb, pockets, cells, side = [], vert = []) => {
        const N = 6, br = new Set([y0, y1]);
        for (const h of [...side, ...vert]) for (let k = 0; k <= N; k++) { const y = h.y - h.r + 2 * h.r * k / N; if (y > y0 + 1e-6 && y < y1 - 1e-6) br.add(y); }
        const ys = [...br].sort((a, b) => a - b);
        for (let k = 0; k + 1 < ys.length; k++) {
          const ya = ys[k], yb = ys[k + 1], ym = (ya + yb) / 2, notch = [], cuts = [];
          for (const h of side) { const d = Math.abs(ym - h.y); if (d < h.r) { const hh = Math.sqrt(h.r * h.r - d * d); for (const e of h.ends) notch.push({ end: e, xc: h.x, h: hh, depth: h.depth }); } }
          for (const h of vert) { const d = Math.abs(ym - h.y); if (d < h.r) { const w = Math.sqrt(h.r * h.r - d * d); cuts.push([h.z - w, h.z + w]); } }
          cuts.sort((a, b) => a[0] - b[0]); const ranges = []; let z0 = za;
          for (const [c0, c1] of cuts) { ranges.push([z0, c0]); z0 = c1; } ranges.push([z0, zb]);
          ranges.forEach(([r0, r1], i) => slice(S, ya, yb, r0, r1, pockets, cells.filter(c => c.a >= r0 - 1e-9 && c.b <= r1 + 1e-9),
            notch.filter(n => (n.end === 'a' && i === 0) || (n.end === 'b' && i === ranges.length - 1))));
        }
      };
      const pilotAt = (y) => ({ y, x: 0.3, r: 0.9, depth: 7.0, ends: ['a', 'b'] });   // self-tapping 2.2 mm screws of the cover: pilot O1.8 x 7
      const tongueH = [{ y: 78.5, z: -1.0, r: 1.7 }, { y: 78.5, z: 11.0, r: 1.7 }];   // M3 button-head screws of the strap tongue: O3.4 through
      const cupPk = [[z1, xd, P.ratchet.rOut + 0.5]], gearPk = [[z1, xd, 20.7], [z2, xd, 20.7]];
      const cA = (s, port) => ({ a: s > 0 ? 14.0 : -20.6, b: s > 0 ? 20.6 : -14.0, port }), cB = (s, port) => ({ a: s > 0 ? 21.8 : -30.9, b: s > 0 ? 30.9 : -21.8, port });
      const cC = (s, port) => ({ a: s > 0 ? 32.4 : -42.0, b: s > 0 ? 42.0 : -32.4, port });
      const AB = (port, signs = [-1, 1]) => signs.flatMap(s => [cA(s, port), cB(s, port)]);
      const S = new Mesh();
      const yCup0 = P.ratchet.web[0] - 0.6, yCup1 = P.plateYm[0] + 0.4;   // >= 0.5 mm from the turning cup and release ring (printed part)
      const cr = P.cornerR - ct, endW = (d) => P.HZ - P.cornerR + Math.sqrt(Math.max(0, cr * cr - (cr - d) * (cr - d))) - 0.1;
      slice(S, yMi, yMi + 0.5, -endW(0), endW(0), [], []); slice(S, yMi + 0.5, yMi + 1.0, -endW(0.5), endW(0.5), [], []);
      slice(S, yMi + 1.0, -61.1, -endW(1.0), endW(1.0), [], []); slice(S, -61.1, -60.6, -wz, wz, [], []);
      slice(S, -60.6, yCup0, -wz, wz, [], AB(true, [1]));
      sliceH(S, yCup0, -51.2, -wz, wz, cupPk, AB(false, [1]), [pilotAt(-56.0)]);
      slice(S, -51.2, -50.4, -wz, 35.5, cupPk, AB(false, [1]));
      slice(S, -50.4, -49.4, -wz, 35.5, cupPk, []);
      slice(S, -49.4, yCup1, -sz, sz, cupPk, []);
      slice(S, yCup1, -48.1, -sz, sz, [], []);
      slice(S, -48.1, -45.1, -sz, sz, [], AB(true));
      slice(S, -45.1, 41.5, -sz, sz, [], AB(false));
      slice(S, 41.5, 44.5, -sz, sz, [], AB(true));
      slice(S, 44.5, 45.3, -sz, sz, [], []);
      slice(S, 45.3, 51.5, -sz, sz, gearPk, []);
      slice(S, 51.5, 53.1, -sz, sz, [], []);
      slice(S, 53.1, 54.1, -wz, 35.5, [], []);
      slice(S, 54.1, 57.1, -wz, 35.5, [], [...AB(true), cC(-1, true)]);
      slice(S, 57.1, 60.6, -wz, 35.5, [], [...AB(false), cC(-1, false)]);
      slice(S, 60.6, 61.6, -wz, wz, [], [...AB(false), cC(-1, false)]);
      slice(S, 61.6, 64.6, -wz, wz, [], [...AB(false), cC(-1, false), cC(1, true)]);
      slice(S, 64.6, 66.5, -wz, wz, [], [...AB(false), cC(-1, false), cC(1, false)]);
      sliceH(S, 66.5, 73.5, -wz, wz, [], AB(false), [pilotAt(70.0)]);
      slice(S, 73.5, 76.5, -wz, wz, [], [...AB(false), cC(-1, true), cC(1, true)]);
      sliceH(S, 76.5, 80.5, -wz, wz, [], [...AB(false), cC(-1, false), cC(1, false)], [], tongueH);
      slice(S, 80.5, 83.5, -wz, wz, [], AB(true));
      slice(S, 83.5, yPi - 1.5, -wz, wz, [], []);
      slice(S, yPi - 1.5, yPi - 1.0, -endW(1.0), endW(1.0), [], []); slice(S, yPi - 1.0, yPi - 0.5, -endW(0.5), endW(0.5), [], []); slice(S, yPi - 0.5, yPi, -endW(0), endW(0), [], []);
      { const ySE = P.trip.spring[0] + 0.82 * 5.5;      // guide ribs for the leg of the trip spring (leg rests on the floor top)
        S.append(box(P.floor, P.floor + 0.6, ySE - 1.1, ySE - 0.55, 31.4, 34.2)); S.append(box(P.floor, P.floor + 0.6, ySE + 0.55, ySE + 1.1, 31.4, 34.2)); }
      for (const y of [-56.0, 70.0]) for (const s of [-1, 1]) S.append(box(P.floor, P.floor + 1.2, y - 2.5, y + 2.5, Math.min(s * (wz - 7.5), s * wz), Math.max(s * (wz - 7.5), s * wz)));   // bosses over the pilot holes: 2 mm of wall above the hole
      add('saddle', 'Сідло (силова основа корпусу)', 'housing', 'pacf', S, { explode: [-30, 0, 0],
        func: 'Коробчаста основа з PA12-CF (SLS): рівна підлога зверху, увігнутий низ радіусом 78 мм. Обшивки 1 мм, між ними поздовжні канали; посередині, де товщина лише 2–3 мм, сідло суцільне. Вікна 3 мм на кінцях каналів — для видалення порошку після друку. Між роликами сідло вузьке (63,8 мм) і працює як балка між двома щоками: щоки спираються на підлогу своїми нижніми кромками. На торцях розширюється до 86 мм і закриває відсіки храповика та привода. Циліндричні кишені під чашку храповика і шестерні.',
        dims: `${(yPi - yMi).toFixed(1)} × 86 мм; між роликами 63,8 мм; висота 2…14,9 мм` });
      const Pd = new Mesh();
      const pseg = (y0, y1, za, zb) => { const N = 16, top = [], bot = []; for (let k = 0; k <= N; k++) { const z = za + (zb - za) * k / N; top.push([z, saddleX(z)]); bot.push([z, saddleX(z) - P.padT]); } Pd.append(extrude(top.concat(bot.reverse()), y0, y1, mapZX)); };
      pseg(yMi, -51.2, -wz, wz); pseg(-51.2, -49.4, -wz, 35.5); pseg(-49.4, 53.1, -sz, sz); pseg(53.1, 60.6, -wz, 35.5); pseg(60.6, 75.0, -wz, wz);
      { const ys = [75.0]; for (let k = 0; k <= 8; k++) ys.push(78.5 - 3.5 + 7.0 * k / 8); ys.push(82.0);   // cut-outs O7 for the button heads of the tongue screws
        for (let k = 0; k + 1 < ys.length; k++) { const ym = (ys[k] + ys[k + 1]) / 2, d = Math.abs(ym - 78.5), w = d < 3.5 ? Math.sqrt(12.25 - d * d) : 0;
          if (ys[k + 1] - ys[k] < 1e-6) continue; if (!w) { pseg(ys[k], ys[k + 1], -wz, wz); continue; }
          pseg(ys[k], ys[k + 1], -wz, -1.0 - w); pseg(ys[k], ys[k + 1], -1.0 + w, 11.0 - w); pseg(ys[k], ys[k + 1], 11.0 + w, wz); } }
      pseg(82.0, yPi, -wz, wz);
      add('pad', 'Підкладка EVA 2,5 мм', 'housing', 'eva', Pd, { explode: [-45, 0, 0], func: 'М’який шар під сідлом; він же закриває головки гвинтів язичка D-кільця. Тиск під сідлом ≈ 310 мм рт. ст. — менший, ніж під стрічкою (≈ 510), бо стрічка лягає на стегно одразу біля ролика і сама притискає тканини.', dims: '2,5 мм' });
      const W = new Mesh();
      for (const s of [-1, 1]) W.append(box(-5.4, 1.6, -P.bandZone + 0.6, P.bandZone - 0.6, Math.min(s * (sz + 0.05), s * (sz + 1.0)), Math.max(s * (sz + 0.05), s * (sz + 1.0))));
      add('slotWipers', 'Губи-ущільнювачі щілин стрічки', 'housing', 'tpu', W, { qty: 2, explode: [-35, 0, 0], func: 'TPU-губи на краях сідла із зазором ≈ 0,3 мм до стрічки: не пускають пісок і бруд у щілини там, де стрічка піднімається від ролика до барабана, і не додають тертя (стрічки не торкаються).', dims: '92 × 7 × 0,95 мм' });

      // ---- cover: top with openings, four walls, labyrinth ribs of the release slot
      const sl0 = z1 + (P.HX - ct / 2 - xd) / Math.tan(CM.a0 + CM.throw) - 1.8, sl1 = z1 + (P.HX - ct / 2 - xd) / Math.tan(CM.a0) + 1.8;
      const topHoles = [circ([z1, P.pinA.y], 1.45, 14), rrect(G.rodZ - 1.2, G.rodZ + 1.2, rodY - 1.0, rodY + 1.0, 0.9, 3), rrect(sl0, sl1, CM.y[0] - 0.4, CM.y[1] + 0.4, 0.6, 3),
        ...bossZ.m.map(z => circ([z, yBossM], 1.4, 12)), ...bossZ.p.map(z => circ([z, yBossP], 1.4, 12))];
      const cR = P.cornerR, top = plate(rrect(P.ZM, P.HZ, P.Ym, P.Yp, cR, 5), topHoles, P.HX - ct, P.HX, mapZY);
      const Wl = new Mesh(), wallO = rrect(P.Ym + cR, P.Yp - cR, P.wallX0, P.HX - ct, 0.01, 1), sideH = sideScrewY.map(y => circ([y, 0.3], 1.4, 12));
      Wl.append(plate(wallO, sideH, P.ZM, P.ZM + ct, mapYX));
      Wl.append(plate(wallO, [...sideH, circ([P.valve.y, P.valve.x], 2.6, 16)], P.HZ - ct, P.HZ, mapYX));
      Wl.append(box(P.wallX0, P.HX - ct, P.Ym, P.Ym + ct, P.ZM + cR, P.HZ - cR));
      Wl.append(plate(rrect(P.ZM + cR, P.HZ - cR, P.wallX0, P.HX - ct, 0.01, 1), [circ([z1 + 7.5, xd - 7.5], 1.7, 14), rrect(-10.6, 20.6, 1.8, 3.5, 0.3, 2)], P.Yp - ct, P.Yp, mapZX));
      for (const [zc, yc, a0] of [[P.ZM + cR, P.Ym + cR, PI], [P.HZ - cR, P.Ym + cR, -PI / 2], [P.HZ - cR, P.Yp - cR, 0], [P.ZM + cR, P.Yp - cR, PI / 2]]) {
        const prof = []; for (let k = 0; k <= 6; k++) { const a = a0 + PI / 2 * k / 6; prof.push([zc + cR * Math.cos(a), yc + cR * Math.sin(a)]); }
        for (let k = 6; k >= 0; k--) { const a = a0 + PI / 2 * k / 6; prof.push([zc + (cR - ct) * Math.cos(a), yc + (cR - ct) * Math.sin(a)]); }
        Wl.append(extrude(prof, P.wallX0, P.HX - ct, mapZY));
      }
      const bossA = alongX(tubeY(1.4, 2.6, P.HX, P.HX + 1.5, 18), P.pinA.y, z1);
      const ribs = merge(box(P.HX - ct - 0.3, P.HX - ct, CM.y[0] - 1.25, CM.y[0] - 0.5, sl0 - 1, sl1 + 1), box(P.HX - ct - 0.9, P.HX - ct, CM.y[1] + 0.5, CM.y[1] + 1.25, sl0 - 1, sl1 + 1));
      add('cover', 'Кожух', 'housing', 'pacf', merge(top, Wl, bossA, ribs), { explode: [70, 0, 0], transparentInCut: true,
        func: 'Тонкостінний кожух 0,9 мм з PA12-CF: захищає механізм від бруду й ударів, навантажень стрічки не несе. Чотири гвинти зверху йдуть у бобишки щік, чотири бокові — у крила сідла: кожух і сідло стягнуті навколо рами. Бокові стінки закінчуються над роликами — там стрічка проходить під роликами. Отвори: штифт A чеки (з напрямною бобишкою), тяга важеля проколу, щілина ручки послаблення з двома ребрами-лабіринтом, резервна кнопка (збоку), гвинт скидання і язичок D-кільця (на торці).',
        dims: `${(P.Yp - P.Ym).toFixed(1)} × ${(P.HZ - P.ZM).toFixed(0)} × ${(P.HX - P.wallX0).toFixed(1)} мм, стінка 0,9 мм` });
      // fasteners (visual; holes and threads are not modelled)
      const scr = new Mesh(), screw = (L, hr = 2.25, r = 1.25) => lathe([[0, 0], [hr, 0], [hr, 1.4], [r, 1.4], [r, 1.4 + L], [0, 1.4 + L]], 14);
      for (const z of bossZ.m) scr.append(screw(5), Mx.chain(Mx.T(P.HX + 1.4, yBossM, z), Mx.RZ(PI / 2)));
      for (const z of bossZ.p) scr.append(screw(5), Mx.chain(Mx.T(P.HX + 1.4, yBossP, z), Mx.RZ(PI / 2)));
      for (const y of sideScrewY) for (const s of [-1, 1]) scr.append(screw(6, 2.0, 1.1), Mx.chain(Mx.T(0.3, y, s * (P.HZ + 1.4)), Mx.RX(-s * PI / 2)));
      for (const a of G.lugA) { const q = pol(C2, G.lugR, a); scr.append(screw(5.2), Mx.T(q[1], G.flange[0] - 1.4, q[0])); }
      add('screws', 'Гвинти кожуха, фланця газового блока', 'housing', 'steelBlk', scr, { qty: 12, func: 'Кожух: 4 × M2,5×5 згори в бобишки щік і 4 самонарізи 2,2×6 збоку в крила сідла. Фланець газового блока: 4 × M2,5×6 крізь фланець осі в щоку з бобишкою (різьба 2,8 мм) — саме вони тримають силу балончика до 1,2 кН (запас зрізу різьби ≈ 9).', dims: 'M2,5; 2,2×6' });
      // belt strap: sewn through the slot of a titanium tongue that is screwed to the saddle floor (+Y end)
      const zc = 5, yT0 = 74.0, yT1 = P.Yp + 6.5, tT = 1.0;
      const tng = plate(rrect(zc - 15, zc + 15, yT0, yT1, 2.0, 3), [rrect(zc - 13, zc + 13, yT1 - 4.4, yT1 - 1.8, 1.0, 3)], P.floor, P.floor + tT, mapZY);
      for (const z of [zc - 6, zc + 6]) tng.append(alongX(tubeY(1.25, 2.4, P.floor + tT, P.floor + tT + 1.6, 14), 78.5, z));
      add('strapTongue', 'Язичок стропи', 'housing', 'ti', tng, { explode: [0, 30, 0],
        func: 'Титанова пластина 1 мм на підлозі сідла: два гвинти M3 × 6 (ISO 7380) знизу, їхні головки лежать у вирізах Ø7 підкладки у відбортовані різьбові отвори. Виходить крізь проріз у торці кожуха; у її пазу 26 × 2,6 мм зашита петля стропи. В ескізі тут було сталеве D-кільце на вушках стінки кожуха 0,9 мм — вона не витримала б ривка 200 Н; язичок до того ж на 5 г легший за кільце.', dims: '30 × 19 × 1 мм' });
      const sy = yT1 - 3.1, loop = [[P.floor + tT + 0.9, sy + 34], [P.floor + tT + 0.9, sy + 4], [P.floor + tT + 0.75, sy + 1.2], [P.floor + tT / 2, sy - 0.75], [P.floor - 0.75, sy + 1.2], [P.floor - 0.9, sy + 4], [P.floor - 0.9, sy + 34]];
      const lp3 = loop.map(([x, y]) => [x, y, zc]), lup = loop.map((p, k) => { const q0 = loop[Math.max(0, k - 1)], q1 = loop[Math.min(loop.length - 1, k + 1)], tx = q1[0] - q0[0], ty = q1[1] - q0[1], L = Math.hypot(tx, ty); return [ty / L, -tx / L, 0]; });
      add('strap', 'Стропа до пояса (25 мм)', 'housing', 'strapPA', strip(lp3, lup, 25, 1.2), { explode: [0, 60, 0], func: 'Проти сповзання: стропа від язичка йде вздовж стегна вгору до поясного ременя; довжина регулюється під зріст.', dims: '25 × 1,2 мм' });
    }

    // ================= frame: two machined 7075 cheeks, tie tube
    const PG = [0, 1, 2].map(pawlGeo);
    const slotB = (rad) => {     // straight slot for the roller-B journal: 2 mm of travel along e, stops at both ends
      const a = [RO.z - 0.15 * e[0], RO.x - 0.15 * e[1]], b = [RO.z + (P.trip.chord - 0.1) * e[0], RO.x + (P.trip.chord - 0.1) * e[1]], n = [-e[1], e[0]], o = [];
      const ang = Math.atan2(n[1], n[0]);
      for (let k = 0; k <= 8; k++) { const t = ang + PI * k / 8; o.push([a[0] + rad * Math.cos(t), a[1] + rad * Math.sin(t)]); }
      for (let k = 0; k <= 8; k++) { const t = ang + PI + PI * k / 8; o.push([b[0] + rad * Math.cos(t), b[1] + rad * Math.sin(t)]); }
      return o;
    };
    const gasProf = lugProfile(G.flangeR, G.lugR, G.lugr, G.lugA);
    const standPts = P.arbor.standA.map(a => pol(C1, P.arbor.standR, a));
    const earE = [-36.0, 37.5], postA = [A.postA.z, A.postA.x];
    {
      const pz = P.plateZ, pt = P.plateTop;
      const outer = [[-32.2, P.floor], [32.2, P.floor], [32.5, 1.7], [32.5, -9.6], [34.5, -11.6], [41.0, -11.6], [pz, -9.6], [pz, 34.5], [36.5, pt], [-36.5, pt], [-pz, 34.5], [-pz, -9.6], [-41.0, -11.6], [-34.5, -11.6], [-32.5, -9.6], [-32.5, 1.7]];
      const common = [circ([-RO.z, RO.x], 1.5, 14), slotB(1.65), circ(P.trip.piv, 2.05, 14), circ([P.tie.z, P.tie.x], 1.6, 12), circ([P.tube.z, P.tube.x], 1.4, 12)];
      // --- cheek on the ratchet side, with the integral stator of the ratchet
      const lightM = [rrect(-3.0, 8.5, 30, 36, 1.5, 3), rrect(-4.0, 8.5, 6.6, 10.5, 1.5, 3), rrect(-40.6, -36.9, 12.8, 29.2, 1.5, 3), rrect(34.5, 40.5, 26, 32.5, 1.5, 3)];
      const pm = plate(outer, [circ(C1, P.stator.hub[0], 32), circ(C2, P.axle2.ro + 0.03, 48), ...common, ...lightM], P.plateYm[0], P.plateYm[1], mapZX);
      const legPoly = (s) => [[32.5, 5.0], [32.5, -9.6], [34.5, -11.6], [41.0, -11.6], [pz, -9.6], [pz, 9.0], [37.0, 9.0]].map(p => [s * p[0], p[1]]);
      const yPadM = P.plateYm[1] + P.legPadM, yReM = P.plateYm[1] + 1.6;                                               // legs 2.6 mm thick, bosses up to the roller ends
      pm.append(plate(legPoly(-1), [circ([-RO.z, RO.x], RO.pin, 14)], P.plateYm[1], yPadM, mapZX));
      pm.append(plate(legPoly(1), [slotB(1.65), circ(P.trip.piv, 2.05, 14)], P.plateYm[1], yPadM, mapZX));
      pm.append(tubeY(RO.pin, 4.0, yPadM, yReM, 20).xf(Mx.T(RO.x, 0, -RO.z)));
      pm.append(plate(slotB(3.6), [slotB(1.65)], yPadM, yReM, mapZX));
      const ST = P.stator, PW = P.pawl, prof = [];
      for (let k = 0; k < 720; k++) {
        const a = TAU * k / 720; let r = ST.rLow;
        for (const g of PG) {
          const d0 = ((a - g.tipA + 0.11) % TAU + TAU) % TAU, dP = (g.heelA - g.tipA) + 0.11 + 1.31 / PW.rHeel; if (d0 < dP) r = ST.rPocket + 0.3; else if (d0 < dP + ST.buttress) r = ST.r;
          const aS = Math.atan2(g.seat[1] - xd, g.seat[0] - z1), dS = Math.atan2(Math.sin(a - aS), Math.cos(a - aS)); if (Math.abs(dS) < 0.95 / 14.2) r = 13.2;
        }
        prof.push([a, r]);
      }
      pm.append(at1(merge(ringY(prof, ST.rIn, ST.y[0], ST.y[1], false), tubeY(ST.hub[0], ST.hub[1], ST.y[0], ST.y[1], 32), tubeY(ST.hub[1], ST.rIn, ST.y[1] - ST.web, ST.y[1], 40))));
      pm.append(at2(ringY(gasProf, P.axle2.ro + 0.03, G.pad[0], G.pad[1], false)));
      for (const z of bossZ.m) pm.append(box(P.boss.x[0], P.boss.x[1], P.plateYm[0] - P.boss.len, P.plateYm[0], z - P.boss.half, z + P.boss.half));
      add('plateYm', 'Щока рами зі статором храповика', 'frame', 'alDark', pm, { explode: [0, -35, 0],
        func: 'Одна фрезерована деталь із 7075-T6: полотно 1,6 мм і, зовні, статор храповика — кільце Ø34,8 × 6 з трьома гніздами п’ят собачок, маточина з втулкою вала барабана 1 і перемичка 3 мм між ними. Собачка працює як розпірка між гніздом і зубом, а сила від вала повертається в той самий диск — петля сил замкнена в площині. Гвинтів у цьому вузлі немає: у попередньому ескізі статор тримали три M2,5, але для їхніх головок між щокою і ребордою барабана лишалося 0,55 мм. Навколо отвору осі барабана 2 — бобишка 1,2 мм під різьбу гвинтів газового блока. Унизу дві ноги до осей роликів, потовщені до 2,6 мм (сила ролика прикладена за 4 мм від площини щоки і крутить ногу); угорі дві бобишки під гвинти кожуха.',
        dims: '85 × 53 × 1,6 мм; статор Ø34,8 × 6; ноги 2,6 мм' });
      // --- cheek on the drive side
      const lightP = [rrect(-4.5, 5.5, 22, 34, 2.5, 3), rrect(-9.0, 12, 7.0, 10.5, 1.5, 3), rrect(-40.6, -35, 11, 30, 1.5, 3), rrect(34.5, 39, 12, 28.5, 1.5, 3)];
      const pp = plate(outer, [circ(C1, 7.0, 32), circ(C2, P.axle2.noseR, 24), ...common, ...lightP, circ(postA, 1.35, 10), circ(earE, 1.35, 10)], P.plateYp[0], P.plateYp[1], mapZX);
      pp.append(tubeY(7.0, 9.0, P.plateYp[0] - 3.4, P.plateYp[0], 32).xf(Mx.T(xd, 0, z1)));                         // drum-1 bush boss (inside the gear recess)
      pp.append(tubeY(P.axle2.noseR, 6.5, P.plateYp[0] - 2.7, P.plateYp[0], 24).xf(Mx.T(xd, 0, z2)));              // nose boss
      const yPad = P.plateYp[0] - P.legPad, yRe = RO.y[1] + 0.3;
      pp.append(plate(legPoly(-1), [circ([-RO.z, RO.x], RO.pin, 14)], yPad, P.plateYp[0], mapZX));
      pp.append(plate(legPoly(1), [slotB(1.65), circ(P.trip.piv, 2.05, 14)], yPad, P.plateYp[0], mapZX));
      pp.append(tubeY(RO.pin, 4.0, yRe, yPad, 20).xf(Mx.T(RO.x, 0, -RO.z)));
      pp.append(plate(slotB(3.6), [slotB(1.65)], yRe, yPad, mapZX));
      pp.append(plate(arcPts(C1, 13.9, 160 * DEG, 476 * DEG, 60).concat(arcPts(C1, 12.75, 476 * DEG, 160 * DEG, 60)), [], P.plateYp[1], P.plateYp[1] + 1.5, mapZX));   // pilot ring of the actuator bell
      for (const z of bossZ.p) pp.append(box(P.boss.x[0], P.boss.x[1], P.plateYp[1], P.plateYp[1] + P.boss.len, z - P.boss.half, z + P.boss.half));
      add('plateYp', 'Щока рами (сторона привода)', 'frame', 'alDark', pp, { explode: [0, 30, 0],
        func: 'Друга щока: втулка вала барабана 1 (бобишка заходить у виточку шестерні), гніздо носка осі барабана 2. Ноги тут потовщені до 4 мм, а бобишки цапф роликів довгі (2,95 мм): між роликом і щокою стоять шестерні, і без потовщення нога гнулася б із площини (≈ 570 МПа). Зовні — центрувальний буртик дзвона привода і дві бобишки гвинтів кожуха.',
        dims: '85 × 53 × 1,6 мм; ноги 4 мм' });
      add('tieRod', 'Стяжка рами', 'frame', 'alDark', tubeY(1.25, 2.5, P.plateYm[1], P.plateYp[0], 16).xf(Mx.T(P.tie.x, 0, P.tie.z)), { explode: [25, 0, 0], func: 'Трубка Ø5/Ø2,5 між щоками над зачепленням шестерень; на обох торцях різьба M3 × 6, гвинти M3 × 5 крізь щоки. (Тонкостінна Ø5/Ø4 різьби M3 не тримала б.)', dims: 'Ø5/Ø2,5 × 98,8 мм, M3 × 6 з торців' });
      const b1 = merge(tubeY(6.03, ST.hub[0] - 0.01, ST.y[0], ST.y[1], 48), tubeY(6.03, 6.99, P.plateYp[0] - 3.4, P.plateYp[1], 48));   // DU-type bushes 12 x 14
      add('bush1', 'Втулки вала барабана 1', 'frame', 'igl', at1(b1), { qty: 2, explode: [0, 0, -25], func: 'PTFE-композитні втулки Ø12: одна в маточині статора, друга в бобишці щоки. У режимі утримання (газ втрачено) втулка статора несе силу однієї собачки — до 1,4 кН, 19 МПа.', dims: 'Ø12/Ø14,2 × 6 і × 5 мм' });
      const tw = P.thrustT, yf = P.flange[0];
      const thr = merge(at1(tubeY(7.3, 16.0, yf - 0.1 - tw, yf - 0.1, 40)), at2(tubeY(P.axle2.ro + 0.2, 16.0, yf - 0.1 - tw, yf - 0.1, 40)),
        at1(tubeY(7.3, 9.0, P.gear.y[0] + P.gear.web + 0.08, P.gear.y[0] + P.gear.web + 0.08 + tw, 32)), at2(tubeY(4.2, 6.4, 48.05, 48.05 + tw, 28)));
      add('thrust', 'Упорні шайби барабанів', 'frame', 'igl', thr, { qty: 4, explode: [0, 0, 0], func: 'Чотири шайби 0,4 мм із PTFE-композиту між торцями барабанів і щоками: осьовий люз барабана ≈ 0,3 мм, стрічка не затирає реборду об щоку.', dims: '0,4 мм' });
    }

    // ================= drum 1 (winds clockwise = -theta): barrel, spring cap, gear cap, ratchet cup, clutch disk
    {
      const S1 = P.shaft1, SP = P.spring, g = P.gear, RT = P.ratchet;
      add('barrel1', 'Барабан 1 (кінець A)', 'drum1', 'cfrp', at1(tubeY(P.R0 - P.barrelT, P.R0, P.barrel[0], P.barrel[1], 56)), { explode: [0, 0, -35],
        func: 'Вуглепластикова труба Ø30 × 0,8 з намоткою ±45°. Намотує тільки кінець A: 0,3 мм на оберт, тож радіус рулону змінюється лише з 15,3 до 15,8 мм. Стрічка підходить збоку (вертикально від ролика) і приклеєна до труби на півоберта. Крутний момент між кінцями труби передає сама труба (21 МПа при 60 допустимих).',
        dims: 'Ø30 × 0,8 × 91,5 мм' });
      const capM = lathe([[S1.ri, RT.hub[1]], [S1.ro, RT.hub[1]], [S1.ro, P.flange[0]], [P.flangeR, P.flange[0]], [P.flangeR, P.flange[1]], [P.R0 - P.barrelT - 0.075, P.flange[1]], [P.R0 - P.barrelT - 0.075, SP.y[1] + 0.5], [SP.Dc / 2, SP.y[1] + 0.5], [SP.Dc / 2, SP.y[0] - 0.2], [S1.ri, SP.y[0] - 0.2], [S1.ri, RT.hub[1]]], 48);
      const capHex = plate(hexP(C1, HX.m10), [circ(C1, S1.ri, 32)], S1.yM, RT.hub[1], mapZX);
      add('capM1', 'Заглушка барабана 1 зі склянкою пружини і валом', 'drum1', 'alDark', merge(at1(capM), capHex), { explode: [0, -30, -30],
        func: 'Одна точена деталь з 7075: реборда Ø34, склянка Ø27,4 для пружини (вклеєна в трубу на 9 мм) і порожнистий вал Ø12/8,3. Вал іде крізь маточину статора; на його шестигранному кінці S10 сидить чашка храповика. Крізь отвір вала проходить нерухомий арбор пружини.',
        dims: 'Ø34 × 22 мм, вал Ø12/Ø8,3' });
      add('spring', 'Заводна пружина 7,5 × 0,5 × 539 мм', 'drum1', 'spring', at1(tubeY(SP.da / 2 + 0.3, SP.Dc / 2 - 0.15, SP.y[0], SP.y[1], 40)), { massOverride: SP.mass, explode: [0, -12, -45],
        func: 'Спіральна стрічкова пружина всередині барабана 1: зовнішній кінець у склянці заглушки, внутрішній — на нерухомому арборі. Заведена на ≈ 3,6 оберта, віддає 0,50 Н·м на початку і 0,35 Н·м після 1,1 оберта — цього досить, щоб вибрати 200 мм слабини й натягнути стрічку до порога датчика (запас 1,37). Пружина постійного моменту з v5 (6 × 0,3 на котушках Ø19/Ø15) фізично не могла дати потрібний момент — стрічка працювала б за межею пружності.',
        dims: '7,5 × 0,5 × 539 мм, Ø27,4/Ø8, 3,9 оберта, 15,9 г' });
      add('springWasher', 'Шайба пружини', 'drum1', 'ptfe', at1(tubeY(S1.ri, SP.Dc / 2 - 0.1, SP.y[1] + 0.05, SP.y[1] + 0.45, 32)), { explode: [0, -6, -45], func: 'Закриває склянку пружини і не дає виткам зміщуватися вздовж осі.', dims: 'Ø27,2 × 0,4 мм' });
      const gp = gearProfile(g.n, g.m, 0);
      const capP = merge(ringY(gp, 15.6, g.y[0], g.y[1], false), lathe([[S1.ro, g.y[0]], [15.7, g.y[0]], [15.7, g.y[0] + g.web], [S1.ro, g.y[0] + g.web], [S1.ro, g.y[0]]], 48),
        tubeY(13.2, P.R0 - P.barrelT - 0.075, P.barrel[1] - 8, g.y[0] + 0.05, 56),
        lathe([[2.5, g.y[0] - 3], [S1.ro, g.y[0] - 3], [S1.ro, P.clutch.disk[0] - 0.05], [2.5, P.clutch.disk[0] - 0.05], [2.5, g.y[0] - 3]], 28));
      const gearHex = plate(hexP(C1, HX.m10), [circ(C1, 2.5, 24)], P.clutch.disk[0] - 0.05, S1.yP, mapZX);
      add('gear1', 'Шестерня 1 із заглушкою і валом (z 30, m 1,25)', 'drum1', 'alDark', merge(at1(capP), gearHex), { explode: [0, 28, -30],
        func: 'Заглушка барабана з боку привода, шестерня і порожнистий вал Ø12 — одна деталь з 7075 (тверде анодування з PTFE). Шестерні зв’язують барабани 1:1, тому обидва кінці стрічки вибираються однаково; торець шестерні править за реборду рулону. Сила в зачепленні 0,47 кН, згин зуба 210 МПа. Контакт зубів — елемент із найменшим запасом у всій конструкції (1,2 у нормальному режимі), його перевіряють ресурсним випробуванням. На шестигранному кінці вала — диск муфти.',
        dims: 'Ø40 × 5 мм, спідниця Ø28,4 × 8, вал Ø12/Ø5' });
      // ratchet cup: tooth ring with a square window for pin A, stiff web, hub
      const pitch = TAU / RT.teeth, inner = [];
      for (let k = 0; k < RT.teeth; k++) for (const gf of [0.0005, 0.04, 0.28, 0.52, 0.76, 0.9995]) inner.push([(k + gf) * pitch, toothR((k + gf) * pitch)]);
      const wO = Math.asin(RT.holeHalf / RT.rOut), wI = Math.asin(RT.holeHalf / RT.rTip), aTop = PI / 2;
      const cPoly = arcPts(C1, RT.rOut, aTop + wO, aTop + TAU - wO, 120);
      const innerC = inner.map(([a, r]) => [a < aTop ? a + TAU : a, r]).filter(([a]) => a > aTop + wI && a < aTop + TAU - wI).sort((p, q) => q[0] - p[0]);
      cPoly.push(pol(C1, toothR(aTop - wI), aTop + TAU - wI));
      for (const [a, r] of innerC) cPoly.push(pol(C1, r, a));
      cPoly.push(pol(C1, toothR(aTop + wI), aTop + wI));
      const cup = merge(at1(ringIn(inner, RT.rOut, RT.y[0], RT.hole[0])), plate(cPoly, [], RT.hole[0], RT.hole[1], mapZX), at1(ringIn(inner, RT.rOut, RT.hole[1], RT.y[1])),
        at1(lathe([[5.9, RT.hub[0]], [RT.hubR, RT.hub[0]], [RT.hubR, RT.webThin], [11.5, RT.webThin], [13.5, RT.web[0]], [RT.rOut, RT.web[0]], [RT.rOut, RT.web[1]], [5.9, RT.web[1]], [5.9, RT.hub[0]]], 64)),
        plate(circ(C1, 5.95, 64), [hexP(C1, HX.f10)], RT.hub[0], RT.web[1], mapZX));
      add('ratchet', 'Чашка храповика (60 внутрішніх зубів)', 'drum1', 'ti', cup, { explode: [0, -48, -30],
        func: 'Титанова (ВТ6) чашка на шестигранному кінці вала барабана 1. Усередині — три собачки в гніздах статора, зсунуті на ⅓ зуба: барабан може віддати назад не більше 2° (≈ 1,1 мм стрічки). Тримає одна собачка: сила стиску до 1,05 кН, з них ≈ 0,53 кН розпирають вінець назовні за 3,6 мм від диска. Тому диск біля вінця потовщено до 2,2 мм, а вінець до 2,0 мм (в ескізі було 0,9 і 1,3 мм — місцевий згин у стику сягав би 500–900 МПа); тепер у корені вінця ≈ 370 МПа, у диску ≈ 285 МПа при межі текучості 880. Це оцінка балкою на пружній основі — вузол треба уточнити МСЕ і перевірити статичним моментом 1,5 × номіналу. У вінці — квадратне вікно 2,2 мм під штифт A чеки, розташоване посередині між собачками.',
        dims: 'Ø42,6 × 7,2 мм, зуб 0,8 мм, вінець 2,0 мм, диск 2,2 мм' });
      const CL = P.clutch;
      add('clutchDisk', 'Диск торцевої муфти', 'drum1', 'ti', plate(circ(C1, CL.r, 54), [hexP(C1, HX.f10)], CL.disk[0], CL.disk[1], mapZX), { explode: [0, 44, -30],
        func: 'Торцеві зуби (54) на шестигранному кінці вала. Поки пружина вибирає слабину, диск обганяє кільце муфти; коли газ тягне шнур, кільце захоплює диск (вільний хід шнура ≤ 1 мм).', dims: 'Ø21 × 0,9 мм, 54 зуби' });
    }

    // ================= pawls with their springs, release ring; spring arbor and its lock
    {
      const PW = P.pawl, RT = P.ratchet;
      const spr = new Mesh();
      PG.forEach((g, k) => {
        const map = (a, b, w) => { const q = g.at(a, b); return [q[1], w, q[0]]; };
        const m = extrude(pawlPoly(g.L), PW.y[0], PW.y[1], map);
        m.append(cylY(0.6, PW.y[1], CM.y[1] - 0.05, 10).xf(Mx.T(g.pin[1], 0, g.pin[0])));
        add('pawl' + (k + 1), 'Собачка храповика ' + (k + 1), 'pawl' + (k + 1), 'steel', m, { explode: [0, -36, 0], pivot: [g.heel[1], 0, g.heel[0]], pawl: g,
          func: 'Загартована сталева розпірка 7 × 2,6 × 3,8 мм: кругла п’ята лежить у гнізді статора, носок — у западині зуба. Лінія сили нахилена до зуба так, що навантаження саме втягує носок у западину. Бічний штифт Ø1,2 заходить у вікно кільця послаблення.', dims: '7 × 2,6 × 3,8 мм' });
        const aS = Math.atan2(g.seat[1] - xd, g.seat[0] - z1), rS = Math.hypot(g.seat[0] - z1, g.seat[1] - xd), ym = (PW.y[0] + PW.y[1]) / 2;
        const p0 = pol(C1, 13.3, aS), p1 = pol(C1, rS - 0.75, aS);
        spr.append(sweep([[p0[1], ym, p0[0]], [p1[1], ym, p1[0]]], 0.75, 8));
      });
      add('pawlSprings', 'Пружини собачок', 'frame', 'spring', spr, { qty: 3, explode: [0, -30, 0], func: 'Три пружини стиску Ø1,5 × 3 у пазах гнізд статора: притискають носки собачок до зубів (≈ 0,3 Н). Замість однієї дротяної пружини-кільця: відмова однієї пружини не вимикає решту собачок.', dims: 'Ø1,5 × 3 мм' });
      // release ring: three windows with inward ramps, fan-shaped dirt shield, stem and low knob (drawn in the locked position)
      const cprof = [];
      for (let k = 0; k < 720; k++) {
        const a = TAU * k / 720; let r = CM.rIn;
        for (const g of PG) { const pa = Math.atan2(g.pin[1] - xd, g.pin[0] - z1), pr = Math.hypot(g.pin[0] - z1, g.pin[1] - xd), d = Math.atan2(Math.sin(a - pa), Math.cos(a - pa)); if (d > -(CM.throw + 0.10) && d < 0.14) r = Math.max(r, pr + 0.75 - 0.95 * clamp01((-d - 0.04) / (CM.throw - 0.06))); }
        cprof.push([a, r]);
      }
      const a0 = CM.a0, dir = [Math.cos(a0), Math.sin(a0)], nrm = [-Math.sin(a0), Math.cos(a0)];
      const bar = (r0, r1, hw) => [[r0, -hw], [r1, -hw], [r1, hw], [r0, hw]].map(([r, t]) => [z1 + dir[0] * r + nrm[0] * t, xd + dir[1] * r + nrm[1] * t]);
      const fan = arcPts(C1, CM.fanR, a0 - CM.fanHalf, a0 + CM.fanHalf, 24).concat(arcPts(C1, CM.rOut - 0.15, a0 + CM.fanHalf, a0 - CM.fanHalf, 24));
      const camM = merge(at1(ringIn(cprof, CM.rOut, CM.y[0], CM.y[1])), plate(fan, [], CM.y[0], CM.y[1], mapZX), plate(bar(CM.fanR - 0.2, CM.knobR[0] + 0.2, 1.1), [], CM.y[0], CM.y[1], mapZX),
        plate(bar(CM.knobR[0], CM.knobR[1], CM.knobHalf), [], CM.knobY[0], CM.knobY[1], mapZX));
      add('cam', 'Кільце послаблення з ручкою', 'cam', 'red', camM, { explode: [25, -30, 0],
        func: 'Плоске кільце між щокою і чашкою, сидить на статорі. Поворот ручки на 24° трьома скосами втягує всі собачки в гнізда. Порядок послаблення: спершу перевести ручку (поки газ тримає силу, собачки розвантажені — до 3 Н), потім повільно відкручувати гвинт скидання: натяг спадає плавно разом із тиском. Якщо газу вже немає, собачка навантажена: на ручці ≈ 35 Н (хід 10,6 мм), стрічка відпускається одразу. Ручка низька (3,3 мм над кожухом), ходить симетрично відносно вертикалі й у замкненому положенні тримається фіксатором у щілині кожуха; віяло під щілиною закриває її від бруду. Зверху ручку прикриває стропа чеки.',
        dims: 'Ø41,8 × 1 мм, ручка на r = 24,5…26,3 мм, хід 24° (10 мм)' });
      const AR = P.arbor;
      add('arbor', 'Арбор пружини (нерухомий)', 'frame', 'alDark', at1(merge(lathe([[0, AR.lock[1]], [AR.r, AR.lock[1]], [AR.r, AR.y[1]], [AR.bore, AR.y[1]], [AR.bore, AR.lock[1] + 1.5], [0, AR.lock[1] + 1.5]], 24), cylY(2.0 / Math.cos(PI / 6), AR.y[0], AR.lock[1], 6))), { explode: [0, -62, -15],
        func: 'Трубчастий стрижень Ø8/Ø4 з 7075 (тверде анодування) крізь порожнистий вал барабана: на ньому закріплений внутрішній кінець пружини. На складанні арбор заводять шестигранним ключем 4 мм на ≈ 3,6 оберта і надягають фіксатор із восьмигранним отвором (крок 45°). Ручного дотягування стрічки через нього немає: для оклюзії потрібно 12–17 Н·м.',
        dims: 'Ø8/Ø4 × 25 мм, хвостовик шестигранник S4' });
      const lk = plate(hull([...circ(C1, 4.6, 16), ...circ(standPts[0], 3.0, 12), ...circ(standPts[1], 3.0, 12)]), [Array.from({ length: 24 }, (_, k) => pol(C1, (k % 2 ? 0.8966 : 1) * 2.025 / Math.cos(PI / 6), k * PI / 12)), circ(standPts[0], 1.3, 10), circ(standPts[1], 1.3, 10), circ([z1 - 14.5, xd], 5.0, 20)], AR.lock[0], AR.lock[1], mapZX);
      for (const p of standPts) lk.append(tubeY(1.3, AR.standr, AR.lock[1], P.plateYm[0], 12).xf(Mx.T(p[1], 0, p[0])));
      add('arborLock', 'Фіксатор арбора зі стійками', 'frame', 'alDark', lk, { explode: [0, -70, -10], func: 'Пластина 1,5 мм на двох стійках M2,5 від щоки: тримає хвостовик арбора (реактивний момент пружини 0,5 Н·м) і прикриває чашку храповика з торця.', dims: '1,5 мм, стійки Ø4,4 × 11' });
      add('cupWasher', 'Шайба чашки храповика', 'frame', 'ptfe', at1(tubeY(AR.r + 0.2, 5.6, P.shaft1.yM - 0.05 - 1.7, P.shaft1.yM - 0.05, 24)), { explode: [0, -56, -15], func: 'Втулка-шайба між маточиною чашки і фіксатором арбора: не дає чашці зійти з шестигранника вала.', dims: 'Ø8,4/Ø11,2 × 1,7 мм' });
    }

    // ================= drum 2 (winds counter-clockwise = +theta) on the fixed capsule axle
    {
      const AX = P.axle2, g = P.gear, gp = gearProfile(g.n, g.m, TAU / g.n / 2);
      add('barrel2', 'Барабан 2 (кінець Б)', 'drum2', 'cfrp', at2(tubeY(P.R0 - P.barrelT, P.R0, P.barrel[0], P.barrel[1], 56)), { explode: [0, 0, 35], func: 'Друга вуглепластикова труба Ø30: обертається навколо нерухомої осі з балончиком і намотує кінець Б.', dims: 'Ø30 × 0,8 × 91,5 мм' });
      add('cap2M', 'Заглушка барабана 2 з ребордою', 'drum2', 'alDark', at2(lathe([[12.25, P.flange[0]], [P.flangeR, P.flange[0]], [P.flangeR, P.flange[1]], [P.R0 - P.barrelT - 0.075, P.flange[1]], [P.R0 - P.barrelT - 0.075, P.barrel[0] + 6], [12.25, P.barrel[0] + 6], [12.25, P.flange[0]]], 48)), { explode: [0, -22, 30], func: 'Кільце з ребордою, вклеєне в трубу; у ньому сидить втулка ковзання.', dims: 'Ø34 × 7,2 мм' });
      add('bush2', 'Втулка барабана 2 на осі Ø22,8', 'drum2', 'igl', at2(tubeY(AX.ro + 0.04, 12.25, P.flange[0], P.flange[0] + 6, 48)), { explode: [0, -30, 40], func: 'PTFE-композит: несе половину сили стрічки (0,28 кН) при 2,7 МПа. Шестерню винесено на інший кінець барабана, де стоїть голчастий підшипник малого радіуса — тертя барабана 2 лише 1,5 %.', dims: 'Ø22,9/Ø24,5 × 6 мм' });
      const cap = merge(ringY(gp, 15.6, g.y[0], g.y[1], false), lathe([[7.6, g.y[0]], [15.7, g.y[0]], [15.7, g.y[0] + g.web], [7.6, g.y[0] + g.web], [7.6, g.y[0]]], 48),
        tubeY(13.2, P.R0 - P.barrelT - 0.075, P.barrel[1] - 8, g.y[0] + 0.05, 56), tubeY(6.0, 7.7, 37.8, 48.0, 32));
      add('gear2', 'Шестерня 2 із заглушкою (z 30, m 1,25)', 'drum2', 'alDark', at2(cap), { explode: [0, 28, 30], func: 'Обертає барабан 2 назустріч барабану 1. У маточині — голчастий підшипник на сталевому носку осі.', dims: 'Ø40 × 5 мм, маточина Ø15,4 × 10' });
      add('needle2', 'Голчастий підшипник HK0810', 'drum2', 'brg', at2(tubeY(AX.noseR + 0.02, 5.96, 37.9, 47.9, 40)), { massOverride: 3.0, explode: [0, 36, 40], func: 'Стандартний голчастий підшипник 8 × 12 × 10: несе половину сили стрічки і всю силу зачеплення (до 1 кН при C0 ≈ 3,9 кН).', dims: '8 × 12 × 10 мм' });
      const y0a = AX.flange[0], axle = merge(lathe([[AX.ri, y0a], [AX.ro, y0a], [AX.ro, AX.plug + AX.bottomT], [AX.bossR, AX.plug + AX.bottomT], [AX.bossR, AX.bossEnd], [AX.noseR, AX.bossEnd], [AX.noseR, AX.nose[0]], [2.0, AX.nose[0]], [2.0, AX.plug], [AX.ri, AX.plug], [AX.ri, y0a]], 48),
        ringY(gasProf, AX.ro - 0.05, AX.flange[0], AX.flange[1], false));
      add('axle2', 'Нерухома вісь барабана 2 (камера балончика)', 'gas-static', 'alDark', at2(axle), { cut: true, explode: [0, 0, 55],
        func: 'Труба Ø22,8 × 0,65 з 7075 з дном 2 мм, бобишкою під цапфу і фланцем 1 мм на зовнішньому кінці. Фланець затиснутий між бобишкою щоки і фланцем газового блока чотирма гвинтами M2,5: балончик тисне на дно осі, газовий блок тягне в інший бік — петля сил (до 1,2 кН при +50 °C) замикається через фланець, труба працює на розтяг 25 МПа. В ескізі вісь була гладкою трубою без осьового упору.',
        dims: 'Ø22,8/Ø21,5 × 84,3 мм, фланець 1 мм' });
      add('nose2', 'Носок осі Ø8', 'gas-static', 'steel', at2(tubeY(AX.noseBore, AX.noseR, AX.nose[0], AX.nose[1], 24)), { cut: true, explode: [0, 40, 55], func: 'Загартована трубчаста цапфа Ø8/Ø5, запресована в дно осі та у щоку: доріжка голчастого підшипника і друга опора осі. Згин 95 МПа (130 у режимі мембрани).', dims: 'Ø8/Ø5 × 25,9 мм' });
      const cz = P.capsule.y0, cL = P.capsule.L;
      const capProf = [[0, cz], [3.2, cz], [4.35, cz + 0.6], [4.35, cz + 8], [7.5, cz + 12], [10.25, cz + 19], [10.25, cz + cL - 5], [8, cz + cL - 1.4], [4, cz + cL], [0, cz + cL]];
      add('capsule', 'Балончик iSi Nitro N₂ 2,4 г, 15 мл', 'gas', 'capsule', at2(lathe(capProf, 48)), { massOverride: 27.8, explode: [0, 0, 75],
        func: 'Серійний одноразовий балончик (паспорт iSi: Ø20,5 × 69 мм, горловина Ø8,7, ковпачок 0,20 мм, 27,8 г; ≈ 153 бар при 20 °C). Витрата газу на спрацювання — до 43 %.', dims: 'Ø20,5 × 69 мм' });
      add('capSpring', 'Пружина балончика', 'gas-static', 'spring', at2(helix(6.3, 0.5, 2.35, 2, G.face + 0.5, 20, 8)), { explode: [0, -8, 60], func: 'Притискає балончик дном до дна осі (≈ 20 Н), щоб голка не втрачала хід на осьовому люзі: довжина балончиків гуляє на ±0,3 мм.', dims: 'Ø13,6, дріт Ø1' });
    }

    // ================= hold-down exit rollers; roller B floats on the trip lever
    {
      const rol = () => merge(tubeY(RO.r - RO.wall, RO.r, RO.y[0], RO.y[1], 32), tubeY(RO.pin + 0.6, RO.r - RO.wall, RO.y[0], RO.y[0] + 3, 20), tubeY(RO.pin + 0.6, RO.r - RO.wall, RO.y[1] - 3, RO.y[1], 20));
      const bush = () => merge(tubeY(RO.pin + 0.03, RO.pin + 0.6, RO.y[0] + 0.3, RO.y[0] + 3, 16), tubeY(RO.pin + 0.03, RO.pin + 0.6, RO.y[1] - 3, RO.y[1] - 0.3, 16));
      const rf = 'Труба Ø10 × 0,8 з 7075 на двох загартованих цапфах Ø3 у PTFE-композитних втулках. Стрічка проходить ПІД роликом і піднімається вертикально до барабана: вона лежить на стегні аж до ролика, без «містка» над шкірою (у v5 стрічка відривалася від стегна за 45 мм до модуля і терлася об край щілини). Сила на ролик ≈ 1,1 × натягу (0,6 кН), згин труби 145 МПа (190 у режимі мембрани).';
      add('rollerA', 'Притискний ролик A', 'housing-int', 'alDark', rol().xf(Mx.T(RO.x, 0, -RO.z)), { explode: [-12, 0, -30], func: rf, dims: 'Ø10 × 91,2 мм' });
      add('rollerBushA', 'Втулки ролика A', 'housing-int', 'igl', bush().xf(Mx.T(RO.x, 0, -RO.z)), { qty: 2, explode: [-12, 0, -36], func: 'Втулки Ø3 × 2,7: тертя ролика — 2,7 % натягу.', dims: 'Ø3/Ø4,2 × 2,7 мм' });
      add('rollerPinsA', 'Цапфи ролика A', 'frame', 'steel', merge(cylY(RO.pin, P.plateYm[0], RO.y[0] + 3.2, 14), cylY(RO.pin, RO.y[1] - 3.2, P.plateYp[1], 14)).xf(Mx.T(RO.x, 0, -RO.z)), { qty: 2, explode: [-12, 0, -42], func: 'Загартовані штифти Ø3, запресовані в бобишки щік; виліт до середини втулки лише 2 мм (згин ≈ 230 МПа, у режимі мембрани 310).', dims: 'Ø3 × 6,7 і × 10,5 мм' });
      add('rollerB', 'Притискний ролик Б (датчик натягу)', 'trip', 'alDark', rol().xf(Mx.T(RO.x, 0, RO.z)), { explode: [-12, 0, 30],
        func: 'Такий самий ролик, але його цапфи сидять на важелі датчика і ходять у прорізах щік. Натяг стрічки зсуває ролик на 2 мм убік (уздовж шкіри); при ≈ 5 Н важіль відкриває клапан подачі газу. Після цього цапфи впираються в кінці прорізів і всю силу несе рама, а не важіль.', dims: 'Ø10 × 91,2 мм' });
      add('rollerBushB', 'Втулки ролика Б', 'trip', 'igl', bush().xf(Mx.T(RO.x, 0, RO.z)), { qty: 2, explode: [-12, 0, 36], func: 'Втулки Ø3 × 2,7.', dims: 'Ø3/Ø4,2 × 2,7 мм' });
      const T = P.trip, VV = P.valve, armPoly = hull([...circ(T.piv, 2.7, 14), ...circ([RO.z, RO.x], 2.6, 14)]);
      const armM = plate(armPoly, [circ(T.piv, 2.0, 16), circ([RO.z, RO.x], RO.pin, 14)], T.armM[0], T.armM[1], mapZX);
      // valve tab: 2.8 mm above the pivot, bent toward -Y, pushes the valve stem (-z) when the roller moves outward
      armM.append(box(T.piv[1] + T.valveArm - 1.4, T.piv[1] + T.valveArm + 1.4, VV.y - 1.6, T.armM[0], T.piv[0] - 1.25, T.piv[0] - 0.25));
      armM.append(box(T.piv[1] + 2.0, T.piv[1] + T.valveArm + 1.4, T.armM[0], T.armM[1], T.piv[0] - 1.6, T.piv[0] + 1.6));
      const armP = plate(armPoly, [circ(T.piv, 2.0, 16), circ([RO.z, RO.x], RO.pin, 14), circ(T.hole, 0.45, 12)], T.armP[0], T.armP[1], mapZX);
      const shaft = tubeY(1.5, 2.0, T.armM[0], T.shaftEnd, 16).xf(Mx.T(T.piv[1], 0, T.piv[0]));
      const pins = merge(cylY(RO.pin, T.armM[0], RO.y[0] + 3.2, 14), cylY(RO.pin, RO.y[1] - 3.2, T.armP[1], 14)).xf(Mx.T(RO.x, 0, RO.z));
      add('tripLever', 'Важіль датчика натягу (два плеча на торсіонній трубці)', 'trip', 'steel', merge(armM, armP), { explode: [-5, 0, 42], pivot: [T.piv[1], 0, T.piv[0]],
        func: 'Два сталеві плеча 1,3 мм зовні щік, жорстко з’єднані трубкою-віссю; на них цапфи ролика Б. Плече 11,3 мм спрямоване майже по нормалі до шкіри, тому сила на ролику мало залежить від товщини стегна (у межах розміру ±8 %). Поки датчик не спрацював, цапфи висять на важелі й не торкаються прорізів — тертя лише в осі Ø4 (2–3 % порога). Відігнутий язичок на плечі з боку храповика натискає шток клапана: хід ролика 2 мм дає хід язичка 0,57 мм (0,1 мм — вільний хід, 0,47 мм — відкриття клапана).',
        dims: 'плече 11,3 мм, язичок на 3,2 мм' });
      add('tripShaft', 'Торсіонна трубка важеля', 'trip', 'alDark', shaft, { explode: [-5, 0, 48], func: 'Трубка Ø4/Ø3 з 7075 крізь обидві щоки: змушує обидва кінці ролика рухатися разом. На її продовженні з боку привода — пружина порога.', dims: `Ø4/Ø3 × ${(T.shaftEnd - T.armM[0]).toFixed(1)} мм` });
      add('rollerPinsB', 'Цапфи ролика Б', 'trip', 'steel', pins, { qty: 2, explode: [-12, 0, 42], func: 'Штифти Ø3 у плечах важеля; проходять крізь прорізи щік.', dims: 'Ø3 × 8,2 і × 11,9 мм' });
      const H = T.hole, cH = [H[1] - T.piv[1], H[0] - T.piv[0]], dH = Math.hypot(cH[0], cH[1]);   // hole in arm P for leg 1, (x, z) offset from the pivot
      const aT = Math.atan2(cH[1], cH[0]) - Math.acos(2.45 / dH);                                                     // tangent point on the coil (angle from +x toward +z)
      const sp = helix(2.45, 0.4, 0.82, 5.5, T.spring[0], 20, 8).xf(Mx.chain(Mx.T(T.piv[1], 0, T.piv[0]), Mx.RY(-(aT - PI / 2))));
      const ySpE = T.spring[0] + 0.82 * 5.5;
      const tP = [T.piv[1] + 2.45 * Math.cos(aT), T.piv[0] + 2.45 * Math.sin(aT)], eP = [T.piv[1] - 2.45 * Math.cos(aT), T.piv[0] - 2.45 * Math.sin(aT)];
      const leg1 = sweep([[tP[0], T.spring[0], tP[1]], [H[1], T.spring[0], H[0]], [H[1], T.armP[0] + 0.2, H[0]]], 0.4, 8);
      const leg2 = sweep([[eP[0], ySpE, eP[1]], [P.floor + 0.45, ySpE, 34.6], [P.floor + 0.45, ySpE, 32.0]], 0.4, 8);
      add('tripSpringLeg', 'Ніжка 1 пружини порога (у отворі плеча)', 'trip', 'spring', leg1, { qty: '—', explode: [-5, 8, 52], func: 'Частина пружини порога: тангенційна ніжка, кінчик загнутий в отвір Ø0,9 у плечі важеля (поворот разом із важелем).', dims: 'дріт Ø0,8' });
      add('tripSpring', 'Пружина порога датчика', 'trip', 'spring', merge(sp, leg2), { explode: [-5, 8, 52],
        func: 'Пружина кручення (дріт Ø0,8, 5,5 витка) на продовженні трубки: ніжка 1 загнута в отвір Ø0,9 у плечі важеля, ніжка 2 лежить на дні сідла між двома напрямними ребрами. Задає поріг 5 Н (3,75…6,25 з допусками); на її частку припадає 79 % порога, решта — сила відкривання клапана. Одна для всіх розмірів: від уставки S/M/L поріг змінюється лише на 2 %.', dims: 'Ø5,7 × 4,7 мм, 36 Н·мм' });
    }

    // ================= gas block: capsule seat, lance with helix cam and pull lever, regulator, trip valve
    {
      const AX = P.axle2, nk = G.neck, ySl0 = G.camLow - 0.4, ySl1 = G.flange[0] - 0.05;      // slotted part of the neck
      const dO = Math.asin(1.05 / G.neckR), dI = Math.asin(1.05 / G.bore);
      const seg = (s) => arcPts(C2, G.neckR, s + dO, s + PI - dO, 16).concat(arcPts(C2, G.bore, s + PI - dI, s + dI, 8));
      const body = lathe([[G.bore, ySl1], [G.neckR, ySl1], [G.neckR, G.flange[0]], [G.flangeR, G.flange[0]], [G.flangeR, G.flange[1]], [AX.ri - 0.01, G.flange[1]], [AX.ri - 0.01, G.pilot[1]], [9.2, G.pilot[1]], [9.2, G.flange[1] + 0.4], [G.bossR, G.flange[1] + 0.4],
        [G.bossR, G.face], [G.pocketR, G.face], [G.pocketR, G.face - 1.0], [5.55, G.face - 1.0], [5.55, G.face - 3.7], [G.pocketR, G.face - 3.7], [G.pocketR, G.pocket], [2.3, G.pocket], [2.3, G.pocket - 2.3], [G.bore, G.pocket - 2.3], [G.bore, ySl1]], 48);   // gland of the lance seal: counterbore O4.6 x 2.3 from the pocket bottom
      const blk = merge(at2(body), at2(lathe([[G.bore, nk[0]], [G.neckR, nk[0]], [G.neckR, G.lever[0] - 1.25], [G.neckR - 0.3, G.lever[0] - 1.25], [G.neckR - 0.3, G.lever[0] - 0.9], [G.neckR, G.lever[0] - 0.9], [G.neckR, ySl0], [G.bore, ySl0], [G.bore, nk[0]]], 28)), plate(seg(0), [], ySl0, ySl1, mapZX), plate(seg(PI), [], ySl0, ySl1, mapZX), at2(ringY(gasProf, G.flangeR - 0.1, G.flange[0], G.flange[1], false)));
      add('gasBlock', 'Газовий блок (сідло балончика, напрямна голки)', 'gas-static', 'alDark', blk, { cut: true, explode: [0, -30, 40],
        func: 'Точений блок із 7075: фланець із чотирма вушками (гвинти M2,5 крізь фланець осі в щоку), центрувальний поясок в осі, стакан із гніздом горловини балончика і шийка-напрямна голки з двома поздовжніми пазами під поперечний штифт. Горловина ущільнена РАДІАЛЬНИМ кільцем по Ø8,7 — герметичність не залежить від довжини балончика (торцева прокладка з ескізу розкрилася б, щойно тиск відсуне балончик на осьовий люз). Канал високого тиску від голки виходить у торець фланця до регулятора.',
        dims: 'фланець Ø24,4 + 4 вушка, шийка Ø8 × 9, стакан Ø13,6' });
      add('capSeal', 'Кільце горловини 8,5 × 1,5 NBR 90 з PTFE-опорним кільцем', 'gas-static', 'nbr', at2(tubeY(4.39, 5.53, G.face - 3.6, G.face - 1.1, 24)), { explode: [0, -12, 40], func: 'O-кільце NBR 90 (до −40 °C) з PTFE-опорним кільцем у канавці стакана: тримає 150–190 бар після проколу.', dims: 'O-кільце 8,5 × 1,5; опорне кільце Ø11,1/Ø8,7 × 0,6' });
      const yTip = P.capsule.y0 - 1.2, yPin = G.camLow + 1.0;
      const lance = merge(at2(lathe([[0, nk[0] + 0.3], [1.5, nk[0] + 0.3], [1.5, yTip - 1.5], [0.6, yTip - 1.2], [0.25, yTip], [0, yTip]], 20)), alongZ(cylY(1.0, z2 - 6.3, z2 + 6.3, 14), xd, yPin));
      add('lance', 'Голка проколу з поперечним штифтом', 'lance', 'ss', lance, { explode: [0, -50, 40],
        func: 'Загартована голка Ø3 з порожнистим вістрям Ø1,2 і фільтром 40 мкм; ущільнена O-кільцем у блоці. Поперечний штифт Ø2 ходить у пазах шийки (голка не обертається) і лежить на двох гвинтових скосах втулки важеля. Хід 3 мм: 1,2 мм до ковпачка і 1,8 мм у нього.', dims: 'Ø3 × 13 мм, штифт Ø2 × 12,6' });
      // piercing lever: plate with an open fork + sleeve whose +Y end face carries two helical ramps (3 mm over 56 deg)
      const aL = PI + G.beta, cA = Math.cos(aL), sA = Math.sin(aL), loc = (u, v) => [z2 + u * cA - v * sA, xd + u * sA + v * cA];
      const hubA = Math.acos((G.sleeveR - 2.65) / G.forkR);                       // tangent from the hub circle to the fork tip
      const lv = [];
      for (let k = 0; k <= 28; k++) { const t = hubA + (TAU - 2 * hubA) * k / 28; lv.push(loc(G.sleeveR * Math.cos(t), G.sleeveR * Math.sin(t))); }
      const rIn = G.arm - 0.15;
      lv.push(loc(G.forkR, -2.65), loc(G.forkR, -1.05), loc(rIn, -1.05));
      for (let k = 1; k < 8; k++) { const t = -PI / 2 - PI * k / 8; lv.push(loc(rIn + 1.05 * Math.cos(t), 1.05 * Math.sin(t))); }
      lv.push(loc(rIn, 1.05), loc(G.forkR, 1.05), loc(G.forkR, 2.65));
      const lev = plate(lv, [circ(C2, G.neckR + 0.1, 24)], G.lever[0], G.lever[1], mapZX);
      const flat = 110 * DEG, face = (a) => { const f = (d) => { let m = ((d + 70 * DEG) % PI + PI) % PI - 70 * DEG; return G.camLow + G.lift * clamp01((m - G.camLead) / G.camSwing); }; return [f(a - 1e-7), f(a + 1e-7)]; };
      lev.append(at2(camTube(G.neckR + 0.1, G.sleeveR, G.lever[1], face, 180)));
      add('pierceLever', 'Важіль проколу з гвинтовим кулачком', 'pierce', 'ti', lev, { explode: [0, -58, 30], pivot: [xd, 0, z2],
        func: 'Пластина 1,6 мм з відкритою вилкою і втулка на шийці газового блока, торець якої має два гвинтові скоси (3 мм на 56°). Чека — жорстка деталь і рухається строго вгору, тому її штифт іде по вертикалі, а в важелі ковзає у вилці: плече сталих 25,75 мм. Ривок повертає важіль, скоси штовхають штифт голки — балончик проколює рука, а не заведена пружина: сила ривка ≈ 31 Н при проколі 165 Н (57 Н, якби знадобилося 300 Н). Наприкінці ходу штифт чеки виходить із вилки, а голка лишається в балончику на пласкій ділянці кулачка. У v5 голку мала звільняти засувка датчика натягу, але енергії датчика було вчетверо менше, ніж треба.',
        dims: 'плече 25,75 мм, хід ±28°, втулка Ø13' });
      add('pierceCap', 'Упорна шайба кулачка і стопорне кільце', 'gas-static', 'steel', at2(merge(tubeY(G.neckR + 0.1, 6.0, G.lever[0] - 0.85, G.lever[0] - 0.1, 24), tubeY(G.neckR - 0.27, 5.2, G.lever[0] - 1.22, G.lever[0] - 0.93, 28))), { explode: [0, -66, 40], func: 'Сприймає осьову силу проколу (до 300 Н): шайба і стопорне кільце в канавці шийки (запас канавки ≈ 8).', dims: 'Ø12 × 0,75 мм, кільце Ø8' });
      // regulator (vertical) + trip valve: one block on the cheek, fed from the gas-block flange through a jumper tube
      const RG = P.reg, VV = P.valve;
      const regB = merge(alongX(tubeY(3.2, RG.r, RG.x[0], RG.x[1], 32), RG.y, RG.z), alongZ(tubeY(1.25, VV.r, VV.z[0], VV.z[1], 24), VV.x, VV.y),
        box(VV.x + VV.r - 0.4, RG.x[0] + 0.3, VV.y - 2.0, VV.y + 2.0, RG.z - 3.2, VV.z[1]),
        box(11.0, RG.x[1] - 0.5, P.plateYm[0] - 1.5, P.plateYm[0], RG.z - 5.4, 42.0), box(11.0, RG.x[1] - 0.5, RG.y + 4.8, P.plateYm[0] - 1.4, RG.z - 3.0, RG.z + 3.0), box(xd - 1.6, xd + 1.6, P.plateYm[0] - 4.2, P.plateYm[0] - 1.6, z2 + G.flangeR + 0.55, RG.z - 3.3));
      { // the regulator and valve body is ONE milled part with the gas block (the radial jumper of the sketch did not fit into the 1.4 mm flange)
        const gb = parts.find(q => q.id === 'gasBlock'), web = box(xd - 1.6, xd + 1.6, -55.5, G.flange[0], z2 + 7.25, RG.z - 3.3);
        gb.mesh = merge(gb.mesh, regB, web); gb.name = 'Газовий блок із корпусом редуктора і клапана';
        gb.func = 'Одна фрезерно-токарна деталь із 7075: фланець із чотирма вушками (гвинти M2,5 крізь фланець осі в щоку), центрувальний поясок в осі, стакан із гніздом горловини балончика, шийка-напрямна голки з двома поздовжніми пазами під поперечний штифт, а збоку — корпус поршневого редуктора і клапана подачі на перемичці. Горловина ущільнена радіальним кільцем по Ø8,7, голка — кільцем 3 × 1 у сальнику на дні гнізда. Газ після проколу йде з дна гнізда похилим каналом Ø0,8 крізь перемичку до входу редуктора; вхідний отвір каналу заглушено запресованою кулькою Ø1. Окрема перехідна трубка з ескізу не влазила у фланець 1,4 мм.';
        gb.dims = 'фланець Ø24,4 + 4 вушка, шийка Ø8 × 9, стакан Ø13,6; редуктор Ø11,2 × 13, клапан Ø5,6 × 10';
        add('lanceSeal', 'Ущільнення голки: O-кільце 3 × 1 NBR 90 і PTFE-опорне кільце', 'gas-static', 'nbr', at2(tubeY(1.52, 2.28, G.pocket - 2.3, G.pocket - 0.5, 20)), { explode: [0, -20, 40],
          func: 'Сальник голки на дні гнізда: опорне кільце з боку пазів шийки, O-кільце з боку балончика. Тиск після проколу притискає кільце до голки.', dims: 'O-кільце 3 × 1; опорне кільце Ø4,6/Ø3 × 0,4' });
        add('lanceWasher', 'Шайба сальника голки', 'gas-static', 'steel', at2(tubeY(1.56, 2.3, G.pocket - 0.5, G.pocket, 20)), { explode: [0, -16, 40],
          func: 'Запресована шайба тримає кільця в сальнику; торець урівень із дном гнізда.', dims: 'Ø4,6 × 0,5 мм, отвір Ø3,12' }); }
      add('regPiston', 'Поршень і тарілчасті пружини регулятора', 'gas-static', 'ss', alongX(lathe([[0, RG.x[0] + 1.2], [3.05, RG.x[0] + 1.2], [3.05, RG.x[0] + 6.5], [1.2, RG.x[0] + 6.5], [1.2, RG.x[1] - 1.0], [0, RG.x[1] - 1.0]], 20), RG.y, RG.z), { explode: [-8, -52, 50], func: 'Чутливий поршень Ø6 із PTFE-манжетою і пакет тарілчастих пружин Ø6 (≈ 140 Н); сідло Ø0,3 — вхідний тиск змінює уставку менш ніж на 1 %. Гвинт уставки стопориться і пломбується на стенді.', dims: 'Ø6,1 × 10,8 мм' });
      add('valveStem', 'Шток і кулька клапана подачі', 'gas-static', 'ss', alongZ(lathe([[0, VV.z[0] + 3.0], [1.0, VV.z[0] + 3.0], [1.0, VV.z[0] + 5.0], [0.3, VV.z[0] + 5.0], [0.3, VV.z[1] + VV.stemOut], [0, VV.z[1] + VV.stemOut]], 12), VV.x, VV.y), { explode: [-8, -40, 62],
        func: 'Кулька Ø2 на сідлі, тиск притискає її до сідла. Щоб відкрити, язичок важеля натискає шток силою ≈ 3,2 Н (21 % порога спрацювання). Якщо стрічка не лягла, клапан відкриває резервна кнопка.', dims: 'сідло Ø0,6, шток Ø0,6, хід 0,47 мм' });
      const TU = P.tube, tb = [[VV.x, VV.y, VV.z[0]], [VV.x, VV.y, TU.z], [TU.x, VV.y, TU.z], [TU.x, A.y, TU.z], [TU.x, A.y, z1], [xd - A.chRo - 0.05, A.y, z1]];
      const tps = []; for (let k = 0; k < tb.length - 1; k++) { const n = Math.max(2, Math.round(V.len(V.sub(tb[k + 1], tb[k])) / 3)); for (let j = 0; j < n; j++) tps.push(V.lerp(tb[k], tb[k + 1], j / n)); } tps.push(tb[tb.length - 1]);
      let tl = 0; for (let k = 1; k < tps.length; k++) tl += V.len(V.sub(tps[k], tps[k - 1]));
      add('gasTube', 'Газова трубка Ø2 × 0,4', 'gas', 'ss', sweep(tps, 1.0, 10), { explode: [-15, 0, 0], flowPath: tps, massOverride: PI / 4 * (4 - 1.44) * tl * 7.93e-3, func: 'Від клапана подачі по підлозі сідла між барабанами (крізь отвори в щоках) до камери шківа.', dims: `Ø2 × 0,4 мм, ≈ ${tl.toFixed(0)} мм` });
    }

    // ================= actuator: one-piece body (bell + pulley chamber + cylinder), pulley shaft, clutch ring, piston
    {
      const PU = P.pulley, CL = P.clutch, atCyl = (m) => alongZ(m, A.x, A.y), zJ = A.zB - A.bottom, dzJ = zJ - z1;
      const aO = Math.atan2(Math.sqrt(A.chRo * A.chRo - dzJ * dzJ), dzJ), aI = Math.atan2(Math.sqrt(A.chR * A.chR - dzJ * dzJ), dzJ);
      const aS = Math.atan2(A.slotX[0] - xd, Math.sqrt(A.chR * A.chR - (A.slotX[0] - xd) ** 2));
      const Dout = [...arcPts(C1, A.chRo, PI / 2, TAU - aO, 64), [zJ, A.lumpX], [z1, A.lumpX]];                    // flat-cut D with the lump over the cord slot
      const slab = [[zJ, A.slotX[1]], [zJ, A.lumpX], [z1, A.lumpX], ...arcPts(C1, A.chRo, PI / 2, TAU - aO, 64), ...arcPts(C1, A.chR, TAU - aI, PI / 2, 64), [z1, A.slotX[1]]];
      const isle = [...arcPts(C1, A.chR, aI, aS, 6), [zJ, A.slotX[0]]];
      const earP = hull([...circ(pol(C1, 14.2, Math.atan2(earE[1] - xd, earE[0] - z1)), 2.2, 12), ...circ(earE, 3.2, 14)]);
      const hub = lathe([[6.0, A.wallA[0] + 1.2], [8.3, A.wallA[0] + 1.2], [8.3, A.flangeY], [5.2, A.flangeY], [5.2, A.flangeY - 0.4], [4.05, A.flangeY - 0.4], [4.05, A.flangeY - 1.0], [6.0, A.flangeY - 1.0], [6.0, A.wallA[0] + 1.2]], 40);
      const bell = lathe([[11.2, P.plateYp[1]], [12.6, P.plateYp[1]], [12.6, A.wallA[0] + 1.2], [6.0, A.wallA[0] + 1.2], [6.0, A.wallA[0]], [11.2, A.wallA[0]], [11.2, P.plateYp[1]]], 48);
      const yPostA = A.y - Math.sqrt(A.ro * A.ro - (postA[1] - A.postA.r - A.x) ** 2);
      const bodyM = merge(at1(hub), at1(bell), plate(earP, [circ(earE, 1.35, 10)], P.plateYp[1], P.plateYp[1] + 2.0, mapZX),
        plate(Dout, [circ(C1, 5.2, 28)], A.flangeY, A.chamber[0] - 0.8, mapZX), plate(Dout, [circ(C1, 4.05, 28)], A.chamber[0] - 0.8, A.chamber[0], mapZX),
        plate(slab, [], A.chamber[0], A.chamber[1], mapZX), plate(isle, [], A.chamber[0], A.chamber[1], mapZX),
        plate(circ([A.x, A.y], A.ro, 56), [rrect(A.slotX[0], A.slotX[1], A.chamber[0], A.chamber[1], 0.01, 1)], zJ, A.zB, mapXY),
        atCyl(lathe([[A.ri, A.zB], [A.ro, A.zB], [A.ro, A.zEnd], [A.ri, A.zEnd], ...Array.from({ length: 9 }, (_, k) => { const f = PI / 2 - PI * k / 8; return [A.ri + A.lockR * Math.cos(f), A.zEnd - A.lockZ + A.lockR * Math.sin(f)]; }), [A.ri, A.zB]], 56)), cylY(A.postA.r, P.plateYp[1], yPostA, 24).xf(Mx.T(postA[1], 0, postA[0])));
      for (let y = yPostA; y < A.y - Math.sqrt(Math.max(0, A.ro * A.ro - (postA[1] + A.postA.r - A.x) ** 2)); y += 0.6) {     // the post blends into the round tube: slices clipped by the tube surface
        const xc = A.x + Math.sqrt(Math.max(0, A.ro * A.ro - (A.y - y - 0.6) ** 2)) - 0.03, pc = circ(postA, A.postA.r, 32).filter(q => q[1] > xc);
        if (pc.length >= 3) bodyM.append(plate(pc, [], y, y + 0.6, mapZX));
      }
      const aD = 595.3, mBody = (PI * (8.3 * 8.3 - 36) * 7.9 + PI * (12.6 * 12.6 - 11.2 * 11.2) * 3.45 + PI * (12.6 * 12.6 - 36) * 1.2 + 6 * 12 * 2.0 + (aD - 84.9) * 1.7 + (aD - 51.5) * 0.8 + (aD - 468.7) * 8
        + (PI * A.ro * A.ro - 54.4) * A.bottom + PI * (A.ro * A.ro - A.ri * A.ri) * (A.zEnd - A.zB) + PI * A.postA.r * A.postA.r * (yPostA - P.plateYp[1])) * 2.81e-3;
      add('actBody', 'Корпус привода: дзвін муфти, камера шківа і циліндр Ø23', 'actuator', 'alDark', bodyM, { cut: true, massOverride: mBody, explode: [0, 34, 0],
        func: 'Одна деталь із 7075 (тверде анодування, хонінгована гільза). Камера шківа фрезерована з торця разом із каналом 6,8 × 8 мм, яким шнур іде в циліндр, — шнур ніде не виходить із газового об’єму, тож ущільнювати треба тільки поршень і вал Ø8, а не гнучкий шнур під 50 бар, як у v5. Дзвін закриває муфту й сідає на центрувальний буртик щоки; реактивний момент шнура (до 23 Н·м) тримають стійка під циліндром і вушко — два гвинти M2,5 поза колами шестерень. Тиск до 66 бар (режим мембрани): гільза 96 МПа, стінки камери до 111 МПа.',
        dims: 'камера Ø26,8 × 8, гільза Ø24,6 × 49,9 мм' });
      const cov = merge(plate(Dout, [circ(C1, 6.0, 28)], A.wallB[0], A.wallB[0] + A.coverT, mapZX), at1(lathe([[6.0, A.wallB[0] + A.coverT], [8.3, A.wallB[0] + A.coverT], [8.3, A.wallB[1]], [0, A.wallB[1]], [0, A.wallB[1] - 1.2], [6.0, A.wallB[1] - 1.2], [6.0, A.wallB[0] + A.coverT]], 40)));
      add('actCover', 'Кришка камери шківа', 'actuator', 'alDark', cov, { cut: true, explode: [0, 52, -10], func: 'Закриває камеру після встановлення шківа і шнура (4 гвинти M3, O-кільце по контуру). Глухе гніздо другого підшипника — без ущільнення вала з цього боку.', dims: '28 × 28 × 2,5 мм, маточина Ø16,6 × 6,7' });
      add('needleA', 'Голчастий підшипник HK0808', 'actuator', 'brg', at1(tubeY(4.02, 5.96, A.wallA[0] + 0.05, A.wallA[0] + 8.05, 40)), { massOverride: 2.7, explode: [0, 40, -22], func: 'Опора вала шківа з боку муфти (8 × 12 × 8). Вал несе всю силу шнура — до 2,0 кН, тому тут голчасті підшипники: з втулками ковзання втрачалося б ще 3 % сили.', dims: '8 × 12 × 8 мм' });
      add('needleB', 'Голчастий підшипник BK0808 (глухий)', 'actuator', 'brg', at1(tubeY(4.02, 5.96, A.wallB[0] + 0.05, A.wallB[0] + 7.95, 40)), { massOverride: 3.0, explode: [0, 60, -22], func: 'Друга опора вала в кришці (8 × 12 × 8, із закритим дном). В ескізі тут стояв BK0608 із запасом лише 1,2 — тепер обидві шийки Ø8, запас 1,9.', dims: '8 × 12 × 8 мм' });
      add('shaftSeal', 'O-кільце вала 8 × 1,5', 'actuator', 'nbr', at1(tubeY(4.03, 5.17, A.flangeY - 0.35, A.chamber[0] - 0.85, 28)), { explode: [0, 46, -22], func: 'Єдине рухоме ущільнення, крім поршня. Вал повертається менш ніж на два оберти, тому замість PTFE-манжети — звичайне O-кільце: воно краще тримає тиск годинами. Тертя ≈ 0,2 % моменту.', dims: '8 × 1,5 мм' });
      const shA = [[2.0, A.wallA[0]], [4.0, A.wallA[0]], [4.0, A.chamber[0] + 0.5], [2.0, A.chamber[0] + 0.5], [2.0, A.wallA[0]]];
      const shB = [[2.0, A.chamber[1] - 0.2], [4.0, A.chamber[1] - 0.2], [4.0, A.wallB[1] - 1.4], [0, A.wallB[1] - 1.4], [0, A.wallB[1] - 5.0], [2.0, A.wallB[1] - 5.0], [2.0, A.chamber[1] - 0.2]];
      const shM = merge(at1(lathe(shA, 28)), at1(lathe(shB, 28)), plate(hexP(C1, HX.m7), [circ(C1, 2.0, 24)], CL.ring[0] + 0.15, A.wallA[0], mapZX), plate(hexP(C1, HX.m10), [circ(C1, 2.0, 24)], A.chamber[0] + 0.5, A.chamber[1] - 0.2, mapZX));
      add('pulleyShaft', 'Вал шківа', 'pulley', 'steel', shM, { explode: [0, 48, -34], func: 'Вал із нержавіючої сталі 440C, гартування 58–60 HRC (шийки — доріжки голчастих підшипників, тому твердість не нижче 58 HRC; 17-4PH H900 з ескізу має лише 40–44 HRC): дві шийки Ø8 під голчасті підшипники, між ними шестигранний бурт S10 під шків, зовні шестигранний хвостовик S7 під кільце муфти. Кручення 344 МПа у найважчому режимі; бурт передає на упорну шайбу осьову силу тиску (0,33 кН).', dims: 'Ø8 × 29,6 мм' });
      const pul = merge(at1(lathe([[5.9, PU.y[0]], [PU.rf, PU.y[0]], [PU.rf, PU.y[0] + 1.0], [PU.root, PU.y[0] + 1.9], [PU.root, PU.y[1] - 1.9], [PU.rf, PU.y[1] - 1.0], [PU.rf, PU.y[1]], [5.9, PU.y[1]], [5.9, PU.y[0]]], 48)), plate(circ(C1, 5.95, 48), [hexP(C1, HX.f10)], PU.y[0], PU.y[1], mapZX));
      add('pulley', 'Шків шнура (r = 9 мм)', 'pulley', 'alDark', pul, { explode: [0, 56, -34],
        func: 'Шнур змотується зі шківа радіусом 9 мм, стрічка намотується на радіусі ≈ 15,6 мм: сила газу 2,0 кН дає 2 × 534 Н на кінцях стрічки. За весь хід шків робить 0,67 оберта, тож шнур лежить в один шар. Кінець шнура заведений у радіальний отвір шківа і залитий.', dims: 'Ø22,4 × 7,1 мм' });
      add('thrustWasher', 'Упорна шайба вала', 'pulley', 'igl', at1(tubeY(4.1, 6.6, A.chamber[0] + 0.03, A.chamber[0] + 0.47, 20)), { explode: [0, 52, -34], func: 'Тиск виштовхує вал назовні з силою до 0,33 кН; шайба з PTFE-композиту сприймає її (≈ 1 % моменту).', dims: 'Ø8,2/Ø13,2 × 0,44 мм' });
      add('clutchRing', 'Кільце торцевої муфти', 'pulley', 'ti', merge(plate(circ(C1, CL.r, 54), [hexP(C1, HX.f7)], CL.ring[0], CL.ring[1], mapZX), plate(circ(C1, 5.6, 36), [hexP(C1, HX.f7)], CL.ring[1], CL.hub[1], mapZX)), { explode: [0, 46, -30],
        func: 'Ковзає по шестигранному хвостовику вала шківа і пружинною шайбою притиснуте до диска на валу барабана. Передає момент лише в бік затягування — як зірчастий храповик велосипедної втулки. Зуби несуть 95 МПа, навіть якщо працює третина з 54.', dims: 'Ø21 × 2,1 мм, 54 зуби' });
      add('clutchSpring', 'Хвиляста шайба муфти', 'actuator', 'spring', at1(waveWasher(6.4, 9.6, 0.25, (CL.ring[1] + A.wallA[0]) / 2, 0.38, 3)), { explode: [0, 43, -30], func: 'Тримає кільце муфти притиснутим до диска (≈ 2 Н); при обгоні кільце відходить на 0,5 мм.', dims: 'Ø19,2/Ø12,8 × 0,25 мм' });
      const pz = A.z0, L5 = A.pistonL, ri = A.ri;
      const piston = lathe([[0, pz + 3.6], [3.0, pz + 3.6], [3.0, pz], [ri - 0.05, pz], [ri - 0.05, pz + 1.0], [ri - 1.45, pz + 1.0], [ri - 1.45, pz + 3.8], [ri - 0.05, pz + 3.8], [ri - 0.05, pz + L5], [ri - 2.2, pz + L5], [ri - 2.2, pz + 2.6], [4.5, pz + 2.6], [4.5, pz + 4.6], [0, pz + 4.6]], 48);   // blind M6 x 0.5 for the cord end, 1 mm bottom
      add('piston', 'Поршень Ø23', 'piston', 'alDark', atCyl(piston), { explode: [0, 34, 14], func: 'Поршень із 7075 без наскрізних отворів: з боку тиску — глухий різьбовий отвір M6 × 0,5 глибиною 3,6, у який до упору в дно вкручено втулку шнура (Loctite 243); дно 1 мм, тому газ не може пройти вздовж шнура. З іншого боку — кільцева виточка для полегшення.', dims: 'Ø23 × 5,5 мм' });
      add('pistonSeal', 'PTFE-манжета поршня', 'piston', 'seal', atCyl(tubeY(ri - 1.42, ri - 0.03, pz + 1.02, pz + 3.78, 56)), { explode: [0, 34, 14], func: 'Однобічна U-манжета з PTFE на нержавіючій пружині, −80…+200 °C; тертя 3–6 % сили газу враховано в розрахунку допусків.', dims: 'під Ø23, переріз 1,45 × 2,8' });
      add('cordEnd', 'Залита втулка шнура', 'piston', 'ti', atCyl(lathe([[1.6, pz - 5.0], [2.2, pz - 5.0], [2.8, pz + 0.6], [2.95, pz + 0.6], [2.95, pz + 3.55], [2.1, pz + 3.55], [1.6, pz - 5.0]], 24)), { explode: [0, 34, 8], func: 'Титанова втулка з різьбою M6 × 0,5 на головці: шнур входить з вузького кінця (Ø3,2, кромка R0,3), розпушений кінець залитий епоксидом у конічній порожнині Ø3,2 → Ø4,2 (ефективність закладання ≥ 70 %). Втулку вкручують у поршень до упору торцем у дно отвору; щоб не скручувати шнур, обертають поршень.', dims: 'M6 × 0,5 / Ø4,4 × 8,55 мм' });
      const zG = A.zEnd - A.lockZ, dzG = Math.sqrt(A.lockR ** 2 - 0.03 ** 2), aG = Math.asin(0.03 / A.lockR);
      const plugP = [[1.2, A.zEnd - 1.3], [ri - 2.0, A.zEnd - 1.3], [ri - 2.0, A.zEnd - 3.0], [ri - 0.03, A.zEnd - 3.0], [ri - 0.03, zG - dzG],
        ...Array.from({ length: 7 }, (_, k) => { const f = -PI / 2 + aG + (PI - 2 * aG) * (k + 1) / 8; return [ri - A.lockR * Math.cos(f), zG + A.lockR * Math.sin(f)]; }), [ri - 0.03, zG + dzG], [ri - 0.03, A.zEnd], [1.2, A.zEnd], [1.2, A.zEnd - 1.3]];
      const wire = sweep(Array.from({ length: 70 }, (_, k) => { const a = 0.35 + (TAU - 0.5) * k / 69; return [A.x + ri * Math.cos(a), A.y + ri * Math.sin(a), zG]; }), A.lockWire / 2 - 0.02, 8);
      add('capA', 'Пробка-упор циліндра з дротяним замком', 'actuator', 'alDark', merge(atCyl(lathe(plugP, 48)), wire), { explode: [0, 34, 30], func: 'Пробка-упор Ø23 з вентиляційним отвором: упор поршня в кінці ходу 38 мм (у найгіршому випадку потрібно 35,5). Тримається дротяним замком: дріт Ø0,8 (1.4310) лягає в напівкруглі канавки R0,42 пробки й гільзи і заводиться крізь тангенційний проріз у стінці. Замок тримає повну силу газу на упорі (2,7 кН при тиску мембрани) з запасом ≥ 3; клей Loctite 638 по пояску лише ущільнює. Різьба M24 і чисте вклеювання не годяться: стінка гільзи 0,8 мм, а клей на анодованому 7075 при +50 °C тримає менше за силу газу.', dims: 'Ø23 × 3 мм, дно 1,3 мм; дріт Ø0,8 × ≈ 72 мм' });
      const rvz = z1 - A.chRo;
      add('relief', 'Скидний клапан (1,10 × уставка)', 'actuator', 'brass', alongZ(lathe([[0, -5.6], [2.4, -5.6], [2.4, 0], [0, 0]], 18), xd, A.y).xf(Mx.T(0, 0, rvz)), { explode: [0, 34, -24], func: 'Пружинний клапан на камері шківа, налаштований на стенді на 1,10 × уставки (±5 %): захищає від перевищення тиску, якщо регулятор заклинить відкритим або газ нагріється. Тиск під стрічкою в цьому режимі — до 642 мм рт. ст.', dims: 'Ø4,8 × 5,6 мм' });
      const bdA = 150 * DEG, bd = pol(C1, A.chRo + 0.65, bdA);
      add('burstDisc', 'Розривна мембрана (1,30 × уставка)', 'actuator', 'brass', lathe([[0, -0.65], [2.6, -0.65], [2.6, 0.65], [0, 0.65]], 20).xf(Mx.chain(Mx.T(bd[1], A.y, bd[0]), Mx.RY(bdA), Mx.RX(PI / 2))), { explode: [14, 34, -20], func: 'Останній рубіж, якщо відмовлять і регулятор, і скидний клапан. Запас 30 % над уставкою більший за розкид самих мембран (±7 %) — у v5 «+3 бар» лежало всередині цього розкиду. Тиск під стрічкою в такому подвійному збої — до 691 мм рт. ст.', dims: 'Ø5,2 × 1,3 мм' });
      const bz = z1 + 7.5, bx = xd - 7.5;
      add('vent', 'Гвинт скидання тиску', 'actuator', 'redSteel', lathe([[0, A.wallB[0] + A.coverT], [1.5, A.wallB[0] + A.coverT], [1.5, P.Yp + 0.5], [3.6, P.Yp + 0.5], [3.6, P.Yp + 3.4], [0, P.Yp + 3.4]], 20).xf(Mx.T(bx, 0, bz)), { explode: [0, 62, 0], func: 'Гвинт із накаткою на торці модуля: конусна голка на сідлі в кришці камери. Повільне відкручування плавно знижує тиск, а з ним і натяг стрічки; регулятор при цьому стравлює і решту газу з балончика.', dims: 'M3, головка Ø7,2' });
    }

    // ================= trigger: pin A (drum lock) + pull rod of the piercing lever on one wire frame, soft pull loop; backup button
    {
      const xh = P.pinA.xh, TR = new Mesh();
      const pA = [xh, P.pinA.y, z1], pB = [xh, rodY, G.rodZ];
      TR.append(sweep([[xd + P.ratchet.rRoot + 0.25, P.pinA.y, z1], pA], 1.0, 12));
      TR.append(sweep([[G.rodX0, rodY, G.rodZ], pB], G.rodR, 12));
      TR.append(sweep([[G.rodX0, rodY, G.rodZ], [G.rodX0, G.lever[1] + 0.5, G.rodZ]], G.rodR, 10));
      TR.append(sweep([pA, pB], 1.6, 12));
      add('trigger', 'Чека: штифт A і тяга важеля проколу', 'trigger', 'redSteel', TR, { explode: [40, 0, 0],
        func: 'Жорстка дротяна рамка. Один ривок робить дві речі: штифт A виходить із вікна вінця чашки храповика — пружина починає вибирати слабину; загнутий кінець тяги повертає важіль проколу — голка пробиває балончик, і газ стає перед клапаном подачі. Хід ривка 27 мм (після 30 мм тяга виходить із вилки, і чека лишається в руці), сила ≈ 31 Н.',
        dims: 'штифт Ø2, тяга Ø1,8, перемичка Ø3,2' });
      const zs = (z1 + G.rodZ) / 2, ym = (P.pinA.y + rodY) / 2, path = [[xh + 2.4, ym - 1.6], [xh + 2.7, ym + 0.6], [xh + 2.4, ym + 3.6], [xh + 1.5, CM.knobY[0] + 1.0], [xh + 1.3, CM.knobY[1] + 0.4], [xh - 0.6, -42], [xh - 2.0, -37], [P.HX + 0.95, -30], [P.HX + 0.95, 8]];
      const sp3 = path.map(([x, y]) => [x, y, zs]), ups = path.map((p, k) => { const a = path[Math.max(0, k - 1)], b = path[Math.min(path.length - 1, k + 1)], tx = b[0] - a[0], ty = b[1] - a[1], L = Math.hypot(tx, ty); return [ty / L, -tx / L, 0]; });
      add('pullLoop', 'Стропа чеки (м’яка петля)', 'trigger', 'strapPA', strip(sp3, ups, 20, 1.5), { explode: [48, 0, 0], func: 'Червона стропа 20 мм, пришита навколо перемички чеки, лежить на кожусі під гумкою. М’яка петля не чіпляється за спорядження, як жорстке кільце, і заодно прикриває ручку послаблення від випадкового зсуву.', dims: '20 × 1,5 мм, 65 мм' });
      const VV = P.valve, T = P.trip, bz0 = T.piv[0] - 0.05;
      add('backupButton', 'Резервна кнопка подачі газу', 'backup', 'green', alongZ(lathe([[0, bz0], [1.0, bz0], [1.0, P.HZ + 0.4], [2.4, P.HZ + 0.4], [2.4, P.HZ + 1.9], [0, P.HZ + 1.9]], 16), T.piv[1] + T.valveArm, VV.y), { explode: [0, -8, 30],
        func: 'Якщо пружина не дотягнула стрічку (зачепилась за спорядження) і датчик не спрацював, натиснути й тримати секунду: шток штовхає язичок важеля співвісно зі штоком клапана і відкриває подачу газу.', dims: 'Ø4,8, шток Ø2' });
      add('buttonGuard', 'Обідок кнопки', 'housing', 'pacf', alongZ(tubeY(2.7, 4.2, P.HZ, P.HZ + 2.6, 24), T.piv[1] + T.valveArm, VV.y), { explode: [0, -8, 38], func: 'Кнопка на 0,7 мм нижче обідка — випадково не натиснеш.', dims: 'Ø8,4 × 2,6 мм' });
    }

    for (const p of parts) fixOrientation(p.mesh);
    return { parts, P, K, D, MAT, PG, pawlGeo };
  }

  // ------------------------------------------------------------------ dynamic geometry (v5.1)
  function perpUps(pts, sign) {
    const n = pts.length, ups = [];
    for (let k = 0; k < n; k++) {
      const a = pts[Math.max(0, k - 1)], b = pts[Math.min(n - 1, k + 1)], t = V.norm(V.sub(b, a));
      ups.push(V.mul([-t[2], 0, t[0]], sign));
    }
    return ups;
  }
  // crossing tangent of two circles: circle 1 on the -n side of the line, circle 2 on the +n side; s = +-1 picks one of the two
  function tanCross(C1, r1, C2, r2, s) {
    const D = [C2[0] - C1[0], C2[1] - C1[1]], d = Math.hypot(D[0], D[1]);
    if (d <= r1 + r2) { const n = [D[0] / d, D[1] / d], q = [C1[0] + n[0] * (r1 - (r1 + r2 - d) / 2), C1[1] + n[1] * (r1 - (r1 + r2 - d) / 2)]; return { n, p1: q, p2: q, touch: true }; }
    const a = Math.atan2(D[1], D[0]) + s * Math.acos((r1 + r2) / d), n = [Math.cos(a), Math.sin(a)];
    return { n, p1: [C1[0] + r1 * n[0], C1[1] + r1 * n[1]], p2: [C2[0] - r2 * n[0], C2[1] - r2 * n[1]] };
  }
  // one side of the band in a frame where the roller is at +z: limb -> under the roller -> straight up -> outer side of the drum
  function sidePath(Cl, Rb, Cr, rr, Cd, R) {
    let a = tanCross(Cl, Rb, Cr, rr, 1), b = tanCross(Cl, Rb, Cr, rr, -1);
    const lim = a.p1[0] >= b.p1[0] ? a : b;                 // leaves the limb on its outer side
    a = tanCross(Cd, R, Cr, rr, 1); b = tanCross(Cd, R, Cr, rr, -1);
    const dr = a.p1[0] >= b.p1[0] ? a : b;                  // touches the drum on its outer side
    let a0 = ang2(Cr, lim.p2), a1 = ang2(Cr, dr.p2);        // on the roller: from the limb side clockwise to the drum side
    while (a1 > a0) a1 -= TAU;
    return { limb: lim.p1, rollIn: lim.p2, a0, a1, rollOut: dr.p2, drum: dr.p1, wrap: a0 - a1 };
  }
  const rollerBpos = (G, trip) => { const P = G.P, e = P.trip.e, s = P.trip.travel * (trip || 0); return [P.roll.z + e[0] * s, P.roll.x + e[1] * s]; };
  // band layout in the (z, x) plane: st = { theta, Rb (band radius on the limb), mountX (limb axis below x = 0), trip (0..1) }
  function bandLayout(G, st) {
    const { P } = G, t = P.band.t, R = G.K.rd(st.theta), rr = P.roll.r + t / 2, Cl = [0, -st.mountX];
    const B = sidePath(Cl, st.Rb, rollerBpos(G, st.trip), rr, [P.z2, P.xd], R);
    const m = (p) => [-p[0], p[1]];
    const A0 = sidePath(Cl, st.Rb, [P.roll.z, P.roll.x], rr, [P.z2, P.xd], R);
    const A = { limb: m(A0.limb), rollIn: m(A0.rollIn), a0: PI - A0.a0, a1: PI - A0.a1, rollOut: m(A0.rollOut), drum: m(A0.drum), wrap: A0.wrap };
    return { R, rr, Cl, C1: [P.z1, P.xd], C2: [P.z2, P.xd], RA: [-P.roll.z, P.roll.x], RB: rollerBpos(G, st.trip), A, B, T1: A.drum, T2: B.drum };
  }
  function bandGeometry(G, st, pressureFn) {
    const L = bandLayout(G, st), cx = -st.mountX, Rb = st.Rb;
    const pts = [], tag = [];
    const push = (p) => { if (pts.length && V.len(V.sub(p, pts[pts.length - 1])) < 0.04) return; pts.push(p); };
    const line = (a, b, N) => { for (let j = 1; j <= N; j++) push(V.lerp(a, b, j / N)); };
    const arc2 = (C, r, a0, a1, N) => { for (let j = 0; j <= N; j++) { const a = a0 + (a1 - a0) * j / N; push(zx3([C[0] + r * Math.cos(a), C[1] + r * Math.sin(a)])); } };
    // ply A: drum 1 -> down to roller A -> under it -> limb
    push(zx3(L.A.drum)); line(zx3(L.A.drum), zx3(L.A.rollOut), 12);
    arc2(L.RA, L.rr, L.A.a1, L.A.a0, 12);
    line(pts[pts.length - 1], zx3(L.A.limb), 4);
    // limb arc from A round the far side to B
    const angL = (p) => Math.atan2(p[0], p[1] - cx);      // from +x toward +z
    let b0 = angL(L.A.limb), b1 = angL(L.B.limb);
    while (b0 > 0) b0 -= TAU; while (b0 < -TAU) b0 += TAU;                    // A is on the -z side: go through -pi (the far side) to B
    b1 = b1 - TAU;
    const Narc = 170, arcStart = pts.length;
    for (let j = 0; j <= Narc; j++) { const a = b0 + (b1 - b0) * j / Narc; push([cx + Rb * Math.cos(a), 0, Rb * Math.sin(a)]); }
    const arcEnd = pts.length - 1;
    // ply B: limb -> under roller B -> up to drum 2
    line(pts[pts.length - 1], zx3(L.B.rollIn), 4);
    arc2(L.RB, L.rr, L.B.a0, L.B.a1, 12);
    line(pts[pts.length - 1], zx3(L.B.drum), 12);
    const ups = perpUps(pts, 1);
    const span = Math.abs(b1 - b0);
    const cols = pts.map((p, k) => {
      if (k < arcStart || k > arcEnd) return null;
      const f = (k - arcStart) / (arcEnd - arcStart), phi = Math.min(f, 1 - f) * span;
      return pressureFn ? pressureFn(phi) : null;
    });
    return { pts, ups, cols, arcStart, arcEnd, Rb, contactAngle: span, layout: L };
  }
  // the two wound rolls (one ply each, pre-wrap included): traced from the tangent point inwards
  function bandRoll(G, st) {
    const { P } = G, t = P.band.t, m = new Mesh(), L = bandLayout(G, st), thw = st.theta + P.preWrap;
    for (const [C, T, dir] of [[L.C1, L.T1, -1], [L.C2, L.T2, 1]]) {
      const pts = [], ups = [], N = Math.max(6, Math.ceil(thw / 0.05)), aT = ang2(C, T);
      for (let j = 0; j <= N; j++) {
        const s = thw * j / N, r = L.R - t * s / TAU, a = aT + dir * s;
        pts.push(zx3([C[0] + r * Math.cos(a), C[1] + r * Math.sin(a)])); ups.push([Math.sin(a), 0, Math.cos(a)]);
      }
      m.append(strip(pts, ups, P.band.w, t * 0.95));
    }
    return m;
  }
  // cord: wrapped on the pulley (leaves its top toward +Z) and straight to the potted end at the piston
  function cableGeometry(G, X) {
    const { P } = G, A = P.act, PU = P.pulley, pts = [];
    const psi = Math.max(0.06, (A.stroke + 5 - X) / PU.r), N = Math.max(4, Math.ceil(psi / 0.08));
    for (let j = 0; j <= N; j++) { const a = PI / 2 + psi * (1 - j / N); pts.push([P.xd + PU.r * Math.sin(a), A.y, P.z1 + PU.r * Math.cos(a)]); }
    const zf = A.z0 + X - 4.6;
    for (let j = 1; j <= 8; j++) pts.push([A.x, A.y, P.z1 + (zf - P.z1) * j / 8]);
    return { mesh: sweep(pts, 1.5, 10), pts };
  }

  // thigh: surface of revolution with local compression under the band (+bulge)
  function thighMesh(R, dR, yb, w, segA = 96, segY = 90, opts = {}) {
    const m = new Mesh(), y0 = -170, y1 = 170, rows = [];
    for (let j = 0; j <= segY; j++) {
      const y = y0 + (y1 - y0) * j / segY;
      // base taper (proximal larger) + compression under the band with smooth edges and a soft bulge
      const taper = 1 + 0.0009 * y;
      const u = Math.abs(y - yb) / (w / 2);
      const comp = u < 1 ? 1 : Math.exp(-Math.pow((u - 1) * 2.2, 2));
      const bulge = u > 1 ? 0.35 * Math.exp(-Math.pow((u - 1.7) * 1.6, 2)) : 0;
      rows.push({ y, r: R * taper - dR * comp + dR * bulge * 0.9 });
    }
    const idx = [];
    for (let j = 0; j <= segY; j++) {
      const r = rows[j].r, dr = j === 0 ? rows[1].r - rows[0].r : j === segY ? rows[j].r - rows[j - 1].r : (rows[j + 1].r - rows[j - 1].r) / 2;
      const dy = (y1 - y0) / segY * (j === 0 || j === segY ? 1 : 1);
      const row = [];
      for (let i = 0; i <= segA; i++) {
        const a = TAU * i / segA, nx = Math.sin(a), nz = Math.cos(a);
        const n = V.norm([nx * dy, -dr, nz * dy]);
        row.push(m.v([r * nx, rows[j].y, r * nz], n));
      }
      idx.push(row);
    }
    for (let j = 0; j < segY; j++) for (let i = 0; i < segA; i++) m.quad(idx[j][i], idx[j][i + 1], idx[j + 1][i + 1], idx[j + 1][i]);
    for (const [j, s] of [[0, -1], [segY, 1]]) { const c = m.v([0, rows[j].y, 0], [0, s, 0]); const ring = []; for (let i = 0; i <= segA; i++) { const a = TAU * i / segA; ring.push(m.v([rows[j].r * Math.sin(a), rows[j].y, rows[j].r * Math.cos(a)], [0, s, 0])); } for (let i = 0; i < segA; i++) m.tri(c, ring[i], ring[i + 1]); }
    return { mesh: m, rows };
  }

  const API = { V, Mx, Mesh, merge, lathe, cylY, tubeY, box, extrude, ringY, ringIn, plate, plateHole, sweep, strip, helix, earclip, P, MAT, DEFAULT_D, makeKin, toothR,
    buildParts, bandGeometry, bandRoll, bandLayout, cableGeometry, thighMesh, alongZ, alongX, pawlAngle, releaseWind, pawlGeo, mountFor, saddleX, rollerBpos, pierceKin, DEG };
  if (typeof module !== 'undefined' && module.exports) module.exports = API; else root.IMPULS_GEOM = API;
})(typeof window !== 'undefined' ? window : globalThis);
