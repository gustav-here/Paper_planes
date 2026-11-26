import * as V from '../math/vec3';
import { invert } from '../math/linalg';
import type { Vec3 } from '../math/vec3';
import { canonicalNormal, facetArea, facetContains, facetNormal, facetVertices } from '../paper/model';
import type { Facet, PaperModel } from '../paper/model';
import type { AeroMesh } from './aeroMesh';
import type { PaperConfig } from './config';
import type { PanelBend } from './simulator';
import type { PanelSet } from './panels';

/*
 * Bending of the flat paper itself (flexible model). Every large, flat panel gets a few smooth
 * out-of-plane shapes φₖ, and its deflection is w(p) = Σ qₖ φₖ(p) along the panel normal. The
 * amplitudes qₖ are extra generalized coordinates of the multibody model:
 *   - panels hanging on a crease are clamped there (w = 0 and ∂w/∂d = 0 at the crease):
 *       φ₁ = d̃²            bending away from the crease (a wing bending up)
 *       φ₂ = d̃² s̃          bending that grows along the crease (twist)
 *       φ₃ = d̃² (s̃² − ⅓)   camber across
 *     with d̃ the distance from the crease and s̃ the position along it, both scaled to ±1;
 *   - the free root panel (the body, or a whole unfolded sheet) gets x̃², ỹ² and x̃ỹ in its
 *     principal axes, made orthogonal to rigid motion (∫ρφ = ∫ρφx = ∫ρφy = 0) and to each other.
 * From the paper (layers counted, areal density ρ_s, bending stiffness D, Poisson ν):
 *   inertia          the panel's rigid motion and the amplitudes together, from the same mass
 *                    points (paper) and elements (air carried along)
 *   stiffness        Kₖₗ = ∫ D [∇²φₖ ∇²φₗ − (1−ν)(φₖ,uu φₗ,vv + φₖ,vv φₗ,uu − 2 φₖ,uv φₗ,uv)] dA
 * and the same mass integrals with the air carried along (strip-theory added mass per element).
 */

/** Polynomial Σ c·uⁱvʲ in a panel's plane coordinates (u, v) — with its derivatives. */
export class Poly {
  constructor(readonly terms: [number, number, number][]) {}
  private sum(u: number, v: number, du: number, dv: number): number {
    let s = 0;
    for (const [i, j, c] of this.terms) {
      if (i < du || j < dv) continue;
      let k = c;
      for (let a = 0; a < du; a++) k *= i - a;
      for (let b = 0; b < dv; b++) k *= j - b;
      s += k * u ** (i - du) * v ** (j - dv);
    }
    return s;
  }
  w = (u: number, v: number) => this.sum(u, v, 0, 0);
  wu = (u: number, v: number) => this.sum(u, v, 1, 0);
  wv = (u: number, v: number) => this.sum(u, v, 0, 1);
  wuu = (u: number, v: number) => this.sum(u, v, 2, 0);
  wvv = (u: number, v: number) => this.sum(u, v, 0, 2);
  wuv = (u: number, v: number) => this.sum(u, v, 1, 1);
  plus(o: Poly, k: number): Poly {
    const t = new Map<string, [number, number, number]>();
    for (const [i, j, c] of this.terms) t.set(`${i},${j}`, [i, j, c]);
    for (const [i, j, c] of o.terms) {
      const e = t.get(`${i},${j}`);
      if (e) e[2] += k * c;
      else t.set(`${i},${j}`, [i, j, k * c]);
    }
    return new Poly([...t.values()]);
  }
  scale(k: number): Poly {
    return new Poly(this.terms.map(([i, j, c]) => [i, j, c * k]));
  }
}

export interface PanelModes {
  panel: number;
  /** Plane frame (model space): origin, in-plane axes and normal. */
  origin: Vec3;
  eu: Vec3;
  ev: Vec3;
  normal: Vec3;
  shapes: Poly[];
  /** Index of the first mode in the list of all modes (generalized coordinate 6 + k + first). */
  first: number;
  /**
   * Inertia of the whole panel (paper + air), (6+m)² row-major in the model frame: rigid motion
   * of the panel frame (CoM velocity, angular velocity) and the bending rates.
   */
  Maug: number[];
  /** ∫ ρ φₖ dA of the paper alone (gravity on the bending). */
  Spaper: number[];
  /** Bending stiffness (m × m). */
  K: number[];
  /**
   * Generalized force of a uniform rest curvature (m × 3): f = Kcurv · (κuu, κvv, κuv), the same
   * plate energy as K with the rest shape's curvatures in place of a second shape's.
   */
  Kcurv: number[];
  /** Amplitude of each shape at which its steepest slope reaches the paper's limit (m). */
  limit: number[];
  /** Aero elements of the panel: index, shape values and in-plane slopes (model frame). */
  elements: { index: number; phi: number[]; grad: Vec3[] }[];
}

const MIN_AREA = 0.0025; // 25 cm²: smaller pieces (elevators, tabs) are stiff enough
const FLAT_COS = Math.cos((12 * Math.PI) / 180);
const POISSON = 0.3;
/** Panels within this angle of closing onto each other lie face to face (as in panels.ts). */
const FACE_TO_FACE_RAD = (25 * Math.PI) / 180;
/** Spacing of the integration points (m): the shapes are low-order polynomials, so this is plenty. */
const SAMPLE_SPACING = 0.012;
/** Layers within this distance of each other (along the normal) count as one pressed stack (m). */
const STACK_GAP_M = 0.002;
/**
 * Extra clamped shapes (powers of d̃ and s̃: d̃³, d̃³s̃, d̃²s̃³, d̃³s̃²) for a panel that carries
 * creases, which stay straight (see keepCreasesStraight).
 */
const EXTRA_CLAMPED: [number, number][] = [
  [3, 0],
  [3, 1],
  [2, 3],
  [3, 2],
];
/** Shapes stiffer than this (natural frequency, Hz) count as rigid. */
const MAX_FREQ_HZ = 120;

/** Does this panel get bending shapes? Large and flat enough, and not a servo-driven flap. */
export function isBendingPanel(facets: Facet[], isControl: boolean): boolean {
  if (isControl || !facets.length) return false;
  let A = 0;
  let n: Vec3 = [0, 0, 0];
  for (const f of facets) {
    const a = facetArea(f);
    A += a;
    n = V.addScaled(n, canonicalNormal(facetNormal(f)), a);
  }
  if (A < MIN_AREA) return false;
  const m = V.normalize(n);
  return facets.every((f) => Math.abs(V.dot(canonicalNormal(facetNormal(f)), m)) > FLAT_COS);
}

/** Sample points over facets (all layers), with their areas. */
function samples(facets: Facet[], spacing: number): { p: Vec3; a: number }[] {
  const out: { p: Vec3; a: number }[] = [];
  for (const f of facets) {
    const vs = facetVertices(f);
    for (let i = 1; i + 1 < vs.length; i++) {
      const [A, B, C] = [vs[0], vs[i], vs[i + 1]];
      const area = V.length(V.cross(V.sub(B, A), V.sub(C, A))) / 2;
      if (area < 1e-12) continue;
      const edge = Math.max(V.dist(A, B), V.dist(B, C), V.dist(C, A));
      const k = Math.min(40, Math.max(2, Math.ceil(edge / spacing)));
      const pts: Vec3[] = [];
      for (let a = 0; a < k; a++)
        for (let b = 0; a + b < k; b++) {
          const u = (a + 1 / 3) / k;
          const v = (b + 1 / 3) / k;
          pts.push(V.add(A, V.add(V.scale(V.sub(B, A), u), V.scale(V.sub(C, A), v))));
        }
      for (const p of pts) out.push({ p, a: area / pts.length });
    }
  }
  return out;
}

/** Normal added mass of every aero element (strip theory, as in addedMass.ts). */
function elementAddedMass(mesh: AeroMesh, rho: number, scale: number, strip: number): Float64Array {
  const ma = new Float64Array(mesh.elements.length);
  for (const s of mesh.surfaces) {
    const ids = Array.from({ length: s.elementCount }, (_, k) => s.firstElement + k);
    let A = 0;
    let mx = 0;
    let my = 0;
    for (const i of ids) {
      const e = mesh.elements[i];
      A += e.area;
      mx += e.area * e.c2[0];
      my += e.area * e.c2[1];
    }
    if (A <= 0) continue;
    mx /= A;
    my /= A;
    let sxx = 0;
    let syy = 0;
    let sxy = 0;
    for (const i of ids) {
      const e = mesh.elements[i];
      sxx += e.area * (e.c2[0] - mx) ** 2;
      syy += e.area * (e.c2[1] - my) ** 2;
      sxy += e.area * (e.c2[0] - mx) * (e.c2[1] - my);
    }
    const th = 0.5 * Math.atan2(2 * sxy, sxx - syy);
    const ax: [number, number] = [Math.cos(th), Math.sin(th)];
    const strips = new Map<number, number>();
    let lo = Infinity;
    let hi = -Infinity;
    for (const i of ids) {
      const e = mesh.elements[i];
      const y = e.c2[0] * ax[0] + e.c2[1] * ax[1];
      lo = Math.min(lo, y);
      hi = Math.max(hi, y);
      const k = Math.floor(y / strip);
      strips.set(k, (strips.get(k) ?? 0) + e.area);
    }
    const span = Math.max(hi - lo + strip, strip);
    const AR = (span * span) / A;
    const kappa = (AR / Math.sqrt(1 + AR * AR)) * scale;
    for (const i of ids) {
      const e = mesh.elements[i];
      const k = Math.floor((e.c2[0] * ax[0] + e.c2[1] * ax[1]) / strip);
      const stripArea = strips.get(k) ?? e.area;
      const chord = stripArea / strip;
      const mStrip = (kappa * rho * Math.PI * chord * chord * strip) / 4;
      ma[i] = (mStrip * e.area) / stripArea;
    }
  }
  return ma;
}

/**
 * The creases panel `pi` carries other panels on, as segments (model space): the hinge line
 * over the extent of the child's paper that lies on it.
 */
function creaseLines(set: PanelSet, pi: number, byId: Map<number, Facet>): [Vec3, Vec3][] {
  const out: [Vec3, Vec3][] = [];
  for (const hg of set.hinges) {
    if (hg.parent !== pi) continue;
    let lo = Infinity;
    let hi = -Infinity;
    for (const id of set.panels[hg.child].facets) {
      const f = byId.get(id);
      if (!f) continue;
      for (const x of facetVertices(f)) {
        if (V.distToLine(x, hg.point, hg.axis) > 1e-4) continue;
        const s = V.dot(V.sub(x, hg.point), hg.axis);
        lo = Math.min(lo, s);
        hi = Math.max(hi, s);
      }
    }
    if (hi - lo > 1e-3) out.push([V.addScaled(hg.point, hg.axis, lo), V.addScaled(hg.point, hg.axis, hi)]);
  }
  return out;
}

/**
 * A crease the panel carries another panel on stays straight: the shapes are restricted to the
 * combinations whose deflection varies linearly along every such crease.
 *
 * A folded crease is an angle section, far stiffer along its length than flat paper (that is why
 * corrugated paper is stiff), and the paper hanging on it is a rigid panel (or bends from it as
 * from a straight clamped edge). Without this the panel could curve along the crease while the
 * flap on it stayed straight: a wing tip twisted and cambered around a turned-up corner as if the
 * corner were not there. The shapes are polynomials of low degree, so second differences at five
 * points along each crease capture its curvature exactly.
 */
function keepCreasesStraight(
  shapes: Poly[],
  lines: [Vec3, Vec3][],
  local: (p: Vec3) => readonly [number, number],
  samplesUV: (readonly [number, number])[],
): Poly[] {
  const m = shapes.length;
  if (!lines.length || !m) return shapes;
  const rows: number[][] = [];
  for (const [a, b] of lines) {
    const w = [0, 0.25, 0.5, 0.75, 1].map((t) => {
      const [u, v] = local(V.lerp(a, b, t));
      return shapes.map((P) => P.w(u, v));
    });
    for (let i = 1; i + 1 < w.length; i++) rows.push(shapes.map((_, k) => w[i - 1][k] - 2 * w[i][k] + w[i + 1][k]));
  }
  // orthonormal basis of the constraint rows, then the shape combinations orthogonal to it
  const dot = (x: number[], y: number[]) => x.reduce((s, xi, i) => s + xi * y[i], 0);
  const scale = Math.max(...rows.map((r) => Math.sqrt(dot(r, r))), 1e-12);
  const rowBasis: number[][] = [];
  for (const r of rows) {
    let q = r.slice();
    for (const b of rowBasis) {
      const c = dot(q, b);
      q = q.map((x, i) => x - c * b[i]);
    }
    const n = Math.sqrt(dot(q, q));
    if (n > 1e-6 * scale) rowBasis.push(q.map((x) => x / n));
  }
  if (!rowBasis.length) return shapes;
  const free: number[][] = [];
  for (let k = 0; k < m; k++) {
    let q: number[] = shapes.map((_, i) => (i === k ? 1 : 0));
    for (const b of [...rowBasis, ...free]) {
      const c = dot(q, b);
      q = q.map((x, i) => x - c * b[i]);
    }
    const n = Math.sqrt(dot(q, q));
    if (n > 1e-6) free.push(q.map((x) => x / n));
  }
  return free.map((c) => {
    let P = new Poly([]);
    c.forEach((ck, k) => (P = P.plus(shapes[k], ck)));
    const mx = Math.max(...samplesUV.map(([u, v]) => Math.abs(P.w(u, v))), 1e-12);
    return P.scale(1 / mx); // amplitude q = largest deflection (m)
  });
}

/**
 * Bending shapes for every panel that gets them. `panelSurfaces[i]` are the aero surfaces of
 * panel i; `com` is the plane's centre of mass (aero element centres are relative to it).
 */
export function buildPanelModes(
  model: PaperModel,
  set: PanelSet,
  mesh: AeroMesh,
  panelSurfaces: number[][],
  com: Vec3,
  controlPanels: Set<number>,
  paper: PaperConfig,
  rho: number,
  addedMassScale: number,
  stripWidth: number,
): PanelModes[] {
  const gsm = model.spec.gsm / 1000; // kg/m²
  const ma = elementAddedMass(mesh, rho, addedMassScale, stripWidth);
  const byId = new Map(model.facets.map((f) => [f.id, f]));
  const out: PanelModes[] = [];
  let first = 0;
  /*
   * Panels that cannot bend on their own:
   *  - the root, when paper hangs on it: it carries the whole plane and its bending would be
   *    shared with everything attached (only a lone sheet bends freely);
   *  - panels lying face to face with another one (the two halves of a keel): the layers press
   *    on each other and bend together, not through each other.
   */
  const faceToFace = new Set<number>();
  for (const h of set.hinges) {
    if (Math.abs(h.closeAt) < FACE_TO_FACE_RAD) {
      faceToFace.add(h.parent);
      faceToFace.add(h.child);
    }
  }
  set.panels.forEach((panel, pi) => {
    const facets = panel.facets.map((id) => byId.get(id)!).filter(Boolean);
    if (panel.parent < 0 && set.panels.some((q) => q.parent === pi)) return;
    if (faceToFace.has(pi)) return;
    if (!isBendingPanel(facets, controlPanels.has(pi))) return;
    // plane frame
    let n: Vec3 = [0, 0, 0];
    for (const f of facets) n = V.addScaled(n, canonicalNormal(facetNormal(f)), facetArea(f));
    n = V.normalize(n);
    const pts = samples(facets, SAMPLE_SPACING);
    let shapes: Poly[];
    let eu: Vec3;
    let ev: Vec3;
    let o: Vec3;
    if (panel.parent >= 0) {
      // clamped at the crease to the parent
      const hg = set.hinges[panel.hinge];
      const a = V.normalize(V.rejectFrom(hg.axis, n));
      let d = V.normalize(V.cross(n, a));
      if (V.dot(V.sub(panel.com, hg.point), d) < 0) d = V.negate(d);
      const s = pts.map(({ p }) => V.dot(V.sub(p, hg.point), a));
      const sMid = (Math.min(...s) + Math.max(...s)) / 2;
      const sHalf = Math.max((Math.max(...s) - Math.min(...s)) / 2, 1e-3);
      const L = Math.max(...pts.map(({ p }) => V.dot(V.sub(p, hg.point), d)), 1e-3);
      o = V.addScaled(hg.point, a, sMid);
      eu = d;
      ev = a;
      // in (u, v) metres; d̃ = u/L, s̃ = v/sHalf
      shapes = [
        new Poly([[2, 0, 1 / (L * L)]]),
        new Poly([[2, 1, 1 / (L * L * sHalf)]]),
        new Poly([
          [2, 2, 1 / (L * L * sHalf * sHalf)],
          [2, 0, -1 / (3 * L * L)],
        ]),
      ];
    } else {
      // free: principal axes of the panel, orthogonal to rigid motion
      o = panel.com;
      let e1 = V.rejectFrom([1, 0, 0], n);
      if (V.length(e1) < 0.3) e1 = V.rejectFrom([0, 0, 1], n);
      e1 = V.normalize(e1);
      const e2 = V.cross(n, e1);
      let sxx = 0;
      let syy = 0;
      let sxy = 0;
      for (const { p, a } of pts) {
        const r = V.sub(p, o);
        const x = V.dot(r, e1);
        const y = V.dot(r, e2);
        sxx += a * x * x;
        syy += a * y * y;
        sxy += a * x * y;
      }
      const th = 0.5 * Math.atan2(2 * sxy, sxx - syy);
      eu = V.normalize(V.add(V.scale(e1, Math.cos(th)), V.scale(e2, Math.sin(th))));
      ev = V.cross(n, eu);
      const us = pts.map(({ p }) => V.dot(V.sub(p, o), eu));
      const vs = pts.map(({ p }) => V.dot(V.sub(p, o), ev));
      const Lu = Math.max(...us.map(Math.abs), 1e-3);
      const Lv = Math.max(...vs.map(Math.abs), 1e-3);
      const rigid = [new Poly([[0, 0, 1]]), new Poly([[1, 0, 1 / Lu]]), new Poly([[0, 1, 1 / Lv]])];
      const cands = [new Poly([[2, 0, 1 / (Lu * Lu)]]), new Poly([[0, 2, 1 / (Lv * Lv)]]), new Poly([[1, 1, 1 / (Lu * Lv)]])];
      const dot = (P1: Poly, P2: Poly) => pts.reduce((acc, _, j) => acc + pts[j].a * P1.w(us[j], vs[j]) * P2.w(us[j], vs[j]), 0);
      // Gram–Schmidt (area-weighted = mass-weighted for uniform paper): rigid first, then shapes
      const basis: Poly[] = [];
      for (const P of [...rigid, ...cands]) {
        let q = P;
        for (const b of basis) q = q.plus(b, -dot(q, b) / dot(b, b));
        if (dot(q, q) > 1e-14) basis.push(q);
      }
      shapes = basis.slice(3).map((P) => {
        const mx = Math.max(...pts.map((_, j) => Math.abs(P.w(us[j], vs[j]))), 1e-9);
        return P.scale(1 / mx); // amplitude q = largest deflection (m)
      });
    }
    const local = (p: Vec3) => {
      const r = V.sub(p, o);
      return [V.dot(r, eu), V.dot(r, ev)] as const;
    };
    const lines = creaseLines(set, pi, byId);
    if (lines.length && panel.parent >= 0) {
      // a straight crease is a local stiffening; with only the three shapes above it would
      // stiffen the whole panel, so the panel gets more shapes to bend around it
      const hg = set.hinges[panel.hinge];
      const s = pts.map(({ p }) => V.dot(V.sub(p, hg.point), ev));
      const sHalf = Math.max((Math.max(...s) - Math.min(...s)) / 2, 1e-3);
      const L = Math.max(...pts.map(({ p }) => V.dot(V.sub(p, hg.point), eu)), 1e-3);
      for (const [i, j] of EXTRA_CLAMPED) shapes.push(new Poly([[i, j, 1 / (L ** i * sHalf ** j)]]));
    }
    shapes = keepCreasesStraight(shapes, lines, local, pts.map(({ p }) => local(p)));
    if (!shapes.length) return;
    // a pressed stack of n layers bends like n^k single sheets (each layer's samples weigh n^(k−1))
    const stackW = pts.map(({ p }) => Math.max(1, facets.filter((f) => facetContains(f, p, 1e-6, STACK_GAP_M)).length) ** (paper.stackStiffnessExponent - 1));
    const stiffnessOf = (sh: Poly[]) => {
      const m = sh.length;
      const K = new Array<number>(m * m).fill(0);
      for (let ip = 0; ip < pts.length; ip++) {
        const { p } = pts[ip];
        const a = pts[ip].a * stackW[ip];
        const [u, v] = local(p);
        const lap = sh.map((P) => P.wuu(u, v) + P.wvv(u, v));
        const uu = sh.map((P) => P.wuu(u, v));
        const vv = sh.map((P) => P.wvv(u, v));
        const uv = sh.map((P) => P.wuv(u, v));
        for (let k = 0; k < m; k++)
          for (let l = 0; l < m; l++)
            K[k * m + l] += paper.bendingStiffness * a * (lap[k] * lap[l] - (1 - POISSON) * (uu[k] * vv[l] + vv[k] * uu[l] - 2 * uv[k] * uv[l]));
      }
      return K;
    };
    // a shape that is far stiffer than the paper's slow motions (across a narrow strip) is
    // effectively rigid: leave it out
    {
      const K = stiffnessOf(shapes);
      const m0 = shapes.length;
      shapes = shapes.filter((P, k) => {
        const mk = pts.reduce((acc, { p, a }) => {
          const [u, v] = local(p);
          return acc + gsm * a * P.w(u, v) ** 2;
        }, 0);
        return Math.sqrt(K[k * m0 + k] / Math.max(mk, 1e-12)) / (2 * Math.PI) < MAX_FREQ_HZ;
      });
    }
    const m = shapes.length;
    if (!m) return;
    const K = stiffnessOf(shapes);
    const Kcurv = new Array<number>(m * 3).fill(0);
    for (let ip = 0; ip < pts.length; ip++) {
      const a = paper.bendingStiffness * pts[ip].a * stackW[ip];
      const [u, v] = local(pts[ip].p);
      shapes.forEach((P, k) => {
        const uu = P.wuu(u, v);
        const vv = P.wvv(u, v);
        const uv = P.wuv(u, v);
        // B(φ, w_r) with w_r,uu = κuu, w_r,vv = κvv, w_r,uv = κuv
        Kcurv[k * 3] += a * (uu + vv - (1 - POISSON) * vv);
        Kcurv[k * 3 + 1] += a * (uu + vv - (1 - POISSON) * uu);
        Kcurv[k * 3 + 2] += a * 2 * (1 - POISSON) * uv;
      });
    }
    /*
     * The panel's whole inertia — rigid motion of the panel frame (v, ω about its CoM) and the
     * bending amplitudes — from the same mass points, so the matrix is always positive definite:
     *   paper point (mass mⱼ):  velocity v − [r]×ω + n Σ φₖ q̇ₖ  →  mⱼ GᵀG,  G = [I, −[r]×, n φ]
     *   air at an element (mₐ): normal velocity n·v + (r×n)·ω + Σ φₖ q̇ₖ  →  mₐ ggᵀ
     */
    const N = 6 + m;
    const Maug = new Array<number>(N * N).fill(0);
    const Spaper = new Array<number>(m).fill(0);
    const addOuter = (rows: number[][], w: number) => {
      for (const g of rows) for (let a = 0; a < N; a++) if (g[a]) for (let b = 0; b < N; b++) Maug[a * N + b] += w * g[a] * g[b];
    };
    for (const { p, a } of pts) {
      const [u, v] = local(p);
      const mass = gsm * a;
      const r = V.sub(p, panel.com);
      const phi = shapes.map((P) => P.w(u, v));
      phi.forEach((f, k) => (Spaper[k] += mass * f));
      // rows of G: for each direction x, y, z
      const rows: number[][] = [0, 1, 2].map((d) => {
        const e: Vec3 = [d === 0 ? 1 : 0, d === 1 ? 1 : 0, d === 2 ? 1 : 0];
        // (−[r]×ω)_d = (ω × r)_d = ω · (r × e_d)
        const rw = V.cross(r, e);
        return [...e, ...rw, ...phi.map((f) => n[d] * f)];
      });
      addOuter(rows, mass);
    }
    // aero elements of the panel, and the air they carry along
    const elements: PanelModes['elements'] = [];
    for (const sid of panelSurfaces[pi]) {
      const s = mesh.surfaces[sid];
      for (let k = 0; k < s.elementCount; k++) {
        const index = s.firstElement + k;
        const pm = V.add(mesh.elements[index].center, com);
        const [u, v] = local(pm);
        const phi = shapes.map((P) => P.w(u, v));
        const grad = shapes.map((P) => V.add(V.scale(eu, P.wu(u, v)), V.scale(ev, P.wv(u, v))));
        elements.push({ index, phi, grad });
        const mae = ma[index];
        if (mae <= 0) continue;
        const r = V.sub(pm, panel.com);
        addOuter([[...n, ...V.cross(r, n), ...phi]], mae);
      }
    }
    const tanMax = Math.tan((paper.bendingLimitSlopeDeg * Math.PI) / 180);
    const limit = shapes.map((P) => {
      let g = 1e-9;
      for (const { p } of pts) {
        const [u, v] = local(p);
        g = Math.max(g, Math.hypot(P.wu(u, v), P.wv(u, v)));
      }
      return tanMax / g;
    });
    out.push({ panel: pi, origin: o, eu, ev, normal: n, shapes, first, Maug, Spaper, K, Kcurv, limit, elements });
    first += m;
  });
  return out;
}

/**
 * Extra floor-contact points on the bending panels: a bent panel no longer lies between its
 * corners, so its edges and its inside must be able to touch the floor too. Points along every
 * facet edge and spread over the inside, about `spacing` apart.
 */
/**
 * Rest amplitudes of the bending shapes for a paper that curls by itself: each panel gets a
 * uniform curvature (κuu, κvv, κuv) from `next` × `sigmaPerM`, and rests in the shape of least
 * plate energy relative to it (K q = Kcurv κ). A free panel takes the curl; shapes held by
 * straight creases take less. One entry per shape, in the order of the generalized coordinates.
 */
export function restCurl(modes: PanelModes[], next: () => number, sigmaPerM: number): number[] {
  const out: number[] = [];
  for (const pm of modes) {
    const m = pm.shapes.length;
    const kappa = [next() * sigmaPerM, next() * sigmaPerM, next() * sigmaPerM];
    const f = Array.from({ length: m }, (_, k) => pm.Kcurv[k * 3] * kappa[0] + pm.Kcurv[k * 3 + 1] * kappa[1] + pm.Kcurv[k * 3 + 2] * kappa[2]);
    const Kinv = invert(pm.K, m);
    for (let k = 0; k < m; k++) {
      let q = 0;
      for (let l = 0; l < m; l++) q += Kinv[k * m + l] * f[l];
      out.push(q);
    }
  }
  return out;
}

export function bendingContacts(model: PaperModel, set: PanelSet, modes: PanelModes[], spacing: number): { panel: number; facet: number; p: Vec3 }[] {
  const byId = new Map(model.facets.map((f) => [f.id, f]));
  const out: { panel: number; facet: number; p: Vec3 }[] = [];
  for (const pm of modes) {
    const mine: Vec3[] = [];
    const add = (facet: number, p: Vec3) => {
      if (mine.some((q) => V.dist(q, p) < spacing / 4)) return;
      mine.push(p);
      out.push({ panel: pm.panel, facet, p });
    };
    for (const id of set.panels[pm.panel].facets) {
      const f = byId.get(id);
      if (!f) continue;
      const vs = facetVertices(f);
      vs.forEach((a, i) => {
        const b = vs[(i + 1) % vs.length];
        const k = Math.ceil(V.dist(a, b) / spacing);
        for (let j = 1; j < k; j++) add(id, V.lerp(a, b, j / k));
      });
      for (const { p } of samples([f], spacing)) add(id, p);
    }
  }
  return out;
}

/**
 * Extra stiffness of shape k at amplitude q beyond the linear range (see
 * PaperConfig.bendingLimitSlopeDeg): k·c·(q/q_lim)⁶. Used as a secant stiffness (force −k_extra·q).
 */
export function limitStiffness(pm: PanelModes, k: number, q: number, paper: PaperConfig): number {
  const m = pm.shapes.length;
  const r = q / pm.limit[k];
  return pm.K[k * m + k] * paper.bendingLimitStiffening * r ** 6;
}

/** Deflection w and its in-plane gradient (model frame) of a panel at model point p. */
export function deflectionAt(pm: PanelModes, q: ArrayLike<number>, p: Vec3): { w: number; grad: Vec3 } {
  const r = V.sub(p, pm.origin);
  const u = V.dot(r, pm.eu);
  const v = V.dot(r, pm.ev);
  let w = 0;
  let gu = 0;
  let gv = 0;
  pm.shapes.forEach((P, k) => {
    const a = q[pm.first + k];
    w += a * P.w(u, v);
    gu += a * P.wu(u, v);
    gv += a * P.wv(u, v);
  });
  return { w, grad: V.add(V.scale(pm.eu, gu), V.scale(pm.ev, gv)) };
}

/**
 * The panel's current bending as one polynomial (all shapes summed with their amplitudes), for
 * evaluating at many points (drawing).
 */
export function bendField(pm: PanelModes, q: ArrayLike<number>): PanelBend {
  let sum = new Poly([]);
  pm.shapes.forEach((P, k) => (sum = sum.plus(P, q[pm.first + k])));
  const terms = sum.terms.filter(([, , c]) => c !== 0);
  const ti = terms.map(([i]) => i);
  const tj = terms.map(([, j]) => j);
  const tc = terms.map(([, , c]) => c);
  const maxPow = Math.max(1, ...ti, ...tj);
  const pu = new Float64Array(maxPow + 1);
  const pv = new Float64Array(maxPow + 1);
  const [ox, oy, oz] = pm.origin;
  const [ux, uy, uz] = pm.eu;
  const [vx, vy, vz] = pm.ev;
  const evalInto = (x: number, y: number, z: number, out: Float64Array) => {
    const rx = x - ox;
    const ry = y - oy;
    const rz = z - oz;
    const u = rx * ux + ry * uy + rz * uz;
    const v = rx * vx + ry * vy + rz * vz;
    pu[0] = pv[0] = 1;
    for (let k = 1; k <= maxPow; k++) {
      pu[k] = pu[k - 1] * u;
      pv[k] = pv[k - 1] * v;
    }
    let w = 0;
    let wu = 0;
    let wv = 0;
    for (let t = 0; t < tc.length; t++) {
      const i = ti[t];
      const j = tj[t];
      const c = tc[t];
      w += c * pu[i] * pv[j];
      if (i > 0) wu += c * i * pu[i - 1] * pv[j];
      if (j > 0) wv += c * j * pu[i] * pv[j - 1];
    }
    out[0] = w;
    out[1] = wu * ux + wv * vx;
    out[2] = wu * uy + wv * vy;
    out[3] = wu * uz + wv * vz;
  };
  const tmp = new Float64Array(4);
  return {
    normal: pm.normal,
    evalInto,
    at: (p: Vec3) => {
      evalInto(p[0], p[1], p[2], tmp);
      return { w: tmp[0], grad: [tmp[1], tmp[2], tmp[3]] };
    },
  };
}
