import * as V from '../math/vec3';
import type { Vec3 } from '../math/vec3';
import { canonicalNormal, facetArea, facetNormal, facetVertices } from '../paper/model';
import type { Facet, PaperModel } from '../paper/model';
import type { AeroMesh } from './aeroMesh';
import type { PaperConfig } from './config';
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
  /** Amplitude of each shape at which its steepest slope reaches the paper's limit (m). */
  limit: number[];
  /** Aero elements of the panel: index, shape values and in-plane slopes (model frame). */
  elements: { index: number; phi: number[]; grad: Vec3[] }[];
}

const MIN_AREA = 0.0025; // 25 cm²: smaller pieces (elevators, tabs) are stiff enough
const FLAT_COS = Math.cos((12 * Math.PI) / 180);
const POISSON = 0.3;
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
  set.panels.forEach((panel, pi) => {
    const facets = panel.facets.map((id) => byId.get(id)!).filter(Boolean);
    if (!isBendingPanel(facets, controlPanels.has(pi))) return;
    // plane frame
    let n: Vec3 = [0, 0, 0];
    for (const f of facets) n = V.addScaled(n, canonicalNormal(facetNormal(f)), facetArea(f));
    n = V.normalize(n);
    const pts = samples(facets, 0.006);
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
    const stiffnessOf = (sh: Poly[]) => {
      const m = sh.length;
      const K = new Array<number>(m * m).fill(0);
      for (const { p, a } of pts) {
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
    out.push({ panel: pi, origin: o, eu, ev, normal: n, shapes, first, Maug, Spaper, K, limit, elements });
    first += m;
  });
  return out;
}

/**
 * Extra floor-contact points on the bending panels: a bent panel no longer lies between its
 * corners, so its edges and its inside must be able to touch the floor too. Points along every
 * facet edge and spread over the inside, about `spacing` apart.
 */
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
