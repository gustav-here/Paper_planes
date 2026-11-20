import * as V from '../math/vec3';
import type { Vec2 } from '../math/vec2';
import type { Vec3 } from '../math/vec3';
import type { Airframe } from './airframe';
import type { AeroConfig } from './config';

/*
 * Quasi-steady thin flat-plate aerodynamics, evaluated per surface element.
 *
 *   uₑ = v + ω × rₑ                 element velocity relative to still air (body frame)
 *   q  = ½ρ|uₑ|²,  α = asin(|n·û|)   (plates are symmetric: no built-in "up")
 *   a  = a₀·AR/(2+√(AR²+4))          Helmbold finite-wing lift slope, AR from geometry + flow
 *   C_L,att = a sin α,      C_D,att = C_D0 + C_L,att²/(π e AR)
 *   C_L,sep = C_N90 sin α cos α,  C_D,sep = C_D0 + C_N90 sin² α
 *   σ(α) = 1/(1+exp(−M(α−α_s))),  C = (1−σ)C_att + σC_sep
 *
 * Chordwise loading: thin-plate theory puts the load near the leading edge (centre of pressure
 * at ¼ chord). Each element gets a weight ḡ from its chordwise extent [ξ₁, ξ₂], where the
 * leading edge is found from the CURRENT in-plane flow direction:
 *   attached:  g(ξ) = (2/π)√((1−ξ)/ξ),  ḡ = (G(ξ₂)−G(ξ₁))/(ξ₂−ξ₁),  G = (2/π)(asin√ξ + √(ξ(1−ξ)))
 *   separated: g(ξ) = 1 + k(1−2ξ),  k = 6Δ cos α   (x_cp = ½ − Δ cos α)
 * Weights are normalized per surface so the total force is unchanged.
 *
 *   Fₑ = q Aₑ [ ḡ (C_L ê_L + (C_D − C_D0) ê_D) + C_D0 ê_D ]
 */

const DEG = Math.PI / 180;

function G(x: number): number {
  const c = Math.min(1, Math.max(0, x));
  return (2 / Math.PI) * (Math.asin(Math.sqrt(c)) + Math.sqrt(c * (1 - c)));
}

/** Mean attached-flow weight over [x1, x2] (exact integral, handles the leading-edge singularity). */
export function attachedWeight(x1: number, x2: number): number {
  const a = Math.min(1, Math.max(0, Math.min(x1, x2)));
  const b = Math.min(1, Math.max(0, Math.max(x1, x2)));
  if (b - a < 1e-9) {
    const m = Math.min(Math.max(a, 1e-6), 1 - 1e-9);
    return (2 / Math.PI) * Math.sqrt((1 - m) / m);
  }
  return (G(b) - G(a)) / (b - a);
}

interface SurfaceCache {
  dir: Vec2 | null;
  /** Normalized attached weights per element. */
  gAtt: Float64Array;
  /** Chordwise midpoints per element. */
  xiMid: Float64Array;
  /** Σ Aₑ(1−2ξₘ) for normalizing the separated distribution. */
  K: number;
  aspect: number;
}

export interface AeroCache {
  surfaces: SurfaceCache[];
}

export function createAeroCache(af: Airframe): AeroCache {
  return {
    surfaces: af.mesh.surfaces.map((s) => ({
      dir: null,
      gAtt: new Float64Array(s.elementCount).fill(1),
      xiMid: new Float64Array(s.elementCount).fill(0.5),
      K: 0,
      aspect: 1,
    })),
  };
}

export interface AeroResult {
  /** Total aerodynamic force and moment about the CoM, body frame. */
  force: Vec3;
  torque: Vec3;
}

/** Optional per-element output for the debug overlay (body frame). */
export interface AeroDebug {
  lift: Vec3[];
  drag: Vec3[];
  alpha: number[];
}

/** Recompute chord coordinates and aspect ratio of one surface for in-plane flow direction w2. */
function updateSurfaceCache(af: Airframe, sid: number, w2: Vec2, w3: Vec3, c: SurfaceCache) {
  const s = af.mesh.surfaces[sid];
  const els = af.mesh.elements;
  let sumAG = 0;
  let K = 0;
  for (let k = 0; k < s.elementCount; k++) {
    const e = els[s.firstElement + k];
    const sc = e.c2[0] * w2[0] + e.c2[1] * w2[1];
    let smin = Infinity;
    let smax = -Infinity;
    for (const p of e.poly2) {
      const sp = p[0] * w2[0] + p[1] * w2[1];
      if (sp < smin) smin = sp;
      if (sp > smax) smax = sp;
    }
    // chord through the element centre along the flow: nearest outline crossings up/downstream
    let tLo = -Infinity;
    let tHi = Infinity;
    for (const [a, b] of s.edges) {
      const ex = b[0] - a[0];
      const ey = b[1] - a[1];
      const den = w2[0] * ey - w2[1] * ex;
      if (Math.abs(den) < 1e-14) continue;
      const ax = a[0] - e.c2[0];
      const ay = a[1] - e.c2[1];
      const t = (ax * ey - ay * ex) / den;
      const u = (ax * w2[1] - ay * w2[0]) / den;
      if (u < 0 || u > 1) continue;
      if (t <= 0 && t > tLo) tLo = t;
      if (t >= 0 && t < tHi) tHi = t;
    }
    let x1 = 0;
    let x2 = 1;
    if (Number.isFinite(tLo) && Number.isFinite(tHi) && tHi - tLo > 1e-6) {
      const le = sc + tLo;
      const chord = tHi - tLo;
      x1 = (smin - le) / chord;
      x2 = (smax - le) / chord;
    }
    const g = attachedWeight(x1, x2);
    c.gAtt[k] = g;
    const xm = Math.min(1, Math.max(0, (x1 + x2) / 2));
    c.xiMid[k] = xm;
    sumAG += e.area * g;
    K += e.area * (1 - 2 * xm);
  }
  const norm = sumAG > 0 ? s.area / sumAG : 1;
  for (let k = 0; k < s.elementCount; k++) c.gAtt[k] *= norm;
  c.K = K;
  // aspect ratio of the lifting system this surface belongs to, across the current flow
  const across = V.normalize(V.cross(s.normal, w3));
  let lo = Infinity;
  let hi = -Infinity;
  let S = 0;
  for (const pid of s.partners) {
    const p = af.mesh.surfaces[pid];
    S += p.area * Math.abs(V.dot(p.normal, s.normal));
    for (const v of p.outline3) {
      const y = V.dot(v, across);
      if (y < lo) lo = y;
      if (y > hi) hi = y;
    }
  }
  const b = hi - lo;
  c.aspect = Math.max((b * b) / Math.max(S, 1e-9), 0.05);
  c.dir = w2;
}

/**
 * Aerodynamic force and moment for body velocity v and angular velocity w (both body frame).
 * `cache` is updated in place (chord coordinates are reused while the flow direction is steady).
 */
export function evaluateAero(
  af: Airframe,
  v: Vec3,
  w: Vec3,
  cfg: AeroConfig,
  rho: number,
  cache: AeroCache,
  debug?: AeroDebug,
): AeroResult {
  const force: Vec3 = [0, 0, 0];
  const torque: Vec3 = [0, 0, 0];
  const els = af.mesh.elements;
  const alphaS = cfg.alphaStallDeg * DEG;
  const cosCache = Math.cos(cfg.chordCacheDeg * DEG);
  if (debug) {
    debug.lift.length = els.length;
    debug.drag.length = els.length;
    debug.alpha.length = els.length;
  }

  for (const s of af.mesh.surfaces) {
    const c = cache.surfaces[s.id];
    const n = s.normal;
    // in-plane direction of the air relative to the surface (air moves opposite to the plate)
    const uc = V.add(v, V.cross(w, s.centroid));
    const inPlane = V.negate(V.rejectFrom(uc, n));
    const ipl = V.length(inPlane);
    let uniform = false;
    if (ipl < 1e-6 * Math.max(V.length(uc), 1e-9) || ipl < 1e-9) {
      uniform = true;
    } else {
      const w3 = V.scale(inPlane, 1 / ipl);
      const w2: Vec2 = [V.dot(w3, s.e1), V.dot(w3, s.e2)];
      if (!c.dir || c.dir[0] * w2[0] + c.dir[1] * w2[1] < cosCache) updateSurfaceCache(af, s.id, w2, w3, c);
    }
    const AR = c.aspect;
    const a = (cfg.a0 * AR) / (2 + Math.sqrt(AR * AR + 4));
    const sepNorm = (k: number) => s.area / Math.max(s.area + k * c.K, 1e-12);

    for (let i = 0; i < s.elementCount; i++) {
      const idx = s.firstElement + i;
      const e = els[idx];
      const u = V.add(v, V.cross(w, e.center));
      const speed = V.length(u);
      if (speed < 1e-6) {
        if (debug) {
          debug.lift[idx] = [0, 0, 0];
          debug.drag[idx] = [0, 0, 0];
          debug.alpha[idx] = 0;
        }
        continue;
      }
      const uh = V.scale(u, 1 / speed);
      const un = V.dot(n, uh);
      const alpha = Math.asin(Math.min(1, Math.abs(un)));
      const q = 0.5 * rho * speed * speed;
      const sig = 1 / (1 + Math.exp(-cfg.stallBlendM * (alpha - alphaS)));
      const sa = Math.sin(alpha);
      const ca = Math.cos(alpha);
      const clAtt = a * sa;
      const cdAtt = cfg.CD0 + (clAtt * clAtt) / (Math.PI * cfg.oswaldE * AR);
      const clSep = cfg.CN90 * sa * ca;
      const cdSep = cfg.CD0 + cfg.CN90 * sa * sa;
      const CL = (1 - sig) * clAtt + sig * clSep;
      const CD = (1 - sig) * cdAtt + sig * cdSep;
      let gbar = 1;
      if (!uniform) {
        const k = 6 * cfg.sepCopShift * ca;
        const gSep = (1 + k * (1 - 2 * c.xiMid[i])) * sepNorm(k);
        gbar = (1 - sig) * c.gAtt[i] + sig * gSep;
      }
      const eD = V.negate(uh);
      const np = V.scale(n, un > 0 ? -1 : 1);
      let eL = V.rejectFrom(np, uh);
      const l = V.length(eL);
      eL = l > 1e-9 ? V.scale(eL, 1 / l) : [0, 0, 0];
      const qa = q * e.area;
      const L = V.scale(eL, qa * gbar * CL);
      const D = V.scale(eD, qa * (gbar * (CD - cfg.CD0) + cfg.CD0));
      const F = V.add(L, D);
      force[0] += F[0];
      force[1] += F[1];
      force[2] += F[2];
      const t = V.cross(e.center, F);
      torque[0] += t[0];
      torque[1] += t[1];
      torque[2] += t[2];
      if (debug) {
        debug.lift[idx] = L;
        debug.drag[idx] = D;
        debug.alpha[idx] = alpha;
      }
    }
  }
  return { force, torque };
}
