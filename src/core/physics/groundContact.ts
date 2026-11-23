import * as V from '../math/vec3';
import * as Q from '../math/quat';
import { mulVec } from '../math/linalg';
import type { Vec3 } from '../math/vec3';
import type { Airframe } from './airframe';
import type { BodyState } from './rigidBody';
import type { GroundConfig } from './config';

export interface ContactResult {
  /** Any point touched or penetrated the floor this step. */
  touching: boolean;
  /** Largest approach speed resolved this step (m/s), for impact reporting. */
  impactSpeed: number;
  /** The contact with that approach speed: body-frame point index and its world velocity. */
  hardest: { index: number; velWorld: Vec3 } | null;
}

/** Paper in a loose layer rests on it when it moves slower than this, and ploughs through it when faster (m/s). */
export const DUST_SINK_SPEED = 1;

/** Penetration allowed without correction (m) and fraction corrected per step (Baumgarte). */
const SLOP = 2e-4;
const BAUMGARTE = 0.2;

/**
 * Solid surfaces (floor, walls, ceiling): sequential impulses with restitution and Coulomb friction; penetration is
 * removed gently through a Baumgarte velocity bias with slop (no position teleporting, which
 * made resting planes jitter). Impulses act through the generalized mass M (incl. added mass).
 * Chosen over a spring–damper: a 5 g object against a stiff spring needs tiny time steps.
 */

export function resolveGround(s: BodyState, af: Airframe, cfg: GroundConfig, dt: number): ContactResult {
  const q = Q.normalize(s.q);
  const qi = Q.conjugate(q);
  const active: { r: Vec3; index: number; depth: number; target: number; jn: number; jt: Vec3; n: Vec3; plough: number }[] = [];
  // deepest penetration per surface (for the safety net)
  const minGap = cfg.planes.map(() => Infinity);
  // points sunk into a loose top layer (dustDepthM): braked by it, firm ground below
  const dust: { r: Vec3; index: number; n: Vec3 }[] = [];
  const firm = cfg.dustDepthM;
  af.contacts.forEach((r, index) => {
    const p = V.add(s.x, Q.rotate(q, r));
    cfg.planes.forEach((pl, k) => {
      const gap = V.dot(pl.n, p) - pl.d + firm;
      minGap[k] = Math.min(minGap[k], gap);
      if (gap > 1e-5) {
        if (gap < firm + 1e-5) {
          // slow paper rests on the dust (it bears a light sheet); only an impact sinks into it
          const vp = V.length(Q.rotate(q, V.add(s.v, V.cross(s.w, r))));
          if (vp < DUST_SINK_SPEED) {
            if (gap < firm - 1e-5) active.push({ r, index, depth: 0, target: 0, jn: 0, jt: [0, 0, 0], n: pl.n, plough: 0 });
          } else dust.push({ r, index, n: pl.n });
        }
        return;
      }
      active.push({ r, index, depth: -gap, target: 0, jn: 0, jt: [0, 0, 0], n: pl.n, plough: 0 });
    });
  });
  if (active.length === 0 && dust.length === 0) return { touching: false, impactSpeed: 0, hardest: null };

  const velAt = (r: Vec3) => Q.rotate(q, V.add(s.v, V.cross(s.w, r)));
  const applyImpulse = (r: Vec3, jWorld: Vec3) => {
    const jb = Q.rotate(qi, jWorld);
    const d = mulVec(af.Minv, [...jb, ...V.cross(r, jb)], 6);
    s.v = V.add(s.v, [d[0], d[1], d[2]]);
    s.w = V.add(s.w, [d[3], d[4], d[5]]);
  };
  /** Effective inverse mass of point r along world direction dir. */
  const invMass = (r: Vec3, dir: Vec3) => {
    const db = Q.rotate(qi, dir);
    const g = [...db, ...V.cross(r, db)];
    const mg = mulVec(af.Minv, g, 6);
    return g.reduce((acc, gi, i) => acc + gi * mg[i], 0);
  };

  let impactSpeed = 0;
  let hardest: ContactResult['hardest'] = null;
  for (const c of active) {
    const vel = velAt(c.r);
    const vn = V.dot(vel, c.n);
    const bounce = vn < 0 && -vn > cfg.restitutionThreshold ? -cfg.restitution * vn : 0;
    const push = (BAUMGARTE * Math.max(0, c.depth - SLOP)) / dt;
    c.target = Math.max(bounce, push);
    // loose ground ploughed ahead of the point during this step
    c.plough = cfg.ploughKgPerM * V.lengthSq(V.rejectFrom(vel, c.n)) * dt;
    if (vn < 0 && -vn > impactSpeed) {
      impactSpeed = -vn;
      hardest = { index: c.index, velWorld: vel };
    }
  }
  for (let it = 0; it < cfg.iterations; it++) {
    for (const c of active) {
      // normal
      const vn = V.dot(velAt(c.r), c.n);
      const kn = invMass(c.r, c.n);
      let dj = (c.target - vn) / kn;
      const old = c.jn;
      c.jn = Math.max(0, old + dj);
      dj = c.jn - old;
      if (dj !== 0) applyImpulse(c.r, V.scale(c.n, dj));
      // friction (isotropic, clamped to the friction cone)
      const vel = velAt(c.r);
      const vt = V.rejectFrom(vel, c.n);
      const vtl = V.length(vt);
      if (vtl < 1e-9) continue;
      const t = V.scale(vt, 1 / vtl);
      const kt = invMass(c.r, t);
      const want = V.addScaled(c.jt, t, -vtl / kt);
      const maxF = cfg.friction * c.jn + c.plough;
      const wl = V.length(want);
      const next = wl > maxF ? V.scale(want, maxF / wl) : want;
      const delta = V.sub(next, c.jt);
      c.jt = next;
      applyImpulse(c.r, delta);
    }
  }
  // paper in the loose layer ploughs through it, whichever way it moves
  for (const c of dust) {
    const vel = velAt(c.r);
    const vn = -V.dot(vel, c.n);
    if (vn > impactSpeed) {
      impactSpeed = vn;
      hardest = { index: c.index, velWorld: vel };
    }
    const vl = V.length(vel);
    if (vl < 1e-9) continue;
    const dir = V.scale(vel, 1 / vl);
    applyImpulse(c.r, V.scale(dir, -Math.min(cfg.ploughKgPerM * vl * vl * dt, vl / invMass(c.r, dir))));
  }
  // loose ground closes round the paper and takes up its spin
  if (cfg.spinDampingPerS > 0) s.w = V.scale(s.w, Math.exp(-cfg.spinDampingPerS * dt));
  // safety net for deep penetration only (hard impacts): never more than 1 mm into a surface
  cfg.planes.forEach((pl, k) => {
    if (minGap[k] < -0.001) s.x = V.addScaled(s.x, pl.n, -minGap[k] - 0.001);
  });
  return { touching: true, impactSpeed, hardest };
}
