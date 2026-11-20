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

/** Penetration allowed without correction (m) and fraction corrected per step (Baumgarte). */
const SLOP = 2e-4;
const BAUMGARTE = 0.2;

/**
 * Floor y = 0: sequential impulses with restitution and Coulomb friction; penetration is
 * removed gently through a Baumgarte velocity bias with slop (no position teleporting, which
 * made resting planes jitter). Impulses act through the generalized mass M (incl. added mass).
 * Chosen over a spring–damper: a 5 g object against a stiff spring needs tiny time steps.
 */

export function resolveGround(s: BodyState, af: Airframe, cfg: GroundConfig, dt: number): ContactResult {
  const q = Q.normalize(s.q);
  const qi = Q.conjugate(q);
  const nWorld: Vec3 = [0, 1, 0];
  const active: { r: Vec3; index: number; depth: number; target: number; jn: number; jt: Vec3 }[] = [];
  let minY = Infinity;
  af.contacts.forEach((r, index) => {
    const y = s.x[1] + Q.rotate(q, r)[1];
    minY = Math.min(minY, y);
    if (y > 1e-5) return;
    active.push({ r, index, depth: -y, target: 0, jn: 0, jt: [0, 0, 0] });
  });
  if (active.length === 0) return { touching: false, impactSpeed: 0, hardest: null };

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
    const vn = V.dot(vel, nWorld);
    const bounce = vn < 0 && -vn > cfg.restitutionThreshold ? -cfg.restitution * vn : 0;
    const push = (BAUMGARTE * Math.max(0, c.depth - SLOP)) / dt;
    c.target = Math.max(bounce, push);
    if (vn < 0 && -vn > impactSpeed) {
      impactSpeed = -vn;
      hardest = { index: c.index, velWorld: vel };
    }
  }
  for (let it = 0; it < cfg.iterations; it++) {
    for (const c of active) {
      // normal
      const vn = V.dot(velAt(c.r), nWorld);
      const kn = invMass(c.r, nWorld);
      let dj = (c.target - vn) / kn;
      const old = c.jn;
      c.jn = Math.max(0, old + dj);
      dj = c.jn - old;
      if (dj !== 0) applyImpulse(c.r, V.scale(nWorld, dj));
      // friction (isotropic, clamped to the friction cone)
      const vel = velAt(c.r);
      const vt = V.rejectFrom(vel, nWorld);
      const vtl = V.length(vt);
      if (vtl < 1e-9) continue;
      const t = V.scale(vt, 1 / vtl);
      const kt = invMass(c.r, t);
      const want = V.addScaled(c.jt, t, -vtl / kt);
      const maxF = cfg.friction * c.jn;
      const wl = V.length(want);
      const next = wl > maxF ? V.scale(want, maxF / wl) : want;
      const delta = V.sub(next, c.jt);
      c.jt = next;
      applyImpulse(c.r, delta);
    }
  }
  // safety net for deep penetration only (hard impacts): never more than 1 mm into the floor
  if (minY < -0.001) s.x = [s.x[0], s.x[1] - minY - 0.001, s.x[2]];
  return { touching: true, impactSpeed, hardest };
}
