import * as V from '../math/vec3';
import type { Vec3 } from '../math/vec3';
import type { Airframe } from './airframe';
import { createAeroCache, evaluateAero } from './aeroModel';
import type { PhysicsConfig } from './config';

const DEG = Math.PI / 180;

export interface TunnelPoint {
  alphaDeg: number;
  CL: number;
  CD: number;
  /** Pitching moment coefficient about the CoM, nose-up positive. */
  Cm: number;
  force: Vec3;
  torque: Vec3;
}

/**
 * Virtual wind tunnel: hold the airframe still at angle of attack α (and sideslip β) in a flow
 * of speed V and read off force and moment. Body velocity: V(cosα cosβ·nose − sinα·up + cosα sinβ·right).
 */
export function tunnelPoint(af: Airframe, cfg: PhysicsConfig, V0: number, alphaDeg: number, betaDeg = 0): TunnelPoint {
  const a = alphaDeg * DEG;
  const b = betaDeg * DEG;
  const v = V.add(
    V.add(V.scale(af.nose, V0 * Math.cos(a) * Math.cos(b)), V.scale(af.up, -V0 * Math.sin(a))),
    V.scale(af.right, V0 * Math.cos(a) * Math.sin(b)),
  );
  const r = evaluateAero(af, v, [0, 0, 0], cfg.aero, cfg.rho, createAeroCache(af));
  const q = 0.5 * cfg.rho * V0 * V0;
  const vh = V.normalize(v);
  const eL = V.normalize(V.add(V.scale(af.nose, Math.sin(a)), V.scale(af.up, Math.cos(a))));
  return {
    alphaDeg,
    CL: V.dot(r.force, eL) / (q * af.refArea),
    CD: -V.dot(r.force, vh) / (q * af.refArea),
    Cm: V.dot(r.torque, af.right) / (q * af.refArea * af.refChord),
    force: r.force,
    torque: r.torque,
  };
}

export function sweep(af: Airframe, cfg: PhysicsConfig, V0 = 5, from = -10, to = 40, step = 1): TunnelPoint[] {
  const out: TunnelPoint[] = [];
  for (let a = from; a <= to + 1e-9; a += step) out.push(tunnelPoint(af, cfg, V0, a));
  return out;
}

export interface Stability {
  /** Trim angle of attack (Cm = 0 with dCm/dα < 0), deg, or null if none in range. */
  trimAlphaDeg: number | null;
  /** dCm/dα at trim (1/rad); negative = statically stable. */
  cmAlpha: number | null;
  CLtrim: number | null;
  /** Lift-to-drag ratio at trim. */
  LDtrim: number | null;
}

export function stability(af: Airframe, cfg: PhysicsConfig, V0 = 5): Stability {
  const pts = sweep(af, cfg, V0, -5, 45, 0.5);
  for (let i = 0; i + 1 < pts.length; i++) {
    const p = pts[i];
    const q = pts[i + 1];
    if (p.Cm >= 0 && q.Cm < 0) {
      const t = p.Cm / (p.Cm - q.Cm);
      const alpha = p.alphaDeg + t * (q.alphaDeg - p.alphaDeg);
      const CL = p.CL + t * (q.CL - p.CL);
      const CD = p.CD + t * (q.CD - p.CD);
      return { trimAlphaDeg: alpha, cmAlpha: (q.Cm - p.Cm) / ((q.alphaDeg - p.alphaDeg) * DEG), CLtrim: CL, LDtrim: CL / CD };
    }
  }
  return { trimAlphaDeg: null, cmAlpha: null, CLtrim: null, LDtrim: null };
}

/**
 * Centre of lift (neutral point): where the change in lift with angle of attack acts, on the
 * line through the centre of mass along the nose. Behind the centre of mass = statically stable
 * in pitch. From the tunnel at α₀ (the trim angle, or 5° if there is none):
 *   x_np = x_cm + (Cmα / CLα) · c̄  along the nose (Cmα < 0, CLα > 0 ⇒ behind).
 * Returns the point (model space) and its distance behind the centre of mass (m), or null when
 * the lift hardly changes with α (no meaningful point).
 */
export function centerOfLift(af: Airframe, cfg: PhysicsConfig, V0 = 5): { point: Vec3; behindCom: number } | null {
  const a0 = stability(af, cfg, V0).trimAlphaDeg ?? 5;
  const lo = tunnelPoint(af, cfg, V0, a0 - 1);
  const hi = tunnelPoint(af, cfg, V0, a0 + 1);
  const CLa = (hi.CL - lo.CL) / (2 * DEG);
  const Cma = (hi.Cm - lo.Cm) / (2 * DEG);
  if (!(Math.abs(CLa) > 0.1)) return null;
  const ahead = (Cma / CLa) * af.refChord;
  return { point: V.addScaled(af.com, af.nose, ahead), behindCom: -ahead };
}
