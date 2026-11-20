import * as V from '../math/vec3';
import * as Q from '../math/quat';
import * as M3 from '../math/mat3';
import { mulVec } from '../math/linalg';
import type { Vec3 } from '../math/vec3';
import type { Quat } from '../math/quat';
import type { Airframe } from './airframe';
import { evaluateAero } from './aeroModel';
import type { AeroCache, AeroDebug } from './aeroModel';
import type { PhysicsConfig } from './config';

/** 6-DOF state. Position/orientation in the world, velocities in the body frame. */
export interface BodyState {
  /** CoM position, world (m). */
  x: Vec3;
  /** Orientation body → world. */
  q: Quat;
  /** CoM velocity, body frame (m/s). */
  v: Vec3;
  /** Angular velocity, body frame (rad/s). */
  w: Vec3;
}

export interface Derivative {
  dx: Vec3;
  dq: Quat;
  dv: Vec3;
  dw: Vec3;
}

export const cloneState = (s: BodyState): BodyState => ({ x: [...s.x], q: [...s.q], v: [...s.v], w: [...s.w] });

/**
 * Equations of motion (Kirchhoff/Fossen form, body frame):
 *   (M_RB + M_A) ν̇ = τ_aero + τ_grav − C_RB(ν)ν − C_A(ν)ν
 * with p = A11 v + A12 ω, h = A21 v + A22 ω (added momentum):
 *   lin: −ω × (m v + p)
 *   ang: −ω × (I ω + h) − v × p        (v × p is the Munk moment; off by default, see config)
 */
export function derivative(
  s: BodyState,
  af: Airframe,
  cfg: PhysicsConfig,
  cache: AeroCache,
  debug?: AeroDebug,
): Derivative {
  const q = Q.normalize(s.q);
  const R = Q.toMat3(q);
  const aero = evaluateAero(af, s.v, s.w, cfg.aero, cfg.rho, cache, debug);
  const gWorld: Vec3 = [0, -af.mass * cfg.g, 0];
  const gBody = M3.mulVec(M3.transpose(R), gWorld);

  const nu = [...s.v, ...s.w];
  const MA = af.MA;
  const am = mulVec(MA, nu, 6);
  const p: Vec3 = [am[0], am[1], am[2]];
  const h: Vec3 = [am[3], am[4], am[5]];
  const Iw = M3.mulVec(af.inertia, s.w);
  const lin = V.sub(V.add(aero.force, gBody), V.cross(s.w, V.add(V.scale(s.v, af.mass), p)));
  let ang = V.sub(aero.torque, V.cross(s.w, V.add(Iw, h)));
  if (cfg.munkMoment) ang = V.sub(ang, V.cross(s.v, p));
  const acc = mulVec(af.Minv, [...lin, ...ang], 6);

  const wq: Quat = [s.w[0], s.w[1], s.w[2], 0];
  const dq = Q.mul(q, wq).map((c) => c / 2) as Quat;
  return { dx: M3.mulVec(R, s.v), dq, dv: [acc[0], acc[1], acc[2]], dw: [acc[3], acc[4], acc[5]] };
}

function addScaled(s: BodyState, d: Derivative, h: number): BodyState {
  return {
    x: V.addScaled(s.x, d.dx, h),
    q: Q.normalize([s.q[0] + d.dq[0] * h, s.q[1] + d.dq[1] * h, s.q[2] + d.dq[2] * h, s.q[3] + d.dq[3] * h]),
    v: V.addScaled(s.v, d.dv, h),
    w: V.addScaled(s.w, d.dw, h),
  };
}

/** Classic 4th-order Runge–Kutta step. */
export function rk4Step(s: BodyState, h: number, f: (s: BodyState) => Derivative): BodyState {
  const k1 = f(s);
  const k2 = f(addScaled(s, k1, h / 2));
  const k3 = f(addScaled(s, k2, h / 2));
  const k4 = f(addScaled(s, k3, h));
  const comb = (a: number[], b: number[], c: number[], d: number[]) => a.map((_, i) => (a[i] + 2 * b[i] + 2 * c[i] + d[i]) / 6);
  return {
    x: V.addScaled(s.x, comb(k1.dx, k2.dx, k3.dx, k4.dx) as Vec3, h),
    q: Q.normalize(s.q.map((c, i) => c + h * (comb(k1.dq, k2.dq, k3.dq, k4.dq)[i])) as Quat),
    v: V.addScaled(s.v, comb(k1.dv, k2.dv, k3.dv, k4.dv) as Vec3, h),
    w: V.addScaled(s.w, comb(k1.dw, k2.dw, k3.dw, k4.dw) as Vec3, h),
  };
}

/** Velocity of a body point r (body frame) expressed in the world frame. */
export function pointVelocityWorld(s: BodyState, r: Vec3): Vec3 {
  return Q.rotate(Q.normalize(s.q), V.add(s.v, V.cross(s.w, r)));
}

export function pointWorld(s: BodyState, r: Vec3): Vec3 {
  return V.add(s.x, Q.rotate(Q.normalize(s.q), r));
}
