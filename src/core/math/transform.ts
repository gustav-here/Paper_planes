import * as Q from './quat';
import * as M from './mat3';
import * as V from './vec3';
import type { Vec3 } from './vec3';
import type { Quat } from './quat';
import type { Mat3 } from './mat3';

/** Proper rigid transform p ↦ q·p + t. */
export interface RigidTransform {
  q: Quat;
  t: Vec3;
}

export const rigidIdentity = (): RigidTransform => ({ q: Q.identity(), t: [0, 0, 0] });

export const applyRigid = (T: RigidTransform, p: Vec3): Vec3 => V.add(Q.rotate(T.q, p), T.t);
export const applyRigidLinear = (T: RigidTransform, v: Vec3): Vec3 => Q.rotate(T.q, v);

/** (a ∘ b)(p) = a(b(p)) */
export function composeRigid(a: RigidTransform, b: RigidTransform): RigidTransform {
  return { q: Q.normalize(Q.mul(a.q, b.q)), t: V.add(Q.rotate(a.q, b.t), a.t) };
}

export function invertRigid(T: RigidTransform): RigidTransform {
  const qi = Q.conjugate(T.q);
  return { q: qi, t: V.negate(Q.rotate(qi, T.t)) };
}

/** Rotation by `angle` (right-hand rule) about the axis through `point` with unit direction `dir`. */
export function rotationAbout(point: Vec3, dir: Vec3, angle: number): RigidTransform {
  const q = Q.fromAxisAngle(dir, angle);
  return { q, t: V.sub(point, Q.rotate(q, point)) };
}

export function sameRigid(a: RigidTransform, b: RigidTransform, eps = 1e-9): boolean {
  return Q.sameRotation(a.q, b.q, 1e-12) && V.almostEqual(a.t, b.t, eps);
}

/** General affine isometry p ↦ m·p + t (m may be improper, e.g. a reflection). */
export interface Affine {
  m: Mat3;
  t: Vec3;
}

export const applyAffine = (A: Affine, p: Vec3): Vec3 => V.add(M.mulVec(A.m, p), A.t);
export const applyAffineLinear = (A: Affine, v: Vec3): Vec3 => M.mulVec(A.m, v);

export function composeAffine(a: Affine, b: Affine): Affine {
  return { m: M.mul(a.m, b.m), t: V.add(M.mulVec(a.m, b.t), a.t) };
}

/** Inverse of an orthogonal affine map. */
export function invertOrthogonal(A: Affine): Affine {
  const mt = M.transpose(A.m);
  return { m: mt, t: V.negate(M.mulVec(mt, A.t)) };
}

export const rigidToAffine = (T: RigidTransform): Affine => ({ m: Q.toMat3(T.q), t: [...T.t] as Vec3 });

export function sameAffine(a: Affine, b: Affine, eps = 1e-7): boolean {
  return M.almostEqual(a.m, b.m, eps) && V.almostEqual(a.t, b.t, eps);
}
