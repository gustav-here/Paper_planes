import * as V from '../math/vec3';
import type { Vec3 } from '../math/vec3';
import type { Quat } from '../math/quat';
import type { Airframe } from './airframe';
import type { BodyState } from './rigidBody';

export interface LaunchParams {
  /** Throw speed (m/s), 1–15 in the game. */
  speed: number;
  /** Climb angle of the throw above horizontal (deg). */
  elevationDeg: number;
  /** Heading in the horizontal plane (deg), 0 = world +X. */
  headingDeg: number;
  /** Release height of the centre of mass (m). */
  heightM: number;
  /** Nose pitched up relative to the throw direction (deg). */
  aoaDeg?: number;
  /** Bank angle (deg), right wing down positive. */
  rollDeg?: number;
}

export const DEFAULT_LAUNCH: LaunchParams = { speed: 5, elevationDeg: 0, headingDeg: 0, heightM: 1.6, aoaDeg: 0, rollDeg: 0 };

const DEG = Math.PI / 180;

/** Quaternion from a rotation matrix given by its columns. */
function quatFromColumns(c0: Vec3, c1: Vec3, c2: Vec3): Quat {
  const m00 = c0[0], m10 = c0[1], m20 = c0[2];
  const m01 = c1[0], m11 = c1[1], m21 = c1[2];
  const m02 = c2[0], m12 = c2[1], m22 = c2[2];
  const tr = m00 + m11 + m22;
  let q: Quat;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    q = [(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, s / 4];
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
    q = [s / 4, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s];
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
    q = [(m01 + m10) / s, s / 4, (m12 + m21) / s, (m02 - m20) / s];
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
    q = [(m02 + m20) / s, (m12 + m21) / s, s / 4, (m10 - m01) / s];
  }
  const l = Math.hypot(...q);
  return q.map((x) => x / l) as Quat;
}

/**
 * Initial state: CoM at (0, height, 0), velocity along the throw direction, nose pitched up by
 * the release angle of attack, wings level (then banked by roll), no rotation.
 */
export function launchState(af: Airframe, p: LaunchParams): BodyState {
  const el = p.elevationDeg * DEG;
  const hd = p.headingDeg * DEG;
  const dir: Vec3 = [Math.cos(el) * Math.cos(hd), Math.sin(el), Math.cos(el) * Math.sin(hd)];
  const pitch = el + (p.aoaDeg ?? 0) * DEG;
  let noseW: Vec3 = [Math.cos(pitch) * Math.cos(hd), Math.sin(pitch), Math.cos(pitch) * Math.sin(hd)];
  let upW: Vec3 = V.normalize(V.rejectFrom([0, 1, 0], noseW));
  const roll = (p.rollDeg ?? 0) * DEG;
  if (roll !== 0) {
    const r0 = V.cross(noseW, upW);
    upW = V.normalize(V.add(V.scale(upW, Math.cos(roll)), V.scale(r0, -Math.sin(roll))));
  }
  noseW = V.normalize(noseW);
  const rightW = V.cross(noseW, upW);
  // R maps body axes (nose, up, right) to world axes: R = W·Bᵀ
  const B = [af.nose, af.up, af.right];
  const W = [noseW, upW, rightW];
  const col = (j: number): Vec3 => {
    const e: Vec3 = [0, 0, 0];
    e[j] = 1;
    // R e_j = Σ_k W_k (B_k · e_j)
    let out: Vec3 = [0, 0, 0];
    for (let k = 0; k < 3; k++) out = V.addScaled(out, W[k], V.dot(B[k], e));
    return out;
  };
  const q = quatFromColumns(col(0), col(1), col(2));
  // body-frame velocity = Rᵀ (speed · dir) = Σ_k B_k (W_k · dir)·speed
  let vb: Vec3 = [0, 0, 0];
  for (let k = 0; k < 3; k++) vb = V.addScaled(vb, B[k], V.dot(W[k], dir) * p.speed);
  return { x: [0, p.heightM, 0], q, v: vb, w: [0, 0, 0] };
}
