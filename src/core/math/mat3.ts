import type { Vec3 } from './vec3';

/** 3×3 matrix, row-major, 9 numbers. */
export type Mat3 = number[];

export const identity = (): Mat3 => [1, 0, 0, 0, 1, 0, 0, 0, 1];
export const diag = (a: number, b: number, c: number): Mat3 => [a, 0, 0, 0, b, 0, 0, 0, c];

export function mul(a: Mat3, b: Mat3): Mat3 {
  const r = new Array<number>(9);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    }
  }
  return r;
}

export function mulVec(m: Mat3, v: Vec3): Vec3 {
  return [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  ];
}

export const transpose = (m: Mat3): Mat3 => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];

export function det(m: Mat3): number {
  return (
    m[0] * (m[4] * m[8] - m[5] * m[7]) -
    m[1] * (m[3] * m[8] - m[5] * m[6]) +
    m[2] * (m[3] * m[7] - m[4] * m[6])
  );
}

export function almostEqual(a: Mat3, b: Mat3, eps = 1e-9): boolean {
  for (let i = 0; i < 9; i++) if (Math.abs(a[i] - b[i]) > eps) return false;
  return true;
}
