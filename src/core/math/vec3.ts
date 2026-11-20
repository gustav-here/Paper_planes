/** 3D vector as a plain tuple (serialises directly to JSON). */
export type Vec3 = [number, number, number];

export const vec3 = (x = 0, y = 0, z = 0): Vec3 => [x, y, z];
export const clone = (a: Vec3): Vec3 => [a[0], a[1], a[2]];
export const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s];
export const negate = (a: Vec3): Vec3 => [-a[0], -a[1], -a[2]];
export const addScaled = (a: Vec3, b: Vec3, s: number): Vec3 => [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s];
export const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const lengthSq = (a: Vec3): number => dot(a, a);
export const length = (a: Vec3): number => Math.sqrt(dot(a, a));
export const dist = (a: Vec3, b: Vec3): number => length(sub(a, b));
export const lerp = (a: Vec3, b: Vec3, t: number): Vec3 => [
  a[0] + (b[0] - a[0]) * t,
  a[1] + (b[1] - a[1]) * t,
  a[2] + (b[2] - a[2]) * t,
];

export function normalize(a: Vec3): Vec3 {
  const l = length(a);
  if (l < 1e-300) return [0, 0, 0];
  return [a[0] / l, a[1] / l, a[2] / l];
}

export function almostEqual(a: Vec3, b: Vec3, eps = 1e-9): boolean {
  return Math.abs(a[0] - b[0]) <= eps && Math.abs(a[1] - b[1]) <= eps && Math.abs(a[2] - b[2]) <= eps;
}

/** Component of `a` perpendicular to unit vector `n`. */
export const rejectFrom = (a: Vec3, n: Vec3): Vec3 => addScaled(a, n, -dot(a, n));

/** Distance from point p to the infinite line through `origin` with unit direction `dir`. */
export function distToLine(p: Vec3, origin: Vec3, dir: Vec3): number {
  const r = sub(p, origin);
  return length(rejectFrom(r, dir));
}

/** Any unit vector perpendicular to unit vector n. */
export function anyPerpendicular(n: Vec3): Vec3 {
  const ax = Math.abs(n[0]);
  const ay = Math.abs(n[1]);
  const az = Math.abs(n[2]);
  const helper: Vec3 = ax <= ay && ax <= az ? [1, 0, 0] : ay <= az ? [0, 1, 0] : [0, 0, 1];
  return normalize(cross(n, helper));
}

export function centroid(points: Vec3[]): Vec3 {
  let x = 0;
  let y = 0;
  let z = 0;
  for (const p of points) {
    x += p[0];
    y += p[1];
    z += p[2];
  }
  const n = Math.max(points.length, 1);
  return [x / n, y / n, z / n];
}
