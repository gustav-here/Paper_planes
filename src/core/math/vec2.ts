/** 2D vector as a plain tuple (serialises directly to JSON). */
export type Vec2 = [number, number];

export const vec2 = (x = 0, y = 0): Vec2 => [x, y];
export const add = (a: Vec2, b: Vec2): Vec2 => [a[0] + b[0], a[1] + b[1]];
export const sub = (a: Vec2, b: Vec2): Vec2 => [a[0] - b[0], a[1] - b[1]];
export const scale = (a: Vec2, s: number): Vec2 => [a[0] * s, a[1] * s];
export const dot = (a: Vec2, b: Vec2): number => a[0] * b[0] + a[1] * b[1];
/** z-component of the 3D cross product. */
export const cross = (a: Vec2, b: Vec2): number => a[0] * b[1] - a[1] * b[0];
export const length = (a: Vec2): number => Math.hypot(a[0], a[1]);
export const dist = (a: Vec2, b: Vec2): number => Math.hypot(a[0] - b[0], a[1] - b[1]);
export const lerp = (a: Vec2, b: Vec2, t: number): Vec2 => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
export const perp = (a: Vec2): Vec2 => [-a[1], a[0]];

export function normalize(a: Vec2): Vec2 {
  const l = length(a);
  if (l < 1e-300) return [0, 0];
  return [a[0] / l, a[1] / l];
}

export function almostEqual(a: Vec2, b: Vec2, eps = 1e-9): boolean {
  return Math.abs(a[0] - b[0]) <= eps && Math.abs(a[1] - b[1]) <= eps;
}
