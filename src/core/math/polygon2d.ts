import * as V2 from './vec2';
import type { Vec2 } from './vec2';

export function signedArea(poly: Vec2[]): number {
  let a = 0;
  for (let i = 0; i < poly.length; i++) {
    const p = poly[i];
    const q = poly[(i + 1) % poly.length];
    a += p[0] * q[1] - q[0] * p[1];
  }
  return a / 2;
}

export const area = (poly: Vec2[]): number => Math.abs(signedArea(poly));

/** Area centroid of a simple polygon. */
export function centroid(poly: Vec2[]): Vec2 {
  let a = 0;
  let cx = 0;
  let cy = 0;
  for (let i = 0; i < poly.length; i++) {
    const p = poly[i];
    const q = poly[(i + 1) % poly.length];
    const c = p[0] * q[1] - q[0] * p[1];
    a += c;
    cx += (p[0] + q[0]) * c;
    cy += (p[1] + q[1]) * c;
  }
  if (Math.abs(a) < 1e-300) {
    // degenerate: vertex average
    let x = 0;
    let y = 0;
    for (const p of poly) {
      x += p[0];
      y += p[1];
    }
    return [x / poly.length, y / poly.length];
  }
  return [cx / (3 * a), cy / (3 * a)];
}

export function bbox(poly: Vec2[]): [number, number, number, number] {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const p of poly) {
    if (p[0] < x0) x0 = p[0];
    if (p[1] < y0) y0 = p[1];
    if (p[0] > x1) x1 = p[0];
    if (p[1] > y1) y1 = p[1];
  }
  return [x0, y0, x1, y1];
}

/** Remove consecutive (and wrap-around) near-duplicate vertices. */
export function dedupe(poly: Vec2[], eps = 1e-12): Vec2[] {
  const out: Vec2[] = [];
  for (const p of poly) {
    if (out.length === 0 || V2.dist(out[out.length - 1], p) > eps) out.push(p);
  }
  while (out.length > 1 && V2.dist(out[0], out[out.length - 1]) <= eps) out.pop();
  return out;
}

/**
 * Split a convex polygon by per-vertex signed values (e.g. signed distances to a plane).
 * Values within ±eps count as "on the line" and go to both halves.
 * Returns null halves when the polygon lies entirely on one side.
 */
export function splitConvexByValues(
  poly: Vec2[],
  values: number[],
  eps: number,
): { neg: Vec2[] | null; pos: Vec2[] | null } {
  const cls = values.map((v) => (v > eps ? 1 : v < -eps ? -1 : 0));
  const hasPos = cls.some((c) => c > 0);
  const hasNeg = cls.some((c) => c < 0);
  if (!hasNeg) return { neg: hasPos ? null : poly, pos: hasPos ? poly : null };
  if (!hasPos) return { neg: poly, pos: null };
  const neg: Vec2[] = [];
  const pos: Vec2[] = [];
  const n = poly.length;
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    if (cls[i] >= 0) pos.push(poly[i]);
    if (cls[i] <= 0) neg.push(poly[i]);
    if ((cls[i] > 0 && cls[j] < 0) || (cls[i] < 0 && cls[j] > 0)) {
      const t = values[i] / (values[i] - values[j]);
      const x = V2.lerp(poly[i], poly[j], t);
      pos.push(x);
      neg.push(x);
    }
  }
  const p = dedupe(pos);
  const q = dedupe(neg);
  return { neg: q.length >= 3 ? q : null, pos: p.length >= 3 ? p : null };
}

/** Signed distance of each vertex to the directed line a→b (positive to the left). */
export function lineValues(poly: Vec2[], a: Vec2, b: Vec2): number[] {
  const d = V2.normalize(V2.sub(b, a));
  return poly.map((p) => V2.cross(d, V2.sub(p, a)));
}

/**
 * Clip segment a→b against a convex polygon (Cyrus–Beck).
 * Returns the parameter interval [t0, t1] ⊂ [0, 1] inside the polygon, or null.
 */
export function clipSegmentConvex(a: Vec2, b: Vec2, poly: Vec2[]): [number, number] | null {
  const orient = Math.sign(signedArea(poly)) || 1;
  let t0 = 0;
  let t1 = 1;
  const d = V2.sub(b, a);
  for (let i = 0; i < poly.length; i++) {
    const p = poly[i];
    const q = poly[(i + 1) % poly.length];
    const e = V2.sub(q, p);
    // inside: orient * cross(e, x - p) >= 0
    const num = orient * V2.cross(e, V2.sub(a, p));
    const den = orient * V2.cross(e, d);
    if (Math.abs(den) < 1e-300) {
      if (num < 0) return null;
      continue;
    }
    const t = -num / den;
    if (den > 0) t0 = Math.max(t0, t);
    else t1 = Math.min(t1, t);
    if (t0 > t1) return null;
  }
  return [t0, t1];
}

/** Minimum signed distance from p to the polygon's edges (positive inside). Convex polygons only. */
export function insideDistance(p: Vec2, poly: Vec2[]): number {
  const orient = Math.sign(signedArea(poly)) || 1;
  let m = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % poly.length];
    const e = V2.sub(b, a);
    const l = V2.length(e);
    if (l < 1e-300) continue;
    const s = (orient * V2.cross(e, V2.sub(p, a))) / l;
    if (s < m) m = s;
  }
  return m;
}

/** Point in convex polygon; margin > 0 requires being strictly inside by that distance. */
export const pointInConvex = (p: Vec2, poly: Vec2[], margin = 0): boolean => insideDistance(p, poly) >= margin;

/**
 * Separating-axis test for convex polygons. Returns true only if they overlap with
 * positive area (penetration depth > eps); touching edges count as not overlapping.
 */
export function convexOverlap(a: Vec2[], b: Vec2[], eps = 1e-9): boolean {
  for (const poly of [a, b]) {
    for (let i = 0; i < poly.length; i++) {
      const p = poly[i];
      const q = poly[(i + 1) % poly.length];
      const axis = V2.normalize(V2.perp(V2.sub(q, p)));
      if (axis[0] === 0 && axis[1] === 0) continue;
      let amin = Infinity, amax = -Infinity, bmin = Infinity, bmax = -Infinity;
      for (const v of a) {
        const s = V2.dot(v, axis);
        if (s < amin) amin = s;
        if (s > amax) amax = s;
      }
      for (const v of b) {
        const s = V2.dot(v, axis);
        if (s < bmin) bmin = s;
        if (s > bmax) bmax = s;
      }
      if (Math.min(amax, bmax) - Math.max(amin, bmin) <= eps) return false;
    }
  }
  return true;
}

export function closestPointOnSegment(p: Vec2, a: Vec2, b: Vec2): { point: Vec2; t: number } {
  const d = V2.sub(b, a);
  const l2 = V2.dot(d, d);
  let t = l2 > 0 ? V2.dot(V2.sub(p, a), d) / l2 : 0;
  t = Math.max(0, Math.min(1, t));
  return { point: V2.lerp(a, b, t), t };
}

/** Distance from p to the infinite line through a, b. */
export function distToLine(p: Vec2, a: Vec2, b: Vec2): number {
  const d = V2.normalize(V2.sub(b, a));
  return Math.abs(V2.cross(d, V2.sub(p, a)));
}

/** Polygon edges as [start, end] pairs. */
export function edges(poly: Vec2[]): [Vec2, Vec2][] {
  return poly.map((p, i) => [p, poly[(i + 1) % poly.length]] as [Vec2, Vec2]);
}
