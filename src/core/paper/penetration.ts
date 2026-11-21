import * as V from '../math/vec3';
import { applyRigid, applyRigidLinear, rotationAbout } from '../math/transform';
import type { Vec3 } from '../math/vec3';
import { facetNormal, facetVertices, parallelNormals } from './model';
import type { Facet } from './model';

interface Poly3 {
  verts: Vec3[];
  n: Vec3;
  box: [Vec3, Vec3];
}

function makePoly(verts: Vec3[], n: Vec3): Poly3 {
  const lo: Vec3 = [Infinity, Infinity, Infinity];
  const hi: Vec3 = [-Infinity, -Infinity, -Infinity];
  for (const v of verts) {
    for (let k = 0; k < 3; k++) {
      lo[k] = Math.min(lo[k], v[k]);
      hi[k] = Math.max(hi[k], v[k]);
    }
  }
  return { verts, n, box: [lo, hi] };
}

const boxesOverlap = (a: Poly3, b: Poly3, m = 1e-6) =>
  a.box[0][0] <= b.box[1][0] + m && b.box[0][0] <= a.box[1][0] + m &&
  a.box[0][1] <= b.box[1][1] + m && b.box[0][1] <= a.box[1][1] + m &&
  a.box[0][2] <= b.box[1][2] + m && b.box[0][2] <= a.box[1][2] + m;

/**
 * Where does convex polygon A cross the plane of B? Returns the crossing segment, or null if A
 * does not strictly straddle that plane (touching is not penetration).
 */
function crossSegment(A: Poly3, B: Poly3): [Vec3, Vec3] | null {
  const o = B.verts[0];
  const d = A.verts.map((v) => V.dot(B.n, V.sub(v, o)));
  const eps = 1e-9;
  if (!d.some((x) => x > eps) || !d.some((x) => x < -eps)) return null;
  const pts: Vec3[] = [];
  for (let i = 0; i < A.verts.length; i++) {
    const j = (i + 1) % A.verts.length;
    if (Math.abs(d[i]) <= eps) pts.push(A.verts[i]);
    else if ((d[i] > eps && d[j] < -eps) || (d[i] < -eps && d[j] > eps)) {
      pts.push(V.lerp(A.verts[i], A.verts[j], d[i] / (d[i] - d[j])));
    }
  }
  if (pts.length < 2) return null;
  // farthest pair (a convex polygon meets a plane in one segment)
  let best: [Vec3, Vec3] = [pts[0], pts[1]];
  let bd = -1;
  for (let i = 0; i < pts.length; i++)
    for (let j = i + 1; j < pts.length; j++) {
      const dd = V.dist(pts[i], pts[j]);
      if (dd > bd) {
        bd = dd;
        best = [pts[i], pts[j]];
      }
    }
  return best;
}

/**
 * Do two non-parallel convex polygons pass through each other? Both must straddle the other's
 * plane, and their crossing segments (on the common line) must overlap — away from the hinge.
 */
function polygonsPenetrate(A: Poly3, B: Poly3, hinge: { p: Vec3; d: Vec3 }): boolean {
  const sa = crossSegment(A, B);
  if (!sa) return false;
  const sb = crossSegment(B, A);
  if (!sb) return false;
  const L = V.normalize(V.cross(A.n, B.n));
  const ia = [V.dot(sa[0], L), V.dot(sa[1], L)].sort((x, y) => x - y);
  const ib = [V.dot(sb[0], L), V.dot(sb[1], L)].sort((x, y) => x - y);
  const lo = Math.max(ia[0], ib[0]);
  const hi = Math.min(ia[1], ib[1]);
  if (hi - lo <= 1e-6) return false;
  // overlap lying on the hinge itself is the fold, not a collision
  const t = (lo + hi) / 2;
  const base = sa[0];
  const mid = V.addScaled(base, L, t - V.dot(base, L));
  return V.distToLine(mid, hinge.p, hinge.d) > 1e-6;
}

/**
 * Sampled sweep check: does any moving facet pass through a stationary facet while rotating
 * about the hinge from 0 to `angle`? Rigid-paper approximation used for warnings in stage 1.
 */
export function sweepPenetrates(
  stationary: Facet[],
  moving: Facet[],
  hingePoint: Vec3,
  hingeDir: Vec3,
  angle: number,
  samples: number,
): boolean {
  return firstContactSample(stationary, moving, hingePoint, hingeDir, angle, samples) !== null;
}

/**
 * Index k (1…samples) of the first sampled rotation angle·k/samples at which a moving facet
 * passes through a stationary one, or null if the whole sweep is free.
 */
export function firstContactSample(
  stationary: Facet[],
  moving: Facet[],
  hingePoint: Vec3,
  hingeDir: Vec3,
  angle: number,
  samples: number,
): number | null {
  const stat = stationary.map((f) => makePoly(facetVertices(f), facetNormal(f)));
  const hinge = { p: hingePoint, d: hingeDir };
  const shared = sharedHinge(stationary, moving, hingePoint, hingeDir);
  // paper hinged on the same line: swinging past it is going through it (lying flat on it is not)
  const side = shared.map((x) => (Math.abs(x.offset) < Math.PI / 2 && Math.abs(x.offset) > 1e-6 ? Math.sign(x.offset) : 0));
  for (let k = 1; k <= samples; k++) {
    for (let i = 0; i < shared.length; i++) {
      const b = wrap(shared[i].offset + (angle * k) / samples);
      if (Math.abs(b) >= Math.PI / 2) side[i] = 0;
      else if (Math.abs(b) > 1e-6) {
        if (side[i] && Math.sign(b) !== side[i]) return k;
        side[i] = Math.sign(b);
      }
    }
    const R = rotationAbout(hingePoint, hingeDir, (angle * k) / samples);
    for (const m of moving) {
      const mv = makePoly(
        facetVertices(m).map((p) => applyRigid(R, p)),
        applyRigidLinear(R, facetNormal(m)),
      );
      for (const s of stat) {
        if (parallelNormals(mv.n, s.n)) continue; // coplanar/parallel stacking is not penetration
        if (!boxesOverlap(mv, s)) continue;
        if (polygonsPenetrate(mv, s, hinge)) return k;
      }
    }
  }
  return null;
}

const wrap = (x: number) => Math.atan2(Math.sin(x), Math.cos(x));

/**
 * Pairs of a moving and a stationary facet that both have an edge on the hinge line, overlapping
 * along it (a flap and paper hinged on the same crease): the moving facet's angle about the hinge
 * relative to the stationary one, at the start. They meet when it reaches 0.
 */
function sharedHinge(stationary: Facet[], moving: Facet[], p: Vec3, d: Vec3): { offset: number }[] {
  const e1 = V.anyPerpendicular(d);
  const e2 = V.cross(d, e1);
  const onLine = (f: Facet): { lo: number; hi: number; angle: number } | null => {
    const vs = facetVertices(f);
    const along = vs.filter((v) => V.distToLine(v, p, d) < 1e-7).map((v) => V.dot(V.sub(v, p), d));
    if (along.length < 2) return null;
    const c = vs.reduce((acc, v) => V.add(acc, v), [0, 0, 0] as Vec3);
    const arm = V.rejectFrom(V.sub(V.scale(c, 1 / vs.length), p), d);
    if (V.length(arm) < 1e-9) return null;
    return { lo: Math.min(...along), hi: Math.max(...along), angle: Math.atan2(V.dot(arm, e2), V.dot(arm, e1)) };
  };
  const st = stationary.map(onLine).filter((x): x is NonNullable<typeof x> => !!x);
  const out: { offset: number }[] = [];
  for (const m of moving) {
    const a = onLine(m);
    if (!a) continue;
    for (const s of st) {
      if (Math.min(a.hi, s.hi) - Math.max(a.lo, s.lo) <= 1e-6) continue;
      out.push({ offset: wrap(a.angle - s.angle) });
    }
  }
  return out;
}
