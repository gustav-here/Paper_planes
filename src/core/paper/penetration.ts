import * as V from '../math/vec3';
import * as P from '../math/polygon2d';
import { applyRigid, applyRigidLinear, rotationAbout } from '../math/transform';
import type { Vec3 } from '../math/vec3';
import { facetNormal, facetVertices, parallelNormals, planeBasis, projectToPlane } from './model';
import type { Facet } from './model';

interface Poly3 {
  verts: Vec3[];
  n: Vec3;
  basis: [Vec3, Vec3];
  poly2: ReturnType<typeof projectToPlane>;
  box: [Vec3, Vec3];
}

function makePoly(verts: Vec3[], n: Vec3): Poly3 {
  const basis = planeBasis(n);
  const lo: Vec3 = [Infinity, Infinity, Infinity];
  const hi: Vec3 = [-Infinity, -Infinity, -Infinity];
  for (const v of verts) {
    for (let k = 0; k < 3; k++) {
      lo[k] = Math.min(lo[k], v[k]);
      hi[k] = Math.max(hi[k], v[k]);
    }
  }
  return { verts, n, basis, poly2: projectToPlane(verts, verts[0], basis), box: [lo, hi] };
}

const boxesOverlap = (a: Poly3, b: Poly3, m = 1e-6) =>
  a.box[0][0] <= b.box[1][0] + m && b.box[0][0] <= a.box[1][0] + m &&
  a.box[0][1] <= b.box[1][1] + m && b.box[0][1] <= a.box[1][1] + m &&
  a.box[0][2] <= b.box[1][2] + m && b.box[0][2] <= a.box[1][2] + m;

/** Does an edge of A pass strictly through the interior of B (away from the hinge line)? */
function edgesPierce(A: Poly3, B: Poly3, hinge: { p: Vec3; d: Vec3 }): boolean {
  const o = B.verts[0];
  for (let i = 0; i < A.verts.length; i++) {
    const a = A.verts[i];
    const b = A.verts[(i + 1) % A.verts.length];
    const da = V.dot(B.n, V.sub(a, o));
    const db = V.dot(B.n, V.sub(b, o));
    if (!((da > 1e-9 && db < -1e-9) || (da < -1e-9 && db > 1e-9))) continue;
    const x = V.lerp(a, b, da / (da - db));
    if (V.distToLine(x, hinge.p, hinge.d) < 1e-6) continue;
    const r = V.sub(x, o);
    if (P.pointInConvex([V.dot(r, B.basis[0]), V.dot(r, B.basis[1])], B.poly2, 1e-7)) return true;
  }
  return false;
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
  const stat = stationary.map((f) => makePoly(facetVertices(f), facetNormal(f)));
  const hinge = { p: hingePoint, d: hingeDir };
  for (let k = 1; k <= samples; k++) {
    const R = rotationAbout(hingePoint, hingeDir, (angle * k) / samples);
    for (const m of moving) {
      const mv = makePoly(
        facetVertices(m).map((p) => applyRigid(R, p)),
        applyRigidLinear(R, facetNormal(m)),
      );
      for (const s of stat) {
        if (parallelNormals(mv.n, s.n)) continue; // coplanar/parallel stacking is not penetration
        if (!boxesOverlap(mv, s)) continue;
        if (edgesPierce(mv, s, hinge) || edgesPierce(s, mv, hinge)) return true;
      }
    }
  }
  return false;
}
