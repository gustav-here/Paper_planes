import * as V from '../math/vec3';
import { applyRigid, applyRigidLinear, composeRigid, rotationAbout } from '../math/transform';
import type { RigidTransform } from '../math/transform';
import { invert, mulVec } from '../math/linalg';
import type { Vec3 } from '../math/vec3';
import { canonicalNormal, coplanar, facetArea, facetCentroid, facetNormal, facetVertices, facetsOverlap, toModel } from './model';
import type { Facet, PaperModel } from './model';
import { computeEdges, isConnected } from './edges';
import { resolveClip } from './clips';
import { sweepPenetrates } from './penetration';
import type { Op } from './ops';

/*
 * Springback: a crease does not stay exactly where it was pressed. Released, it opens by a
 * fraction of its fold angle (elastic recovery of the bent fibres), so a well-pressed 180° fold
 * gapes a few degrees. Applied after the player's folds, per fold operation, last fold first:
 *   - the paper that fold moved rotates back about the fold's crease, as one unit (layers folded
 *     together open together);
 *   - a crease that a later fold has bent is locked (it is no longer a straight line);
 *   - layers held by a clip or tape across the crease, or sandwiched by other paper, stay;
 *   - finally the whole model is re-aligned (least squares) with the folded model, so the body
 *     axes still fit (a gaping centre fold opens symmetrically).
 */

const DEG = Math.PI / 180;

interface Hinge {
  p: Vec3;
  d: Vec3;
  /** A facet on each side of the crease (moving side, other side). */
  a: Facet;
  b: Facet;
}

/** The crease of fold `id`, if it is still one straight line. */
function creaseOf(model: PaperModel, moving: Set<number>): Hinge | null {
  const byId = new Map(model.facets.map((f) => [f.id, f]));
  const links = computeEdges(model).links.filter((l) => isConnected(l) && l.kind === 'crease' && moving.has(l.a) !== moving.has(l.b));
  if (!links.length) return null;
  const pts: Vec3[] = [];
  let best = links[0];
  let bestLen = -1;
  for (const l of links) {
    const f = byId.get(l.a)!;
    const A = toModel(f, l.seg[0]);
    const B = toModel(f, l.seg[1]);
    pts.push(A, B);
    if (V.dist(A, B) > bestLen) {
      bestLen = V.dist(A, B);
      best = l;
    }
  }
  let [p, q] = [pts[0], pts[1]];
  let far = 0;
  for (const x of pts)
    for (const y of pts)
      if (V.dist(x, y) > far) {
        far = V.dist(x, y);
        p = x;
        q = y;
      }
  if (far < 1e-6) return null;
  const d = V.normalize(V.sub(q, p));
  if (pts.some((x) => V.distToLine(x, p, d) > 1e-6)) return null;
  const fa = byId.get(best.a)!;
  const fb = byId.get(best.b)!;
  return moving.has(fa.id) ? { p, d, a: fa, b: fb } : { p, d, a: fb, b: fa };
}

/** Unit vector perpendicular to the line (p, d) towards the facet's centroid. */
function arm(f: Facet, p: Vec3, d: Vec3): Vec3 {
  const r = V.sub(facetCentroid(f), p);
  return V.normalize(V.rejectFrom(r, d));
}

/**
 * Signed opening rotation (rad) about d for the crease: positive values open it. Returns the
 * current fold angle too (0 = flat, π = pressed flat onto itself).
 */
function opening(h: Hinge): { sign: number; foldAngle: number } {
  const w = arm(h.a, h.p, h.d);
  const v = arm(h.b, h.p, h.d);
  const psi = Math.atan2(V.dot(h.d, V.cross(v, w)), V.dot(v, w)); // dihedral, signed about d
  const foldAngle = Math.PI - Math.abs(psi);
  if (Math.abs(psi) > 1e-4) return { sign: Math.sign(psi), foldAngle };
  // pressed flat (180°): open towards the side the moving layer lies on in the stack
  const c = canonicalNormal(facetNormal(h.b));
  const above = h.a.layer > h.b.layer ? 1 : -1;
  return { sign: Math.sign(V.dot(V.cross(h.d, w), c)) * above || 1, foldAngle };
}

/** Would rotating `mov` by `angle` lift it into stationary layers stacked on that side? */
function blocked(mov: Facet[], stat: Facet[], p: Vec3, d: Vec3, angle: number): boolean {
  for (const m of mov) {
    const c = canonicalNormal(facetNormal(m));
    const motion = V.cross(d, V.sub(facetCentroid(m), p));
    const dir = Math.sign(V.dot(motion, c)) * Math.sign(angle);
    if (!dir) continue;
    for (const s of stat) {
      if (!coplanar(m, s) || !facetsOverlap(m, s)) continue;
      const sameSide = Math.sign(V.dot(canonicalNormal(facetNormal(s)), c)) || 1;
      if (dir * (s.layer * sameSide - m.layer) > 0) return true;
    }
  }
  return sweepPenetrates(stat, mov, p, d, angle, 3);
}

function rotateFacets(model: PaperModel, ids: Set<number>, R: RigidTransform): PaperModel {
  return {
    ...model,
    facets: model.facets.map((f) => {
      if (!ids.has(f.id)) return f;
      const cOld = canonicalNormal(facetNormal(f));
      const T = composeRigid(R, f.T);
      const nNew = applyRigidLinear(T, [0, 1, 0]);
      const flipped = V.dot(applyRigidLinear(R, cOld), canonicalNormal(nNew)) < 0;
      return { ...f, T, layer: flipped ? -f.layer : f.layer };
    }),
  };
}

/** Facets squeezed together by clips or tape. */
function heldTogether(model: PaperModel): number[][] {
  const out: number[][] = [];
  for (const c of model.clips) {
    const rc = resolveClip(model, c);
    if (rc && rc.clamped.length > 1) out.push(rc.clamped);
  }
  return out;
}

/** Rigid transform that best maps `from` onto `to` (area-weighted points), small rotations. */
function bestFit(from: Vec3[], to: Vec3[], w: number[]): RigidTransform {
  let R: RigidTransform = { q: [0, 0, 0, 1], t: [0, 0, 0] };
  for (let it = 0; it < 4; it++) {
    // linearised: x' = R(x) + ω × R(x) + t, minimise Σ w |x' − y|²
    const A = new Array<number>(36).fill(0);
    const b = new Array<number>(6).fill(0);
    from.forEach((x0, i) => {
      const x = applyRigid(R, x0);
      const r = V.sub(to[i], x);
      // rows of the 3×6 Jacobian [−[x]×  I]
      const Jr = [
        [0, x[2], -x[1], 1, 0, 0],
        [-x[2], 0, x[0], 0, 1, 0],
        [x[1], -x[0], 0, 0, 0, 1],
      ];
      for (let k = 0; k < 3; k++)
        for (let a = 0; a < 6; a++) {
          b[a] += w[i] * Jr[k][a] * r[k];
          for (let c = 0; c < 6; c++) A[a * 6 + c] += w[i] * Jr[k][a] * Jr[k][c];
        }
    });
    const s = mulVec(invert(A, 6), b, 6);
    const om: Vec3 = [s[0], s[1], s[2]];
    const ang = V.length(om);
    const step = composeRigid({ q: [0, 0, 0, 1], t: [s[3], s[4], s[5]] }, ang > 1e-12 ? rotationAbout([0, 0, 0], V.scale(om, 1 / ang), ang) : { q: [0, 0, 0, 1], t: [0, 0, 0] });
    R = composeRigid(step, R);
    if (ang < 1e-10 && Math.hypot(s[3], s[4], s[5]) < 1e-12) break;
  }
  return R;
}

/**
 * The plane as it rests after folding: every fold opened by `fraction` of its angle where the
 * paper lets it. `ops` are the design operations that produced `model` (in order).
 */
export function relaxCreases(model: PaperModel, ops: Op[], fraction: number): PaperModel {
  if (fraction <= 0) return model;
  const ids: string[] = [];
  for (const op of ops) if (op.type === 'fold') ids.push(op.id, `${op.id}'`);
  // folds not in the list (e.g. an editor preview) come last, in the order the paper saw them
  const known = new Set(ids);
  for (const f of model.facets)
    for (const id of f.foldPath)
      if (!known.has(id)) {
        known.add(id);
        ids.push(id);
      }
  const held = heldTogether(model);
  let cur = model;
  for (const id of ids.reverse()) {
    const moving = new Set(cur.facets.filter((f) => f.foldPath.includes(id)).map((f) => f.id));
    if (moving.size === 0 || moving.size === cur.facets.length) continue;
    if (held.some((g) => g.some((x) => moving.has(x)) && g.some((x) => !moving.has(x)))) continue;
    const h = creaseOf(cur, moving);
    if (!h) continue;
    const { sign, foldAngle } = opening(h);
    if (foldAngle < 0.5 * DEG) continue;
    const mov = cur.facets.filter((f) => moving.has(f.id));
    const stat = cur.facets.filter((f) => !moving.has(f.id));
    let angle = sign * fraction * foldAngle;
    // layers on top hold it down; try a smaller opening before giving up
    while (Math.abs(angle) > 0.25 * DEG && blocked(mov, stat, h.p, h.d, angle)) angle /= 2;
    if (Math.abs(angle) <= 0.25 * DEG) continue;
    cur = rotateFacets(cur, moving, rotationAbout(h.p, h.d, angle));
  }
  if (cur === model) return model;
  // keep the plane where it was (the body axes refer to the folded model)
  const from: Vec3[] = [];
  const to: Vec3[] = [];
  const w: number[] = [];
  model.facets.forEach((f, i) => {
    const g = cur.facets[i];
    const a = facetArea(f);
    facetVertices(g).forEach((p, k) => {
      from.push(p);
      to.push(facetVertices(f)[k]);
      w.push(a);
    });
  });
  const fit = bestFit(from, to, w);
  return { ...cur, facets: cur.facets.map((f) => ({ ...f, T: composeRigid(fit, f.T) })) };
}
