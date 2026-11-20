import * as V from '../math/vec3';
import { applyRigid, applyRigidLinear, composeRigid, rotationAbout } from '../math/transform';
import type { RigidTransform } from '../math/transform';
import { invert, mulVec } from '../math/linalg';
import type { Vec3 } from '../math/vec3';
import * as P from '../math/polygon2d';
import { canonicalNormal, coplanar, facetArea, facetCentroid, facetNormal, facetVertices, facetsOverlap, toModel } from './model';
import type { Facet, PaperModel } from './model';
import { computeEdges, isConnected } from './edges';
import { holderSpec, resolveClip } from './clips';
import { sweepPenetrates } from './penetration';
import type { Op } from './ops';

/*
 * Springback: a crease does not stay exactly where it was pressed. Released, it opens by a
 * fraction of its fold angle (elastic recovery of the bent fibres), so a well-pressed 180° fold
 * gapes a few degrees. Applied after the player's folds, per fold operation, last fold first:
 *   - the paper that fold moved rotates back about the fold's crease, as one unit (layers folded
 *     together open together);
 *   - a crease that a later fold has bent is locked (it is no longer a straight line);
 *   - both sides of the crease turn away from each other (a centre fold opens symmetrically);
 *   - at a clip or tape across the crease the paper stays shut, and opens linearly away from it;
 *   - layers sandwiched by other paper stay;
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


/**
 * Where clips or tape hold this crease shut: intervals along the crease axis (m, measured from
 * h.p along h.d) under their footprint. A holder only counts if it squeezes paper from both
 * sides of the crease.
 */
function heldZones(model: PaperModel, moving: Set<number>, h: { p: Vec3; d: Vec3 }): [number, number][] {
  const out: [number, number][] = [];
  for (const c of model.clips) {
    const rc = resolveClip(model, c);
    if (!rc || !rc.clamped.some((x) => moving.has(x)) || !rc.clamped.some((x) => !moving.has(x))) continue;
    const spec = holderSpec(c);
    const hw = spec.widthM / 2;
    const corners = [
      V.addScaled(rc.position, rc.edgeDirModel, -hw),
      V.addScaled(rc.position, rc.edgeDirModel, hw),
      V.addScaled(V.addScaled(rc.position, rc.edgeDirModel, hw), rc.inward, spec.lengthM),
      V.addScaled(V.addScaled(rc.position, rc.edgeDirModel, -hw), rc.inward, spec.lengthM),
    ].map((q) => V.dot(V.sub(q, h.p), h.d));
    out.push([Math.min(...corners), Math.max(...corners)]);
  }
  return out;
}

/** Distance (m) along the axis from s to the nearest held zone (0 inside one). */
function distToZones(s: number, zones: [number, number][]): number {
  let d = Infinity;
  for (const [a, b] of zones) d = Math.min(d, s < a ? a - s : s > b ? s - b : 0);
  return d;
}

/**
 * Open a crease by `angle`, shared by its two sides about the crease (each side turns away
 * from the other; the lighter side turns more, in proportion to the other side's paper area, so
 * a centre fold opens symmetrically about the middle).
 *
 * Where clips or tape hold the crease, it stays shut there, and the opening grows linearly with
 * the distance from the holder, reaching the full springback at the end of the paper: the two
 * sides run as straight lines from the clip, a tight V. Facets along the way are cut into strips
 * (narrow enough that the twisted surface is matched to ≈ 0.2 mm) and triangles, each placed
 * rigidly on the twisted surface, so the paper bends smoothly instead of breaking.
 */
function openCrease(
  model: PaperModel,
  moving: Set<number>,
  h: { p: Vec3; d: Vec3 },
  angle: number,
  zones: [number, number][],
  origT: Map<number, RigidTransform>,
): PaperModel {
  const sOf = (q: Vec3) => V.dot(V.sub(q, h.p), h.d);
  let aM = 0;
  let aS = 0;
  let sMin = Infinity;
  let sMax = -Infinity;
  for (const f of model.facets) {
    if (moving.has(f.id)) aM += facetArea(f);
    else aS += facetArea(f);
    for (const v of facetVertices(f)) {
      const sv = sOf(v);
      sMin = Math.min(sMin, sv);
      sMax = Math.max(sMax, sv);
    }
  }
  const shareM = aS / (aM + aS);
  const shareS = -aM / (aM + aS);
  const zLo = Math.min(...zones.map((z) => z[0]));
  const zHi = Math.max(...zones.map((z) => z[1]));
  // fraction of the full opening at s: 0 at a holder, 1 at the end of the paper
  const ramp = (s: number) => {
    if (!zones.length) return 1;
    const d = distToZones(s, zones);
    if (d === 0) return 0;
    const span = s < zLo ? zLo - sMin : s > zHi ? sMax - zHi : Math.max(sMax - zHi, zLo - sMin);
    return Math.min(1, d / Math.max(span, 1e-3));
  };
  const facets: Facet[] = [];
  let nextId = model.nextFacetId;
  const place = (q: Facet, R: RigidTransform, T: RigidTransform): Facet => {
    const cOld = canonicalNormal(facetNormal(q));
    const flipped = V.dot(applyRigidLinear(R, cOld), canonicalNormal(applyRigidLinear(T, [0, 1, 0]))) < 0;
    return { ...q, T, layer: flipped ? -q.layer : q.layer };
  };
  for (const f of model.facets) {
    const share = moving.has(f.id) ? shareM : shareS;
    const theta = (sv: number) => angle * share * ramp(sv);
    const vs = facetVertices(f);
    const ss = vs.map(sOf);
    const th = ss.map(theta);
    const spread = Math.max(...th) - Math.min(...th);
    if (spread < 0.05 * DEG) {
      const a = (Math.max(...th) + Math.min(...th)) / 2;
      if (Math.abs(a) < 1e-12) facets.push(f);
      else {
        const R = rotationAbout(h.p, h.d, a);
        facets.push(place(f, R, composeRigid(R, f.T)));
      }
      continue;
    }
    // strips across the axis: twist rate × strip × reach ≈ 0.2 mm
    const lo = Math.min(...ss);
    const hi = Math.max(...ss);
    const rate = spread / Math.max(hi - lo, 1e-6);
    const reach = Math.max(...vs.map((v) => V.distToLine(v, h.p, h.d)), 1e-3);
    const strip = Math.min(0.03, Math.max(0.003, 4e-4 / (rate * reach)));
    let pieces: Facet[] = [f];
    for (let c = lo + strip; c < hi - 1e-6; c += strip) {
      const next: Facet[] = [];
      for (const q of pieces) {
        const vals = facetVertices(q).map((v) => sOf(v) - c);
        const { neg, pos } = P.splitConvexByValues(q.poly, vals, 1e-9);
        if (neg && pos) next.push({ ...q, id: nextId++, poly: neg }, { ...q, id: nextId++, poly: pos });
        else next.push(q);
      }
      pieces = next;
    }
    const mapPoint = (x: Vec3) => applyRigid(rotationAbout(h.p, h.d, theta(sOf(x))), x);
    for (const q of pieces) {
      for (let i = 1; i + 1 < q.poly.length; i++) {
        const tri = [q.poly[0], q.poly[i], q.poly[i + 1]];
        if (Math.abs(P.signedArea(tri)) < 1e-12) continue;
        const sheet = tri.map((t) => [t[0], 0, t[1]] as Vec3);
        const target = sheet.map((x) => mapPoint(applyRigid(q.T, x)));
        // start from the rotation at the triangle's centre, then the best rigid fit
        const Rc = rotationAbout(h.p, h.d, theta(sOf(applyRigid(q.T, V.centroid(sheet)))));
        const R0 = composeRigid(Rc, q.T);
        const fit = bestFit(sheet.map((x) => applyRigid(R0, x)), target, [1, 1, 1]);
        const piece = place({ ...q, id: q.poly.length === 3 ? q.id : nextId++, poly: tri }, Rc, composeRigid(fit, R0));
        facets.push(piece);
        origT.set(piece.id, origT.get(f.id) ?? f.T);
      }
    }
  }
  return { ...model, facets, nextFacetId: nextId };
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
  // where each piece of paper was before relaxing (pieces cut off a facet share its transform)
  const origT = new Map(model.facets.map((f) => [f.id, f.T]));
  let cur = model;
  for (const id of ids.reverse()) {
    const moving = new Set(cur.facets.filter((f) => f.foldPath.includes(id)).map((f) => f.id));
    if (moving.size === 0 || moving.size === cur.facets.length) continue;
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
    cur = openCrease(cur, moving, h, angle, heldZones(cur, moving, h), origT);
  }
  if (cur === model) return model;
  // keep the plane where it was (the body axes refer to the folded model)
  const from: Vec3[] = [];
  const to: Vec3[] = [];
  const w: number[] = [];
  for (const g of cur.facets) {
    const T0 = origT.get(g.id) ?? g.T;
    const a = facetArea(g);
    for (const q of g.poly) {
      const x: Vec3 = [q[0], 0, q[1]];
      from.push(applyRigid(g.T, x));
      to.push(applyRigid(T0, x));
      w.push(a);
    }
  }
  const fit = bestFit(from, to, w);
  return { ...cur, facets: cur.facets.map((f) => ({ ...f, T: composeRigid(fit, f.T) })) };
}
