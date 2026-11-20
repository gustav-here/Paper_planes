import * as V from '../math/vec3';
import * as P from '../math/polygon2d';
import { applyRigid, applyRigidLinear, composeRigid, rotationAbout } from '../math/transform';
import type { Vec3 } from '../math/vec3';
import {
  EPS_PLANE,
  canonicalNormal,
  coplanar,
  facetCentroid,
  facetContains,
  facetInPlane,
  facetNormal,
  facetVertices,
  facetsOverlap,
  heightAlong,
  toModel,
} from './model';
import type { Facet, PaperModel } from './model';
import { computeEdges, isConnected } from './edges';
import { PaperError } from './errors';
import type { PaperWarning } from './errors';
import type { FoldOp } from './ops';
import { sweepPenetrates } from './penetration';

export interface FoldFrame {
  p0: Vec3;
  /** Unit direction of the fold line. */
  d: Vec3;
  /** Unit working-plane normal (towards the viewer). */
  N: Vec3;
  /** Unit normal of the cutting plane, pointing to the moving side. */
  m: Vec3;
  len: number;
}

export function foldFrame(op: Pick<FoldOp, 'line' | 'planeNormal' | 'movingSide'>): FoldFrame {
  const N = V.normalize(op.planeNormal);
  const raw = V.sub(op.line.p1, op.line.p0);
  const d = V.normalize(V.rejectFrom(raw, N));
  const len = V.length(raw);
  if (len < 1e-6 || V.length(d) < 0.5) throw new PaperError('degenerateLine');
  const mRaw = V.rejectFrom(V.rejectFrom(V.normalize(op.movingSide), d), N);
  if (V.length(mRaw) < 1e-6) throw new PaperError('degenerateLine', 'moving side is parallel to the line');
  const m = V.normalize(mRaw);
  return { p0: op.line.p0, d, N, m, len };
}

export interface FoldResult {
  model: PaperModel;
  moved: number[];
  warnings: PaperWarning[];
}

export interface FoldOptions {
  /** Penetration check: none (fast previews), or a sampled sweep. */
  penetration?: 'none' | 'sweep';
}

const DEG = Math.PI / 180;

/**
 * Apply a single fold (no mirroring). Splits every facet crossed by the cutting plane,
 * selects the moving set by scope, checks that no paper would tear, rotates, and updates
 * stacking order.
 */
export function applyFold(model: PaperModel, op: FoldOp, opts: FoldOptions = {}): FoldResult {
  const fr = foldFrame(op);
  const { p0, d, N, m } = fr;
  const sd = (p: Vec3) => V.dot(m, V.sub(p, p0));

  // 1. Split facets crossing the cutting plane.
  let nextId = model.nextFacetId;
  const facets: Facet[] = [];
  const side = new Map<number, -1 | 0 | 1>();
  for (const f of model.facets) {
    const vals = facetVertices(f).map(sd);
    const max = Math.max(...vals);
    const min = Math.min(...vals);
    if (max > EPS_PLANE && min < -EPS_PLANE) {
      const { neg, pos } = P.splitConvexByValues(f.poly, vals, EPS_PLANE);
      if (neg && pos) {
        const a: Facet = { ...f, id: nextId++, poly: neg };
        const b: Facet = { ...f, id: nextId++, poly: pos };
        facets.push(a, b);
        side.set(a.id, -1);
        side.set(b.id, 1);
        continue;
      }
    }
    facets.push(f);
    side.set(f.id, max > EPS_PLANE ? 1 : min < -EPS_PLANE ? -1 : 0);
  }
  const split: PaperModel = { ...model, facets, nextFacetId: nextId };
  const byId = new Map(facets.map((f) => [f.id, f]));

  // 2. Connectivity; links lying on the fold line act as hinges.
  const graph = computeEdges(split);
  const onLine = (a: number, seg: [number, number][]) => {
    const f = byId.get(a)!;
    return seg.every((s) => V.distToLine(toModel(f, s), p0, d) < 1e-7);
  };
  const adj = new Map<number, { other: number; hinge: boolean }[]>();
  for (const l of graph.links) {
    if (!isConnected(l)) continue;
    const hinge = onLine(l.a, l.seg);
    (adj.get(l.a) ?? adj.set(l.a, []).get(l.a)!).push({ other: l.b, hinge });
    (adj.get(l.b) ?? adj.set(l.b, []).get(l.b)!).push({ other: l.a, hinge });
  }

  const positive = facets.filter((f) => side.get(f.id) === 1);
  if (positive.length === 0) throw new PaperError('nothingToFold');

  // 3. Moving set.
  let moving: Set<number>;
  if (op.scope === 'all') {
    moving = new Set(positive.map((f) => f.id));
  } else {
    const seed = op.scope === 'flap' ? seedFromPick(positive, op, N) : seedAtLine(positive, fr, op.scope);
    moving = new Set([seed.id]);
    const grow = (start: number[]) => {
      const stack = [...start];
      while (stack.length) {
        const id = stack.pop()!;
        for (const { other, hinge } of adj.get(id) ?? []) {
          if (hinge || moving.has(other)) continue;
          moving.add(other);
          stack.push(other);
        }
      }
    };
    grow([seed.id]);
    // Loose flaps sandwiched between moving layers come along (you cannot fold around them).
    for (;;) {
      const mov = positive.filter((f) => moving.has(f.id));
      const sandwiched = positive.filter((s) => {
        if (moving.has(s.id)) return false;
        const hs = heightAlong(s, N);
        let below = false;
        let above = false;
        for (const mf of mov) {
          if (!coplanar(mf, s) || !facetsOverlap(mf, s)) continue;
          const hm = heightAlong(mf, N);
          if (hm < hs) below = true;
          if (hm > hs) above = true;
          if (below && above) return true;
        }
        return false;
      });
      if (sandwiched.length === 0) break;
      for (const s of sandwiched) moving.add(s.id);
      grow(sandwiched.map((s) => s.id));
    }
  }

  // Tear check: moving paper may only be attached to the rest along the fold line.
  for (const id of moving) {
    if (side.get(id) !== 1) throw new PaperError('tear');
    for (const { other, hinge } of adj.get(id) ?? []) {
      if (!moving.has(other) && !hinge) throw new PaperError('tear');
    }
  }

  // 4. Rotate. Valley = flap swings towards +N (towards the viewer).
  const theta = op.angleDeg * DEG;
  const sgn = Math.sign(V.dot(V.cross(d, m), N)) || 1;
  const phi = (op.kind === 'valley' ? 1 : -1) * sgn * theta;
  const rot = rotationAbout(p0, d, phi);

  const warnings: PaperWarning[] = [];
  if (opts.penetration === 'sweep') {
    const mov = facets.filter((f) => moving.has(f.id));
    const stat = facets.filter((f) => !moving.has(f.id));
    // Zero-thickness stacks: a flap leaving its stack must not have paper on the side it swings towards.
    const dirSign = op.kind === 'valley' ? 1 : -1;
    const blockedInStack = mov.some((mf) =>
      stat.some(
        (s) =>
          coplanar(mf, s) &&
          facetsOverlap(mf, s) &&
          dirSign * (heightAlong(s, N) - heightAlong(mf, N)) > 0,
      ),
    );
    if (blockedInStack || sweepPenetrates(stat, mov, p0, d, phi, Math.max(4, Math.ceil(op.angleDeg / 15)))) {
      warnings.push({ code: 'penetration' });
    }
  }

  const out: Facet[] = facets.map((f) => {
    if (!moving.has(f.id)) return f;
    const cOld = canonicalNormal(facetNormal(f));
    const T = composeRigid(rot, f.T);
    const nNew = applyRigidLinear(T, [0, 1, 0]);
    const flipped = V.dot(applyRigidLinear(rot, cOld), canonicalNormal(nNew)) < 0;
    return { ...f, T, layer: flipped ? -f.layer : f.layer, foldPath: [...f.foldPath, op.id] };
  });

  // 5. Restack moved facets that landed on stationary paper.
  restack(out, moving, (f) => {
    // centroid just before arrival tells from which side the flap approached
    const pre = rotationAbout(p0, d, phi - Math.sign(phi) * Math.min(2 * DEG, theta / 2));
    const orig = byId.get(f.id)!;
    return applyRigid(pre, facetCentroid(orig));
  });

  return { model: { ...split, facets: out }, moved: [...moving], warnings };
}

function restack(facets: Facet[], moving: Set<number>, preArrival: (f: Facet) => Vec3) {
  const movedFacets = facets.filter((f) => moving.has(f.id));
  const stationary = facets.filter((f) => !moving.has(f.id));
  const done = new Set<number>();
  for (const f of movedFacets) {
    if (done.has(f.id)) continue;
    const group = movedFacets.filter((g) => !done.has(g.id) && (g.id === f.id || coplanar(f, g)));
    group.forEach((g) => done.add(g.id));
    const under = stationary.filter((s) => coplanar(f, s) && group.some((g) => facetsOverlap(g, s)));
    if (under.length === 0) continue;
    const c = canonicalNormal(facetNormal(f));
    const o = facetVertices(f)[0];
    const approach = V.centroid(group.map(preArrival));
    const above = V.dot(c, V.sub(approach, o)) >= 0;
    const gl = group.map((g) => g.layer);
    const sl = under.map((s) => s.layer);
    const shift = above ? Math.max(...sl) + 1 - Math.min(...gl) : Math.min(...sl) - 1 - Math.max(...gl);
    for (const g of group) {
      const idx = facets.findIndex((x) => x.id === g.id);
      facets[idx] = { ...facets[idx], layer: facets[idx].layer + shift };
    }
  }
}

/** Topmost/bottommost facet (seen from N) on the moving side that touches the drawn line. */
function seedAtLine(positive: Facet[], fr: FoldFrame, scope: 'top' | 'bottom'): Facet {
  const { p0, d, N, len } = fr;
  const touching = (f: Facet, restrict: boolean) => {
    const vs = facetVertices(f);
    for (let i = 0; i < vs.length; i++) {
      const a = vs[i];
      const b = vs[(i + 1) % vs.length];
      if (V.distToLine(a, p0, d) > 1e-7 || V.distToLine(b, p0, d) > 1e-7) continue;
      if (!restrict) return true;
      const ta = V.dot(V.sub(a, p0), d);
      const tb = V.dot(V.sub(b, p0), d);
      const lo = Math.max(Math.min(ta, tb), 0);
      const hi = Math.min(Math.max(ta, tb), len);
      if (hi - lo > 1e-6) return true;
    }
    return false;
  };
  const inPlane = positive.filter((f) => facetInPlane(f, p0, N));
  let cands = inPlane.filter((f) => touching(f, true));
  if (cands.length === 0) cands = inPlane.filter((f) => touching(f, false));
  if (cands.length === 0) throw new PaperError('noLayerAtLine');
  const sign = scope === 'top' ? 1 : -1;
  return cands.reduce((best, f) => (sign * heightAlong(f, N) > sign * heightAlong(best, N) ? f : best));
}

/** The visible facet under `pick` on the moving side. */
function seedFromPick(positive: Facet[], op: FoldOp, N: Vec3): Facet {
  const pick = op.pick;
  if (!pick) throw new PaperError('pickNotOnMovingSide');
  const cands = positive.filter((f) => facetContains(f, pick, -1e-7, 1e-4));
  if (cands.length === 0) throw new PaperError('pickNotOnMovingSide');
  return cands.reduce((best, f) => (heightAlong(f, N) > heightAlong(best, N) ? f : best));
}
