import * as V from '../math/vec3';
import * as V2 from '../math/vec2';
import * as Q from '../math/quat';
import * as P from '../math/polygon2d';
import type { Vec3 } from '../math/vec3';
import { facetCentroid, facetContains, facetNormal, toModel } from '../paper/model';
import type { PaperModel } from '../paper/model';
import type { CutOp, FoldOp, Op } from '../paper/ops';
import { applyOp } from '../paper/replay';
import { foldFrame } from '../paper/fold';
import { applyRigid, applyRigidLinear, composeRigid, rigidIdentity, rotationAbout } from '../math/transform';
import type { PaperConfig } from './config';
import { interiorCutEnds } from './panels';
import type { DamageEvent } from './simulator';

/*
 * Damage becomes ordinary design operations appended after the player's folds, so a damaged
 * plane is still just "folds and cuts": deterministic, replayable, and the same physics flies it.
 *   crumple → the edge is pushed in: an accordion of small alternating folds at the impact
 *   bend    → the crease's fold angle changes by the plastic bend
 *   tear    → the cut is extended from its end
 * An operation that cannot be applied (it would tear paper etc.) is skipped.
 */

export interface DamageResult {
  ops: Op[];
  model: PaperModel;
}

let seq = 0;
const nextId = (kind: string) => `dmg-${kind}-${++seq}`;

function tryOps(model: PaperModel, candidates: Op[]): { op: Op; model: PaperModel } | null {
  for (const op of candidates) {
    try {
      const r = applyOp(model, op, { penetration: 'sweep' });
      if (r.warnings.some((w) => w.code === 'penetration')) continue;
      return { op, model: r.model };
    } catch {
      /* try the next variant */
    }
  }
  return null;
}

/** The rigid motion a fold gives the paper it moves (same convention as applyFold). */
function foldMotion(op: FoldOp) {
  const fr = foldFrame(op);
  const sgn = Math.sign(V.dot(V.cross(fr.d, fr.m), fr.N)) || 1;
  const phi = (op.kind === 'valley' ? 1 : -1) * sgn * (op.angleDeg * Math.PI) / 180;
  return rotationAbout(fr.p0, fr.d, phi);
}

/**
 * A crushed edge: the paper ahead of the impact is pushed in by `depth` and buckles into an
 * accordion of alternating folds over a zone a few times that long; the edge itself stays
 * pointing the way it did, `depth` closer. Folds go through all layers
 * there if possible, else through the hit layer. Returns the folds of the first variant that
 * the paper allows, applied in order from the innermost fold outwards.
 */
function crumpleOps(model: PaperModel, e: Extract<DamageEvent, { kind: 'crumple' }>, up: Vec3, cfg: PaperConfig): { ops: Op[]; model: PaperModel } | null {
  // earlier damage (tears) may have split the facet: then take the paper at the impact point
  const f = model.facets.find((x) => x.id === e.facet) ?? model.facets.find((x) => facetContains(x, e.point, -1e-6, 1e-4));
  if (!f) return null;
  let N = facetNormal(f);
  if (V.dot(N, up) < 0) N = V.negate(N);
  const inPlane = V.rejectFrom(e.dir, N);
  // a flat slap onto the floor does not crumple an edge
  if (V.length(inPlane) < 0.3) return null;
  const dImpact = V.normalize(inPlane);
  const excess = e.speed - cfg.crumpleSpeed;
  const fullDepth = Math.min(cfg.crumpleMaxDepth, 0.004 + cfg.crumpleDepthPerMs * excess);
  const k = Math.max(1, Math.round(cfg.crumpleFolds));
  // k folds: the first and last turn by α, the ones between by 2α, so the k − 1 pieces between
  // them zigzag at ±α and the tip continues straight on, (k − 1)/k·zone·(1 − cos α) = depth shorter
  const cosA = k > 1 ? 1 - k / ((k - 1) * cfg.crumpleZoneFactor) : 1 - 1 / cfg.crumpleZoneFactor;
  const alpha = (Math.acos(Math.max(-1, cosA)) * 180) / Math.PI;
  const id = nextId('crumple');
  // straight across the edge first; a blunt edge that cannot fold across (the fold would run
  // into other paper) crushes at a corner instead (diagonal folds)
  for (const [turnDeg, scope, first, scale] of variants()) {
    {
      // where folded paper around the edge stiffens it, the crush stays shallower
      const depth = fullDepth * scale;
      const zone = cfg.crumpleZoneFactor * depth;
      const din = Q.rotate(Q.fromAxisAngle(N, (turnDeg * Math.PI) / 180), dImpact);
      const along = V.normalize(V.cross(N, din));
      let cur = model;
      let moved = rigidIdentity();
      const ops: Op[] = [];
      let ok = true;
      for (let i = 0; i < k && ok; i++) {
        // fold i sits (k − i)/k of the zone in from the edge, on the paper as the earlier folds left it
        const s = (zone * (k - i)) / k;
        const p = applyRigid(moved, V.addScaled(e.point, din, -s));
        const kind: FoldOp['kind'] = (i % 2 === 0) === (first === 'valley') ? 'valley' : 'mountain';
        const op: FoldOp = {
          type: 'fold',
          id: k === 1 ? id : `${id}${String.fromCharCode(97 + i)}`,
          line: { p0: V.addScaled(p, applyRigidLinear(moved, along), -0.3), p1: V.addScaled(p, applyRigidLinear(moved, along), 0.3) },
          planeNormal: applyRigidLinear(moved, N),
          movingSide: applyRigidLinear(moved, din),
          kind,
          angleDeg: Math.round((i === 0 || i === k - 1 ? alpha : 2 * alpha) * 10) / 10,
          scope,
          pick: applyRigid(moved, V.addScaled(e.point, din, -s / 2)),
        };
        const r = tryOps(cur, [op]);
        if (!r) {
          ok = false;
          break;
        }
        ops.push(op);
        cur = r.model;
        moved = composeRigid(foldMotion(op), moved);
      }
      if (ok) return { ops, model: cur };
    }
  }
  return null;
}

function variants(): [number, 'all' | 'flap', FoldOp['kind'], number][] {
  const out: [number, 'all' | 'flap', FoldOp['kind'], number][] = [];
  for (const scale of [1, 0.6, 0.35])
    for (const turn of [0, 45, -45])
      for (const scope of ['all', 'flap'] as const) for (const first of ['valley', 'mountain'] as const) out.push([turn, scope, first, scale]);
  return out;
}

function bendOps(model: PaperModel, e: Extract<DamageEvent, { kind: 'bend' }>): Op[] {
  const parent = model.facets.find((x) => x.id === e.parentFacet);
  const child = model.facets.find((x) => x.id === e.childFacet);
  if (!parent || !child) return [];
  const N = facetNormal(parent);
  const d = V.normalize(e.axis);
  // the moving side in the parent's plane is away from the parent's own paper
  let S = V.rejectFrom(V.rejectFrom(V.sub(facetCentroid(parent), e.point), d), N);
  if (V.length(S) < 1e-6) return [];
  S = V.negate(V.normalize(S));
  const variants: Op[] = [];
  for (const side of [S, V.negate(S)]) {
    const m = side;
    const sgn = Math.sign(V.dot(V.cross(d, m), N)) || 1;
    const kind: FoldOp['kind'] = Math.sign(e.deltaDeg) === sgn ? 'valley' : 'mountain';
    variants.push({
      type: 'fold',
      id: nextId('bend'),
      line: { p0: e.point, p1: V.add(e.point, V.scale(d, 0.1)) },
      planeNormal: N,
      movingSide: side,
      kind,
      angleDeg: Math.min(180, Math.abs(e.deltaDeg)),
      scope: 'flap',
      pick: facetCentroid(child),
    });
  }
  return variants;
}

function tearOps(model: PaperModel, e: Extract<DamageEvent, { kind: 'tear' }>): Op[] {
  const f = model.facets.find((x) => x.id === e.facet) ?? model.facets.find((x) => P.pointInConvex(e.at, x.poly, -1e-9));
  if (!f) return [];
  const from = V2.add(e.at, V2.scale(e.dir, -0.0005));
  const to = V2.add(e.at, V2.scale(e.dir, e.length));
  const op: CutOp = {
    type: 'cut',
    id: nextId('tear'),
    path: [toModel(f, from), toModel(f, to)],
    planeNormal: facetNormal(f),
    depth: 'all',
  };
  return [op];
}

/**
 * Express damage recorded on the flown plane (relaxed by springback, maybe split at slit tabs)
 * on the plane exactly as folded, where the design's operations live. Points and directions
 * move with the paper they lie on (matched through sheet coordinates).
 */
export function damageOnFolded(flown: PaperModel, folded: PaperModel, events: DamageEvent[]): DamageEvent[] {
  if (flown === folded) return events;
  const flownById = new Map(flown.facets.map((f) => [f.id, f]));
  const owner = (sheet: [number, number]) => folded.facets.find((g) => P.pointInConvex(sheet, g.poly, -1e-9));
  const sheetOf = (id: number) => {
    const f = flownById.get(id);
    return f ? P.centroid(f.poly) : null;
  };
  const mapPoint = (id: number, p: Vec3): { p: Vec3; R: (v: Vec3) => Vec3; id: number } | null => {
    const f = flownById.get(id);
    if (!f) return null;
    const s3 = V.sub(p, f.T.t);
    const local = Q.rotate(Q.conjugate(f.T.q), s3);
    const g = owner([local[0], local[2]]) ?? owner(P.centroid(f.poly));
    if (!g) return null;
    const R = (v: Vec3) => Q.rotate(g.T.q, Q.rotate(Q.conjugate(f.T.q), v));
    return { p: V.add(Q.rotate(g.T.q, [local[0], 0, local[2]]), g.T.t), R, id: g.id };
  };
  const facetIdOf = (id: number) => {
    const c = sheetOf(id);
    return c ? owner(c)?.id ?? null : null;
  };
  const out: DamageEvent[] = [];
  for (const e of events) {
    if (e.kind === 'crumple') {
      const m = mapPoint(e.facet, e.point);
      if (m) out.push({ ...e, facet: m.id, point: m.p, dir: m.R(e.dir) });
    } else if (e.kind === 'bend') {
      const m = mapPoint(e.parentFacet, e.point);
      const parent = facetIdOf(e.parentFacet);
      const child = facetIdOf(e.childFacet);
      if (m && parent !== null && child !== null) out.push({ ...e, point: m.p, axis: m.R(e.axis), parentFacet: parent, childFacet: child });
    } else {
      const g = owner(e.at);
      if (g) out.push({ ...e, facet: g.id });
    }
  }
  return out;
}

/** Turn the damage of one flight into operations on `model` (the plane as it flew). */
export function damageToOps(model: PaperModel, events: DamageEvent[], up: Vec3, cfg: PaperConfig): DamageResult {
  let cur = model;
  const ops: Op[] = [];
  const all = [...events];
  // a hard impact next to a cut end tears it too
  const ends = interiorCutEnds(model);
  for (const e of events) {
    if (e.kind !== 'crumple') continue;
    for (const c of ends) {
      if (V.dist(c.model, e.point) < cfg.tearImpactRadius) all.push({ kind: 'tear', facet: c.facet, at: c.at, dir: c.dir, length: cfg.tearMaxLength / 2 });
    }
  }
  // crumples first (the crushed edge is what the impact did first), then bends, then tears;
  // events are pinned to the paper, so they land right after the others have moved paper
  const order = { crumple: 0, bend: 1, tear: 2 } as const;
  all.sort((a, b) => order[a.kind] - order[b.kind]);
  // crumples are pinned to the paper (sheet point + direction in the facet's frame), so they
  // still land in the right place after the bends and tears have moved paper
  const pinned = new Map<DamageEvent, { sheet: [number, number]; dirLocal: Vec3 }>();
  for (const e of all) {
    if (e.kind !== 'crumple') continue;
    const f = model.facets.find((x) => x.id === e.facet);
    if (!f) continue;
    const local = Q.rotate(Q.conjugate(f.T.q), V.sub(e.point, f.T.t));
    pinned.set(e, { sheet: [local[0], local[2]], dirLocal: Q.rotate(Q.conjugate(f.T.q), e.dir) });
  }
  for (const e0 of all) {
    if (e0.kind === 'crumple') {
      const pin = pinned.get(e0);
      const g = pin ? cur.facets.find((x) => P.pointInConvex(pin.sheet, x.poly, -1e-9)) : undefined;
      const e = pin && g ? { ...e0, facet: g.id, point: toModel(g, pin.sheet), dir: Q.rotate(g.T.q, pin.dirLocal) } : e0;
      const c = crumpleOps(cur, e, up, cfg);
      if (!c) continue;
      ops.push(...c.ops);
      cur = c.model;
      continue;
    }
    let e = e0;
    if (e.kind === 'bend') {
      // the crease's two sides, found again by their paper
      const at = (id: number) => {
        const f = model.facets.find((x) => x.id === id);
        const c = f ? P.centroid(f.poly) : null;
        return c ? cur.facets.find((x) => P.pointInConvex(c, x.poly, -1e-9))?.id : undefined;
      };
      const parentFacet = at(e.parentFacet);
      const childFacet = at(e.childFacet);
      if (parentFacet === undefined || childFacet === undefined) continue;
      e = { ...e, parentFacet, childFacet };
    }
    const r = tryOps(cur, e.kind === 'bend' ? bendOps(cur, e) : tearOps(cur, e));
    if (!r) continue;
    ops.push(r.op);
    cur = r.model;
  }
  return { ops, model: cur };
}
