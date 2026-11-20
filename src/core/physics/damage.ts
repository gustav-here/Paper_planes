import * as V from '../math/vec3';
import * as V2 from '../math/vec2';
import * as P from '../math/polygon2d';
import type { Vec3 } from '../math/vec3';
import { facetCentroid, facetNormal, toModel } from '../paper/model';
import type { PaperModel } from '../paper/model';
import type { CutOp, FoldOp, Op } from '../paper/ops';
import { applyOp } from '../paper/replay';
import type { PaperConfig } from './config';
import { interiorCutEnds } from './panels';
import type { DamageEvent } from './simulator';

/*
 * Damage becomes ordinary design operations appended after the player's folds, so a damaged
 * plane is still just "folds and cuts": deterministic, replayable, and the same physics flies it.
 *   crumple → a small fold of the paper around the impact point (tip bent up/back)
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

function crumpleOps(model: PaperModel, e: Extract<DamageEvent, { kind: 'crumple' }>, up: Vec3, cfg: PaperConfig): Op[] {
  const f = model.facets.find((x) => x.id === e.facet);
  if (!f) return [];
  let N = facetNormal(f);
  if (V.dot(N, up) < 0) N = V.negate(N);
  const inPlane = V.rejectFrom(e.dir, N);
  // a flat slap onto the floor does not crumple an edge
  if (V.length(inPlane) < 0.3) return [];
  const din = V.normalize(inPlane);
  const excess = e.speed - cfg.crumpleSpeed;
  const depth = Math.min(cfg.crumpleMaxDepth, 0.004 + cfg.crumpleDepthPerMs * excess);
  const angle = Math.min(90, Math.max(10, cfg.crumpleDegPerMs * excess));
  const p = V.addScaled(e.point, din, -depth);
  const along = V.normalize(V.cross(N, din));
  const base: FoldOp = {
    type: 'fold',
    id: nextId('crumple'),
    line: { p0: V.addScaled(p, along, -0.3), p1: V.addScaled(p, along, 0.3) },
    planeNormal: N,
    movingSide: din,
    kind: 'valley',
    angleDeg: Math.round(angle),
    scope: 'flap',
    pick: V.addScaled(e.point, din, -depth / 2),
  };
  return [base, { ...base, kind: 'mountain' }, { ...base, scope: 'all' }, { ...base, scope: 'all', kind: 'mountain' }];
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
  // bends first (they refer to the undamaged creases), then tears, then crumples
  const order = { bend: 0, tear: 1, crumple: 2 } as const;
  all.sort((a, b) => order[a.kind] - order[b.kind]);
  for (const e of all) {
    const cands = e.kind === 'crumple' ? crumpleOps(cur, e, up, cfg) : e.kind === 'bend' ? bendOps(cur, e) : tearOps(cur, e);
    const r = tryOps(cur, cands);
    if (!r) continue;
    ops.push(r.op);
    cur = r.model;
  }
  return { ops, model: cur };
}
