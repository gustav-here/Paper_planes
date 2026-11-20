import * as V2 from '../math/vec2';
import * as V from '../math/vec3';
import * as P from '../math/polygon2d';
import type { Vec2 } from '../math/vec2';
import type { Vec3 } from '../math/vec3';
import { facetArea, facetInPlane, heightAlong, toSheet } from './model';
import type { Facet, PaperModel } from './model';
import { connectedComponents } from './edges';
import { PaperError } from './errors';
import type { PaperWarning } from './errors';
import type { CutOp } from './ops';
import { clipStillAttached } from './clips';

export interface CutPiece {
  /** A sheet point inside the piece (use in CutOp.discard). */
  flat: Vec2;
  area: number;
  kept: boolean;
}

export interface CutResult {
  model: PaperModel;
  /** All pieces after the cut (only reported when the cut detached something). */
  pieces: CutPiece[];
  warnings: PaperWarning[];
}

interface Hit {
  f: Facet;
  t0: number;
  t1: number;
  a: Vec2;
  b: Vec2;
  la: Vec2;
  lb: Vec2;
}

function cutSegment(model: PaperModel, A: Vec3, B: Vec3, N: Vec3, depth: CutOp['depth'], opId: string) {
  const hits: Hit[] = [];
  for (const f of model.facets) {
    if (!facetInPlane(f, A, N)) continue;
    const la = toSheet(f, A);
    const lb = toSheet(f, B);
    const r = P.clipSegmentConvex(la, lb, f.poly);
    if (!r) continue;
    const [t0, t1] = r;
    if ((t1 - t0) * V2.dist(la, lb) < 1e-7) continue;
    hits.push({ f, t0, t1, a: V2.lerp(la, lb, t0), b: V2.lerp(la, lb, t1), la, lb });
  }
  let kept = hits;
  if (depth === 'top') {
    kept = hits.filter((h) => {
      const tm = (h.t0 + h.t1) / 2;
      return !hits.some((o) => o !== h && o.t0 < tm && o.t1 > tm && heightAlong(o.f, N) > heightAlong(h.f, N));
    });
  }
  const keptIds = new Set(kept.map((h) => h.f.id));
  let nextId = model.nextFacetId;
  const facets: Facet[] = [];
  const cuts = [...model.cuts];
  for (const f of model.facets) {
    const h = kept.find((k) => k.f.id === f.id);
    if (!h || !keptIds.has(f.id)) {
      facets.push(f);
      continue;
    }
    cuts.push({ a: h.a, b: h.b, opId });
    const vals = P.lineValues(f.poly, h.la, h.lb);
    const { neg, pos } = P.splitConvexByValues(f.poly, vals, 1e-9);
    if (neg && pos) {
      facets.push({ ...f, id: nextId++, poly: neg }, { ...f, id: nextId++, poly: pos });
    } else {
      facets.push(f); // cut runs along an existing edge
    }
  }
  return { model: { ...model, facets, cuts, nextFacetId: nextId }, count: kept.length };
}

/** Apply a single cut (no mirroring). */
export function applyCut(model: PaperModel, op: CutOp): CutResult {
  const N = V.normalize(op.planeNormal);
  let cur = model;
  let total = 0;
  for (let i = 0; i + 1 < op.path.length; i++) {
    const r = cutSegment(cur, op.path[i], op.path[i + 1], N, op.depth, op.id);
    cur = r.model;
    total += r.count;
  }
  if (total === 0) throw new PaperError('nothingToCut');

  const warnings: PaperWarning[] = [];
  const comps = connectedComponents(cur);
  if (comps.length <= 1) return { model: cur, pieces: [], warnings };

  const areaOf = (c: Facet[]) => c.reduce((s, f) => s + facetArea(f), 0);
  const containsFlat = (c: Facet[], p: Vec2) => c.some((f) => P.pointInConvex(p, f.poly, -1e-9));
  let keep: Facet[][];
  if (op.discard && op.discard.length) {
    keep = comps.filter((c) => !op.discard!.some((d) => containsFlat(c, d.flat)));
    if (keep.length === 0) keep = [comps.reduce((a, b) => (areaOf(a) >= areaOf(b) ? a : b))];
  } else {
    keep = [comps.reduce((a, b) => (areaOf(a) >= areaOf(b) ? a : b))];
  }
  const keepIds = new Set(keep.flat().map((f) => f.id));
  const pieces: CutPiece[] = comps.map((c) => {
    const big = c.reduce((a, b) => (facetArea(a) >= facetArea(b) ? a : b));
    return { flat: P.centroid(big.poly), area: areaOf(c), kept: keepIds.has(big.id) };
  });
  const remaining = cur.facets.filter((f) => keepIds.has(f.id));
  let next: PaperModel = { ...cur, facets: remaining };
  const clips = next.clips.filter((c) => clipStillAttached(next, c));
  if (clips.length !== next.clips.length) warnings.push({ code: 'clipRemoved' });
  next = { ...next, clips };
  warnings.push({ code: 'piecesDiscarded', detail: String(comps.length - keep.length) });
  return { model: next, pieces, warnings };
}
