import * as V2 from '../math/vec2';
import * as P from '../math/polygon2d';
import * as I from '../math/interval';
import { sameRigid } from '../math/transform';
import type { Vec2 } from '../math/vec2';
import { EPS } from './model';
import type { Facet, PaperModel } from './model';

/**
 * Edge kinds (derived from geometry, never stored):
 * - crease: paper connected, facets have different transforms (a fold)
 * - seam:   paper connected, same transform (facet subdivided but flat there)
 * - cut:    facets touch but the paper is cut — not connected
 */
export type EdgeKind = 'crease' | 'seam' | 'cut';

export interface Link {
  a: number;
  b: number;
  kind: EdgeKind;
  /** Shared segment in sheet coordinates. */
  seg: [Vec2, Vec2];
}

/** Free edge of a facet: paper outline or the lip of a cut. */
export interface Border {
  facet: number;
  seg: [Vec2, Vec2];
  isCut: boolean;
}

export interface EdgeGraph {
  links: Link[];
  borders: Border[];
}

const TOL = 1e-8;

function overlapBoxes(a: number[], b: number[]): boolean {
  return a[0] <= b[2] + TOL && b[0] <= a[2] + TOL && a[1] <= b[3] + TOL && b[1] <= a[3] + TOL;
}

function collinear(p: Vec2, q: Vec2, a: Vec2, b: Vec2): boolean {
  return P.distToLine(a, p, q) < TOL && P.distToLine(b, p, q) < TOL;
}

/** Parameter intervals (along p→q, in metres) of cuts collinear with that edge. */
function cutIntervals(p: Vec2, q: Vec2, cuts: PaperModel['cuts']): I.Interval[] {
  const u = V2.normalize(V2.sub(q, p));
  const out: I.Interval[] = [];
  for (const c of cuts) {
    if (!collinear(p, q, c.a, c.b)) continue;
    out.push([V2.dot(V2.sub(c.a, p), u), V2.dot(V2.sub(c.b, p), u)]);
  }
  return out;
}

export function computeEdges(model: PaperModel): EdgeGraph {
  const facets = model.facets;
  const boxes = facets.map((f) => P.bbox(f.poly));
  const links: Link[] = [];

  for (let i = 0; i < facets.length; i++) {
    const fi = facets[i];
    const ei = P.edges(fi.poly);
    for (let j = i + 1; j < facets.length; j++) {
      if (!overlapBoxes(boxes[i], boxes[j])) continue;
      const fj = facets[j];
      const ej = P.edges(fj.poly);
      const same = sameRigid(fi.T, fj.T);
      for (const [p, q] of ei) {
        const L = V2.dist(p, q);
        if (L < EPS) continue;
        const u = V2.scale(V2.sub(q, p), 1 / L);
        for (const [a, b] of ej) {
          if (!collinear(p, q, a, b)) continue;
          const ta = V2.dot(V2.sub(a, p), u);
          const tb = V2.dot(V2.sub(b, p), u);
          const lo = Math.max(0, Math.min(ta, tb));
          const hi = Math.min(L, Math.max(ta, tb));
          if (hi - lo <= TOL) continue;
          const at = (t: number): Vec2 => V2.add(p, V2.scale(u, t));
          const cuts = cutIntervals(p, q, model.cuts);
          for (const [c0, c1] of I.intersect([lo, hi], cuts, TOL)) {
            links.push({ a: fi.id, b: fj.id, kind: 'cut', seg: [at(c0), at(c1)] });
          }
          for (const [s0, s1] of I.subtract([lo, hi], cuts, TOL)) {
            links.push({ a: fi.id, b: fj.id, kind: same ? 'seam' : 'crease', seg: [at(s0), at(s1)] });
          }
        }
      }
    }
  }

  // Borders: facet edges minus connected shared segments.
  const byFacet = new Map<number, Link[]>();
  for (const l of links) {
    if (l.kind === 'cut') continue;
    (byFacet.get(l.a) ?? byFacet.set(l.a, []).get(l.a)!).push(l);
    (byFacet.get(l.b) ?? byFacet.set(l.b, []).get(l.b)!).push(l);
  }
  const borders: Border[] = [];
  for (const f of facets) {
    const mine = byFacet.get(f.id) ?? [];
    for (const [p, q] of P.edges(f.poly)) {
      const L = V2.dist(p, q);
      if (L < EPS) continue;
      const u = V2.scale(V2.sub(q, p), 1 / L);
      const at = (t: number): Vec2 => V2.add(p, V2.scale(u, t));
      const shared: I.Interval[] = [];
      for (const l of mine) {
        if (!collinear(p, q, l.seg[0], l.seg[1])) continue;
        shared.push([V2.dot(V2.sub(l.seg[0], p), u), V2.dot(V2.sub(l.seg[1], p), u)]);
      }
      const cuts = cutIntervals(p, q, model.cuts);
      for (const piece of I.subtract([0, L], shared, TOL)) {
        for (const [c0, c1] of I.intersect(piece, cuts, TOL)) {
          borders.push({ facet: f.id, seg: [at(c0), at(c1)], isCut: true });
        }
        for (const [s0, s1] of I.subtract(piece, cuts, TOL)) {
          borders.push({ facet: f.id, seg: [at(s0), at(s1)], isCut: false });
        }
      }
    }
  }
  return { links, borders };
}

export const isConnected = (l: Link): boolean => l.kind !== 'cut';

/** Connected components of paper (over crease/seam links, never over cuts). */
export function connectedComponents(model: PaperModel, graph = computeEdges(model)): Facet[][] {
  const parent = new Map<number, number>();
  const find = (x: number): number => {
    let r = x;
    while (parent.get(r) !== r) r = parent.get(r)!;
    let c = x;
    while (parent.get(c) !== r) {
      const n = parent.get(c)!;
      parent.set(c, r);
      c = n;
    }
    return r;
  };
  for (const f of model.facets) parent.set(f.id, f.id);
  for (const l of graph.links) {
    if (!isConnected(l)) continue;
    const ra = find(l.a);
    const rb = find(l.b);
    if (ra !== rb) parent.set(ra, rb);
  }
  const groups = new Map<number, Facet[]>();
  for (const f of model.facets) {
    const r = find(f.id);
    (groups.get(r) ?? groups.set(r, []).get(r)!).push(f);
  }
  return [...groups.values()];
}
