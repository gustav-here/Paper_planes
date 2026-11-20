import * as V2 from '../math/vec2';
import * as P from '../math/polygon2d';
import { sameRigid } from '../math/transform';
import type { Vec2 } from '../math/vec2';
import type { Facet, PaperModel } from '../paper/model';
import { computeEdges, connectedComponents } from '../paper/edges';
import { interiorCutEnds } from './panels';

/*
 * Slit tabs: two cuts whose ends face each other across flat, uncreased paper leave a tab that
 * hangs on the narrow strip between the cut ends. In flexible paper that strip bends like a
 * hinge (with the bending stiffness of flat paper), and the cut ends at it can tear further.
 * Found purely from the geometry of the cuts; nothing knows what the tab is for.
 */

/** A hinge line between two cut ends, in sheet coordinates. */
export interface SlitTab {
  a: Vec2;
  b: Vec2;
}

const MIN_LEN = 0.002;
const MAX_LEN = 0.15;
const TOL = 1e-7;

/** Facets crossed by segment ab (sheet space) with the covered parameter interval. */
function crossings(model: PaperModel, a: Vec2, b: Vec2): { f: Facet; t0: number; t1: number }[] {
  const out: { f: Facet; t0: number; t1: number }[] = [];
  const L = V2.dist(a, b);
  for (const f of model.facets) {
    const r = P.clipSegmentConvex(a, b, f.poly);
    if (!r || (r[1] - r[0]) * L < 1e-9) continue;
    out.push({ f, t0: r[0], t1: r[1] });
  }
  return out;
}

/** Whole segment lies on paper that is flat across it (one rigid transform, no crease). */
function onFlatPaper(model: PaperModel, a: Vec2, b: Vec2): boolean {
  const cr = crossings(model, a, b);
  if (!cr.length) return false;
  if (!cr.every((c) => sameRigid(c.f.T, cr[0].f.T))) return false;
  const iv = cr.map((c) => [c.t0, c.t1] as [number, number]).sort((x, y) => x[0] - y[0]);
  let reach = 0;
  for (const [lo, hi] of iv) {
    if (lo > reach + 1e-6) return false;
    reach = Math.max(reach, hi);
  }
  return reach >= 1 - 1e-6;
}

function properlyCrosses(a: Vec2, b: Vec2, c: Vec2, d: Vec2): boolean {
  const o = (p: Vec2, q: Vec2, r: Vec2) => (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
  const d1 = o(a, b, c);
  const d2 = o(a, b, d);
  const d3 = o(c, d, a);
  const d4 = o(c, d, b);
  return ((d1 > 1e-12 && d2 < -1e-12) || (d1 < -1e-12 && d2 > 1e-12)) && ((d3 > 1e-12 && d4 < -1e-12) || (d3 < -1e-12 && d4 > 1e-12));
}

/** Is a link segment (sheet space) part of the tab hinge? */
export function onTab(seg: [Vec2, Vec2], tab: SlitTab): boolean {
  if (P.distToLine(seg[0], tab.a, tab.b) > TOL || P.distToLine(seg[1], tab.a, tab.b) > TOL) return false;
  const t = P.closestPointOnSegment(V2.lerp(seg[0], seg[1], 0.5), tab.a, tab.b).t;
  return t > 1e-6 && t < 1 - 1e-6;
}

/** Split facets along the tab lines so the hinge strip is a facet edge. */
export function splitAtTabs(model: PaperModel, tabs: SlitTab[]): PaperModel {
  let facets = model.facets;
  let nextId = model.nextFacetId;
  for (const tab of tabs) {
    const out: Facet[] = [];
    for (const f of facets) {
      const r = P.clipSegmentConvex(tab.a, tab.b, f.poly);
      if (r && (r[1] - r[0]) * V2.dist(tab.a, tab.b) > 1e-9) {
        const { neg, pos } = P.splitConvexByValues(f.poly, P.lineValues(f.poly, tab.a, tab.b), 1e-9);
        if (neg && pos) {
          out.push({ ...f, id: nextId++, poly: neg }, { ...f, id: nextId++, poly: pos });
          continue;
        }
      }
      out.push(f);
    }
    facets = out;
  }
  return { ...model, facets, nextFacetId: nextId };
}

/**
 * Drop tab lines that do not free anything: with all candidate lines cut at once, a line is a
 * hinge only if the paper on its two sides ends up in different pieces. Stacked layers (a keel
 * cut through both halves) count together, since each layer alone stays attached via the other.
 */
function separating(model: PaperModel, tabs: SlitTab[]): SlitTab[] {
  let cur = tabs;
  for (let iter = 0; iter < 20 && cur.length; iter++) {
    const m = splitAtTabs(model, cur);
    const g = computeEdges(m);
    const links = g.links.filter((l) => !cur.some((t) => onTab(l.seg, t)));
    const comp = new Map<number, number>();
    connectedComponents(m, { ...g, links }).forEach((c, i) => c.forEach((f) => comp.set(f.id, i)));
    const keep = cur.filter((t) => {
      const on = g.links.filter((l) => onTab(l.seg, t));
      return on.length > 0 && on.every((l) => comp.get(l.a) !== comp.get(l.b));
    });
    if (keep.length === cur.length) return cur;
    cur = keep;
  }
  return cur;
}

/** Tab hinges of a paper model (sheet coordinates), shortest first. */
export function slitTabs(model: PaperModel): SlitTab[] {
  if (model.cuts.length < 2) return [];
  const ends = interiorCutEnds(model);
  const cands: SlitTab[] = [];
  for (let i = 0; i < ends.length; i++) {
    for (let j = i + 1; j < ends.length; j++) {
      const a = ends[i].at;
      const b = ends[j].at;
      const L = V2.dist(a, b);
      if (L < MIN_LEN || L > MAX_LEN) continue;
      if (model.cuts.some((c) => properlyCrosses(a, b, c.a, c.b))) continue;
      // no other cut end on the way (then the shorter tabs are the real ones)
      if (ends.some((e, k) => k !== i && k !== j && V2.dist(e.at, a) > 1e-6 && V2.dist(e.at, b) > 1e-6 && P.distToLine(e.at, a, b) < 5e-4 && P.closestPointOnSegment(e.at, a, b).t > 0 && P.closestPointOnSegment(e.at, a, b).t < 1)) continue;
      if (!onFlatPaper(model, a, b)) continue;
      cands.push({ a, b });
    }
  }
  cands.sort((x, y) => V2.dist(x.a, x.b) - V2.dist(y.a, y.b));
  const tabs: SlitTab[] = [];
  for (const c of cands) {
    if (tabs.some((t) => properlyCrosses(c.a, c.b, t.a, t.b))) continue;
    tabs.push(c);
  }
  return separating(model, tabs);
}
