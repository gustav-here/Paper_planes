import * as V from '../math/vec3';
import * as P from '../math/polygon2d';
import type { Vec2 } from '../math/vec2';
import type { Vec3 } from '../math/vec3';
import type { Mat3 } from '../math/mat3';
import { coplanar, facetVertices, facetsOverlap, toModel } from '../paper/model';
import type { PaperModel } from '../paper/model';
import { computeEdges, isConnected } from '../paper/edges';
import { resolveClip } from '../paper/clips';
import { computeMassProperties } from './massProperties';
import type { Airframe } from './airframe';
import type { PaperConfig } from './config';

/**
 * Flexible paper as hinged rigid panels. A panel is paper that cannot bend in this model:
 * facets joined by flat seams or pressed-flat (180°) creases, and layers squeezed by a clip.
 * Creases at other angles are hinges with the rotational stiffness of a paper crease
 * (proportional to the total crease length over all layers). Control flaps hinge on their fold
 * and are driven by a stiff servo spring.
 */
export interface Panel {
  facets: number[];
  mass: number;
  /** Centre of mass (model space) and inertia about it (model axes). */
  com: Vec3;
  inertia: Mat3;
  /** Parent panel and the hinge joining to it (−1 for the root). */
  parent: number;
  hinge: number;
  /** Hinges from the root down to this panel, in order. */
  path: number[];
}

export interface Hinge {
  parent: number;
  child: number;
  /** A point on the hinge line and its unit axis (model space). */
  point: Vec3;
  axis: Vec3;
  /** Total crease length over all layers (m). */
  length: number;
  /** Rotational stiffness (N·m/rad) and damping (N·m·s/rad). */
  stiffness: number;
  damping: number;
  /** Index into Airframe.controls if this hinge is a control flap, else −1. */
  control: number;
  /** Interior cut ends lying on this hinge line (sheet coordinates + direction), for tearing. */
  cutEnds: { facet: number; at: Vec2; dir: Vec2 }[];
  /** A facet on each side of the hinge (parent side, child side). */
  parentFacet: number;
  childFacet: number;
}

export interface PanelSet {
  panels: Panel[];
  hinges: Hinge[];
  /** Facet id → panel index. */
  panelOf: Map<number, number>;
  /** The root panel (heaviest). */
  root: number;
}

class UnionFind {
  private p = new Map<number, number>();
  find(x: number): number {
    if (!this.p.has(x)) this.p.set(x, x);
    let r = x;
    while (this.p.get(r) !== r) r = this.p.get(r)!;
    let c = x;
    while (this.p.get(c) !== r) {
      const n = this.p.get(c)!;
      this.p.set(c, r);
      c = n;
    }
    return r;
  }
  union(a: number, b: number) {
    const ra = this.find(a);
    const rb = this.find(b);
    if (ra !== rb) this.p.set(ra, rb);
  }
}

const LINE_TOL = 5e-4;

/** Interior cut ends: cut endpoints that are not on the paper's outline. */
export function interiorCutEnds(model: PaperModel): { facet: number; at: Vec2; dir: Vec2; model: Vec3 }[] {
  const out: { facet: number; at: Vec2; dir: Vec2; model: Vec3 }[] = [];
  for (const c of model.cuts) {
    for (const [end, other] of [
      [c.b, c.a],
      [c.a, c.b],
    ] as [Vec2, Vec2][]) {
      // inside some facet (or on a connected seam between facets) = paper continues beyond the end
      const owners = model.facets.filter((f) => P.pointInConvex(end, f.poly, -1e-9));
      if (owners.length === 0) continue;
      const step: Vec2 = [end[0] + (end[0] - other[0]) * 1e-3, end[1] + (end[1] - other[1]) * 1e-3];
      const beyond = model.facets.find((f) => P.pointInConvex(step, f.poly, -1e-9));
      if (!beyond) continue;
      const covered = model.cuts.some(
        (d) => d !== c && P.distToLine(step, d.a, d.b) < 1e-7 && P.closestPointOnSegment(step, d.a, d.b).t > 0 && P.closestPointOnSegment(step, d.a, d.b).t < 1,
      );
      if (covered) continue;
      const len = Math.hypot(end[0] - other[0], end[1] - other[1]) || 1;
      out.push({
        facet: beyond.id,
        at: end,
        dir: [(end[0] - other[0]) / len, (end[1] - other[1]) / len],
        model: toModel(beyond, end),
      });
    }
  }
  return out;
}

export function buildPanels(model: PaperModel, af: Airframe, paper: PaperConfig): PanelSet {
  const control = new Map<number, number>();
  af.controls.forEach((g, i) => g.facets.forEach((f) => control.set(f, i)));
  const byId = new Map(model.facets.map((f) => [f.id, f]));
  const links = computeEdges(model).links.filter(isConnected);
  const uf = new UnionFind();
  for (const f of model.facets) uf.find(f.id);
  const hingeLinks: typeof links = [];
  for (const l of links) {
    const fa = byId.get(l.a)!;
    const fb = byId.get(l.b)!;
    const sameControl = (control.get(l.a) ?? -1) === (control.get(l.b) ?? -1);
    if (sameControl && coplanar(fa, fb)) uf.union(l.a, l.b);
    else hingeLinks.push(l);
  }
  // layers lying flat on each other move together (pressed stacks)
  for (let i = 0; i < model.facets.length; i++) {
    for (let j = i + 1; j < model.facets.length; j++) {
      const fa = model.facets[i];
      const fb = model.facets[j];
      if ((control.get(fa.id) ?? -1) !== (control.get(fb.id) ?? -1)) continue;
      if (coplanar(fa, fb) && facetsOverlap(fa, fb)) uf.union(fa.id, fb.id);
    }
  }
  // clips squeeze layers together
  for (const clip of model.clips) {
    const rc = resolveClip(model, clip);
    if (!rc) continue;
    for (let i = 1; i < rc.clamped.length; i++) uf.union(rc.clamped[0], rc.clamped[i]);
  }

  // hinges must be straight lines between two panels, and the panel graph must be a tree
  for (let iter = 0; iter < 50; iter++) {
    const pairs = new Map<string, Vec3[]>();
    for (const l of hingeLinks) {
      const ra = uf.find(l.a);
      const rb = uf.find(l.b);
      if (ra === rb) continue;
      const key = ra < rb ? `${ra}:${rb}` : `${rb}:${ra}`;
      const f = byId.get(l.a)!;
      (pairs.get(key) ?? pairs.set(key, []).get(key)!).push(toModel(f, l.seg[0]), toModel(f, l.seg[1]));
    }
    let changed = false;
    // not a straight line → it cannot rotate: same panel
    for (const [key, pts] of pairs) {
      const [a, b] = farthest(pts);
      const d = V.normalize(V.sub(b, a));
      if (pts.some((p) => V.distToLine(p, a, d) > LINE_TOL)) {
        const [ra, rb] = key.split(':').map(Number);
        uf.union(ra, rb);
        changed = true;
      }
    }
    if (changed) continue;
    // cycles → rigid loop: merge the panels on the cycle
    const nodes = [...new Set(model.facets.map((f) => uf.find(f.id)))];
    const adj = new Map<number, number[]>(nodes.map((n) => [n, []]));
    for (const key of pairs.keys()) {
      const [ra, rb] = key.split(':').map(Number);
      adj.get(ra)!.push(rb);
      adj.get(rb)!.push(ra);
    }
    const parent = new Map<number, number>();
    for (const start of nodes) {
      if (parent.has(start) || changed) continue;
      parent.set(start, -1);
      const queue = [start];
      while (queue.length && !changed) {
        const u = queue.shift()!;
        for (const w of adj.get(u)!) {
          if (w === parent.get(u)) continue;
          if (parent.has(w)) {
            // cycle u…w: merge the tree paths from both up to their common ancestor
            const pathU = ancestry(u, parent);
            const pathW = ancestry(w, parent);
            const common = pathU.find((x) => pathW.includes(x))!;
            for (const x of [...pathU.slice(0, pathU.indexOf(common) + 1), ...pathW.slice(0, pathW.indexOf(common) + 1)]) uf.union(x, u);
            changed = true;
            break;
          }
          parent.set(w, u);
          queue.push(w);
        }
      }
    }
    if (!changed) break;
  }

  // panels
  const roots = [...new Set(model.facets.map((f) => uf.find(f.id)))];
  const panelOf = new Map<number, number>();
  model.facets.forEach((f) => panelOf.set(f.id, roots.indexOf(uf.find(f.id))));
  const panelFacets = roots.map((r) => model.facets.filter((f) => uf.find(f.id) === r));
  const props = panelFacets.map((fs) => {
    const ids = new Set(fs.map((f) => f.id));
    const clips = model.clips.filter((c) => {
      const rc = resolveClip(model, c);
      return rc ? rc.clamped.some((id) => ids.has(id)) : false;
    });
    return computeMassProperties({ ...model, facets: fs, clips });
  });
  const rootPanel = props.reduce((best, p, i) => (p.mass > props[best].mass ? i : best), 0);

  // hinges (pairs of panels), then orient the tree away from the root
  const pairPts = new Map<string, { pts: Vec3[]; length: number; control: number; facets: Map<number, number> }>();
  for (const l of hingeLinks) {
    const pa = panelOf.get(l.a)!;
    const pb = panelOf.get(l.b)!;
    if (pa === pb) continue;
    const key = pa < pb ? `${pa}:${pb}` : `${pb}:${pa}`;
    const f = byId.get(l.a)!;
    const e = pairPts.get(key) ?? { pts: [], length: 0, control: -1, facets: new Map<number, number>() };
    if (!e.facets.has(pa)) e.facets.set(pa, l.a);
    if (!e.facets.has(pb)) e.facets.set(pb, l.b);
    e.pts.push(toModel(f, l.seg[0]), toModel(f, l.seg[1]));
    e.length += Math.hypot(l.seg[1][0] - l.seg[0][0], l.seg[1][1] - l.seg[0][1]);
    const ca = control.get(l.a) ?? -1;
    const cb = control.get(l.b) ?? -1;
    if (ca !== cb) e.control = Math.max(ca, cb);
    pairPts.set(key, e);
  }
  const adj = new Map<number, { other: number; key: string }[]>();
  for (const key of pairPts.keys()) {
    const [a, b] = key.split(':').map(Number);
    (adj.get(a) ?? adj.set(a, []).get(a)!).push({ other: b, key });
    (adj.get(b) ?? adj.set(b, []).get(b)!).push({ other: a, key });
  }
  const panels: Panel[] = props.map((p, i) => ({
    facets: panelFacets[i].map((f) => f.id),
    mass: p.mass,
    com: p.com,
    inertia: p.inertia,
    parent: -1,
    hinge: -1,
    path: [],
  }));
  const hinges: Hinge[] = [];
  const cutEnds = interiorCutEnds(model);
  const seen = new Set([rootPanel]);
  const queue = [rootPanel];
  while (queue.length) {
    const u = queue.shift()!;
    for (const { other, key } of adj.get(u) ?? []) {
      if (seen.has(other)) continue;
      seen.add(other);
      const e = pairPts.get(key)!;
      const [a, b] = farthest(e.pts);
      let axis = V.normalize(V.sub(b, a));
      if (e.control >= 0) {
        const g = af.controls[e.control];
        if (V.dot(axis, g.hingeA) < 0) axis = V.negate(axis);
      }
      const isControl = e.control >= 0 && af.controls[e.control].facets.includes(panels[other].facets[0]);
      const k = paper.creaseStiffnessPerM * e.length * (isControl ? paper.servoStiffnessFactor : 1);
      const ends = cutEnds.filter((c) => V.distToLine(c.model, a, axis) < 3e-3 && segParam(c.model, a, b) > -0.003 && segParam(c.model, a, b) < V.dist(a, b) + 0.003);
      hinges.push({
        parent: u,
        child: other,
        point: a,
        axis,
        length: e.length,
        stiffness: k,
        damping: k * paper.creaseDampingS,
        control: isControl ? e.control : -1,
        cutEnds: ends.map(({ facet, at, dir }) => ({ facet, at, dir })),
        parentFacet: e.facets.get(u)!,
        childFacet: e.facets.get(other)!,
      });
      panels[other].parent = u;
      panels[other].hinge = hinges.length - 1;
      panels[other].path = [...panels[u].path, hinges.length - 1];
      queue.push(other);
    }
  }
  return { panels, hinges, panelOf, root: rootPanel };
}

function segParam(p: Vec3, a: Vec3, b: Vec3): number {
  return V.dot(V.sub(p, a), V.normalize(V.sub(b, a)));
}

function farthest(pts: Vec3[]): [Vec3, Vec3] {
  let best: [Vec3, Vec3] = [pts[0], pts[1] ?? pts[0]];
  let bd = -1;
  for (let i = 0; i < pts.length; i++)
    for (let j = i + 1; j < pts.length; j++) {
      const d = V.dist(pts[i], pts[j]);
      if (d > bd) {
        bd = d;
        best = [pts[i], pts[j]];
      }
    }
  return best;
}

function ancestry(x: number, parent: Map<number, number>): number[] {
  const out = [x];
  let c = x;
  while (parent.get(c) !== undefined && parent.get(c) !== -1) {
    c = parent.get(c)!;
    out.push(c);
  }
  return out;
}

/** Facet vertices of each panel (unique), model space — contact points for the flexible model. */
export function panelContacts(model: PaperModel, set: PanelSet): { panel: number; facet: number; p: Vec3 }[] {
  const out: { panel: number; facet: number; p: Vec3 }[] = [];
  for (const f of model.facets) {
    const panel = set.panelOf.get(f.id)!;
    for (const v of facetVertices(f)) {
      if (out.some((c) => c.panel === panel && V.dist(c.p, v) < 1e-6)) continue;
      out.push({ panel, facet: f.id, p: v });
    }
  }
  return out;
}
