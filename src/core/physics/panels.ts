import * as V from '../math/vec3';
import * as P from '../math/polygon2d';
import type { Vec2 } from '../math/vec2';
import type { Vec3 } from '../math/vec3';
import type { Mat3 } from '../math/mat3';
import { coplanar, facetCentroid, facetNormal, facetVertices, facetsOverlap, toModel } from '../paper/model';
import type { Facet, PaperModel } from '../paper/model';
import { computeEdges, isConnected } from '../paper/edges';
import { resolveClip } from '../paper/clips';
import { computeMassProperties } from './massProperties';
import type { Airframe } from './airframe';
import type { PaperConfig } from './config';
import { onTab, slitTabs } from './slits';
import { isBendingPanel } from './bendModes';

/**
 * Flexible paper as hinged rigid panels. A panel is paper that cannot bend in this model:
 * facets joined by flat seams or pressed-flat (180°) creases, and layers squeezed by a clip.
 * Creases at other angles are hinges with the rotational stiffness of a paper crease
 * (proportional to the total crease length over all layers). Control flaps hinge on their fold
 * and are driven by a stiff servo spring. The strip between two facing cut ends (a slit tab, see
 * slits.ts) is a hinge with the bending stiffness of flat paper.
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
  /** A fold crease, or flat paper bending between two cut ends (no plastic yield there). */
  kind: 'crease' | 'slit';
  /**
   * Rotational stiffness (N·m/rad) and damping (N·m·s/rad). For a crease this is the crease in
   * series with the bending of the flat paper beyond it (see plateStiffness).
   */
  stiffness: number;
  /** The crease alone (plastic yield happens in the crease, not in the elastic paper). */
  creaseStiffness: number;
  /**
   * Hinge angle (rad, relative to the start pose) at which the two sides lie flat on each other,
   * and the direction (±1) of closing. Folding further would push paper through paper.
   */
  closeAt: number;
  closeDir: number;
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
  /** The root panel (the body: the middle of the panel graph). */
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

/**
 * Hinge links this close to one straight line rotate together (m). Layers of a stack that
 * springback left a few degrees open meet their hinge a millimetre or two apart; they still bend
 * as one hinge (about the crease zone width).
 */
const LINE_TOL = 3e-3;

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
  const tabs = slitTabs(model);
  const slitLinks = new Set<(typeof links)[number]>();
  for (const l of links) {
    const fa = byId.get(l.a)!;
    const fb = byId.get(l.b)!;
    const sameControl = (control.get(l.a) ?? -1) === (control.get(l.b) ?? -1);
    if (tabs.some((t) => onTab(l.seg, t))) {
      slitLinks.add(l);
      hingeLinks.push(l);
    } else if (sameControl && (coplanar(fa, fb) || flatContinuation(fa, fb, l.seg) || unfolded(fa, fb) || crumpled(fa, fb))) uf.union(l.a, l.b);
    else hingeLinks.push(l);
  }
  // layers lying flat on each other move together (pressed stacks)
  for (let i = 0; i < model.facets.length; i++) {
    for (let j = i + 1; j < model.facets.length; j++) {
      const fa = model.facets[i];
      const fb = model.facets[j];
      if ((control.get(fa.id) ?? -1) !== (control.get(fb.id) ?? -1)) continue;
      if ((coplanar(fa, fb) || nearlyStacked(fa, fb)) && facetsOverlap(fa, fb)) uf.union(fa.id, fb.id);
    }
  }
  // clips squeeze layers together. Tape across a seam squeezes the layers on each side, but does
  // not make the two sides one rigid piece: a 2 cm strip at the nose holds the halves together
  // there, it does not stop each wing from flexing at its root and bending along its length (as
  // one panel, both wings lost their hinges and their bending shapes, and a taped glider flew
  // 6 m shorter in a hard throw)
  for (const clip of model.clips) {
    const rc = resolveClip(model, clip);
    if (!rc) continue;
    const across = new Set(rc.across?.clamped ?? []);
    for (const side of [rc.clamped.filter((id) => !across.has(id)), rc.clamped.filter((id) => across.has(id))])
      for (let i = 1; i < side.length; i++) uf.union(side[0], side[i]);
  }

  // hinges must be straight lines between two panels, and the panel graph must be a tree
  for (let iter = 0; iter < 50; iter++) {
    const pairs = new Map<string, Vec3[]>();
    const stackPair = new Map<string, boolean>();
    for (const l of hingeLinks) {
      const ra = uf.find(l.a);
      const rb = uf.find(l.b);
      if (ra === rb) continue;
      const key = ra < rb ? `${ra}:${rb}` : `${rb}:${ra}`;
      const f = byId.get(l.a)!;
      (pairs.get(key) ?? pairs.set(key, []).get(key)!).push(toModel(f, l.seg[0]), toModel(f, l.seg[1]));
      // a stack: two sides of a fold lying almost on each other (not flat paper, e.g. a slit tab)
      if (l.kind === 'crease') stackPair.set(key, stackPair.get(key) || Math.abs(V.dot(facetNormal(f), facetNormal(byId.get(l.b)!))) > STACK_COS);
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
            const loop = [...pathU.slice(0, pathU.indexOf(common) + 1), ...pathW.slice(0, pathW.indexOf(common)).reverse()];
            // a loop through layers that springback left slightly open: press those layers
            // together first (they are held by the loop anyway); otherwise the loop is rigid
            const edges = loop.map((x, i) => [x, loop[(i + 1) % loop.length]] as const);
            const stack = edges.find(([x, y]) => stackPair.get(x < y ? `${x}:${y}` : `${y}:${x}`));
            if (stack) uf.union(stack[0], stack[1]);
            else for (const x of loop) uf.union(x, u);
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

  // hinges (pairs of panels), then orient the tree away from the root
  const pairPts = new Map<string, { pts: Vec3[]; length: number; slit: boolean; control: number; facets: Map<number, number> }>();
  for (const l of hingeLinks) {
    const pa = panelOf.get(l.a)!;
    const pb = panelOf.get(l.b)!;
    if (pa === pb) continue;
    const key = pa < pb ? `${pa}:${pb}` : `${pb}:${pa}`;
    const f = byId.get(l.a)!;
    const e = pairPts.get(key) ?? { pts: [], length: 0, slit: true, control: -1, facets: new Map<number, number>() };
    if (!slitLinks.has(l)) e.slit = false;
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
  // The root is the body: the panel in the middle of the panel graph (smallest number of hinges
  // to the farthest panel; ties: the heavier), so loads run from wings and flaps inwards.
  const ecc = props.map((_, start) => {
    const dist = new Map([[start, 0]]);
    const q = [start];
    while (q.length) {
      const x = q.shift()!;
      for (const { other } of adj.get(x) ?? []) {
        if (dist.has(other)) continue;
        dist.set(other, dist.get(x)! + 1);
        q.push(other);
      }
    }
    return Math.max(...dist.values());
  });
  const rootPanel = props.reduce((best, p, i) => (ecc[i] < ecc[best] || (ecc[i] === ecc[best] && p.mass > props[best].mass) ? i : best), 0);
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
      const slit = e.slit && !isControl;
      const kCrease = paper.creaseStiffnessPerM * e.length * (isControl ? paper.servoStiffnessFactor : 1);
      let k = kCrease;
      if (!slit && !isControl && !(paper.bendingModes && isBendingPanel(panelFacets[other], false))) {
        // the flat paper beyond the crease (away from the body) bends too, in series with it;
        // large flat panels get their own bending shapes instead (bendModes.ts)
        const kp = plateStiffness(panelFacets[other], a, axis, paper.bendingStiffness);
        k = 1 / (1 / kCrease + 1 / kp);
      }
      if (slit) {
        // cantilever plate: root rotation under a tip load ⇒ k = 3·D·w/ℓ, with ℓ the tab length
        // (bending spreads over the tab), but not longer than the strip is wide (a big piece on a
        // narrow strip bends in the strip) and not shorter than a crease zone
        const reach = Math.max(...panelFacets[other].flatMap((f) => facetVertices(f).map((p) => V.distToLine(p, a, axis))));
        const span = V.dist(a, b);
        const ell = Math.max(paper.creaseZoneM, Math.min(reach, span));
        k = (3 * paper.bendingStiffness * e.length) / ell;
      }
      const ends = cutEnds.filter((c) => V.distToLine(c.model, a, axis) < 3e-3 && segParam(c.model, a, b) > -0.003 && segParam(c.model, a, b) < V.dist(a, b) + 0.003);
      hinges.push({
        parent: u,
        child: other,
        point: a,
        axis,
        length: e.length,
        kind: slit ? 'slit' : 'crease',
        stiffness: k,
        creaseStiffness: slit ? k : kCrease,
        ...closing(byId.get(e.facets.get(u)!)!, byId.get(e.facets.get(other)!)!, a, axis),
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

/**
 * Two pieces with the same fold history were never folded against each other: the line between
 * them is no crease. They are one piece of paper split only to show an elastic twist (springback
 * between clips), however much it twists.
 */
function unfolded(fa: Facet, fb: Facet): boolean {
  return fa.foldPath.length === fb.foldPath.length && fa.foldPath.every((id, i) => fb.foldPath[i] === id);
}

/**
 * The folds a hard landing crushes into the paper (the accordion of a crumpled nose) are a tight
 * wad, not hinges: the paper on either side of such a fold is one stiff piece.
 */
function crumpled(fa: Facet, fb: Facet): boolean {
  const n = Math.min(fa.foldPath.length, fb.foldPath.length);
  let i = 0;
  while (i < n && fa.foldPath[i] === fb.foldPath[i]) i++;
  // the fold that separated them is the first one only one of them went through
  const op = fa.foldPath[i] ?? fb.foldPath[i];
  return !!op && op.startsWith('dmg-crumple');
}

const FLAT_COS = Math.cos((3 * Math.PI) / 180);

/**
 * Paper that carries on almost flat across a link (within 3°, e.g. a gently twisted strip) is
 * one panel; only real folds are hinges.
 */
function flatContinuation(fa: Facet, fb: Facet, seg: [Vec2, Vec2]): boolean {
  if (Math.abs(V.dot(facetNormal(fa), facetNormal(fb))) < FLAT_COS) return false;
  const A = toModel(fa, seg[0]);
  const d = V.normalize(V.sub(toModel(fa, seg[1]), A));
  const wa = V.rejectFrom(V.sub(facetCentroid(fa), A), d);
  const wb = V.rejectFrom(V.sub(facetCentroid(fb), A), d);
  return V.dot(wa, wb) < 0;
}

/** Layers this close to each other's plane (m) still lie on each other (see nearlyStacked). */
const STACK_GAP_M = 0.002;

/**
 * Layers that lie on each other but not exactly in one plane: within 3° and 2 mm. Springback
 * held by a clip or tape twists the paper a little (a degree or two), and the layers folded onto
 * a wing then no longer lay exactly on it: they became a panel of their own, hinged at the wing
 * root beside the wing (a taped glider's wing flapped as two thin wings).
 */
function nearlyStacked(fa: Facet, fb: Facet): boolean {
  const na = facetNormal(fa);
  if (Math.abs(V.dot(na, facetNormal(fb))) < FLAT_COS) return false;
  return Math.abs(V.dot(na, V.sub(facetCentroid(fb), facetVertices(fa)[0]))) < STACK_GAP_M;
}

/** Panels whose facets are this close to parallel at a crease are a (slightly open) stack. */
const STACK_COS = Math.cos((25 * Math.PI) / 180);

/** Unit vector from the line (p, d) towards the facet centroid, perpendicular to the line. */
function armOf(f: Facet, p: Vec3, d: Vec3): Vec3 {
  return V.normalize(V.rejectFrom(V.sub(facetCentroid(f), p), d));
}

/** Where the child would lie flat on the parent (fold angle 180°), as a hinge angle. */
function closing(parent: Facet, child: Facet, p: Vec3, d: Vec3): { closeAt: number; closeDir: number } {
  const v = armOf(parent, p, d);
  const w = armOf(child, p, d);
  const psi = Math.atan2(V.dot(d, V.cross(v, w)), V.dot(v, w));
  // rotating the child by +θ about d turns ψ by +θ; it closes when ψ reaches 0
  const closeAt = -psi;
  return { closeAt, closeDir: Math.sign(closeAt) || 1 };
}

/**
 * Equivalent hinge stiffness of the flat paper of a panel bending as a cantilever from the
 * hinge line: a load at the panel's load centre r_c deflects it by
 *   δ = P ∫₀^{r_c} (r_c − r)² / (D·w(r)) dr,
 * where w(r) is the paper width (all layers) at distance r from the hinge, so the hinge that
 * gives the same work has k = r_c² / ∫₀^{r_c} (r_c − r)² / (D·w(r)) dr.
 */
export function plateStiffness(facets: Facet[], p: Vec3, d: Vec3, D: number): number {
  const BIN = 0.002;
  const hist: number[] = [];
  let total = 0;
  let moment = 0;
  for (const f of facets) {
    const vs = facetVertices(f);
    for (let i = 1; i + 1 < vs.length; i++) {
      const [A, B, C] = [vs[0], vs[i], vs[i + 1]];
      const area = V.length(V.cross(V.sub(B, A), V.sub(C, A))) / 2;
      if (area < 1e-12) continue;
      const edge = Math.max(V.dist(A, B), V.dist(B, C), V.dist(C, A));
      const k = Math.min(60, Math.max(2, Math.ceil(edge / BIN)));
      const pts: Vec3[] = [];
      for (let a = 0; a < k; a++)
        for (let b = 0; a + b < k; b++) {
          const u = (a + 1 / 3) / k;
          const v = (b + 1 / 3) / k;
          pts.push(V.add(A, V.add(V.scale(V.sub(B, A), u), V.scale(V.sub(C, A), v))));
        }
      const wgt = area / pts.length;
      for (const q of pts) {
        const r = V.distToLine(q, p, d);
        const bin = Math.floor(r / BIN);
        hist[bin] = (hist[bin] ?? 0) + wgt;
        total += wgt;
        moment += wgt * r;
      }
    }
  }
  if (total <= 0) return Infinity;
  const rc = moment / total;
  let integral = 0;
  const wMin = 0.005;
  for (let bin = 0; bin * BIN < rc; bin++) {
    const r = (bin + 0.5) * BIN;
    const width = Math.max(wMin, (hist[bin] ?? 0) / BIN);
    const dr = Math.min(BIN, rc - bin * BIN);
    integral += ((rc - r) ** 2 / (D * width)) * dr;
  }
  return integral > 0 ? (rc * rc) / integral : Infinity;
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
