import polygonClipping from 'polygon-clipping';
import type { MultiPolygon, Ring } from 'polygon-clipping';
import * as V2 from '../math/vec2';
import * as V from '../math/vec3';
import * as P from '../math/polygon2d';
import { applyRigidLinear } from '../math/transform';
import type { Vec2 } from '../math/vec2';
import type { Vec3 } from '../math/vec3';
import {
  canonicalNormal,
  coplanar,
  facetContains,
  facetNormal,
  facetVertices,
  planeBasis,
  projectToPlane,
  toModel,
} from './model';
import type { Clip, Facet, PaperModel } from './model';
import { computeEdges } from './edges';
import type { EdgeGraph } from './edges';

export interface HolderSpec {
  massKg: number;
  /** How far it reaches in from the edge (on each side of the stack). */
  lengthM: number;
  /** Width along the edge. */
  widthM: number;
}

/** Standard small paper clip (≈ 28 mm "No. 3"). Mass is set per clip op (default 1 g). */
export const CLIP_SPEC: HolderSpec = {
  massKg: 0.001,
  lengthM: 0.028,
  widthM: 0.008,
};

/**
 * A strip of ordinary 19 mm clear tape, always 20 mm long: folded over an edge (10 mm on each
 * side), or laid flat across the seam between two pieces in one plane (10 mm on each piece).
 * `lengthM` is the half length. Tape is ≈ 50 µm film + adhesive ≈ 55 g/m² ⇒ 19 × 20 mm ≈ 0.02 g.
 */
export const TAPE_SPEC: HolderSpec = {
  massKg: 0.00002,
  lengthM: 0.01,
  widthM: 0.019,
};

/** A clip that would run into paper leaving its plane leans back, by at most this much (deg). */
const MAX_TILT_DEG = 80;

export const holderSpec = (c: Pick<Clip, 'kind'>): HolderSpec => (c.kind === 'tape' ? TAPE_SPEC : CLIP_SPEC);

export interface ClipPlacement {
  /** Tape laid flat across a seam (two pieces of one plane meeting), rather than over an edge. */
  bridge?: boolean;
  facetId: number;
  at: Vec2;
  edgeDir: Vec2;
  position: Vec3;
  /** Unit vector in the stack plane pointing from the edge into the paper. */
  inward: Vec3;
  /** Stack plane normal (canonical). */
  normal: Vec3;
  /** Unit edge direction in model space. */
  edgeDirModel: Vec3;
  distance: number;
}

function onBoundary(f: Facet, p: Vec2, tol = 1e-7): boolean {
  for (const [a, b] of P.edges(f.poly)) {
    const { point } = P.closestPointOnSegment(p, a, b);
    if (V2.dist(point, p) < tol) return true;
  }
  return false;
}

export function clipStillAttached(model: PaperModel, clip: Clip): boolean {
  return model.facets.some((f) => onBoundary(f, clip.at));
}

function inwardFor(f: Facet, at: Vec2, edgeDir: Vec2): { inward: Vec3; inward2: Vec2 } | null {
  const perp: Vec2 = V2.normalize(V2.perp(edgeDir));
  for (const s of [1, -1]) {
    const q = V2.add(at, V2.scale(perp, s * 1e-4));
    if (P.pointInConvex(q, f.poly, 0)) {
      const inward2 = V2.scale(perp, s);
      return { inward: V.normalize(applyRigidLinear(f.T, [inward2[0], 0, inward2[1]])), inward2 };
    }
  }
  return null;
}

/**
 * Find the nearest point on a clippable stack edge to `p` (model space).
 * Clippable: paper outline, cut lips and 180° creases — i.e. the outline of a flat stack,
 * where a clip can slide over all layers. Edges where paper continues in another plane are excluded.
 */
export function findClipPlacement(
  model: PaperModel,
  p: Vec3,
  maxDist: number,
  graph: EdgeGraph = computeEdges(model),
  spec: HolderSpec = CLIP_SPEC,
): ClipPlacement | null {
  const byId = new Map(model.facets.map((f) => [f.id, f]));
  const segs: { f: Facet; seg: [Vec2, Vec2] }[] = [];
  for (const b of graph.borders) segs.push({ f: byId.get(b.facet)!, seg: b.seg });
  for (const l of graph.links) {
    if (l.kind !== 'crease') continue;
    const fa = byId.get(l.a)!;
    const fb = byId.get(l.b)!;
    if (!coplanar(fa, fb)) continue; // paper continues out of plane: not a stack edge
    segs.push({ f: fa, seg: l.seg });
  }

  let best: ClipPlacement | null = null;
  for (const { f, seg } of segs) {
    const a3 = toModel(f, seg[0]);
    const b3 = toModel(f, seg[1]);
    const ab = V.sub(b3, a3);
    const L2 = V.dot(ab, ab);
    if (L2 < 1e-14) continue;
    let t = V.dot(V.sub(p, a3), ab) / L2;
    // keep the clip fully on the edge where possible
    const halfW = spec.widthM / 2 / Math.sqrt(L2);
    t = halfW < 0.5 ? Math.min(1 - halfW, Math.max(halfW, t)) : 0.5;
    const pos = V.lerp(a3, b3, t);
    const distance = V.dist(pos, p);
    if (distance > maxDist || (best && distance >= best.distance)) continue;
    const at = V2.lerp(seg[0], seg[1], t);
    const edgeDir = V2.normalize(V2.sub(seg[1], seg[0]));
    const inw = inwardFor(f, at, edgeDir);
    if (!inw) continue;
    // the outside of the edge must be free of coplanar paper (otherwise it is inside a stack)
    const outside = V.addScaled(pos, inw.inward, -5e-4);
    const covered = model.facets.some((g) => g.id !== f.id && coplanar(f, g) && facetContains(g, outside, 1e-9, 1e-6));
    if (covered) continue;
    best = {
      facetId: f.id,
      at,
      edgeDir,
      position: pos,
      inward: inw.inward,
      normal: canonicalNormal(facetNormal(f)),
      edgeDirModel: V.normalize(ab),
      distance,
    };
  }
  return best;
}

/**
 * Bridge placement for tape: the nearest point to `p` on a seam, where a piece of paper leaves
 * the plane (a crease to paper at an angle, like a wing root on the keel) while another piece
 * continues in the same plane just across (the other wing half). The tape is laid flat across.
 */
export function findTapeBridge(model: PaperModel, p: Vec3, maxDist: number, graph: EdgeGraph = computeEdges(model)): ClipPlacement | null {
  const byId = new Map(model.facets.map((f) => [f.id, f]));
  let best: ClipPlacement | null = null;
  for (const l of graph.links) {
    if (l.kind !== 'crease') continue;
    const fa = byId.get(l.a)!;
    const fb = byId.get(l.b)!;
    if (coplanar(fa, fb)) continue;
    for (const f of [fa, fb]) {
      const a3 = toModel(f, l.seg[0]);
      const b3 = toModel(f, l.seg[1]);
      const ab = V.sub(b3, a3);
      const L2 = V.dot(ab, ab);
      if (L2 < 1e-14) continue;
      let t = V.dot(V.sub(p, a3), ab) / L2;
      t = Math.min(1, Math.max(0, t));
      const pos = V.lerp(a3, b3, t);
      const distance = V.dist(pos, p);
      if (distance > maxDist || (best && distance >= best.distance - 1e-9)) continue;
      const at = V2.lerp(l.seg[0], l.seg[1], t);
      const edgeDir = V2.normalize(V2.sub(l.seg[1], l.seg[0]));
      const inw = inwardFor(f, at, edgeDir);
      if (!inw) continue;
      // another piece (in about the same plane) just across the seam
      if (!acrossSeam(model, f, pos, V.normalize(ab), inw.inward)) continue;
      best = {
        bridge: true,
        facetId: f.id,
        at,
        edgeDir,
        position: pos,
        inward: inw.inward,
        normal: canonicalNormal(facetNormal(f)),
        edgeDirModel: V.normalize(ab),
        distance,
      };
    }
  }
  return best;
}

/** Pieces this close to the plane on the other side of a seam can be taped across (wings in a shallow V). */
const BRIDGE_MAX_ANGLE_COS = Math.cos((35 * Math.PI) / 180);

/**
 * The paper just across a seam from facet f (at `pos`, seam along `axis`, `inward` into f): the
 * facets there at most 35° out of f's plane, with the direction into them (in their plane).
 */
function acrossSeam(model: PaperModel, f: Facet, pos: Vec3, axis: Vec3, inward: Vec3): { g: Facet; inward: Vec3 } | null {
  const nf = canonicalNormal(facetNormal(f));
  for (const g of model.facets) {
    if (g.id === f.id) continue;
    const ng = canonicalNormal(facetNormal(g));
    if (Math.abs(V.dot(nf, ng)) < BRIDGE_MAX_ANGLE_COS) continue;
    let d = V.normalize(V.cross(ng, axis));
    if (V.dot(d, inward) > 0) d = V.negate(d);
    if (V.dot(d, inward) > -0.5) continue;
    if (facetContains(g, V.addScaled(pos, d, 1e-3), 1e-9, 1e-6)) return { g, inward: d };
  }
  return null;
}

/** Where tape would go near `p`: across a seam if there is one, otherwise folded over an edge. */
export function findTapePlacement(model: PaperModel, p: Vec3, maxDist: number, graph: EdgeGraph = computeEdges(model)): ClipPlacement | null {
  const bridge = findTapeBridge(model, p, maxDist, graph);
  const wrap = findClipPlacement(model, p, maxDist, graph, TAPE_SPEC);
  if (bridge && (!wrap || bridge.distance <= wrap.distance + 0.002)) return bridge;
  return wrap;
}

export interface ResolvedClip {
  clip: Clip;
  position: Vec3;
  /** Into the paper along the holder (a clip leaning back is turned in its plane). */
  inward: Vec3;
  normal: Vec3;
  /** Across the holder, in its plane (along the edge for an upright clip). */
  edgeDirModel: Vec3;
  /** Facets squeezed by the clip (all coplanar layers under its footprint). */
  clamped: number[];
  /** Centre of mass of the clip (half its length in from the edge). */
  centerOfMass: Vec3;
  /** How far a clip leans back to clear paper leaving its plane (deg, signed about `normal`). */
  tiltDeg: number;
  /** Tape laid flat across a seam, and the side of the stack it lies on (along ±normal). */
  bridge: boolean;
  side: 1 | -1;
  /**
   * Bridging tape: its other half, on the piece across the seam (which may sit at an angle, like
   * a wing half in a shallow V): the same kind of frame (x along `inward` from the seam) and its
   * own trimmed outline. The facets it covers are in `clamped` too.
   */
  across: { inward: Vec3; normal: Vec3; side: 1 | -1; clamped: number[]; tapeShape: Vec2[][] } | null;
  /**
   * Tape after trimming to the paper (what sticks out past the paper is cut off): polygons in
   * the holder's frame, x along `inward`, z along inward × normal. Tape covers 0 ≤ x ≤ length:
   * folded tape on both sides of the stack, bridging tape on one side (the rest is `across`).
   */
  tapeShape: Vec2[][] | null;
}

/** Edges where paper leaves the plane of facet f: crease segments to paper at an angle. */
function outOfPlaneEdges(model: PaperModel, f: Facet, graph: EdgeGraph): { a: Vec3; b: Vec3; other: Facet }[] {
  const byId = new Map(model.facets.map((g) => [g.id, g]));
  const out: { a: Vec3; b: Vec3; other: Facet }[] = [];
  for (const l of graph.links) {
    if (l.kind !== 'crease') continue;
    const fa = byId.get(l.a)!;
    const fb = byId.get(l.b)!;
    if (coplanar(fa, fb)) continue;
    for (const [mine, other] of [
      [fa, fb],
      [fb, fa],
    ]) {
      if (mine.id === f.id || coplanar(f, mine)) out.push({ a: toModel(mine, l.seg[0]), b: toModel(mine, l.seg[1]), other });
    }
  }
  return out;
}

const graphs = new WeakMap<PaperModel, EdgeGraph>();
function graphOf(model: PaperModel): EdgeGraph {
  let g = graphs.get(model);
  if (!g) graphs.set(model, (g = computeEdges(model)));
  return g;
}

const resolved = new WeakMap<PaperModel, Map<Clip, ResolvedClip | null>>();

/** Footprint rectangle of a holder: from x0 to x1 along `inward`, ±hw across. */
function rect(position: Vec3, inward: Vec3, across: Vec3, x0: number, x1: number, hw: number): Vec3[] {
  return [
    V.addScaled(V.addScaled(position, across, -hw), inward, x0),
    V.addScaled(V.addScaled(position, across, hw), inward, x0),
    V.addScaled(V.addScaled(position, across, hw), inward, x1),
    V.addScaled(V.addScaled(position, across, -hw), inward, x1),
  ];
}

export function resolveClip(model: PaperModel, clip: Clip): ResolvedClip | null {
  let cache = resolved.get(model);
  if (!cache) resolved.set(model, (cache = new Map()));
  if (cache.has(clip)) return cache.get(clip)!;
  const r = resolveClipUncached(model, clip);
  // the editor's hover preview makes a new clip on every mouse move: do not keep those
  if (clip.id !== 'ghost') cache.set(clip, r);
  return r;
}

function resolveClipUncached(model: PaperModel, clip: Clip): ResolvedClip | null {
  // a point on a crease is on the edge of the paper on both sides of it: bridging tape belongs
  // to the side with paper across the seam
  const candidates = model.facets.filter((g) => onBoundary(g, clip.at));
  const bridgeSide = (g: Facet) => {
    const iw = inwardFor(g, clip.at, clip.edgeDir);
    const axis = V.normalize(applyRigidLinear(g.T, [clip.edgeDir[0], 0, clip.edgeDir[1]]));
    return !!iw && !!acrossSeam(model, g, toModel(g, clip.at), axis, iw.inward);
  };
  const f = (clip.kind === 'tape' && clip.bridge ? candidates.find(bridgeSide) : undefined) ?? candidates[0];
  if (!f) return null;
  const inw = inwardFor(f, clip.at, clip.edgeDir);
  if (!inw) return null;
  const position = toModel(f, clip.at);
  const normal = canonicalNormal(facetNormal(f));
  let inward = inw.inward;
  let edgeDirModel = V.normalize(applyRigidLinear(f.T, [clip.edgeDir[0], 0, clip.edgeDir[1]]));
  const spec = holderSpec(clip);
  const hw = spec.widthM / 2;
  const isTape = clip.kind === 'tape';
  const bridge = isTape && !!clip.bridge;
  const basis = planeBasis(normal);
  // paper leaving the plane: what a clip must lean away from, and the seam under bridging tape
  const obstacles = !isTape || bridge ? outOfPlaneEdges(model, f, graphOf(model)) : [];
  let tiltDeg = 0;
  if (!isTape && (obstacles.length || clip.tiltDeg)) {
    // a clip cannot pass through paper that leaves its plane (a wing above the keel): lean it
    // back in its plane, about the point on the edge, as little as needed
    const segs = obstacles.map((o) => projectToPlane([o.a, o.b], position, basis));
    const along = V.cross(normal, inward); // in the plane, ⟂ inward
    const blocked = (deg: number) => {
      const a = (deg * Math.PI) / 180;
      const iw = V.add(V.scale(inward, Math.cos(a)), V.scale(along, Math.sin(a)));
      const ac = V.cross(iw, normal);
      // a hair smaller than the clip, so paper it merely touches does not count
      const fp = projectToPlane(rect(position, iw, ac, 3e-4, spec.lengthM, hw - 3e-4), position, basis);
      return segs.some(([a2, b2]) => {
        const r = P.clipSegmentConvex(a2, b2, fp);
        return !!r && r[1] - r[0] > 1e-9;
      });
    };
    if (clip.tiltDeg !== undefined) tiltDeg = clip.tiltDeg;
    else if (blocked(0)) {
      // the chosen way first (backwards), the other way only if that never clears
      const pref = clip.lean ?? 1;
      for (const sgn of [pref, -pref]) {
        for (let d = 1; d <= MAX_TILT_DEG && !tiltDeg; d++) if (!blocked(sgn * d)) tiltDeg = sgn * d;
        if (tiltDeg) break;
      }
    }
    if (tiltDeg) {
      const a = (tiltDeg * Math.PI) / 180;
      inward = V.normalize(V.add(V.scale(inward, Math.cos(a)), V.scale(along, Math.sin(a))));
      edgeDirModel = V.normalize(V.cross(inward, normal));
      // keep the across direction's sense as along the edge
      if (V.dot(edgeDirModel, applyRigidLinear(f.T, [clip.edgeDir[0], 0, clip.edgeDir[1]])) < 0) edgeDirModel = V.negate(edgeDirModel);
    }
  }
  const covered = (fc: Facet, n: Vec3, iw: Vec3, across: Vec3) => {
    const b = planeBasis(n);
    const fp2 = projectToPlane(rect(position, iw, across, 0, spec.lengthM, hw), position, b);
    return model.facets.filter((g) => (g.id === fc.id || coplanar(fc, g)) && P.convexOverlap(fp2, projectToPlane(facetVertices(g), position, b), 1e-9));
  };
  const under = covered(f, normal, inward, edgeDirModel);
  // bridging tape lies on the side away from the paper that leaves the plane at the seam
  let side: 1 | -1 = 1;
  let across: ResolvedClip['across'] = null;
  if (bridge) {
    const seam = obstacles.find((o) => V.length(V.cross(V.sub(position, o.a), V.normalize(V.sub(o.b, o.a)))) < 1e-6);
    const below = seam ? V.scale(facetVertices(seam.other).reduce((acc, v) => V.add(acc, v), [0, 0, 0] as Vec3), 1 / facetVertices(seam.other).length) : null;
    const sideOf = (n: Vec3): 1 | -1 => (below && V.dot(V.sub(below, position), n) > 0 ? -1 : 1);
    side = sideOf(normal);
    const axis = V.normalize(V.cross(normal, inward));
    const other = acrossSeam(model, f, position, axis, inward);
    if (other) {
      const n2 = canonicalNormal(facetNormal(other.g));
      const ac2 = V.normalize(V.cross(other.inward, n2));
      const under2 = covered(other.g, n2, other.inward, ac2);
      across = {
        inward: other.inward,
        normal: n2,
        side: sideOf(n2),
        clamped: under2.map((g) => g.id),
        tapeShape: trimTape(position, other.inward, n2, 0, spec.lengthM, hw, under2),
      };
    }
  }
  return {
    clip,
    position,
    inward,
    normal,
    edgeDirModel,
    clamped: [...new Set([...under.map((g) => g.id), ...(across?.clamped ?? [])])],
    centerOfMass: V.addScaled(position, inward, bridge ? 0 : spec.lengthM / 2),
    tiltDeg,
    bridge,
    side,
    across,
    tapeShape: isTape ? trimTape(position, inward, normal, 0, spec.lengthM, hw, under) : null,
  };
}

/**
 * The tape's outline after cutting away what sticks out past the paper under it (at the nose,
 * past a narrow edge): the rectangle intersected with the paper, in the holder's frame.
 */
function trimTape(position: Vec3, inward: Vec3, normal: Vec3, x0: number, x1: number, hw: number, under: Facet[]): Vec2[][] {
  const across = V.normalize(V.cross(inward, normal));
  const local = (q: Vec3): [number, number] => {
    const r = V.sub(q, position);
    return [V.dot(r, inward), V.dot(r, across)];
  };
  const box: Ring = [
    [x0, -hw],
    [x1, -hw],
    [x1, hw],
    [x0, hw],
    [x0, -hw],
  ];
  const full: Vec2[][] = [box.slice(0, 4).map(([x, z]) => [x, z] as Vec2)];
  if (!under.length) return full;
  const snap = (v: number, q: number) => Math.round(v / q) * q;
  for (const q of [1e-7, 1e-6, 1e-5]) {
    try {
      const rings = under.map((g) => {
        const r = facetVertices(g).map((v) => local(v).map((c) => snap(c, q)) as [number, number]);
        return [[...r, r[0]]] as [number, number][][];
      });
      const paper = polygonClipping.union(rings[0], ...rings.slice(1));
      const cut: MultiPolygon = polygonClipping.intersection([box.map(([x, z]) => [snap(x, q), snap(z, q)] as [number, number])], paper);
      // outer rings only (tape over a hole in the paper is rare and harmless)
      return cut.map((poly) => poly[0].slice(0, -1).map(([x, z]) => [x, z] as Vec2)).filter((r) => r.length >= 3);
    } catch {
      // nearly coincident edges: retry on a coarser grid
    }
  }
  return full;
}
