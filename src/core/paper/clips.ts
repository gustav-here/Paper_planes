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
 * A strip of ordinary 19 mm clear tape, 30 mm long, folded over the edge (15 mm on each side).
 * Tape is ≈ 50 µm film + adhesive ≈ 55 g/m² ⇒ 19 × 30 mm ≈ 0.03 g.
 */
export const TAPE_SPEC: HolderSpec = {
  massKg: 0.00003,
  lengthM: 0.015,
  widthM: 0.019,
};

export const holderSpec = (c: Pick<Clip, 'kind'>): HolderSpec => (c.kind === 'tape' ? TAPE_SPEC : CLIP_SPEC);

export interface ClipPlacement {
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

export interface ResolvedClip {
  clip: Clip;
  position: Vec3;
  inward: Vec3;
  normal: Vec3;
  edgeDirModel: Vec3;
  /** Facets squeezed by the clip (all coplanar layers under its footprint). */
  clamped: number[];
  /** Centre of mass of the clip (half its length in from the edge). */
  centerOfMass: Vec3;
}

export function resolveClip(model: PaperModel, clip: Clip): ResolvedClip | null {
  const f = model.facets.find((g) => onBoundary(g, clip.at));
  if (!f) return null;
  const inw = inwardFor(f, clip.at, clip.edgeDir);
  if (!inw) return null;
  const position = toModel(f, clip.at);
  const edgeDirModel = V.normalize(applyRigidLinear(f.T, [clip.edgeDir[0], 0, clip.edgeDir[1]]));
  const normal = canonicalNormal(facetNormal(f));
  const spec = holderSpec(clip);
  const hw = spec.widthM / 2;
  const footprint: Vec3[] = [
    V.addScaled(position, edgeDirModel, -hw),
    V.addScaled(position, edgeDirModel, hw),
    V.addScaled(V.addScaled(position, edgeDirModel, hw), inw.inward, spec.lengthM),
    V.addScaled(V.addScaled(position, edgeDirModel, -hw), inw.inward, spec.lengthM),
  ];
  const basis = planeBasis(normal);
  const fp2 = projectToPlane(footprint, position, basis);
  const clamped = model.facets
    .filter((g) => (g.id === f.id || coplanar(f, g)) && P.convexOverlap(fp2, projectToPlane(facetVertices(g), position, basis), 1e-9))
    .map((g) => g.id);
  return {
    clip,
    position,
    inward: inw.inward,
    normal,
    edgeDirModel,
    clamped,
    centerOfMass: V.addScaled(position, inw.inward, spec.lengthM / 2),
  };
}
