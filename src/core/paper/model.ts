import * as V from '../math/vec3';
import * as P from '../math/polygon2d';
import { applyRigid, applyRigidLinear, invertRigid, rigidIdentity } from '../math/transform';
import type { RigidTransform } from '../math/transform';
import type { Vec2 } from '../math/vec2';
import type { Vec3 } from '../math/vec3';
import type { PaperSpec } from './paperSpec';

/**
 * Geometric tolerances (metres). Paper has zero thickness in the model, so stacked layers
 * are exactly coplanar; these only absorb floating-point noise.
 */
export const EPS = 1e-9;
export const EPS_PLANE = 1e-7;

/**
 * A convex piece of paper. `poly` is in sheet coordinates (u, v) of the unfolded sheet.
 * The sheet is embedded as (u, 0, v) with its front side facing +Y; `T` maps that
 * embedding into model space.
 */
export interface Facet {
  id: number;
  poly: Vec2[];
  T: RigidTransform;
  /** Stacking order among coplanar overlapping facets, measured along the plane's canonical normal. */
  layer: number;
  /** Ids of operations that have rotated this facet (for future flaps/hinges). */
  foldPath: string[];
}

/** A scissor cut, in sheet coordinates. Paper is disconnected along it. */
export interface CutSegment {
  a: Vec2;
  b: Vec2;
  opId: string;
}

/** A paper clip or a piece of tape attached at a point on a stack edge (sheet coordinates). */
export interface Clip {
  id: string;
  at: Vec2;
  edgeDir: Vec2;
  massKg: number;
  /** Paper clip (default) or tape folded over the edge. Both hold the layers together. */
  kind?: 'clip' | 'tape';
  /** Tape laid flat across a seam (two wing halves), `at` on one side's edge. */
  bridge?: boolean;
  /**
   * Which way a clip leans when paper leaving its plane is in the way (+1: turned from `inward`
   * towards normal × inward, −1: the other way). The editor picks backwards, away from the nose.
   */
  lean?: 1 | -1;
}

export interface PaperModel {
  spec: PaperSpec;
  facets: Facet[];
  cuts: CutSegment[];
  clips: Clip[];
  nextFacetId: number;
}

export function createSheet(spec: PaperSpec): PaperModel {
  const a = spec.heightM / 2;
  const b = spec.widthM / 2;
  return {
    spec,
    facets: [
      {
        id: 0,
        poly: [
          [-a, -b],
          [a, -b],
          [a, b],
          [-a, b],
        ],
        T: rigidIdentity(),
        layer: 0,
        foldPath: [],
      },
    ],
    cuts: [],
    clips: [],
    nextFacetId: 1,
  };
}

export const sheetPoint = (p: Vec2): Vec3 => [p[0], 0, p[1]];

export const facetVertices = (f: Facet): Vec3[] => f.poly.map((p) => applyRigid(f.T, sheetPoint(p)));
export const facetNormal = (f: Facet): Vec3 => applyRigidLinear(f.T, [0, 1, 0]);
export const facetArea = (f: Facet): number => P.area(f.poly);
export const facetCentroid = (f: Facet): Vec3 => applyRigid(f.T, sheetPoint(P.centroid(f.poly)));
export const toModel = (f: Facet, p: Vec2): Vec3 => applyRigid(f.T, sheetPoint(p));

/** Model-space point → facet's sheet frame (u, w, v); w is the height above the facet plane. */
export const toSheet3 = (f: Facet, p: Vec3): Vec3 => applyRigid(invertRigid(f.T), p);
export function toSheet(f: Facet, p: Vec3): Vec2 {
  const s = toSheet3(f, p);
  return [s[0], s[2]];
}

export const totalArea = (m: PaperModel): number => m.facets.reduce((s, f) => s + facetArea(f), 0);

export function findFacet(m: PaperModel, id: number): Facet | undefined {
  return m.facets.find((f) => f.id === id);
}

/**
 * Canonical orientation of a plane normal: the sign is chosen so the first clearly
 * non-zero component among (y, x, z) is positive. Layers are measured along it.
 */
export function canonicalNormal(n: Vec3): Vec3 {
  const t = 1e-6;
  if (n[1] > t) return n;
  if (n[1] < -t) return V.negate(n);
  if (n[0] > t) return n;
  if (n[0] < -t) return V.negate(n);
  return n[2] >= 0 ? n : V.negate(n);
}

export function parallelNormals(a: Vec3, b: Vec3, tol = 1e-6): boolean {
  return V.length(V.cross(a, b)) < tol;
}

/** True if facet f lies in the plane through `point` with normal `n`. */
export function facetInPlane(f: Facet, point: Vec3, n: Vec3, tol = EPS_PLANE): boolean {
  if (!parallelNormals(facetNormal(f), n)) return false;
  return Math.abs(V.dot(n, V.sub(facetVertices(f)[0], point))) < tol;
}

export function coplanar(f: Facet, g: Facet): boolean {
  return facetInPlane(g, facetVertices(f)[0], facetNormal(f));
}

/** Height of facet in its stack as seen from direction N (larger = closer to a viewer on the +N side). */
export function heightAlong(f: Facet, N: Vec3): number {
  return f.layer * Math.sign(V.dot(canonicalNormal(facetNormal(f)), N) || 1);
}

/** Orthonormal 2D basis of a plane, for projecting coplanar facets into a common frame. */
export function planeBasis(n: Vec3): [Vec3, Vec3] {
  const e1 = V.anyPerpendicular(n);
  return [e1, V.cross(n, e1)];
}

export function projectToPlane(points: Vec3[], origin: Vec3, basis: [Vec3, Vec3]): Vec2[] {
  return points.map((p) => {
    const r = V.sub(p, origin);
    return [V.dot(r, basis[0]), V.dot(r, basis[1])] as Vec2;
  });
}

/** Positive-area overlap between two coplanar facets (in model space). */
export function facetsOverlap(f: Facet, g: Facet): boolean {
  const n = canonicalNormal(facetNormal(f));
  const basis = planeBasis(n);
  const o = facetVertices(f)[0];
  return P.convexOverlap(projectToPlane(facetVertices(f), o, basis), projectToPlane(facetVertices(g), o, basis), 1e-9);
}

/** Is model-space point p (assumed near the facet plane) inside facet f? */
export function facetContains(f: Facet, p: Vec3, margin = 0, planeTol = 1e-4): boolean {
  const s = toSheet3(f, p);
  if (Math.abs(s[1]) > planeTol) return false;
  return P.pointInConvex([s[0], s[2]], f.poly, margin);
}
