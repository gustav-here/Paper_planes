import polygonClipping from 'polygon-clipping';
import type { MultiPolygon, Polygon, Ring } from 'polygon-clipping';
import * as V from '../math/vec3';
import type { Vec2 } from '../math/vec2';
import type { Vec3 } from '../math/vec3';
import { canonicalNormal, facetNormal, facetVertices, parallelNormals } from '../paper/model';
import type { Facet, PaperModel } from '../paper/model';

/**
 * Aerodynamic surface: the union of all coplanar facets (stacked layers are ONE plate for the
 * air, although every layer counts for mass). All vectors in the body frame (model axes,
 * origin at the centre of mass).
 */
export interface AeroSurface {
  id: number;
  /** Canonical plane normal. */
  normal: Vec3;
  /** Point on the plane (projection of the CoM). */
  origin: Vec3;
  e1: Vec3;
  e2: Vec3;
  /** Outline edges of the union in plane coordinates (outer rings and holes). */
  edges: [Vec2, Vec2][];
  /** Outline vertices in the body frame (for span measurements). */
  outline3: Vec3[];
  area: number;
  centroid: Vec3;
  firstElement: number;
  elementCount: number;
  /** Surfaces (including this one) whose planes are close enough to form one lifting system. */
  partners: number[];
}

export interface AeroElement {
  surface: number;
  /** Area centroid, body frame. */
  center: Vec3;
  /** Centroid in plane coordinates. */
  c2: Vec2;
  area: number;
  /** Outer ring in plane coordinates (for chordwise extent). */
  poly2: Vec2[];
}

export interface AeroMesh {
  surfaces: AeroSurface[];
  elements: AeroElement[];
}

function ringCentroid(r: Ring): { a: number; cx: number; cy: number } {
  let a = 0;
  let cx = 0;
  let cy = 0;
  for (let i = 0; i + 1 < r.length; i++) {
    const c = r[i][0] * r[i + 1][1] - r[i + 1][0] * r[i][1];
    a += c;
    cx += (r[i][0] + r[i + 1][0]) * c;
    cy += (r[i][1] + r[i + 1][1]) * c;
  }
  return { a: a / 2, cx, cy };
}

/** Area and centroid of a polygon with holes (outer ring first). */
function polygonAreaCentroid(p: Polygon): { area: number; c: Vec2 } {
  let A = 0;
  let X = 0;
  let Y = 0;
  p.forEach((ring, i) => {
    const r = ringCentroid(ring);
    // outer ring counts positive, holes negative, whatever their winding
    const sign = (i === 0 ? 1 : -1) * Math.sign(r.a || 1);
    A += sign * r.a;
    X += (sign * r.cx) / 6;
    Y += (sign * r.cy) / 6;
  });
  return { area: A, c: A > 0 ? [X / A, Y / A] : [0, 0] };
}

/** Snap coordinates to a grid of size q (removes near-degenerate slivers from folded stacks). */
function snapGeom<T>(g: T, q: number): T {
  if (typeof g === 'number') return (Math.round(g / q) * q) as T;
  if (Array.isArray(g)) return g.map((x) => snapGeom(x, q)) as T;
  return g;
}

/**
 * polygon-clipping can fail on nearly collinear, nearly coincident edges (common in folded
 * stacks). Retry on progressively coarser snapping grids (0.1 µm … 10 µm), all far below
 * anything that matters aerodynamically.
 */
function robust<T, R>(op: (g: T[]) => R, geoms: T[]): R {
  let last: unknown;
  for (const q of [1e-7, 1e-6, 1e-5]) {
    try {
      return op(geoms.map((g) => snapGeom(g, q)));
    } catch (e) {
      last = e;
    }
  }
  throw last;
}

function groupByPlane(facets: Facet[]): Facet[][] {
  const groups: { n: Vec3; d: number; fs: Facet[] }[] = [];
  for (const f of facets) {
    const n = canonicalNormal(facetNormal(f));
    const d = V.dot(n, facetVertices(f)[0]);
    const g = groups.find((x) => parallelNormals(x.n, n) && Math.abs(x.d - d) < 1e-7);
    if (g) g.fs.push(f);
    else groups.push({ n, d, fs: [f] });
  }
  return groups.map((g) => g.fs);
}

/**
 * Build aero surfaces and square-grid elements from the folded model.
 * @param com centre of mass (model space); output is relative to it.
 */
export function buildAeroMesh(model: PaperModel, com: Vec3, elementSize: number, liftingSystemAngleDeg: number): AeroMesh {
  const surfaces: AeroSurface[] = [];
  const elements: AeroElement[] = [];
  for (const group of groupByPlane(model.facets)) {
    const n = canonicalNormal(facetNormal(group[0]));
    let e1 = V.rejectFrom([1, 0, 0], n);
    if (V.length(e1) < 0.3) e1 = V.rejectFrom([0, 0, 1], n);
    e1 = V.normalize(e1);
    const e2 = V.cross(n, e1);
    const d = V.dot(n, facetVertices(group[0])[0]);
    const originModel = V.addScaled(com, n, d - V.dot(n, com));
    const to2 = (p: Vec3): [number, number] => {
      const r = V.sub(p, originModel);
      return [V.dot(r, e1), V.dot(r, e2)];
    };
    const polys: Polygon[] = group.map((f) => {
      const ring = facetVertices(f).map(to2);
      ring.push(ring[0]);
      return [ring];
    });
    const union: MultiPolygon = polys.length === 1 ? [polys[0]] : robust((g) => polygonClipping.union(g[0], ...g.slice(1)), polys);
    let area = 0;
    let cx = 0;
    let cy = 0;
    const edges: [Vec2, Vec2][] = [];
    const outline3: Vec3[] = [];
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    const from2 = (q: Vec2): Vec3 => V.sub(V.add(V.add(originModel, V.scale(e1, q[0])), V.scale(e2, q[1])), com);
    for (const poly of union) {
      const ac = polygonAreaCentroid(poly);
      area += ac.area;
      cx += ac.c[0] * ac.area;
      cy += ac.c[1] * ac.area;
      for (const ring of poly) {
        for (let i = 0; i + 1 < ring.length; i++) {
          edges.push([[ring[i][0], ring[i][1]], [ring[i + 1][0], ring[i + 1][1]]]);
          outline3.push(from2([ring[i][0], ring[i][1]]));
          minX = Math.min(minX, ring[i][0]);
          maxX = Math.max(maxX, ring[i][0]);
          minY = Math.min(minY, ring[i][1]);
          maxY = Math.max(maxY, ring[i][1]);
        }
      }
    }
    if (area < 1e-8) continue;
    const id = surfaces.length;
    const firstElement = elements.length;
    const h = elementSize;
    for (let gx = Math.floor(minX / h); gx * h < maxX; gx++) {
      for (let gy = Math.floor(minY / h); gy * h < maxY; gy++) {
        const x0 = gx * h;
        const y0 = gy * h;
        const cell: Polygon = [[[x0, y0], [x0 + h, y0], [x0 + h, y0 + h], [x0, y0 + h], [x0, y0]]];
        const pieces = robust((g) => polygonClipping.intersection(g[0], g[1]), [union, cell]);
        for (const piece of pieces) {
          const ac = polygonAreaCentroid(piece);
          if (ac.area < 1e-8) continue;
          elements.push({
            surface: id,
            center: from2(ac.c),
            c2: ac.c,
            area: ac.area,
            poly2: piece[0].slice(0, -1).map((p) => [p[0], p[1]] as Vec2),
          });
        }
      }
    }
    surfaces.push({
      id,
      normal: n,
      origin: V.sub(originModel, com),
      e1,
      e2,
      edges,
      outline3,
      area,
      centroid: from2([cx / area, cy / area]),
      firstElement,
      elementCount: elements.length - firstElement,
      partners: [],
    });
  }
  const cosLim = Math.cos((liftingSystemAngleDeg * Math.PI) / 180);
  for (const s of surfaces) {
    s.partners = surfaces.filter((o) => Math.abs(V.dot(o.normal, s.normal)) >= cosLim).map((o) => o.id);
  }
  return { surfaces, elements };
}
