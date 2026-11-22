import polygonClipping from 'polygon-clipping';
import type { MultiPolygon, Polygon, Ring } from 'polygon-clipping';
import * as V from '../math/vec3';
import type { Vec2 } from '../math/vec2';
import type { Vec3 } from '../math/vec3';
import { canonicalNormal, facetNormal, facetVertices, parallelNormals, toModel } from '../paper/model';
import { computeEdges } from '../paper/edges';
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
  /** Grouping tag of its facets (control flap / panel index), −1 if untagged. */
  tag: number;
  /**
   * A wing sitting on top of a keel (see markKeelJunctions): the keel's plane normal and a point
   * on the junction (body frame), and the equivalent dihedral (rad) the keel gives this wing.
   */
  keel?: { normal: Vec3; point: Vec3; dihedral: number };
  /**
   * A keel with a wing on top. Slender-body theory: a body's side force arises where the lateral
   * apparent mass of its cross-section grows along it. A bare keel's grows only at the nose (where
   * it deepens), so its side force acts far forward and turns the plane away from the wind. The
   * wing on top is an end plate that widens all the way back, so the keel's apparent mass, and
   * its side force, grow along its whole length: the attached-flow load is spread evenly over it.
   */
  underWing?: boolean;
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

function groupByPlane(facets: Facet[], tagOf: (f: Facet) => number): { fs: Facet[]; tag: number }[] {
  const groups: { n: Vec3; d: number; tag: number; fs: Facet[] }[] = [];
  for (const f of facets) {
    const n = canonicalNormal(facetNormal(f));
    const d = V.dot(n, facetVertices(f)[0]);
    const tag = tagOf(f);
    const g = groups.find((x) => x.tag === tag && parallelNormals(x.n, n) && Math.abs(x.d - d) < 1e-7);
    if (g) g.fs.push(f);
    else groups.push({ n, d, tag, fs: [f] });
  }
  return groups.map((g) => ({ fs: g.fs, tag: g.tag }));
}

/** One plane group of facets: its frame and the union outline in plane coordinates. */
interface PlaneGroup {
  n: Vec3;
  e1: Vec3;
  e2: Vec3;
  /** Model-space point of the plane closest to the centre of mass (plane-coordinate origin). */
  originModel: Vec3;
  union: MultiPolygon;
  area: number;
  tag: number;
}

/** Layers this close in angle that overlap are one stack for the air (a gaping fold). */
const STACK_ANGLE_DEG = 25;
/** …and no farther apart than this where they overlap (m). */
const STACK_GAP_M = 0.012;

function areaOf(mp: MultiPolygon): number {
  return mp.reduce((s, p) => s + polygonAreaCentroid(p).area, 0);
}

/** Group `b`'s outline expressed in group `a`'s plane coordinates (orthogonal projection). */
function projectInto(b: PlaneGroup, a: PlaneGroup): MultiPolygon {
  return b.union.map((poly) =>
    poly.map((ring) =>
      ring.map((q) => {
        const p = V.add(V.add(b.originModel, V.scale(b.e1, q[0])), V.scale(b.e2, q[1]));
        const r = V.sub(p, a.originModel);
        return [V.dot(r, a.e1), V.dot(r, a.e2)] as [number, number];
      }),
    ),
  );
}

/**
 * Layers that a fold left slightly open (springback) still act as one plate for the air: a
 * near-coplanar layer overlapping a larger one joins it (same tag) or, if it moves on its own
 * (another panel), keeps only the part the larger one does not cover. Exactly coplanar layers
 * are already one group. The area that overlaps is counted once.
 */
function mergeStacks(groups: PlaneGroup[]): PlaneGroup[] {
  const cosLim = Math.cos((STACK_ANGLE_DEG * Math.PI) / 180);
  const order = [...groups].sort((x, y) => y.area - x.area);
  const out: PlaneGroup[] = [];
  for (const g of order) {
    let cur: PlaneGroup | null = g;
    for (const big of out) {
      if (!cur) break;
      if (Math.abs(V.dot(big.n, cur.n)) < cosLim) continue;
      if (big.tag === cur.tag && parallelNormals(big.n, cur.n, 1e-9)) continue;
      const proj = projectInto(cur, big);
      let inter: MultiPolygon;
      try {
        inter = robust((x) => polygonClipping.intersection(x[0], x[1]), [big.union, proj]);
      } catch {
        continue;
      }
      const ia = areaOf(inter);
      if (ia < 0.1 * Math.min(cur.area, big.area) || ia < 1e-6) continue;
      // close enough where they overlap?
      const c = polygonAreaCentroid(inter[0]).c;
      const pc = V.add(V.add(big.originModel, V.scale(big.e1, c[0])), V.scale(big.e2, c[1]));
      const dist = Math.abs(V.dot(cur.n, V.sub(pc, cur.originModel)));
      if (dist > STACK_GAP_M) continue;
      if (big.tag === cur.tag) {
        big.union = robust((x) => polygonClipping.union(x[0], x[1]), [big.union, proj]);
        big.area = areaOf(big.union);
        cur = null;
      } else {
        const back = projectInto(big, cur);
        cur.union = robust((x) => polygonClipping.difference(x[0], x[1]), [cur.union, back]);
        cur.area = areaOf(cur.union);
        if (cur.area < 1e-8) cur = null;
      }
    }
    if (cur) out.push(cur);
  }
  return out;
}

/**
 * Build aero surfaces and square-grid elements from the folded model.
 * @param com centre of mass (model space); output is relative to it.
 */
export function buildAeroMesh(
  model: PaperModel,
  com: Vec3,
  elementSize: number,
  liftingSystemAngleDeg: number,
  /** Facets with different tags never merge into one surface (they can move independently). */
  tags?: Map<number, number>,
): AeroMesh {
  const surfaces: AeroSurface[] = [];
  const elements: AeroElement[] = [];
  const groups: PlaneGroup[] = [];
  for (const { fs: group, tag } of groupByPlane(model.facets, (f) => tags?.get(f.id) ?? -1)) {
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
    groups.push({ n, e1, e2, originModel, union, area: areaOf(union), tag });
  }
  for (const { n, e1, e2, originModel, union, tag } of mergeStacks(groups)) {
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
      tag,
    });
  }
  const cosLim = Math.cos((liftingSystemAngleDeg * Math.PI) / 180);
  for (const s of surfaces) {
    s.partners = surfaces.filter((o) => Math.abs(V.dot(o.normal, s.normal)) >= cosLim).map((o) => o.id);
  }
  return { surfaces, elements };
}

/**
 * Wings on top of a keel. In sideslip the keel blocks the air crossing under the wing roots, and
 * the wing turned into the wind is lifted more, as if the wings had some dihedral: the "high
 * wing" effect, worth a few degrees of dihedral on aircraft (DATCOM wing–body term), more the
 * deeper the body is compared with the span. A paper keel is a thin plate, not a body, so this is
 * a semi-empirical equivalent dihedral: perDepth × keel depth / semi-span, at most maxDeg.
 * A wing is a surface within 45° of level, a keel one within 45° of upright, joined by a crease
 * with the keel hanging below the joint.
 */
export function markKeelJunctions(
  mesh: AeroMesh,
  model: PaperModel,
  com: Vec3,
  up: Vec3,
  right: Vec3,
  semiSpan: number,
  keelCfg: { perDepth: number; maxDeg: number },
  tags?: Map<number, number>,
): void {
  const cos45 = Math.SQRT1_2;
  const byId = new Map(model.facets.map((f) => [f.id, f]));
  const surfaceOf = (f: Facet): AeroSurface | undefined => {
    const n = canonicalNormal(facetNormal(f));
    const p = V.sub(facetVertices(f)[0], com);
    const tag = tags?.get(f.id) ?? -1;
    return mesh.surfaces.find((s) => s.tag === tag && Math.abs(V.dot(s.normal, n)) > 0.999 && Math.abs(V.dot(s.normal, V.sub(p, s.origin))) < 1e-6);
  };
  for (const l of computeEdges(model).links) {
    if (l.kind !== 'crease') continue;
    for (const [w, k] of [
      [l.a, l.b],
      [l.b, l.a],
    ]) {
      const fw = byId.get(w)!;
      const fk = byId.get(k)!;
      const nw = facetNormal(fw);
      const nk = facetNormal(fk);
      if (Math.abs(V.dot(nw, up)) < cos45 || Math.abs(V.dot(nk, right)) < cos45) continue;
      const wing = surfaceOf(fw);
      const keel = surfaceOf(fk);
      if (!wing || !keel) continue;
      const point = V.sub(toModel(fw, l.seg[0]), com);
      // the keel hangs below the joint: its depth under it
      const depth = Math.max(0, ...keel.outline3.map((v) => V.dot(V.sub(point, v), up)));
      if (depth < 1e-3) continue;
      const deg = Math.min(keelCfg.maxDeg, (keelCfg.perDepth * depth) / Math.max(semiSpan, 1e-3));
      const dihedral = (deg * Math.PI) / 180;
      if (!wing.keel || dihedral > wing.keel.dihedral) wing.keel = { normal: canonicalNormal(nk), point, dihedral };
      keel.underWing = true;
    }
  }
  // the effect is on the wing's lift as a whole: the parts of a folded wing (a W, turned-up tips)
  // that share its lifting system feel it too
  for (const s of mesh.surfaces) {
    if (!s.keel) continue;
    for (const pid of s.partners) {
      const o = mesh.surfaces[pid];
      if (o.keel || Math.abs(V.dot(o.normal, up)) < cos45) continue;
      o.keel = { ...s.keel };
    }
  }
}
