import * as V from '../../core/math/vec3';
import type { Vec3 } from '../../core/math/vec3';
import { facetInPlane, facetVertices, toModel } from '../../core/paper/model';
import type { PaperModel } from '../../core/paper/model';
import { edgesOf } from '../../render/modelCache';

export type SnapKind = 'point' | 'edge' | 'angle' | 'free';

export interface SnapResult {
  point: Vec3;
  kind: SnapKind;
}

export interface Plane {
  point: Vec3;
  normal: Vec3;
}

const POINT_PX = 12;
const EDGE_PX = 8;
const ANGLE_STEP = (15 * Math.PI) / 180;

type ToScreen = (p: Vec3) => [number, number] | null;

/** Snap candidates for one model and working plane. */
export class Snapper {
  private points: Vec3[] = [];
  private segs: [Vec3, Vec3][] = [];

  constructor(
    model: PaperModel,
    readonly plane: Plane,
  ) {
    const inPlane = (p: Vec3) => Math.abs(V.dot(plane.normal, V.sub(p, plane.point))) < 1e-6;
    const pts: Vec3[] = [];
    for (const f of model.facets) {
      if (!facetInPlane(f, plane.point, plane.normal, 1e-6)) continue;
      pts.push(...facetVertices(f));
    }
    const byId = new Map(model.facets.map((f) => [f.id, f]));
    const g = edgesOf(model);
    const addSeg = (fid: number, a: [number, number], b: [number, number]) => {
      const f = byId.get(fid)!;
      const A = toModel(f, a);
      const B = toModel(f, b);
      if (!inPlane(A) || !inPlane(B)) return;
      this.segs.push([A, B]);
      pts.push(V.lerp(A, B, 0.5));
    };
    for (const b of g.borders) addSeg(b.facet, b.seg[0], b.seg[1]);
    for (const l of g.links) if (l.kind === 'crease') addSeg(l.a, l.seg[0], l.seg[1]);
    for (const c of model.cuts) {
      for (const f of model.facets) {
        // cut endpoints are useful anchors (e.g. folding exactly at the end of a slit)
        const A = toModel(f, c.a);
        const B = toModel(f, c.b);
        if (inPlane(A) && inPlane(B) && facetInPlane(f, plane.point, plane.normal, 1e-6)) {
          pts.push(A, B);
          break;
        }
      }
    }
    // dedupe
    for (const p of pts) if (!this.points.some((q) => V.dist(p, q) < 1e-6)) this.points.push(p);
  }

  snap(raw: Vec3, mouse: [number, number], toScreen: ToScreen, anchor: Vec3 | null, disabled: boolean): SnapResult {
    if (disabled) return { point: raw, kind: 'free' };
    let best: Vec3 | null = null;
    let bestD = POINT_PX;
    for (const p of this.points) {
      const s = toScreen(p);
      if (!s) continue;
      const d = Math.hypot(s[0] - mouse[0], s[1] - mouse[1]);
      if (d < bestD && (!anchor || V.dist(p, anchor) > 1e-5)) {
        bestD = d;
        best = p;
      }
    }
    if (best) return { point: best, kind: 'point' };

    // angle snap relative to the model axes in the working plane
    let angled: Vec3 | null = null;
    if (anchor) {
      const n = this.plane.normal;
      let e1 = V.rejectFrom([1, 0, 0], n);
      if (V.length(e1) < 0.3) e1 = V.rejectFrom([0, 0, 1], n);
      e1 = V.normalize(e1);
      const e2 = V.cross(n, e1);
      const d = V.sub(raw, anchor);
      const len = V.length(d);
      if (len > 1e-4) {
        const ang = Math.atan2(V.dot(d, e2), V.dot(d, e1));
        const sa = Math.round(ang / ANGLE_STEP) * ANGLE_STEP;
        const dir = V.add(V.scale(e1, Math.cos(sa)), V.scale(e2, Math.sin(sa)));
        const cand = V.addScaled(anchor, dir, len);
        const sc = toScreen(cand);
        if (sc && Math.hypot(sc[0] - mouse[0], sc[1] - mouse[1]) < 14) angled = cand;
        // a point on an edge along the snapped angle is best of all
        if (angled) {
          for (const [A, B] of this.segs) {
            const hit = intersectLines(anchor, dir, A, B, n);
            if (!hit) continue;
            const hs = toScreen(hit);
            if (hs && Math.hypot(hs[0] - mouse[0], hs[1] - mouse[1]) < EDGE_PX) return { point: hit, kind: 'point' };
          }
        }
      }
    }

    let edgeBest: Vec3 | null = null;
    let edgeD = EDGE_PX;
    for (const [A, B] of this.segs) {
      const sa = toScreen(A);
      const sb = toScreen(B);
      if (!sa || !sb) continue;
      const dx = sb[0] - sa[0];
      const dy = sb[1] - sa[1];
      const l2 = dx * dx + dy * dy;
      if (l2 < 1) continue;
      const t = Math.max(0, Math.min(1, ((mouse[0] - sa[0]) * dx + (mouse[1] - sa[1]) * dy) / l2));
      const d = Math.hypot(sa[0] + dx * t - mouse[0], sa[1] + dy * t - mouse[1]);
      if (d < edgeD) {
        edgeD = d;
        edgeBest = V.lerp(A, B, t);
      }
    }
    if (edgeBest) return { point: edgeBest, kind: 'edge' };
    if (angled) return { point: angled, kind: 'angle' };
    return { point: raw, kind: 'free' };
  }
}

/** Intersection of ray (o + s·d) with segment AB, all in a plane with normal n. */
function intersectLines(o: Vec3, d: Vec3, A: Vec3, B: Vec3, n: Vec3): Vec3 | null {
  const e = V.sub(B, A);
  const denom = V.dot(V.cross(d, e), n);
  if (Math.abs(denom) < 1e-12) return null;
  const t = V.dot(V.cross(V.sub(A, o), e), n) / denom;
  const u = V.dot(V.cross(V.sub(A, o), d), n) / denom;
  if (u < -1e-9 || u > 1 + 1e-9 || t < 1e-5) return null;
  return V.addScaled(o, d, t);
}
