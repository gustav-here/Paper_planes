import * as V from '../math/vec3';
import type { Vec3 } from '../math/vec3';
import { facetArea, facetCentroid, toModel } from '../paper/model';
import type { PaperModel } from '../paper/model';
import { computeEdges, isConnected } from '../paper/edges';
import type { Op } from '../paper/ops';

/**
 * A control flap: the paper moved by a fold marked `control`, hinged on that fold's crease.
 * Its response to the stick follows from where it sits: behind the CoM it acts as an elevator,
 * ahead of it as a canard, off-centre as an aileron (so mirrored pairs become elevons).
 */
export interface ControlGroup {
  /** Fold op id (a mirrored twin has the id followed by '). */
  id: string;
  facets: number[];
  /** Aero surfaces belonging to the flap (filled by the airframe). */
  surfaces: number[];
  /** Hinge point and unit axis (body frame); +deflection moves the flap towards body "up". */
  hingeP: Vec3;
  hingeA: Vec3;
  /** Area centroid (body frame). */
  centroid: Vec3;
  /** +1 behind the CoM (elevator), −1 ahead of it (canard), 0 at the CoM. */
  pitchSign: number;
  /** +1 on the right wing, −1 on the left, 0 on the centre line. */
  rollSign: number;
}

/** Facet → control group index, from the fold ops marked as control. */
export function controlFacetTags(model: PaperModel, ops: Op[]): { tags: Map<number, number>; ids: string[] } {
  const ids: string[] = [];
  for (const op of ops) {
    if (op.type !== 'fold' || !op.control) continue;
    ids.push(op.id);
    if (op.mirror) ids.push(`${op.id}'`);
  }
  const tags = new Map<number, number>();
  if (ids.length === 0) return { tags, ids };
  for (const f of model.facets) {
    // the most recent control fold that moved this facet owns it
    for (let k = f.foldPath.length - 1; k >= 0; k--) {
      const g = ids.indexOf(f.foldPath[k]);
      if (g >= 0) {
        tags.set(f.id, g);
        break;
      }
    }
  }
  return { tags, ids };
}

const SIDE_TOL = 0.005;

export function buildControlGroups(
  model: PaperModel,
  com: Vec3,
  tags: Map<number, number>,
  ids: string[],
  axes: { nose: Vec3; up: Vec3; right: Vec3 },
): ControlGroup[] {
  const groups: ControlGroup[] = [];
  const graph = computeEdges(model);
  const byId = new Map(model.facets.map((f) => [f.id, f]));
  ids.forEach((id, g) => {
    const facets = model.facets.filter((f) => tags.get(f.id) === g);
    if (facets.length === 0) return;
    // hinge: connected crease/seam segments between the flap and the rest of the paper
    const pts: Vec3[] = [];
    for (const l of graph.links) {
      if (!isConnected(l)) continue;
      const ina = tags.get(l.a) === g;
      const inb = tags.get(l.b) === g;
      if (ina === inb) continue;
      const f = byId.get(l.a)!;
      pts.push(toModel(f, l.seg[0]), toModel(f, l.seg[1]));
    }
    if (pts.length < 2) return;
    let p: Vec3 = pts[0];
    let q: Vec3 = pts[1];
    let best = -1;
    for (let i = 0; i < pts.length; i++)
      for (let j = i + 1; j < pts.length; j++) {
        const d = V.dist(pts[i], pts[j]);
        if (d > best) {
          best = d;
          p = pts[i];
          q = pts[j];
        }
      }
    if (best < 1e-4) return;
    let A = 0;
    let c: Vec3 = [0, 0, 0];
    for (const f of facets) {
      const a = facetArea(f);
      A += a;
      c = V.addScaled(c, facetCentroid(f), a);
    }
    const centroid = V.sub(V.scale(c, 1 / A), com);
    const hingeP = V.sub(p, com);
    let hingeA = V.normalize(V.sub(q, p));
    if (V.dot(V.cross(hingeA, V.sub(centroid, hingeP)), axes.up) < 0) hingeA = V.negate(hingeA);
    const x = V.dot(centroid, axes.nose);
    const y = V.dot(centroid, axes.right);
    groups.push({
      id,
      facets: facets.map((f) => f.id),
      surfaces: [],
      hingeP,
      hingeA,
      centroid,
      pitchSign: x < -SIDE_TOL ? 1 : x > SIDE_TOL ? -1 : 0,
      rollSign: y > SIDE_TOL ? 1 : y < -SIDE_TOL ? -1 : 0,
    });
  });
  return groups;
}
