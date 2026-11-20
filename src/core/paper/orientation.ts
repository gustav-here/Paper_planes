import * as V from '../math/vec3';
import * as P from '../math/polygon2d';
import type { Vec2 } from '../math/vec2';
import type { Vec3 } from '../math/vec3';
import { facetArea, facetCentroid, toModel } from './model';
import type { PaperModel } from './model';
import { findSymmetry } from './symmetry';

export interface Orientation {
  nose: Vec3;
  up: Vec3;
}

export interface DetectedOrientation extends Orientation {
  /** False when the model is not symmetric: the fallback was returned unchanged. */
  detected: boolean;
}

/** Points of the sheet's long centre line (v = 0) in model space: the spine / centre fold. */
export function spinePoints(model: PaperModel, samples = 41): Vec3[] {
  const a = model.spec.heightM / 2;
  const out: Vec3[] = [];
  for (let i = 0; i < samples; i++) {
    const p: Vec2 = [-a + (2 * a * i) / (samples - 1), 0];
    const f = model.facets.find((g) => P.pointInConvex(p, g.poly, -1e-9));
    if (f) out.push(toModel(f, p));
  }
  return out;
}

/**
 * Body axes of a folded plane, found from its geometry:
 * - the mirror symmetry of the sheet gives the plane of symmetry; its normal is the wing axis,
 * - the nose is the preferred nose direction (default: the sheet's top edge, +X) projected into
 *   that plane,
 * - "up" is perpendicular to both, pointing from the centre fold (the sheet's centre line, i.e.
 *   the keel/spine) towards the centre of the paper — so the centre fold ends up underneath.
 * For a flat or otherwise ambiguous model the preferred up direction decides the sign.
 */
export function detectOrientation(model: PaperModel, preferred: Orientation): DetectedOrientation {
  const R = findSymmetry(model);
  if (!R) return { ...preferred, detected: false };
  // R is a reflection: (I − M)/2 = n nᵀ, n = normal of the symmetry plane
  const M = R.m;
  const cols: Vec3[] = [0, 1, 2].map((j) => [
    ((j === 0 ? 1 : 0) - M[j]) / 2,
    ((j === 1 ? 1 : 0) - M[3 + j]) / 2,
    ((j === 2 ? 1 : 0) - M[6 + j]) / 2,
  ]);
  const best = cols.reduce((a, b) => (V.length(b) > V.length(a) ? b : a));
  if (V.length(best) < 0.5) return { ...preferred, detected: false };
  const right = V.normalize(best);

  let nose = V.rejectFrom(preferred.nose, right);
  if (V.length(nose) < 0.3) nose = V.rejectFrom([1, 0, 0], right);
  if (V.length(nose) < 0.3) nose = V.rejectFrom([0, 0, 1], right);
  nose = V.normalize(nose);
  let up = V.normalize(V.cross(right, nose));

  // centre of the paper (area-weighted over all layers) vs. the centre fold
  let A = 0;
  let c: Vec3 = [0, 0, 0];
  for (const f of model.facets) {
    const a = facetArea(f);
    A += a;
    c = V.addScaled(c, facetCentroid(f), a);
  }
  c = V.scale(c, 1 / A);
  const spine = spinePoints(model);
  const s = spine.length ? V.dot(up, V.sub(c, V.centroid(spine))) : 0;
  if (Math.abs(s) > 1e-4) {
    if (s < 0) up = V.negate(up);
  } else if (V.dot(up, preferred.up) < 0) {
    up = V.negate(up);
  }
  return { nose, up, detected: true };
}
