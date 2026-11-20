import * as V2 from '../math/vec2';
import * as V from '../math/vec3';
import * as P from '../math/polygon2d';
import * as M from '../math/mat3';
import {
  applyAffine,
  applyAffineLinear,
  composeAffine,
  invertOrthogonal,
  rigidToAffine,
  sameAffine,
} from '../math/transform';
import type { Affine } from '../math/transform';
import type { Vec2 } from '../math/vec2';
import type { Vec3 } from '../math/vec3';
import { facetArea } from './model';
import type { Facet, PaperModel } from './model';
import type { CutOp, FoldOp } from './ops';

/** Sheet mirror σ: reflection in the sheet's long centre line (v → −v). */
const SIGMA: Affine = { m: M.diag(1, 1, -1), t: [0, 0, 0] };
const sigma2 = (p: Vec2): Vec2 => [p[0], -p[1]];

/**
 * If the folded model is mirror-symmetric with respect to the sheet symmetry σ, return the
 * model-space isometry R that maps every facet onto its σ-twin: R = T_twin ∘ σ ∘ T_f⁻¹.
 * Returns null when the model is not symmetric (or has been cut asymmetrically).
 */
export function findSymmetry(model: PaperModel): Affine | null {
  const fs = model.facets;
  const twin = new Map<number, Facet>();
  for (const f of fs) {
    const c = sigma2(P.centroid(f.poly));
    const a = facetArea(f);
    const g = fs.find((h) => V2.dist(P.centroid(h.poly), c) < 1e-7 && Math.abs(facetArea(h) - a) < 1e-10);
    if (!g) return null;
    twin.set(f.id, g);
  }
  const ref = fs.find((f) => twin.get(f.id)!.id !== f.id) ?? fs[0];
  if (!ref) return null;
  const R = composeAffine(composeAffine(rigidToAffine(twin.get(ref.id)!.T), SIGMA), invertOrthogonal(rigidToAffine(ref.T)));
  for (const f of fs) {
    const expected = composeAffine(composeAffine(R, rigidToAffine(f.T)), SIGMA);
    if (!sameAffine(expected, rigidToAffine(twin.get(f.id)!.T), 1e-6)) return null;
  }
  if (!symmetricCuts(model)) return null;
  return R;
}

function symmetricCuts(model: PaperModel): boolean {
  return model.cuts.every((c) =>
    model.cuts.some(
      (d) =>
        (V2.almostEqual(d.a, sigma2(c.a), 1e-7) && V2.almostEqual(d.b, sigma2(c.b), 1e-7)) ||
        (V2.almostEqual(d.a, sigma2(c.b), 1e-7) && V2.almostEqual(d.b, sigma2(c.a), 1e-7)),
    ),
  );
}

const rp = (R: Affine, p: Vec3) => applyAffine(R, p);
const rv = (R: Affine, v: Vec3) => applyAffineLinear(R, v);

export function mirrorFold(op: FoldOp, R: Affine): FoldOp {
  return {
    ...op,
    id: `${op.id}'`,
    mirror: false,
    line: { p0: rp(R, op.line.p0), p1: rp(R, op.line.p1) },
    planeNormal: rv(R, op.planeNormal),
    movingSide: rv(R, op.movingSide),
    pick: op.pick ? rp(R, op.pick) : undefined,
  };
}

export function mirrorCut(op: CutOp, R: Affine): CutOp {
  return {
    ...op,
    id: `${op.id}'`,
    mirror: false,
    path: op.path.map((p) => rp(R, p)),
    planeNormal: rv(R, op.planeNormal),
    discard: op.discard?.map((d) => ({ flat: sigma2(d.flat) })),
  };
}

/** A mirrored fold that would act on the same paper from the same side is redundant/conflicting. */
export function foldIsSelfMirror(op: FoldOp, R: Affine): boolean {
  const tw = mirrorFold(op, R);
  const d = V.normalize(V.sub(op.line.p1, op.line.p0));
  const onLine = (p: Vec3) => V.distToLine(p, op.line.p0, d) < 1e-6;
  const sameLine = onLine(tw.line.p0) && onLine(tw.line.p1);
  const sameSide = V.dot(V.normalize(tw.planeNormal), V.normalize(op.planeNormal)) > 0.999;
  return sameLine && sameSide;
}

export function cutIsSelfMirror(op: CutOp, R: Affine): boolean {
  const tw = mirrorCut(op, R);
  return tw.path.every((p, i) => V.dist(p, op.path[i]) < 1e-6) || tw.path.every((p, i) => V.dist(p, op.path[op.path.length - 1 - i]) < 1e-6);
}
