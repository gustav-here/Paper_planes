import * as V from '../math/vec3';
import * as P from '../math/polygon2d';
import type { Vec3 } from '../math/vec3';
import { applyRigid, applyRigidLinear, composeRigid, invertRigid } from '../math/transform';
import type { RigidTransform } from '../math/transform';
import { createSheet, facetContains, toSheet } from './model';
import type { PaperModel } from './model';
import type { CutOp, DesignFile, FoldOp, Op } from './ops';
import { applyOp } from './replay';
import { cutIsSelfMirror, findSymmetry, foldIsSelfMirror, mirrorCut, mirrorFold } from './symmetry';

/**
 * Nobody folds exactly along the line. A folded plane is crooked in its own way, the same in
 * every throw: each fold line (and each mirror twin on its own) is turned a little in its plane
 * about its middle, normally distributed with standard deviation `sigmaDeg` (cut off at 2.5σ).
 * A wing root folded 1° off gives that wing about 1° more or less incidence than the other, and
 * a plane with flat wings has almost nothing to right itself against that.
 *
 * Deterministic: the turn of a fold depends only on `seed` and the fold's id (a mirror twin's id
 * ends in ').
 */
export function crookedFolds(seed: number, sigmaDeg: number): (op: FoldOp) => FoldOp {
  return (op) => {
    const g = normal(hash(`${seed}:${op.id}`));
    const ang = (Math.max(-2.5, Math.min(2.5, g)) * sigmaDeg * Math.PI) / 180;
    if (ang === 0) return op;
    const n = V.normalize(op.planeNormal);
    const mid = V.lerp(op.line.p0, op.line.p1, 0.5);
    const turn = (p: Vec3) => V.add(mid, rotate(V.sub(p, mid), n, ang));
    return { ...op, line: { p0: turn(op.line.p0), p1: turn(op.line.p1) } };
  };
}

/**
 * The design folded by a hand that is a little off (crookedFolds), or null if it cannot be made
 * that way. Fold and cut lines are given in the space of the exactly folded paper, so each step
 * is carried along with the paper it lies on: the exact and the crooked plane are folded side by
 * side, and every step is moved as the piece of paper under it has moved. Mirrored steps are
 * made as two separate, separately crooked steps (the crooked plane has no mirror symmetry).
 */
export function replayCrooked(design: DesignFile, seed: number, sigmaDeg: number): PaperModel | null {
  const skew = crookedFolds(seed, sigmaDeg);
  let exact = createSheet(design.paper);
  let crooked = exact;
  try {
    for (const op of design.ops) {
      if (op.type === 'fold') {
        const R = op.mirror ? findSymmetry(exact) : null;
        const steps: FoldOp[] = [{ ...op, mirror: false }];
        if (R && !foldIsSelfMirror(op, R)) steps.push(mirrorFold(op, R));
        let next = crooked;
        for (const step of steps) {
          const moved = carryFold(step, exact, crooked);
          next = foldEither(next, skew(moved), moved);
        }
        crooked = next;
      } else if (op.type === 'cut') {
        const R = op.mirror ? findSymmetry(exact) : null;
        const steps: CutOp[] = [{ ...op, mirror: false }];
        if (R && !cutIsSelfMirror(op, R)) steps.push(mirrorCut(op, R));
        let next = crooked;
        for (const step of steps) next = applyOp(next, carryCut(step, exact, crooked)).model;
        crooked = next;
      } else {
        // clips are placed in sheet coordinates: nothing to carry
        crooked = applyOp(crooked, op).model;
      }
      exact = applyOp(exact, op).model;
    }
  } catch {
    return null;
  }
  return crooked;
}

/** The crooked fold, or the fold exactly where the paper now is if the crooked one cannot be made. */
function foldEither(model: PaperModel, crooked: FoldOp, straight: FoldOp): PaperModel {
  try {
    return applyOp(model, crooked).model;
  } catch {
    return applyOp(model, straight).model;
  }
}

/** How the paper at the first on-paper point of `pts` has moved from the exact to the crooked plane. */
function paperMotion(pts: Vec3[], exact: PaperModel, crooked: PaperModel): RigidTransform | null {
  const samples: Vec3[] = [];
  for (let i = 0; i + 1 < pts.length; i++)
    for (let k = 0; k <= 20; k++) samples.push(V.lerp(pts[i], pts[i + 1], k / 20));
  // the middle of the stretch that lies on paper
  const on = samples.map((p) => exact.facets.find((f) => facetContains(f, p, 0, 1e-4)) ?? null);
  const idx = on.map((f, i) => (f ? i : -1)).filter((i) => i >= 0);
  if (!idx.length) return null;
  const i = idx[Math.floor(idx.length / 2)];
  const fe = on[i]!;
  const s = toSheet(fe, samples[i]);
  const fc = crooked.facets.find((f) => P.pointInConvex(s, f.poly, -1e-9));
  if (!fc) return null;
  return composeRigid(fc.T, invertRigid(fe.T));
}

function carryFold(op: FoldOp, exact: PaperModel, crooked: PaperModel): FoldOp {
  const D = paperMotion(op.pick ? [op.pick, op.pick] : [op.line.p0, op.line.p1], exact, crooked) ?? paperMotion([op.line.p0, op.line.p1], exact, crooked);
  if (!D) return op;
  return {
    ...op,
    line: { p0: applyRigid(D, op.line.p0), p1: applyRigid(D, op.line.p1) },
    planeNormal: applyRigidLinear(D, op.planeNormal),
    movingSide: applyRigidLinear(D, op.movingSide),
    ...(op.pick ? { pick: applyRigid(D, op.pick) } : {}),
  };
}

function carryCut(op: CutOp, exact: PaperModel, crooked: PaperModel): CutOp {
  const D = paperMotion(op.path, exact, crooked);
  if (!D) return op;
  return { ...op, path: op.path.map((p) => applyRigid(D, p)), planeNormal: applyRigidLinear(D, op.planeNormal) };
}

/** A seed for a design's crookedness from its operations (the same plane is crooked the same way). */
export function designSeed(ops: Op[]): number {
  return hash(JSON.stringify(ops));
}

/** Rodrigues rotation of v about unit axis k. */
function rotate(v: Vec3, k: Vec3, a: number): Vec3 {
  const c = Math.cos(a);
  const s = Math.sin(a);
  return V.add(V.add(V.scale(v, c), V.scale(V.cross(k, v), s)), V.scale(k, V.dot(k, v) * (1 - c)));
}

/** 32-bit FNV-1a hash of a string. */
function hash(s: string): number {
  let h = 0x811c9dc5;
  for (let i = 0; i < s.length; i++) {
    h ^= s.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h >>> 0;
}

/** A standard normal number from a 32-bit seed (mulberry32 + Box–Muller). */
function normal(seed: number): number {
  let s = seed >>> 0;
  const uni = () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return (((t ^ (t >>> 14)) >>> 0) + 0.5) / 4294967296;
  };
  return Math.sqrt(-2 * Math.log(uni())) * Math.cos(2 * Math.PI * uni());
}
