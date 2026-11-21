import * as V from '../math/vec3';
import * as P from '../math/polygon2d';
import { coplanar, createSheet, facetVertices, toSheet3 } from './model';
import type { Facet } from './model';
import * as V2 from '../math/vec2';
import { firstContactSample } from './penetration';
import type { PaperModel } from './model';
import { applyFold } from './fold';
import type { FoldOptions } from './fold';
import { applyCut } from './cut';
import type { CutPiece } from './cut';
import { clipStillAttached, resolveClip } from './clips';
import { PaperError } from './errors';
import type { PaperWarning } from './errors';
import { cutIsSelfMirror, findSymmetry, foldIsSelfMirror, mirrorCut, mirrorFold } from './symmetry';
import type { DesignFile, FoldOp, Op } from './ops';

export interface OpResult {
  model: PaperModel;
  warnings: PaperWarning[];
  /** For cuts that detached pieces. */
  pieces?: CutPiece[];
  /** Whether a mirror twin was applied. */
  mirrored?: boolean;
}

/** Apply one design operation (including its mirror twin) to a model. Throws PaperError. */
export function applyOp(model: PaperModel, op: Op, opts: FoldOptions = {}): OpResult {
  switch (op.type) {
    case 'fold': {
      const warnings: PaperWarning[] = [];
      const R = op.mirror ? findSymmetry(model) : null;
      if (op.mirror && !R) warnings.push({ code: 'mirrorUnavailable' });
      const first = applyFold(model, op, opts);
      warnings.push(...first.warnings);
      if (!R) return { model: first.model, warnings, mirrored: false };
      if (foldIsSelfMirror(op, R)) {
        warnings.push({ code: 'mirrorOnAxis' });
        return { model: first.model, warnings, mirrored: false };
      }
      const second = applyFold(first.model, mirrorFold(op, R), opts);
      for (const w of second.warnings) if (!warnings.some((x) => x.code === w.code)) warnings.push(w);
      return { model: second.model, warnings, mirrored: true };
    }
    case 'cut': {
      const warnings: PaperWarning[] = [];
      const R = op.mirror ? findSymmetry(model) : null;
      if (op.mirror && !R) warnings.push({ code: 'mirrorUnavailable' });
      const twin = R && !cutIsSelfMirror(op, R) ? mirrorCut(op, R) : null;
      const first = applyCut(model, op);
      if (!twin) return { model: first.model, warnings: [...warnings, ...first.warnings], pieces: first.pieces };
      const second = applyCut(first.model, twin);
      return {
        model: second.model,
        warnings: [...warnings, ...first.warnings, ...second.warnings],
        pieces: second.pieces.length ? second.pieces : first.pieces,
        mirrored: true,
      };
    }
    case 'clip': {
      const clip = { id: op.id, at: op.at, edgeDir: op.edgeDir, massKg: op.massKg, ...(op.kind ? { kind: op.kind } : {}), ...(op.bridge ? { bridge: true } : {}), ...(op.lean ? { lean: op.lean } : {}) };
      if (!clipStillAttached(model, clip)) throw new PaperError('clipNotOnEdge');
      // a clip leans as it goes on the plane as folded; that lean stays with it
      if (op.kind !== 'tape') {
        const rc = resolveClip({ ...model, clips: [...model.clips, clip] }, clip);
        if (rc?.tiltDeg) return { model: { ...model, clips: [...model.clips, { ...clip, tiltDeg: rc.tiltDeg }] }, warnings: [] };
      }
      return { model: { ...model, clips: [...model.clips, clip] }, warnings: [] };
    }
    case 'unclip': {
      if (!model.clips.some((c) => c.id === op.target)) throw new PaperError('unknownClip');
      return { model: { ...model, clips: model.clips.filter((c) => c.id !== op.target) }, warnings: [] };
    }
  }
}

/**
 * Largest angle (deg, up to 180) a fold can reach before paper would pass through paper,
 * for the fold and (if mirrored) its twin. 0 = blocked from the start. Throws PaperError
 * if the fold itself is invalid.
 */
export function foldAngleLimit(model: PaperModel, op: FoldOp): number {
  const full = { ...op, angleDeg: 180 };
  let limit = 180;
  const first = applyFold(model, full, { penetration: 'scan' });
  if (first.contactAngleDeg != null) limit = Math.min(limit, first.contactAngleDeg - 1);
  const R = op.mirror ? findSymmetry(model) : null;
  if (R && !foldIsSelfMirror(op, R)) {
    // the twin is checked against the unfolded state (the two halves rarely meet)
    const twin = applyFold(model, mirrorFold(full, R), { penetration: 'scan' });
    if (twin.contactAngleDeg != null) limit = Math.min(limit, twin.contactAngleDeg - 1);
  }
  return Math.max(0, limit);
}

/**
 * Can this fold go all the way over (180°) by pushing the paper through a seam? Some paper is in
 * the way on the swing (a keel tab swung sideways meets the underside of the wing), but where it
 * ends up nothing is pierced, and on the way it passes a line where two pieces of paper meet at
 * an angle (the wing roots on top of the keel): real paper parts there for a moment and closes
 * again behind it. That is how a tab cut in the keel is pushed up between the wings as a fin.
 * The fold and (if mirrored) its twin must both qualify.
 */
export function canPushThrough(model: PaperModel, op: FoldOp): boolean {
  const ops: FoldOp[] = [{ ...op, angleDeg: 180 }];
  const R = op.mirror ? findSymmetry(model) : null;
  if (R && !foldIsSelfMirror(op, R)) ops.push(mirrorFold(ops[0], R));
  return ops.every((full) => {
    let scan;
    try {
      scan = applyFold(model, full, { penetration: 'scan' });
    } catch {
      return false;
    }
    // nothing in the way: an ordinary fold
    if (scan.contactAngleDeg == null) return false;
    const moved = new Set(scan.model.facets.filter((f) => f.foldPath.at(-1) === full.id).map((f) => f.id));
    if (!moved.size) return false;
    const mov = scan.model.facets.filter((f) => moved.has(f.id));
    const stat = scan.model.facets.filter((f) => !moved.has(f.id));
    const p = full.line.p0;
    const d = V.normalize(V.sub(full.line.p1, full.line.p0));
    // where it ends up, it pierces nothing
    if (firstContactSample(stat, mov, p, d, 0, 1) !== null) return false;
    // and it passes between two pieces meeting at an angle: both have an edge along a line that
    // runs through it (or along its hinge, for a tab cut right up to the seam)
    for (const m of mov) {
      const cands: { f: Facet; a: [number, number]; b: [number, number] }[] = [];
      for (const f of stat) {
        const vs = facetVertices(f);
        vs.forEach((A, i) => {
          const B = vs[(i + 1) % vs.length];
          const a = toSheet3(m, A);
          const b = toSheet3(m, B);
          if (Math.abs(a[1]) > 1e-6 || Math.abs(b[1]) > 1e-6) return;
          const r = P.clipSegmentConvex([a[0], a[2]], [b[0], b[2]], m.poly);
          if (r && (r[1] - r[0]) * V.dist(A, B) > 1e-4) cands.push({ f, a: [a[0], a[2]], b: [b[0], b[2]] });
        });
      }
      for (let i = 0; i < cands.length; i++)
        for (let j = i + 1; j < cands.length; j++) {
          const c1 = cands[i];
          const c2 = cands[j];
          if (c1.f.id === c2.f.id || coplanar(c1.f, c2.f)) continue;
          // on one line, overlapping
          const u = V2.normalize(V2.sub(c1.b, c1.a));
          if (Math.abs(V2.cross(u, V2.sub(c2.a, c1.a))) > 1e-6 || Math.abs(V2.cross(u, V2.sub(c2.b, c1.a))) > 1e-6) continue;
          const t1 = [0, V2.dot(V2.sub(c1.b, c1.a), u)].sort((x, y) => x - y);
          const t2 = [V2.dot(V2.sub(c2.a, c1.a), u), V2.dot(V2.sub(c2.b, c1.a), u)].sort((x, y) => x - y);
          if (Math.min(t1[1], t2[1]) - Math.max(t1[0], t2[0]) > 1e-4) return true;
        }
    }
    return false;
  });
}

export interface ReplayResult {
  /** models[0] is the flat sheet, models[i] the state after ops[i-1]. */
  models: PaperModel[];
  warnings: PaperWarning[][];
}

/** Deterministically replay a design. Throws PaperError (with op index in the message) on failure. */
export function replayDesign(design: DesignFile, opts: FoldOptions = {}): ReplayResult {
  const models: PaperModel[] = [createSheet(design.paper)];
  const warnings: PaperWarning[][] = [];
  design.ops.forEach((op, i) => {
    try {
      const r = applyOp(models[models.length - 1], op, opts);
      models.push(r.model);
      warnings.push(r.warnings);
    } catch (e) {
      if (e instanceof PaperError) throw new PaperError(e.code, `op ${i} (${op.id}): ${e.message}`);
      throw e;
    }
  });
  return { models, warnings };
}
