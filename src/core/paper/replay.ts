import { createSheet } from './model';
import type { PaperModel } from './model';
import { applyFold } from './fold';
import type { FoldOptions } from './fold';
import { applyCut } from './cut';
import type { CutPiece } from './cut';
import { clipStillAttached } from './clips';
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
      const clip = { id: op.id, at: op.at, edgeDir: op.edgeDir, massKg: op.massKg, ...(op.kind ? { kind: op.kind } : {}) };
      if (!clipStillAttached(model, clip)) throw new PaperError('clipNotOnEdge');
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
