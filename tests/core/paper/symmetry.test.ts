import { describe, expect, it } from 'vitest';
import * as M from '../../../src/core/math/mat3';
import { createSheet, facetVertices, totalArea } from '../../../src/core/paper/model';
import { A4_80GSM } from '../../../src/core/paper/paperSpec';
import { findSymmetry } from '../../../src/core/paper/symmetry';
import { applyOp } from '../../../src/core/paper/replay';
import { CENTER_LINE, cut, fold } from './helpers';

const sheet = () => createSheet(A4_80GSM);
const a = 0.1485;
const b = 0.105;
const cornerFold = (mirror: boolean) =>
  fold([a, 0, 0], [a - b, 0, b], [1, 0, 1], { mirror });

describe('symmetry', () => {
  it('flat sheet is symmetric under z-reflection', () => {
    const R = findSymmetry(sheet())!;
    expect(R).not.toBeNull();
    expect(M.almostEqual(R.m, M.diag(1, 1, -1), 1e-9)).toBe(true);
  });

  it('mirrored corner fold folds both corners and keeps symmetry', () => {
    const r = applyOp(sheet(), cornerFold(true));
    expect(r.mirrored).toBe(true);
    expect(findSymmetry(r.model)).not.toBeNull();
    // both nose corners now lie on the centre line
    const vs = r.model.facets.flatMap(facetVertices);
    expect(vs.some((v) => Math.abs(v[0] - (a - b)) < 1e-9 && Math.abs(v[2]) < 1e-9)).toBe(true);
    expect(Math.max(...vs.map((v) => Math.abs(v[2])))).toBeCloseTo(b, 9);
  });

  it('a single corner fold breaks symmetry', () => {
    const r = applyOp(sheet(), cornerFold(false));
    expect(findSymmetry(r.model)).toBeNull();
  });

  it('after folding in half the twin of a top-layer fold is the bottom layer (other wing)', () => {
    let m = applyOp(sheet(), fold(CENTER_LINE[0], CENTER_LINE[1], [0, 0, -1], { kind: 'mountain' })).model;
    const R = findSymmetry(m)!;
    expect(R).not.toBeNull();
    expect(M.det(R.m)).toBeCloseTo(-1, 9);
    const wing = fold([-0.2, 0, 0.02], [0.2, 0, 0.02], [0, 0, 1], { scope: 'top', angleDeg: 90, mirror: true });
    const r = applyOp(m, wing);
    expect(r.mirrored).toBe(true);
    m = r.model;
    const ys = m.facets.flatMap(facetVertices).map((v) => v[1]);
    expect(Math.max(...ys)).toBeCloseTo(0.085, 6);
    expect(Math.min(...ys)).toBeCloseTo(-0.085, 6);
    expect(totalArea(m)).toBeCloseTo(0.21 * 0.297, 10);
    expect(findSymmetry(m)).not.toBeNull();
  });

  it('mirrored cuts', () => {
    const r = applyOp(sheet(), cut([
      [-0.16, 0, 0.05],
      [-0.12, 0, 0.05],
    ], { mirror: true }));
    expect(r.mirrored).toBe(true);
    expect(r.model.cuts).toHaveLength(2);
    expect(findSymmetry(r.model)).not.toBeNull();
  });

  it('mirror on the symmetry axis is skipped with a warning', () => {
    const r = applyOp(sheet(), fold(CENTER_LINE[0], CENTER_LINE[1], [0, 0, 1], { mirror: true }));
    expect(r.mirrored).toBe(false);
    expect(r.warnings.map((w) => w.code)).toContain('mirrorOnAxis');
  });
});
