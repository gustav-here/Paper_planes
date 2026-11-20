import { describe, expect, it } from 'vitest';
import { computeMassProperties } from '../../../src/core/physics/massProperties';
import { createSheet } from '../../../src/core/paper/model';
import { A4_80GSM } from '../../../src/core/paper/paperSpec';
import { replayDesign, applyOp } from '../../../src/core/paper/replay';
import { findClipPlacement } from '../../../src/core/paper/clips';
import { makeDart, makeGlider } from '../../../src/core/presets';

const M = 0.08 * 0.21 * 0.297; // 4.99 g

describe('mass properties', () => {
  it('flat A4: 4.99 g, centred, analytic plate inertia', () => {
    const mp = computeMassProperties(createSheet(A4_80GSM));
    expect(mp.mass).toBeCloseTo(M, 12);
    expect(mp.mass * 1000).toBeCloseTo(4.99, 2);
    for (const c of mp.com) expect(Math.abs(c)).toBeLessThan(1e-12);
    const a = 0.297; // along x
    const b = 0.21; // along z
    expect(mp.inertia[0]).toBeCloseTo((M * b * b) / 12, 12); // about x
    expect(mp.inertia[4]).toBeCloseTo((M * (a * a + b * b)) / 12, 12); // about y (normal)
    expect(mp.inertia[8]).toBeCloseTo((M * a * a) / 12, 12); // about z
    for (const k of [1, 2, 3, 5, 6, 7]) expect(Math.abs(mp.inertia[k])).toBeLessThan(1e-14);
  });

  it('folding conserves mass; symmetric models keep the centre of mass in the symmetry plane', () => {
    for (const d of [makeDart(), makeGlider()]) {
      const m = replayDesign(d).models.at(-1)!;
      const mp = computeMassProperties(m);
      expect(mp.mass).toBeCloseTo(M, 12);
      expect(Math.abs(mp.com[1])).toBeLessThan(1e-12); // folded models are mirror-symmetric in y
      // inertia tensor is symmetric positive definite
      expect(mp.inertia[1]).toBeCloseTo(mp.inertia[3], 14);
      expect(mp.inertia[0]).toBeGreaterThan(0);
      expect(mp.inertia[4]).toBeGreaterThan(0);
      expect(mp.inertia[8]).toBeGreaterThan(0);
      // triangle inequality for principal moments holds for the diagonal too
      expect(mp.inertia[0] + mp.inertia[4]).toBeGreaterThanOrEqual(mp.inertia[8] - 1e-15);
    }
  });

  it('a 1 g clip adds mass and pulls the centre of mass towards it', () => {
    const m = replayDesign(makeDart()).models.at(-1)!;
    const before = computeMassProperties(m);
    const p = findClipPlacement(m, [0.1, 0, 0], 0.01)!;
    const withClip = applyOp(m, { type: 'clip', id: 'c', at: p.at, edgeDir: p.edgeDir, massKg: 0.001 }).model;
    const after = computeMassProperties(withClip);
    expect(after.mass).toBeCloseTo(M + 0.001, 12);
    expect(after.clipMass).toBeCloseTo(0.001, 12);
    expect(after.com[0]).toBeGreaterThan(before.com[0]);
  });

  it('cutting a piece off removes exactly its mass', () => {
    const r = applyOp(createSheet(A4_80GSM), {
      type: 'cut',
      id: 'k',
      path: [
        [0.1, 0, -0.2],
        [0.1, 0, 0.2],
      ],
      planeNormal: [0, 1, 0],
      depth: 'all',
    });
    expect(computeMassProperties(r.model).mass).toBeCloseTo(0.08 * 0.21 * (0.1485 + 0.1), 12);
  });
});
