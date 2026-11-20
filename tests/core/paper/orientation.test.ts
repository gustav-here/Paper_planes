import { describe, expect, it } from 'vitest';
import * as V from '../../../src/core/math/vec3';
import { detectOrientation } from '../../../src/core/paper/orientation';
import { createSheet } from '../../../src/core/paper/model';
import { A4_80GSM } from '../../../src/core/paper/paperSpec';
import { replayDesign, applyOp } from '../../../src/core/paper/replay';
import { makeDart, makeGlider } from '../../../src/core/presets';
import type { DesignFile, FoldOp } from '../../../src/core/paper/ops';

const DEFAULT = { nose: [1, 0, 0] as V.Vec3, up: [0, 1, 0] as V.Vec3 };
const last = (d: DesignFile) => replayDesign(d).models.at(-1)!;
const close = (a: V.Vec3, b: V.Vec3) => expect(V.dist(a, b)).toBeLessThan(1e-6);

describe('automatic orientation (centre fold down)', () => {
  it('flat sheet: nose +X, front side up', () => {
    const o = detectOrientation(createSheet(A4_80GSM), DEFAULT);
    expect(o.detected).toBe(true);
    close(o.nose, [1, 0, 0]);
    close(o.up, [0, 1, 0]);
  });

  it('dart and glider: keel down → up = +Z, nose +X (matches the presets)', () => {
    for (const d of [makeDart(), makeGlider()]) {
      const o = detectOrientation(last(d), DEFAULT);
      expect(o.detected).toBe(true);
      close(o.nose, [1, 0, 0]);
      close(o.up, [0, 0, 1]);
    }
  });

  it('works whatever way the plane was folded in half (valley or mountain, either half moving)', () => {
    for (const kind of ['valley', 'mountain'] as const) {
      for (const side of [1, -1]) {
        const half: FoldOp = {
          type: 'fold',
          id: 'h',
          line: { p0: [-0.2, 0, 0], p1: [0.2, 0, 0] },
          planeNormal: [0, 1, 0],
          movingSide: [0, 0, side],
          kind,
          angleDeg: 180,
          scope: 'all',
        };
        const m = applyOp(createSheet(A4_80GSM), half).model;
        // wings: fold top/bottom flaps 80° so the plane has a keel
        const wings: FoldOp = {
          ...half,
          id: 'w',
          line: { p0: [-0.2, 0, -side * 0.03], p1: [0.2, 0, -side * 0.03] },
          movingSide: [0, 0, -side],
          kind: 'valley',
          angleDeg: 80,
          scope: 'top',
          mirror: true,
        };
        const plane = applyOp(m, wings).model;
        const o = detectOrientation(plane, DEFAULT);
        expect(o.detected).toBe(true);
        // the keel (spine at z = 0) must be below: up points from z = 0 towards the wings
        expect(Math.sign(o.up[2])).toBe(-side);
        expect(Math.abs(o.up[2])).toBeGreaterThan(0.99);
      }
    }
  });

  it('asymmetric model: falls back to the preferred orientation', () => {
    const corner: FoldOp = {
      type: 'fold',
      id: 'c',
      line: { p0: [0.1485, 0, 0], p1: [0.0435, 0, 0.105] },
      planeNormal: [0, 1, 0],
      movingSide: [1, 0, 1],
      kind: 'valley',
      angleDeg: 180,
      scope: 'all',
    };
    const o = detectOrientation(applyOp(createSheet(A4_80GSM), corner).model, DEFAULT);
    expect(o.detected).toBe(false);
    close(o.up, [0, 1, 0]);
  });
});
