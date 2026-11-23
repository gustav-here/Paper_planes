import { describe, expect, it } from 'vitest';
import { crookedFolds, designSeed, replayCrooked } from '../../../src/core/paper/crooked';
import { replayDesign } from '../../../src/core/paper/replay';
import { facetCentroid, facetNormal, totalArea } from '../../../src/core/paper/model';
import { makeDart, trimmedDart } from '../../../src/core/presets';
import { prepare } from '../../../src/core/physics/factory';
import { DEFAULT_CONFIG } from '../../../src/core/physics/config';
import * as V from '../../../src/core/math/vec3';
import type { FoldOp } from '../../../src/core/paper/ops';

const fold: FoldOp = { type: 'fold', id: 'x', line: { p0: [-0.1, 0, 0], p1: [0.1, 0, 0] }, planeNormal: [0, 1, 0], movingSide: [0, 0, 1], kind: 'valley', angleDeg: 90, scope: 'all' };

describe('crooked folds: a hand never folds exactly along the line', () => {
  it('turn each fold line a little in its plane, the same way every time, twins on their own', () => {
    const skew = crookedFolds(42, 1);
    const a = skew(fold);
    expect(skew(fold)).toEqual(a);
    const dir = (o: FoldOp) => V.normalize(V.sub(o.line.p1, o.line.p0));
    const angle = Math.acos(Math.min(1, V.dot(dir(a), dir(fold)))) * (180 / Math.PI);
    expect(angle).toBeGreaterThan(0);
    expect(angle).toBeLessThanOrEqual(2.5 + 1e-9);
    // still in the fold's plane (y = 0) and about the same middle
    expect(Math.abs(a.line.p0[1])).toBeLessThan(1e-12);
    expect(V.length(V.lerp(a.line.p0, a.line.p1, 0.5))).toBeLessThan(1e-12);
    const twin = skew({ ...fold, id: "x'" });
    expect(dir(twin)).not.toEqual(dir(a));
  });

  it('fold a whole plane crooked: same paper, no longer mirror-symmetric', () => {
    for (const d of [makeDart(), trimmedDart()]) {
      const exact = replayDesign(d).models.at(-1)!;
      const crooked = replayCrooked(d, designSeed(d.ops), 1)!;
      expect(crooked).not.toBeNull();
      expect(totalArea(crooked)).toBeCloseTo(totalArea(exact), 9);
      // the wings' incidence (fore-aft tilt of their normals) differs left and right
      const tilt = (m: typeof exact, side: number) => {
        const ws = m.facets.filter((f) => Math.abs(facetNormal(f)[2]) > 0.9 && Math.sign(facetCentroid(f)[1]) === side);
        return ws.reduce((a, f) => a + Math.abs(facetNormal(f)[0]), 0) / Math.max(ws.length, 1);
      };
      expect(Math.abs(tilt(crooked, 1) - tilt(crooked, -1))).toBeGreaterThan(1e-3);
      expect(Math.abs(tilt(exact, 1) - tilt(exact, -1))).toBeLessThan(1e-9);
    }
  });

  it('prepare folds exactly unless asked (tests and the wind tunnel stay exact)', () => {
    const d = makeDart();
    const a = prepare(d, DEFAULT_CONFIG).folded;
    const b = prepare(d, DEFAULT_CONFIG, { crooked: true }).folded;
    expect(a).toEqual(replayDesign(d).models.at(-1)!);
    expect(b).not.toEqual(a);
    expect(prepare(d, DEFAULT_CONFIG, { crooked: true }).folded).toEqual(b);
  });
});
