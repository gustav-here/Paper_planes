import { describe, expect, it } from 'vitest';
import { reachEdges } from '../../../src/core/paper/cut';
import { replayDesign } from '../../../src/core/paper/replay';
import { makeDart } from '../../../src/core/presets';
import type { Vec3 } from '../../../src/core/math/vec3';

/* The flat dart's keel lies in the plane y = 0, from its fold (z = 0) up to the wing root (z = 25 mm). */
const KEEL = 0.025;
const model = replayDesign(makeDart()).models.at(-1)!;
const N: Vec3 = [0, 1, 0];

describe('reachEdges: a cut drawn up to an edge goes through it', () => {
  it('carries ends that stop just short of the keel fold and the wing root past them', () => {
    const [a, b] = reachEdges(model, [[-0.08, 0, 0.0005], [-0.08, 0, KEEL - 0.001]], N, 0.002);
    expect(a[2]).toBeLessThan(0);
    expect(a[2]).toBeGreaterThan(-0.001);
    expect(b[2]).toBeGreaterThan(KEEL);
    expect(b[2]).toBeLessThan(KEEL + 0.001);
    // only along the cut
    expect(a[0]).toBeCloseTo(-0.08, 9);
    expect(b[0]).toBeCloseTo(-0.08, 9);
  });

  it('leaves ends that stop well inside the paper, or off it, as drawn', () => {
    const inside: Vec3[] = [[-0.08, 0, 0.005], [-0.08, 0, 0.02]];
    expect(reachEdges(model, inside, N, 0.002)).toEqual(inside);
    const off: Vec3[] = [[-0.08, 0, -0.01], [-0.08, 0, KEEL + 0.003]];
    expect(reachEdges(model, off, N, 0.002)).toEqual(off);
  });
});
