import { describe, expect, it } from 'vitest';
import { PRESETS, makeDart } from '../../../src/core/presets';
import { replayDesign } from '../../../src/core/paper/replay';
import { validateDesign } from '../../../src/core/paper/ops';
import { facetVertices, totalArea } from '../../../src/core/paper/model';
import { findSymmetry } from '../../../src/core/paper/symmetry';
import { findClipPlacement, resolveClip } from '../../../src/core/paper/clips';
import { applyOp } from '../../../src/core/paper/replay';
import { maxLayers } from '../../../src/core/paper/stack';
import { PaperError } from '../../../src/core/paper/errors';

const A4_AREA = 0.21 * 0.297;

describe.each(PRESETS)('preset $name', ({ design }) => {
  it('replays without errors or penetration warnings, conserves area and symmetry', () => {
    const { models, warnings } = replayDesign(design, { penetration: 'sweep' });
    const m = models[models.length - 1];
    expect(warnings.flat().map((w) => w.code)).not.toContain('penetration');
    expect(warnings.flat().map((w) => w.code)).not.toContain('mirrorUnavailable');
    expect(totalArea(m)).toBeCloseTo(A4_AREA, 10);
    expect(findSymmetry(m)).not.toBeNull();
  });

  it('survives a JSON round trip with identical geometry', () => {
    const copy = validateDesign(JSON.parse(JSON.stringify(design)));
    const a = replayDesign(design).models.at(-1)!;
    const b = replayDesign(copy).models.at(-1)!;
    expect(JSON.stringify(b.facets)).toBe(JSON.stringify(a.facets));
  });
});

describe('dart geometry', () => {
  const m = replayDesign(makeDart()).models.at(-1)!;

  it('has wings on both sides of the keel and many layers at the nose', () => {
    const vs = m.facets.flatMap(facetVertices);
    const ys = vs.map((v) => v[1]);
    expect(Math.max(...ys)).toBeGreaterThan(0.05);
    expect(Math.min(...ys)).toBeLessThan(-0.05);
    expect(maxLayers(m)).toBeGreaterThanOrEqual(6);
  });

  it('accepts a clip on the keel edge that clamps several layers', () => {
    const p = findClipPlacement(m, [0.08, 0, 0], 0.01);
    expect(p).not.toBeNull();
    const withClip = applyOp(m, { type: 'clip', id: 'c1', at: p!.at, edgeDir: p!.edgeDir, massKg: 0.001 }).model;
    const r = resolveClip(withClip, withClip.clips[0])!;
    expect(r.clamped.length).toBeGreaterThanOrEqual(2);
    expect(Math.abs(r.position[2])).toBeLessThan(1e-9); // on the spine
  });

  it('rejects a clip that is not on an edge', () => {
    expect(() => applyOp(m, { type: 'clip', id: 'x', at: [0.3, 0.3], edgeDir: [1, 0], massKg: 0.001 })).toThrow(PaperError);
  });
});

describe('design validation', () => {
  it('rejects malformed files', () => {
    expect(() => validateDesign({ format: 'nope' })).toThrow(PaperError);
    const d = JSON.parse(JSON.stringify(makeDart()));
    d.ops[0].angleDeg = 400;
    expect(() => validateDesign(d)).toThrow(/angleDeg/);
  });
});
