import { describe, expect, it } from 'vitest';
import * as V from '../../../src/core/math/vec3';
import { makeDart, makeGlider } from '../../../src/core/presets';
import { replayDesign, applyOp } from '../../../src/core/paper/replay';
import { relaxCreases } from '../../../src/core/paper/springback';
import { findClipPlacement, TAPE_SPEC } from '../../../src/core/paper/clips';
import { facetNormal, totalArea } from '../../../src/core/paper/model';
import type { PaperModel } from '../../../src/core/paper/model';
import { computeMassProperties } from '../../../src/core/physics/massProperties';

const DEG = 180 / Math.PI;

/** Angle (deg) between the two keel halves of a plane folded along the long centre line. */
function keelGap(m: PaperModel): number {
  // keel facets lie in the band 0 < z < keel next to the centre fold, one on each side
  const keel = m.facets.filter((f) => f.foldPath.length > 0 && Math.abs(facetNormal(f)[1]) > 0.9);
  let gap = 0;
  for (const a of keel) for (const b of keel) gap = Math.max(gap, Math.acos(Math.min(1, Math.abs(V.dot(facetNormal(a), facetNormal(b))))) * DEG);
  return gap;
}

describe('crease springback', () => {
  const dart = makeDart();
  const folded = replayDesign(dart).models.at(-1)!;

  it('a firmly pressed 180° centre fold gapes a few degrees, symmetrically', () => {
    const r = relaxCreases(folded, dart.ops, 0.05);
    expect(keelGap(folded)).toBeLessThan(0.01);
    const gap = keelGap(r);
    expect(gap).toBeGreaterThan(7);
    expect(gap).toBeLessThan(11);
    // the plane stays where it was: the two halves open by the same amount either way
    const tilts = r.facets
      .filter((f) => Math.abs(facetNormal(f)[1]) > 0.9)
      .map((f) => Math.asin(Math.abs(facetNormal(f)[2])) * DEG);
    expect(Math.max(...tilts) - Math.min(...tilts)).toBeLessThan(0.5);
  });

  it('keeps paper, mass and centre of mass (it only bends)', () => {
    for (const d of [makeDart(), makeGlider()]) {
      const m = replayDesign(d).models.at(-1)!;
      const r = relaxCreases(m, d.ops, 0.05);
      expect(totalArea(r)).toBeCloseTo(totalArea(m), 12);
      const a = computeMassProperties(m);
      const b = computeMassProperties(r);
      expect(b.mass).toBeCloseTo(a.mass, 12);
      expect(V.dist(a.com, b.com)).toBeLessThan(0.002);
    }
  });

  it('no springback means the folded model, and it is deterministic', () => {
    expect(relaxCreases(folded, dart.ops, 0)).toBe(folded);
    expect(JSON.stringify(relaxCreases(folded, dart.ops, 0.05))).toBe(JSON.stringify(relaxCreases(folded, dart.ops, 0.05)));
  });

  it('tape over the keel edge holds the centre fold shut', () => {
    const p = findClipPlacement(folded, [-0.1, 0, 0], 0.02, undefined, TAPE_SPEC)!;
    expect(p).not.toBeNull();
    const ops = [...dart.ops, { type: 'clip' as const, id: 'tejp1', at: p.at, edgeDir: p.edgeDir, massKg: TAPE_SPEC.massKg, kind: 'tape' as const }];
    const taped = applyOp(folded, ops.at(-1)!).model;
    expect(keelGap(relaxCreases(taped, ops, 0.05))).toBeLessThan(0.01);
  });
});
