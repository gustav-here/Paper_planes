import { describe, expect, it } from 'vitest';
import * as V from '../../../src/core/math/vec3';
import { makeDart, makeGlider } from '../../../src/core/presets';
import { replayDesign, applyOp } from '../../../src/core/paper/replay';
import { relaxCreases } from '../../../src/core/paper/springback';
import { CLIP_SPEC, findClipPlacement, TAPE_SPEC } from '../../../src/core/paper/clips';
import { facetCentroid, facetNormal, totalArea } from '../../../src/core/paper/model';
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
    // both halves turn away from the middle: one by +gap/2, the other by −gap/2
    const signed = r.facets.filter((f) => Math.abs(facetNormal(f)[1]) > 0.9).map((f) => Math.asin(facetNormal(f)[2] * Math.sign(facetNormal(f)[1])) * DEG);
    expect(Math.max(...signed) + Math.min(...signed)).toBeCloseTo(0, 1);
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

  it('a clip or tape holds the centre fold shut only where it sits; further back it twists open', () => {
    // opening of the keel about the centre fold (along x) among keel facets within 2 mm of x
    // (a twisted surface also tilts a little along x; that is not the gap)
    const gapNear = (m: PaperModel, x: number, w = 0.002) => {
      const tilts = m.facets
        .filter((f) => Math.abs(facetNormal(f)[1]) > 0.9 && Math.abs(facetCentroid(f)[0] - x) < w)
        .map((f) => Math.asin(facetNormal(f)[2] * Math.sign(facetNormal(f)[1])) * DEG);
      return Math.max(...tilts) - Math.min(...tilts);
    };
    const free = relaxCreases(folded, dart.ops, 0.05);
    for (const spec of [TAPE_SPEC, CLIP_SPEC]) {
      const p = findClipPlacement(folded, [0.1, 0, 0], 0.02, undefined, spec)!;
      expect(p).not.toBeNull();
      const op = { type: 'clip' as const, id: 'h1', at: p.at, edgeDir: p.edgeDir, massKg: spec.massKg, ...(spec === TAPE_SPEC ? { kind: 'tape' as const } : {}) };
      const ops = [...dart.ops, op];
      const held = applyOp(folded, op).model;
      const r = relaxCreases(held, ops, 0.05);
      expect(gapNear(r, p.position[0])).toBeLessThan(0.5);
      // behind it the gap grows steadily (a tight V from the clip) to the full gap at the tail
      const x0 = p.position[0];
      const tail = -0.125;
      const g1 = gapNear(r, x0 + (tail - x0) / 3, 0.012);
      const g2 = gapNear(r, x0 + (2 * (tail - x0)) / 3, 0.012);
      const gEnd = gapNear(r, tail, 0.012);
      expect(g1).toBeGreaterThan(0.5);
      expect(g2).toBeGreaterThan(g1 + 1);
      expect(gEnd).toBeGreaterThan(g2 + 1);
      const full = gapNear(free, 0, 1);
      expect(gEnd).toBeLessThan(full + 0.3);
      expect(gEnd).toBeGreaterThan(0.6 * full);
      // the paper is not torn apart: it still covers the same area
      expect(totalArea(r)).toBeCloseTo(totalArea(held), 10);
    }
  });
});
